
#include <gnss_comm/gnss_spp.hpp>

#include "StateManager.h"

#include "GvioAligner.h"

#include "GnssUpdate.h"

#include "GnssManager.h"

namespace ingvio
{
    void GnssUpdate::checkYofStatus(std::shared_ptr<State> state, 
                                    std::shared_ptr<GvioAligner> gvio_aligner)
    {
        if (!state->_state_params._enable_gnss || !gvio_aligner->isAlign())
            return;
        
        if (state->_gnss.find(State::GNSSType::YOF) == state->_gnss.end())
            StateManager::addGNSSVariable(state, State::GNSSType::YOF,
                                          gvio_aligner->getYawOffset(), 
                                          state->_state_params._init_cov_yof);
    }
    
    
    void GnssUpdate::getSysInGnssMeas(const GnssMeas& gnss_meas)
    {
        _gnss_sys.clear();
        _gnss_sys.insert(State::GNSSType::YOF);
        
        bool flag = false;
        
        for (const gnss_comm::ObsPtr& obs : gnss_meas.first)
            if (_gnss_sys.find(GnssManager::convertSatsys2GnssType(gnss_comm::satsys(obs->sat, NULL))) == _gnss_sys.end())
            {
                flag = true;
                _gnss_sys.insert(GnssManager::convertSatsys2GnssType(gnss_comm::satsys(obs->sat, NULL)));
            }
            else
                continue;
            
        if (flag)
            _gnss_sys.insert(State::GNSSType::FS);
    }
    
    void GnssUpdate::removeUntrackedSys(std::shared_ptr<State> state, 
                                        const GnssMeas& gnss_meas)
    {
        if (!state->_state_params._enable_gnss)
            return;
        
        this->getSysInGnssMeas(gnss_meas);
        
        std::vector<State::GNSSType> gstate_to_marg;
        
        for (auto& gnss_state : state->_gnss)
            if (_gnss_sys.find(gnss_state.first) == _gnss_sys.end())
                gstate_to_marg.push_back(gnss_state.first);
            
        for (const auto& gs : gstate_to_marg)
            StateManager::margGNSSVariable(state, gs);
    }
    
    void GnssUpdate::updateTrackedSys(std::shared_ptr<State> state,
                                      const GnssMeas& gnss_meas,
                                      std::shared_ptr<GvioAligner> gvio_aligner,
                                      const std::vector<double>& iono_params)
    {
        if (!state->_state_params._enable_gnss || !gvio_aligner->isAlign())
            return;
        
        if (gnss_meas.first.size() <= 0 || iono_params.size() != 8)
            return;
        
        if (!GnssManager::checkGnssStates(state))
            return;
        
        std::vector<gnss_comm::SatStatePtr> all_sat_states =
            gnss_comm::sat_states(gnss_meas.first, gnss_meas.second);
        
        Eigen::Matrix<double, 7, 1> xyzt;
        
        xyzt.block<3, 1>(0, 0) = gvio_aligner->getTenu2ecef()*GnssManager::calcTw2enu(state->_gnss.at(State::GNSSType::YOF)->value())*state->_extended_pose->valueTrans1();
        
        xyzt.block<4, 1>(3, 0) = GnssManager::getClockbiasVec(state);
        
        Eigen::Matrix<double, 4, 1> dopp;
        
        dopp.block<3, 1>(0, 0) = gvio_aligner->getRenu2ecef()*GnssManager::calcRw2enu(state->_gnss.at(State::GNSSType::YOF)->value())*state->_extended_pose->valueTrans2();
        
        dopp(3, 0) = state->_gnss.at(State::GNSSType::FS)->value();
        
        std::vector<Eigen::Vector2d> atmos_delay;
        std::vector<Eigen::Vector2d> all_sv_azel;
        
        Eigen::VectorXd res_pos;
        Eigen::VectorXd res_vel;
        Eigen::MatrixXd J_pos_ecef;
        Eigen::MatrixXd J_vel_ecef;
        
        gnss_comm::psr_res(xyzt, gnss_meas.first, all_sat_states, iono_params, res_pos, J_pos_ecef, atmos_delay, all_sv_azel);
        gnss_comm::dopp_res(dopp, xyzt.block<3, 1>(0, 0), gnss_meas.first, all_sat_states, res_vel, J_vel_ecef);
        
        std::vector<std::shared_ptr<Type>> var_order;
        std::map<std::shared_ptr<Type>, int> local_var_index;
        
        var_order.push_back(state->_extended_pose);
        local_var_index[state->_extended_pose] = 0;
        
        var_order.push_back(state->_gnss.at(State::GNSSType::YOF));
        local_var_index[state->_gnss.at(State::GNSSType::YOF)] = 9;
        
        const int max_possible_rows = 2*gnss_meas.first.size();
        const int max_possible_cols = state->_extended_pose->size() + 6;
        
        const Eigen::Matrix3d Rw2ecef = gvio_aligner->getRenu2ecef()*GnssManager::calcRw2enu(state);
        
        Eigen::VectorXd res(max_possible_rows);
        Eigen::MatrixXd H(max_possible_rows, max_possible_cols);
        Eigen::MatrixXd R(max_possible_rows, max_possible_rows);
        res.setZero();
        H.setZero();
        R.setZero();
        
        int row_cnt = 0;
        int col_cnt = 10;
        
        for (int i = 0; i < res_pos.rows(); ++i)
        {
            const Eigen::Vector3d unit_rv2sv = -J_pos_ecef.block<1, 3>(i, 0).transpose();
            const uint32_t sys = gnss_comm::satsys(all_sat_states[i]->sat_id, NULL);
            
            if (state->_gnss.find(GnssManager::satsys_to_gnsstype.at(sys)) == state->_gnss.end())
                continue;
            
            auto cb_state = state->_gnss.at(GnssManager::satsys_to_gnsstype.at(sys));
            
            Eigen::MatrixXd H_i(1, state->_extended_pose->size()+2);
            H_i.setZero();
            
            H_i.block<1, 3>(0, 0) = unit_rv2sv.transpose()*Rw2ecef*skew(state->_extended_pose->valueTrans1());
            H_i.block<1, 3>(0, 3) = -unit_rv2sv.transpose()*Rw2ecef;
            
            if (_is_adjust_yof)
            {
                H_i(0, 9) = -unit_rv2sv.transpose()*gvio_aligner->getRenu2ecef()*GnssManager::dotRw2enu(state)*state->_extended_pose->valueTrans1();
            }
            
            H_i(0, 10) = 1.0;
            
            Eigen::VectorXd res_i(1);
            res_i(0) = -res_pos(i);
            
            std::vector<std::shared_ptr<Type>> sub_order(3);
            
            sub_order[0] = state->_extended_pose;
            sub_order[1] = state->_gnss.at(State::GNSSType::YOF);
            sub_order[2] = cb_state;
            
            double ns = gnss_meas.second[i]->ura;
            int l1_idx = -1;
            gnss_comm::L1_freq(gnss_meas.first[i], &l1_idx);
            double npr = gnss_meas.first[i]->psr_std[l1_idx];
            double sin_el = std::sin(all_sv_azel[i].y());
            if (std::fabs(sin_el) < 1e-6) sin_el = 1e-6;
            
            double psr_noise = _psr_noise_amp*std::pow(ns*npr/(sin_el*sin_el), 0.5);
            
 
            if (_is_gnss_chi2_test && !testChiSquared(state, res_i, H_i, sub_order, psr_noise))
                continue;
            
            
            res(row_cnt, 0) = res_i(0);
            R(row_cnt, row_cnt) = psr_noise*psr_noise;
            
            H.block<1, 9>(row_cnt, local_var_index.at(state->_extended_pose)) = H_i.block<1, 9>(0, 0);
            H(row_cnt, local_var_index.at(state->_gnss.at(State::GNSSType::YOF))) = H_i(0, 9);
            
            if (local_var_index.find(cb_state) == local_var_index.end())
            {
                local_var_index[cb_state] = col_cnt;
                col_cnt += cb_state->size();
                
                var_order.push_back(cb_state);
            }
            
            H(row_cnt, local_var_index.at(cb_state)) = 1.0;
            
            ++row_cnt;
        }
        
        auto cs_state = state->_gnss.at(State::GNSSType::FS);
        
        local_var_index[cs_state] = col_cnt;
        col_cnt += cs_state->size();
        
        var_order.push_back(cs_state);
        
        for (int i = 0; i < res_vel.rows(); ++i)
        {
            const Eigen::Vector3d unit_rv2sv = -J_vel_ecef.block<1, 3>(i, 0).transpose();
            const uint32_t sys = gnss_comm::satsys(all_sat_states[i]->sat_id, NULL);
            
            if (state->_gnss.find(GnssManager::satsys_to_gnsstype.at(sys)) == state->_gnss.end())
                continue;
            
            std::vector<std::shared_ptr<Type>> sub_order(3);
            sub_order[0] = state->_extended_pose;
            sub_order[1] = state->_gnss.at(State::GNSSType::YOF);
            sub_order[2] = state->_gnss.at(State::GNSSType::FS);
            
            Eigen::MatrixXd H_i(1, 11);
            H_i.setZero();
            
            H_i.block<1, 3>(0, 0) = unit_rv2sv.transpose()*Rw2ecef*skew(state->_extended_pose->valueTrans2());
            H_i.block<1, 3>(0, 6) = -unit_rv2sv.transpose()*Rw2ecef;
            
            if (_is_adjust_yof)
            {
                H_i(0, 9) = -unit_rv2sv.transpose()*gvio_aligner->getRenu2ecef()*GnssManager::dotRw2enu(state)*state->_extended_pose->valueTrans2();
            }
            
            H_i(0, 10) = 1.0;
            
            Eigen::VectorXd res_i(1);
            res_i(0) = -res_vel(i);
            
            double ns = gnss_meas.second[i]->ura;
            int l1_idx = -1;
            const double obs_freq = gnss_comm::L1_freq(gnss_meas.first[i], &l1_idx);
            double ndp = gnss_meas.first[i]->dopp_std[l1_idx]*LIGHT_SPEED/obs_freq;
            double sin_el = std::sin(all_sv_azel[i].y());
            if (std::fabs(sin_el) < 1e-6) sin_el = 1e-6;
            
            double dopp_noise = _dopp_noise_amp*std::pow(ns*ndp/(sin_el*sin_el), 0.5);
            

            if (_is_gnss_chi2_test && !testChiSquared(state, res_i, H_i, sub_order, dopp_noise))
                continue;
            
            
            res(row_cnt, 0) = res_i(0);
            R(row_cnt, row_cnt) = dopp_noise*dopp_noise;
            
            H.block<1, 9>(row_cnt, local_var_index.at(state->_extended_pose)) = H_i.block<1, 9>(0, 0);
            H(row_cnt, local_var_index.at(state->_gnss.at(State::GNSSType::YOF))) = H_i(0, 9);
            
            H(row_cnt, local_var_index.at(state->_gnss.at(State::GNSSType::FS))) = H_i(0, 10);
            
            ++row_cnt;
        }
        
        if (row_cnt < max_possible_rows)
        {
            res.conservativeResize(row_cnt, 1);
            
            R.conservativeResize(row_cnt, row_cnt);
            
            H.conservativeResize(row_cnt, H.cols());
        }
        
        if (col_cnt < max_possible_cols)
            H.conservativeResize(H.rows(), col_cnt);
        
        StateManager::ekfUpdate(state, var_order, H, res, R);
        
        // GnssManager::printYOF(state);
    }
    
    void GnssUpdate::getSysInSppMeas(const SppMeas& spp_meas)
    {
        _spp_sys.clear();
        
        if (std::fabs(spp_meas.velSpp(3, 0)) > 1e-03)
            _spp_sys.insert(State::GNSSType::FS);
        
        for (int i = 0; i < 4; ++i)
            if (std::fabs(spp_meas.posSpp(3+i, 0)) > 1e-03)
                _spp_sys.insert(GnssManager::idx_to_gnsstype.at(i));
    }
    
    void GnssUpdate::calcSysToAdd(const std::shared_ptr<State> state,
                                  std::unordered_set<State::GNSSType>& sys_to_add)
    {
        sys_to_add.clear();
        
        for (const auto& gtype : _spp_sys)
            if (state->_gnss.find(gtype) == state->_gnss.end())
                sys_to_add.insert(gtype);
    }
    
    void GnssUpdate::addNewTrackedSys(std::shared_ptr<State> state,
                                      const GnssMeas& gnss_meas,
                                      const SppMeas& spp_meas,
                                      std::shared_ptr<GvioAligner> gvio_aligner,
                                      const std::vector<double>& iono_params)
    {
        if (!state->_state_params._enable_gnss || !gvio_aligner->isAlign())
            return;
        
        if (gnss_meas.first.size() <= 0 || iono_params.size() != 8)
            return;
        
        if (state->_gnss.find(State::GNSSType::YOF) == state->_gnss.end())
            return;
        
        const Eigen::Matrix3d Rw2ecef = gvio_aligner->getRenu2ecef()*GnssManager::calcRw2enu(state);
        
        this->getSysInSppMeas(spp_meas);
        
        std::unordered_set<State::GNSSType> sys_to_add;
        this->calcSysToAdd(state, sys_to_add);
        
        if (sys_to_add.size() == 0)
            return;
        
        std::vector<gnss_comm::SatStatePtr> all_sat_states =
        gnss_comm::sat_states(gnss_meas.first, gnss_meas.second);
        
        Eigen::Matrix<double, 7, 1> xyzt;
        
        xyzt.block<3, 1>(0, 0) = gvio_aligner->getTenu2ecef()*GnssManager::calcTw2enu(state->_gnss.at(State::GNSSType::YOF)->value())*state->_extended_pose->valueTrans1();
        
        xyzt.block<4, 1>(3, 0) = GnssManager::getClockbiasVec(state);
        
        for (const auto& item : sys_to_add)
            if (item != State::GNSSType::FS)
            {
                int idx_to_add = 3 + gnss_comm::sys2idx.at(GnssManager::gnsstype_to_satsys.at(item));
                xyzt(idx_to_add, 0) = spp_meas.posSpp(idx_to_add, 0);
            }
        
        Eigen::Matrix<double, 4, 1> dopp = Eigen::Matrix<double, 4, 1>::Zero();
        
        dopp.block<3, 1>(0, 0) = gvio_aligner->getRenu2ecef()*GnssManager::calcRw2enu(state->_gnss.at(State::GNSSType::YOF)->value())*state->_extended_pose->valueTrans2();
        
        if (sys_to_add.find(State::GNSSType::FS) == sys_to_add.end())
        {
            if (state->_gnss.find(State::GNSSType::FS) == state->_gnss.end())
                std::cout << "[GnssUpdate]: Fs is either added or in the state!" << std::endl;
            else
                dopp(3, 0) = state->_gnss.at(State::GNSSType::FS)->value();
        }
        else
            dopp(3, 0) = spp_meas.velSpp(3, 0);
        
        std::vector<Eigen::Vector2d> atmos_delay;
        std::vector<Eigen::Vector2d> all_sv_azel;
        
        Eigen::VectorXd res_pos;
        Eigen::VectorXd res_vel;
        Eigen::MatrixXd J_pos_ecef;
        Eigen::MatrixXd J_vel_ecef;
        
        gnss_comm::psr_res(xyzt, gnss_meas.first, all_sat_states, iono_params, res_pos, J_pos_ecef, atmos_delay, all_sv_azel);
        gnss_comm::dopp_res(dopp, xyzt.block<3, 1>(0, 0), gnss_meas.first, all_sat_states, res_vel, J_vel_ecef);
    
        
        for (const auto& gtype : sys_to_add)
            if (gtype == State::GNSSType::FS)
            {
                Eigen::VectorXd res = -res_vel;
                
                Eigen::MatrixXd Hf(res.rows(), 1);
                Hf.setOnes();
                
                Eigen::MatrixXd Hx(res.rows(), state->_extended_pose->size()+1);
                Hx.setZero();
                
                Eigen::MatrixXd batch_unit_rv2sv_T = -J_vel_ecef.block(0, 0, J_vel_ecef.rows(), 3);
                
                Hx.block(0, 0, Hx.rows(), 3) = batch_unit_rv2sv_T*Rw2ecef*skew(state->_extended_pose->valueTrans2());
                
                Hx.block(0, 6, Hx.rows(), 3) = -batch_unit_rv2sv_T*Rw2ecef;
                
                if (_is_adjust_yof)
                {
                    Hx.block(0, 9, Hx.rows(), 1) = -batch_unit_rv2sv_T*gvio_aligner->getRecef2enu()*GnssManager::dotRw2enu(state)*state->_extended_pose->valueTrans2();
                }
                
                std::vector<std::shared_ptr<Type>> x_order(2);
                x_order[0] = state->_extended_pose;
                x_order[1] = state->_gnss.at(State::GNSSType::YOF);
                
                std::shared_ptr<Scalar> fs = std::make_shared<Scalar>();
                fs->setValue(spp_meas.velSpp(3, 0));
                
                double avg_noise = 0.0;
                
                for (int i = 0; i < gnss_meas.second.size(); ++i)
                {
                    double ns = gnss_meas.second[i]->ura;
                    int l1_idx = -1;
                    const double obs_freq = gnss_comm::L1_freq(gnss_meas.first[i], &l1_idx);
                    double ndp = gnss_meas.first[i]->dopp_std[l1_idx]*LIGHT_SPEED/obs_freq;
                    double sin_el = std::sin(all_sv_azel[i].y());
                    if (std::fabs(sin_el) < 1e-6) sin_el = 1e-6;
                    
                    avg_noise += ns*ndp/(sin_el*sin_el);
                }
                
                avg_noise /= gnss_meas.second.size();
                avg_noise = _dopp_noise_amp*std::sqrt(avg_noise);
                
                if (!StateManager::addVariableDelayed(state, fs, x_order, Hx, Hf, res,
                    avg_noise, 0.95, true))
                    continue;
                
                state->_gnss[State::GNSSType::FS] = fs;
            }
            else
            {
                Eigen::VectorXd res;
                Eigen::MatrixXd batch_unit_rv2sv_T;
                double avg_noise;
                
                this->getResJacobianOfSys(gtype, gnss_meas, all_sv_azel,
                                          res_pos, J_pos_ecef,
                                          res, batch_unit_rv2sv_T, avg_noise);
                
                avg_noise *= _psr_noise_amp;
                
                Eigen::MatrixXd Hf(res.rows(), 1);
                Hf.setOnes();
                
                std::vector<std::shared_ptr<Type>> x_order(2);
                x_order[0] = state->_extended_pose;
                x_order[1] = state->_gnss.at(State::GNSSType::YOF);
                
                Eigen::MatrixXd Hx(res.rows(), 10);
                Hx.setZero();
                
                Hx.block(0, 0, Hx.rows(), 3) = batch_unit_rv2sv_T*Rw2ecef*skew(state->_extended_pose->valueTrans1());
                
                Hx.block(0, 3, Hx.rows(), 3) = -batch_unit_rv2sv_T*Rw2ecef;
                
                if (_is_adjust_yof)
                {
                    Hx.block(0, 9, Hx.rows(), 1) = -batch_unit_rv2sv_T*gvio_aligner->getRecef2enu()*GnssManager::dotRw2enu(state)*state->_extended_pose->valueTrans1();
                }
                
                std::shared_ptr<Scalar> cb = std::make_shared<Scalar>();
                cb->setValue(spp_meas.posSpp(3+gnss_comm::sys2idx.at(GnssManager::gnsstype_to_satsys.at(gtype)), 0));
                
                if (!StateManager::addVariableDelayed(state, cb, x_order, Hx, Hf, res,
                    avg_noise, 0.95, true))
                    continue;
                
                state->_gnss[gtype] = cb;
            }
    }
    
    void GnssUpdate::getResJacobianOfSys(const State::GNSSType& gtype,
                                         const GnssMeas& gnss_meas,
                                         const std::vector<Eigen::Vector2d>& all_sv_azel,
                                         const Eigen::VectorXd& res_total,
                                         const Eigen::MatrixXd& H_total,
                                         Eigen::VectorXd& res_sys,
                                         Eigen::MatrixXd& batch_unit_rv2sv_T_sys,
                                         double& noise)
    {
        assert(H_total.cols() == 7);
        
        const int col_idx = 3 + gnss_comm::sys2idx.at(GnssManager::gnsstype_to_satsys.at(gtype));
        
        std::vector<int> this_sys_row;
        
        for (int i = 0; i < H_total.rows(); ++i)
            if (H_total(i, col_idx) == 1.0)
                this_sys_row.push_back(i);
            
        res_sys = Eigen::VectorXd::Zero(this_sys_row.size());
        batch_unit_rv2sv_T_sys = Eigen::MatrixXd::Zero(this_sys_row.size(), 3);
        
        int row_cnt = 0;
        noise = 0.0;
        for (const int& row_idx : this_sys_row)
        {
            res_sys(row_cnt) = -res_total(row_idx);
            batch_unit_rv2sv_T_sys.row(row_cnt) = -H_total.block<1, 3>(row_idx, 0);
            
            ++row_cnt;
            
            double ns = gnss_meas.second[row_idx]->ura;
            int l1_idx = -1;
            gnss_comm::L1_freq(gnss_meas.first[row_idx], &l1_idx);
            double npr = gnss_meas.first[row_idx]->psr_std[l1_idx];
            double sin_el = std::sin(all_sv_azel[row_idx].y());
            if (std::fabs(sin_el) < 1e-6) sin_el = 1e-6;
            
            noise += ns*npr/(sin_el*sin_el);
        }
        
        noise /= this_sys_row.size();
        noise = std::sqrt(noise);
    }
}

