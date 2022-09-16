#include <boost/math/distributions/chi_squared.hpp>

#include "StateManager.h"

namespace ingvio
{
    bool StateManager::checkStateContinuity(const std::shared_ptr<State> state)
    {
        int idx = 0;
        for (int i = 0; i < state->_err_variables.size(); ++i)
            if (state->_err_variables[i]->idx() == idx)
                idx += state->_err_variables[i]->size();
            else
                return false;
            
        if (state->_cov.rows() == idx && state->_cov.cols() == idx)
            return true;
        else
            return false;
    }  
    
    void StateManager::propagateStateCov(std::shared_ptr<State> state, const Eigen::Matrix<double, 15, 15>& Phi_imu, const Eigen::Matrix<double, 15, 12>& G_imu, double dt)
    {
        assert(StateManager::checkStateContinuity(state));
        
        Eigen::MatrixXd cov_tmp(state->curr_cov_size(), state->curr_cov_size());
        
        cov_tmp.block<15, 15>(0, 0) = Phi_imu*state->_cov.block<15, 15>(0, 0)*Phi_imu.transpose();
        
        Eigen::MatrixXd cov21 = state->_cov.block(15, 0, state->_cov.rows()-15, 15)*Phi_imu.transpose();
        Eigen::MatrixXd cov22 = state->_cov.block(15, 15, state->_cov.rows()-15, state->_cov.cols()-15);
        
        if (state->_state_params._enable_gnss && state->_gnss.find(State::GNSSType::FS) != state->_gnss.end())
        {
            Eigen::MatrixXd cov21_tmp = cov21;
            Eigen::MatrixXd cov22_tmp = cov22;
            
            const int local_col_idx = state->_gnss.find(State::GNSSType::FS)->second->idx() - 15;
            
            for (int i = 0; i < 4; ++i)
            {
                auto item = state->_gnss.find(static_cast<State::GNSSType>(i));
                if (item != state->_gnss.end())
                {
                    const int local_row_idx = item->second->idx() - 15;                    
                    cov21_tmp.row(local_row_idx) += dt*cov21.row(local_col_idx);
                    cov22_tmp.row(local_row_idx) += dt*cov22.row(local_col_idx);
                }
            }
            cov21 = cov21_tmp;
            
            cov22 = cov22_tmp;
            for (int i = 0; i < 4; ++i)
            {
                auto item = state->_gnss.find(static_cast<State::GNSSType>(i));
                if (item != state->_gnss.end())
                {
                    const int local_row_idx = item->second->idx() - 15;                    
                    cov22_tmp.col(local_row_idx) += dt*cov22.col(local_col_idx);
                }
            }
            cov22 = cov22_tmp;
        }
        
        cov_tmp.block(15, 0, cov_tmp.rows()-15, 15) = cov21;
        cov_tmp.block(0, 15, 15, cov_tmp.cols()-15) = cov21.transpose();
        cov_tmp.block(15, 15, cov_tmp.rows()-15, cov_tmp.cols()-15) = cov22;
        
        Eigen::Matrix<double, 15, 12> G_tmp = G_imu;
        G_tmp.middleCols<3>(0) *= state->_state_params._noise_g;
        G_tmp.middleCols<3>(3) *= state->_state_params._noise_a;
        G_tmp.middleCols<3>(6) *= state->_state_params._noise_bg;
        G_tmp.middleCols<3>(9) *= state->_state_params._noise_ba;
        cov_tmp.block<15, 15>(0, 0) += dt*Phi_imu*G_tmp*G_tmp.transpose()*Phi_imu.transpose();
        
        if (state->_state_params._enable_gnss)
            for (int i = 0; i < 5; ++i)
            {
                auto item1 = state->_gnss.find(static_cast<State::GNSSType>(i));
                if (item1 == state->_gnss.end()) continue;
                for (int j = 0; j < 5; ++j)
                {
                    auto item2 = state->_gnss.find(static_cast<State::GNSSType>(j));
                    if (item2 == state->_gnss.end()) continue;
                    
                    if (item1->first != State::GNSSType::FS && item2->first != State::GNSSType::FS)
                        cov_tmp(item1->second->idx(), item2->second->idx()) += dt*std::pow(state->_state_params._noise_clockbias, 2) + std::pow(dt, 3)*std::pow(state->_state_params._noise_cb_rw, 2);
                    else if (item1->first == State::GNSSType::FS && item2->first == State::GNSSType::FS)
                        cov_tmp(item1->second->idx(), item2->second->idx()) += dt*std::pow(state->_state_params._noise_cb_rw, 2); 
                    else
                        cov_tmp(item1->second->idx(), item2->second->idx()) += std::pow(dt, 2)*std::pow(state->_state_params._noise_cb_rw, 2);
                }
            }
            
        state->_cov = 0.5*(cov_tmp + cov_tmp.transpose());
    }
    
    Eigen::MatrixXd StateManager::getFullCov(std::shared_ptr<State> state)
    {
        Eigen::MatrixXd cov(state->curr_cov_size(), state->curr_cov_size());
        cov = state->_cov;
        return cov;
    }
    
    Eigen::MatrixXd StateManager::getMarginalCov(std::shared_ptr<State> state, const std::vector<std::shared_ptr<Type>>& small_variables)
    {
        int small_cov_size = 0;
        
        for (int i = 0; i < small_variables.size(); ++i)
            small_cov_size += small_variables[i]->size();
        
        Eigen::MatrixXd small_cov(small_cov_size, small_cov_size);
        
        int row_idx = 0;
        for (int i = 0; i < small_variables.size(); ++i)
        {
            int col_idx = 0;
            for(int j = 0; j < small_variables.size(); ++j)
            {
                small_cov.block(row_idx, col_idx, small_variables[i]->size(), small_variables[j]->size()) = state->_cov.block(small_variables[i]->idx(), small_variables[j]->idx(), small_variables[i]->size(), small_variables[j]->size());
                
                col_idx += small_variables[j]->size();
            }
            
            row_idx += small_variables[i]->size();
        }
        
        return small_cov;
    }
    
    void StateManager::marginalize(std::shared_ptr<State> state, std::shared_ptr<Type> marg)
    {
        if (std::find(state->_err_variables.begin(), state->_err_variables.end(), marg) == state->_err_variables.end() )
        {
            std::cout << "[StateManager]: Marg is not in the current state!" << std::endl;
            std::exit(EXIT_FAILURE);
        }
        
        int marg_size = marg->size();
        int marg_start = marg->idx();
        int marg_end = marg->idx() + marg->size();
        
        Eigen::MatrixXd cov_tmp(state->_cov.rows() - marg_size, state->_cov.cols() - marg_size);
        
        cov_tmp.block(0, 0, marg_start, marg_start) = state->_cov.block(0, 0, marg_start, marg_start);
        
        cov_tmp.block(marg_start, 0, cov_tmp.rows()-marg_start, marg_start) = state->_cov.block(marg_end, 0, cov_tmp.rows()-marg_start, marg_start);
        
        cov_tmp.block(0, marg_start, marg_start, cov_tmp.cols()-marg_start) = state->_cov.block(0, marg_end, marg_start, cov_tmp.cols()-marg_start);
        
        cov_tmp.block(marg_start, marg_start, cov_tmp.rows()-marg_start, cov_tmp.cols()-marg_start) = state->_cov.block(marg_end, marg_end, cov_tmp.rows()-marg_start, cov_tmp.cols()-marg_start);
        
        state->_cov = cov_tmp;
        
        std::vector<std::shared_ptr<Type>> remaining_variables;
        for (int i = 0; i < state->_err_variables.size(); ++i)
            if (state->_err_variables[i] != marg)
            {
                if (state->_err_variables[i]->idx() > marg_start)
                    state->_err_variables[i]->set_cov_idx(state->_err_variables[i]->idx()-marg_size);
                    
                remaining_variables.push_back(state->_err_variables[i]);
            }
            
        marg->set_cov_idx(-1);
        
        state->_err_variables = remaining_variables;
    }
    
    void StateManager::addVariableIndependent(std::shared_ptr<State> state, std::shared_ptr<Type> new_state, const Eigen::MatrixXd& new_state_cov_block)
    {
        assert(new_state->size() == new_state_cov_block.rows());
        
        int old_cov_size = state->curr_cov_size();
        
        Eigen::MatrixXd new_cov(old_cov_size+new_state->size(), old_cov_size+new_state->size());
        new_cov.setZero();
        
        new_cov.block(0, 0, old_cov_size, old_cov_size) = state->_cov;
        new_cov.block(old_cov_size, old_cov_size, new_state->size(), new_state->size()) = new_state_cov_block;
        
        new_state->set_cov_idx(old_cov_size);
        
        state->_cov = new_cov;
        state->_err_variables.push_back(new_state);
        
        assert(StateManager::checkStateContinuity(state));
    }
    
    void StateManager::addGNSSVariable(std::shared_ptr<State> state, const State::GNSSType& gtype, double value, double cov)
    {
        if (state->_gnss.find(gtype) != state->_gnss.end())
            std::cout << "[StateManager]: GNSS variable already in the state, adding operation will rewrite such var!" << std::endl;
        
        state->_gnss[gtype] = std::make_shared<Scalar>();
        state->_gnss[gtype]->setValue(value);
        
        Eigen::MatrixXd scalar_cov(1, 1);
        scalar_cov(0, 0) = cov;
        
        StateManager::addVariableIndependent(state, state->_gnss[gtype], scalar_cov);
    }
    
    void StateManager::margGNSSVariable(std::shared_ptr<State> state, const State::GNSSType& gtype)
    {
        if (state->_gnss.find(gtype) == state->_gnss.end())
            std::cout << "[StateManager]: GNSS variable not in the state, no need to marg!" << std::endl;
        
        StateManager::marginalize(state, state->_gnss.at(gtype));
        
        state->_gnss.erase(gtype);
    }
    
    void StateManager::boxPlus(std::shared_ptr<State> state, const Eigen::VectorXd& dx)
    {
        assert(dx.rows() == state->curr_cov_size());
        assert(StateManager::checkStateContinuity(state));
        
        for (int i = 0; i < state->_err_variables.size(); ++i)
            state->_err_variables[i]->update(dx);
    }
    
    void StateManager::augmentSlidingWindowPose(std::shared_ptr<State> state)
    {
        if (state->_sw_camleft_poses.find(state->_timestamp) != state->_sw_camleft_poses.end())
        {
            std::cout << "[StateManager]: Curr pose already in the sw, cannot clone!" << std::endl;
            return;
        }
        
        std::shared_ptr<SE3> leftcam_pose_clone = std::shared_ptr<SE3>(new SE3());
        
        Eigen::Isometry3d T_i2w = Eigen::Isometry3d::Identity();
        T_i2w.linear() = state->_extended_pose->valueLinearAsMat();
        T_i2w.translation() = state->_extended_pose->valueTrans1();
        
        Eigen::Isometry3d T_cl2i = Eigen::Isometry3d::Identity();
        T_cl2i.linear() = state->_camleft_imu_extrinsics->valueLinearAsMat();
        T_cl2i.translation() = state->_camleft_imu_extrinsics->valueTrans();
        
        leftcam_pose_clone->setValueByIso(T_i2w*T_cl2i);
        leftcam_pose_clone->setFejByIso(T_i2w*T_cl2i);
        
        leftcam_pose_clone->set_cov_idx(state->curr_cov_size());
        
        state->_sw_camleft_poses[state->_timestamp] = leftcam_pose_clone;
        state->_err_variables.push_back(leftcam_pose_clone);
        
        Eigen::Matrix<double, 6, 21> J = Eigen::Matrix<double, 6, 21>::Zero();
        J.block<6, 6>(0, 0).setIdentity();
        J.block<3, 3>(0, 15) = state->_extended_pose->valueLinearAsMat();
        J.block<3, 3>(3, 18) = state->_extended_pose->valueLinearAsMat();
        
        Eigen::MatrixXd cov_new(state->_cov.rows()+leftcam_pose_clone->size(), state->_cov.cols()+leftcam_pose_clone->size());
        cov_new.setZero();
        
        cov_new.block(0, 0, state->curr_cov_size(), state->curr_cov_size()) = state->_cov;
        cov_new.block(state->curr_cov_size(), state->curr_cov_size(), leftcam_pose_clone->size(), leftcam_pose_clone->size()) = J*state->_cov.block<21, 21>(0, 0)*J.transpose();
        
        cov_new.block(state->curr_cov_size(), 0, leftcam_pose_clone->size(), state->curr_cov_size()) = J*state->_cov.block(0, 0, 21, state->curr_cov_size());
        cov_new.block(0, state->curr_cov_size(), state->curr_cov_size(), leftcam_pose_clone->size()) = cov_new.block(state->curr_cov_size(), 0, leftcam_pose_clone->size(), state->curr_cov_size()).transpose();
        
        state->_cov = 0.5*(cov_new+cov_new.transpose());
        
        assert(checkStateContinuity(state));
    }
    
    void StateManager::addAnchoredLandmarkInState(std::shared_ptr<State> state, std::shared_ptr<AnchoredLandmark> anchored_landmark, int lm_id, const Eigen::Matrix3d& cov)
    {
        if (state->_anchored_landmarks.find(lm_id) != state->_anchored_landmarks.end())
        {
            std::cout << "[StateManager]: Landmark already in the state, cannot add!" << std::endl;
            return;
        }
        
        state->_anchored_landmarks[lm_id] = anchored_landmark;
        
        StateManager::addVariableIndependent(state, anchored_landmark, cov);
        
        assert(StateManager::checkStateContinuity(state));
    }
    
    void StateManager::margSlidingWindowPose(std::shared_ptr<State> state, double marg_time)
    {
        if (state->_sw_camleft_poses.find(marg_time) == state->_sw_camleft_poses.end())
            std::cout << "[StateManager]: Marg pose time not exists! Cannot marg!" << std::endl;
        
        StateManager::marginalize(state, state->_sw_camleft_poses.at(marg_time));
        
        state->_sw_camleft_poses.erase(marg_time);
        
        assert(StateManager::checkStateContinuity(state));
    }
    
    void StateManager::margSlidingWindowPose(std::shared_ptr<State> state)
    {
        double marg_time = state->nextMargTime();
        if (marg_time == INFINITY)
        {
            std::cout << "[StateManager]: Auto marg pose gives inf time! Cannot marg!" << std::endl;
            return;
        }
        
        StateManager::margSlidingWindowPose(state, marg_time);
    }
    
    void StateManager::margAnchoredLandmarkInState(std::shared_ptr<State> state, int lm_id)
    {
        if (state->_anchored_landmarks.find(lm_id) == state->_anchored_landmarks.end())
        {
            std::cout << "[StateManager]: Landmark id not exists in state! Cannot marg!" << std::endl;
            return;
        }
        
        StateManager::marginalize(state, state->_anchored_landmarks.at(lm_id));
        
        state->_anchored_landmarks.erase(lm_id);
        
        assert(StateManager::checkStateContinuity(state));
    }
    
    void StateManager::ekfUpdate(std::shared_ptr<State> state, const std::vector<std::shared_ptr<Type>>& var_order, const Eigen::MatrixXd& H, const Eigen::VectorXd& res, const Eigen::MatrixXd& R)
    {
        assert(res.rows() == R.rows());
        assert(H.rows() == res.rows());
        assert(R.rows() == R.cols());
        
        assert(StateManager::checkSubOrder(state, var_order));
        
        int small_idx = 0;
        std::vector<int> H_idx;
        
        for (int i = 0; i < var_order.size(); ++i)
        {
            H_idx.push_back(small_idx);
            
            small_idx += var_order[i]->size();
        }
        
        Eigen::MatrixXd PH_T(state->_cov.rows(), H.rows());
       
        PH_T.setZero();
        
        for (const auto& total_var : state->_err_variables)
        {
            Eigen::MatrixXd PH_T_i = Eigen::MatrixXd::Zero(total_var->size(), res.rows());
            
            for (int i = 0; i < var_order.size(); ++i)
            {
                std::shared_ptr<Type> meas_var = var_order[i];
                
                PH_T_i.noalias() += state->_cov.block(total_var->idx(), meas_var->idx(), total_var->size(), meas_var->size())*H.block(0, H_idx[i], H.rows(), meas_var->size()).transpose();
            }
            
            PH_T.block(total_var->idx(), 0, total_var->size(), res.rows()) = PH_T_i;
        }
        
        Eigen::MatrixXd small_cov = StateManager::getMarginalCov(state, var_order);
        
        Eigen::MatrixXd S(R.rows(), R.cols());
        
        S = H*small_cov*H.transpose() + R;
        
        Eigen::MatrixXd K = PH_T*S.inverse();
        
        Eigen::MatrixXd cov_tmp = state->_cov;
        
        cov_tmp -= K*PH_T.transpose();
        
        state->_cov = 0.5*(cov_tmp + cov_tmp.transpose());
        
        Eigen::VectorXd diags = state->_cov.diagonal();
        
        for (int i = 0; i < diags.rows(); ++i)
            if (diags(i) < 0.0)
            {
                std::cout << "[StateManager]: EKF Update and found negative diag cov elements! " << std::endl;
                
                assert(false);
            }
            
        Eigen::VectorXd dx = K*res;
        
        StateManager::boxPlus(state, dx);
    }
    
    bool StateManager::checkSubOrder(std::shared_ptr<State> state, const std::vector<std::shared_ptr<Type>>& sub_order)
    {
        auto isFound = [&state] (const std::shared_ptr<Type>& var)
        {
            return std::find(state->_err_variables.begin(), state->_err_variables.end(), var) !=  state->_err_variables.end();
        };
        
        for (const auto& item : sub_order)
            if (!isFound(item))
            {
                std::cout << "[StateManager]: Existing sub order var not in state! " << std::endl;
                
                return false;
            }
                
        return true;
    }
    
    int StateManager::calcSubVarSize(const std::vector<std::shared_ptr<Type>>& sub_var)
    {
        int total_size = 0;
        
        for (const auto& item : sub_var)
            if (item != nullptr)
                total_size += item->size();
        
        return total_size;
    }
    
    void StateManager::addVariableDelayedInvertible(std::shared_ptr<State> state, std::shared_ptr<Type> var_new, const std::vector<std::shared_ptr<Type>>& var_old_order, const Eigen::MatrixXd& H_old, const Eigen::MatrixXd& H_new, const Eigen::VectorXd& res, double noise_iso_meas)
    {
        if (std::find(state->_err_variables.begin(), state->_err_variables.end(), var_new) != state->_err_variables.end())
        {
            std::cout << "[StateManager]: New var already in state! Cannot perform add var delayed inv!" << std::endl;
            return;
        }
        
        assert(StateManager::checkSubOrder(state, var_old_order));
        
        int old_sub_var_size = StateManager::calcSubVarSize(var_old_order);
        
        assert(res.rows() == H_old.rows());
        assert(res.rows() == H_new.rows());
        assert(H_new.rows() == H_new.cols());
        assert(H_new.cols() == var_new->size());
        assert(H_old.cols() == old_sub_var_size);
        
        assert(H_new.determinant() != 0);
        
        int small_idx = 0;
        std::vector<int> H_idx;
        
        for (int i = 0; i < var_old_order.size(); ++i)
        {
            H_idx.push_back(small_idx);
            small_idx += var_old_order[i]->size();
        }
        
        Eigen::MatrixXd PH_T = Eigen::MatrixXd::Zero(state->_cov.rows(), res.rows());
        
        for (int i = 0; i < state->_err_variables.size(); ++i)
        {
            Eigen::MatrixXd PH_T_i = Eigen::MatrixXd::Zero(state->_err_variables[i]->size(), res.rows());
            
            for (int j = 0; j < var_old_order.size(); ++j)
            {
                std::shared_ptr<Type> meas_var = var_old_order[j];
                
                PH_T_i += state->_cov.block(state->_err_variables[i]->idx(), meas_var->idx(), state->_err_variables[i]->size(), meas_var->size())*H_old.block(0, H_idx[j], H_old.rows(), meas_var->size()).transpose();
            }
            
            PH_T.block(state->_err_variables[i]->idx(), 0, state->_err_variables[i]->size(), res.rows()) = PH_T_i;
        }
        
        Eigen::MatrixXd small_cov = StateManager::getMarginalCov(state, var_old_order);
        
        Eigen::MatrixXd S(res.rows(), res.rows());
        
        S = H_old*small_cov*H_old.transpose();
        
        for (int i = 0; i < S.cols(); ++i)
            S(i, i) += std::pow(noise_iso_meas, 2.0);
        
        Eigen::MatrixXd H_new_inv = H_new.inverse();
        
        Eigen::MatrixXd cov_newnew = H_new_inv*S*H_new_inv.transpose();
        
        Eigen::MatrixXd cov_tmp = Eigen::MatrixXd::Zero(state->_cov.rows() + var_new->size(), state->_cov.cols() + var_new->size());
        
        cov_tmp.block(0, 0, state->_cov.rows(), state->_cov.cols()) = state->_cov;
        
        cov_tmp.block(state->_cov.rows(), state->_cov.cols(), var_new->size(), var_new->size()) = cov_newnew;
        
        cov_tmp.block(0, state->_cov.cols(), state->_cov.rows(), var_new->size()) = -PH_T*H_new_inv.transpose();
        
        cov_tmp.block(state->_cov.rows(), 0, var_new->size(), state->_cov.cols()) = cov_tmp.block(0, state->_cov.cols(), state->_cov.rows(), var_new->size()).transpose();
        
        var_new->set_cov_idx(state->_cov.cols());
        
        state->_err_variables.push_back(var_new);
        
        state->_cov = 0.5*(cov_tmp + cov_tmp.transpose());
    }
    
    void StateManager::addVariableDelayed(std::shared_ptr<State> state, std::shared_ptr<Type> var_new, const std::vector<std::shared_ptr<Type>>& var_old_order, Eigen::MatrixXd& H_old, Eigen::MatrixXd& H_new, Eigen::VectorXd& res, double noise_iso_meas, double chi2_mult_factor, bool do_chi2)
    {
        if (std::find(state->_err_variables.begin(), state->_err_variables.end(), var_new) != state->_err_variables.end())
        {
            std::cout << "[StateManager]: New var already in state! Cannot perform add var delayed inv!" << std::endl;
            return;
        }
        
        int old_sub_var_size = StateManager::calcSubVarSize(var_old_order);
        int new_var_size = var_new->size();
        
        assert(StateManager::checkSubOrder(state, var_old_order));
        
        assert(res.rows() == H_old.rows());
        assert(res.rows() == H_new.rows());

        assert(H_new.cols() == new_var_size);
        assert(H_old.cols() == old_sub_var_size);
        
        if (H_new.rows() <= H_new.cols())
        {
            std::cout << "[StateManager]: H_new rows should be larger than H_new cols!" << std::endl;
            return;
        }
        
        Eigen::JacobiRotation<double> tmpG;
        
        for (int n = 0; n < H_new.cols(); ++n)
            for (int m = H_new.rows()-1; m > n; --m)
            {
                tmpG.makeGivens(H_new(m-1, n), H_new(m, n));
                
                (H_new.block(m-1, n, 2, H_new.cols()-n)).applyOnTheLeft(0, 1,tmpG.adjoint());
                
                (res.block(m-1, 0, 2, 1)).applyOnTheLeft(0, 1, tmpG.adjoint());
                
                (H_old.block(m-1, 0, 2, H_old.cols())).applyOnTheLeft(0, 1, tmpG.adjoint());
            }
        
        Eigen::MatrixXd Hxinit = H_old.block(0, 0, new_var_size, H_old.cols());
        
        Eigen::MatrixXd Hfinit = H_new.block(0, 0, new_var_size, new_var_size);
        
        Eigen::VectorXd resinit = res.block(0, 0, new_var_size, 1);
        
        Eigen::MatrixXd Hup = H_old.block(new_var_size, 0, H_old.rows()-new_var_size, H_old.cols());
        
        Eigen::VectorXd resup = res.block(new_var_size, 0, res.rows()-new_var_size, 1);
        
        Eigen::MatrixXd small_cov = StateManager::getMarginalCov(state, var_old_order);
        
        Eigen::MatrixXd S = Hup*small_cov*Hup.transpose();
        
        for (int i = 0; i < S.rows(); ++i)
            S(i, i) += std::pow(noise_iso_meas, 2.0);
        
        double chi2 = resup.dot(S.llt().solve(resup));
        
        boost::math::chi_squared chi_squared_dist(res.rows());
        
        double chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
        
        if (chi2 > chi2_mult_factor * chi2_check && do_chi2)
        {
            std::cout << "[StateManager]: Cannot add variable due to chi2 test failure!" << std::endl;
            return;
        }
        
        StateManager::addVariableDelayedInvertible(state, var_new, var_old_order, Hxinit, Hfinit, resinit, noise_iso_meas);
        
        
        if (Hup.rows() > 0)
            StateManager::ekfUpdate(state, var_old_order, Hup, resup, std::pow(noise_iso_meas, 2.0)*Eigen::MatrixXd::Identity(resup.rows(), resup.rows()));
    }
    
    void StateManager::replaceVarLinear(std::shared_ptr<State> state, const std::shared_ptr<Type> target_var, const std::vector<std::shared_ptr<Type>>& dependence_order, const Eigen::MatrixXd& H)
    {
        bool notFound = true;
        for (const auto& item : state->_err_variables)
            if (target_var == item)
            {
                notFound = false;
                break;
            }
            
        if (notFound)
        {
            std::cout << "[StateManager]: Target var not in state, cannot linearly replace!" << std::endl;
            return;
        }
        
        assert(StateManager::checkSubOrder(state, dependence_order));
        assert(target_var->size() == H.rows());
        
        int small_idx = 0;
        std::vector<int> H_idx;
        
        for (int i = 0; i < dependence_order.size(); ++i)
        {
            H_idx.push_back(small_idx);
            
            small_idx += dependence_order[i]->size();
        }
        
        assert(small_idx == H.cols());
        
        Eigen::MatrixXd PH_T(state->_cov.rows(), H.rows());
        
        PH_T.setZero();
        
        for (const auto& total_var : state->_err_variables)
        {
            Eigen::MatrixXd PH_T_i = Eigen::MatrixXd::Zero(total_var->size(), H.rows());
            
            for (int i = 0; i < dependence_order.size(); ++i)
            {
                std::shared_ptr<Type> meas_var = dependence_order[i];
                
                PH_T_i.noalias() += state->_cov.block(total_var->idx(), meas_var->idx(), total_var->size(), meas_var->size())*H.block(0, H_idx[i], H.rows(), meas_var->size()).transpose();
            }
            
            PH_T.block(total_var->idx(), 0, total_var->size(), H.rows()) = PH_T_i;
        }
        
        Eigen::MatrixXd small_cov = StateManager::getMarginalCov(state, dependence_order);
        
        Eigen::MatrixXd HPH_T = H*small_cov*H.transpose();
        
        state->_cov.block(0, target_var->idx(), state->curr_cov_size(), target_var->size()) = PH_T;
        
        state->_cov.block(target_var->idx(), 0, target_var->size(), state->curr_cov_size()) = PH_T.transpose();
        
        state->_cov.block(target_var->idx(), target_var->idx(), target_var->size(), target_var->size()) = HPH_T;
    }
}
