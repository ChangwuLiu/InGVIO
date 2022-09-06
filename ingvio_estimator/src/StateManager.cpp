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
}
