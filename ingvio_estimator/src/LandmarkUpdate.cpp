#include "StateManager.h"

#include "MapServer.h"
#include "MapServerManager.h"

#include "LandmarkUpdate.h"

namespace ingvio
{
    void LandmarkUpdate::updateLandmarkMono(std::shared_ptr<State> state,
                                            std::shared_ptr<MapServer> map_server)
    {
        if (state->_anchored_landmarks.size() == 0) return;
        
        std::vector<std::shared_ptr<Type>> var_order;
        var_order.push_back(state->_extended_pose);
        var_order.push_back(state->_camleft_imu_extrinsics);
        
        std::map<std::shared_ptr<Type>, int> sub_var_col_idx;
        sub_var_col_idx[state->_extended_pose] = 0;
        sub_var_col_idx[state->_camleft_imu_extrinsics] = 9;
        
        const int max_possible_rows = 2*state->_anchored_landmarks.size();
        const int max_possible_cols = 15 + 6*state->_sw_camleft_poses.size() + 3*state->_anchored_landmarks.size();
        
        Eigen::MatrixXd H_large(max_possible_rows, max_possible_cols);
        Eigen::VectorXd res_large(max_possible_rows);
        H_large.setZero();
        res_large.setZero();
        
        int row_cnt = 0;
        int col_cnt = 15;
        
        for (const auto& item : state->_anchored_landmarks)
        {
            const int& id = item.first;
            if (map_server->find(id) == map_server->end())
            {
                std::cout << "[LandmarkUpdate]: Landmark in state not in map server!" << std::endl;
                std::exit(EXIT_FAILURE);
            }
            else if (map_server->at(id)->_ftype != FeatureInfo::FeatureType::SLAM)
            {
                std::cout << "[LandmarkUpdate]: Landmark in state not marked SLAM type in map server!" << std::endl;
                std::exit(EXIT_FAILURE);
            }
            else if (map_server->at(id)->_mono_obs.find(state->_timestamp) == map_server->at(id)->_mono_obs.end())
            {
                std::cout << "[LandmarkUpdate]: Landmark in state not tracked to curr time! Should have been marged before!" << std::endl;
                std::exit(EXIT_FAILURE);
            }
            
            if (map_server->at(id)->_landmark != state->_anchored_landmarks.at(id))
            {
                std::cout << "[LandmarkUpdate]: Landmark ptr in state not the same as that in map server!" << std::endl;
                std::exit(EXIT_FAILURE);
            }
            
            std::shared_ptr<SE3> anchored_pose_ptr = item.second->getAnchoredPose();
            std::shared_ptr<AnchoredLandmark> lm_ptr = item.second;
            
            Eigen::Vector2d res;
            Eigen::Matrix<double, 2, 9> H_fj_epose;
            Eigen::Matrix<double, 2, 6> H_fj_ext;
            Eigen::Matrix<double, 2, 6> H_fj_anch;
            Eigen::Matrix<double, 2, 3> H_fj_pf;
            
            this->calcResJacobianSingleLandmarkMono(map_server->at(id), state, res, H_fj_epose,
                                                    H_fj_ext, H_fj_anch, H_fj_pf);
            
            std::vector<std::shared_ptr<Type>> var_order_chi2(4);
            var_order_chi2[0] = state->_extended_pose;
            var_order_chi2[1] = state->_camleft_imu_extrinsics;
            var_order_chi2[2] = anchored_pose_ptr;
            var_order_chi2[3] = lm_ptr;
            
            Eigen::MatrixXd H_chi2(2, 24);
            H_chi2.block<2, 9>(0, 0) = H_fj_epose;
            H_chi2.block<2, 6>(0, 9) = H_fj_ext;
            H_chi2.block<2, 6>(0, 15) = H_fj_anch;
            H_chi2.block<2, 3>(0, 21) = H_fj_pf;
            
            if (!testChiSquared(state, res, H_chi2, var_order_chi2, this->_noise))
                continue;
            
            res_large.block<2, 1>(row_cnt, 0) = res;
            
            H_large.block<2, 9>(row_cnt, sub_var_col_idx.at(state->_extended_pose)) = H_fj_epose;
            
            H_large.block<2, 6>(row_cnt, sub_var_col_idx.at(state->_camleft_imu_extrinsics)) = H_fj_ext;
            
            if (sub_var_col_idx.find(anchored_pose_ptr) == sub_var_col_idx.end())
            {
                sub_var_col_idx[anchored_pose_ptr] = col_cnt;
                col_cnt += 6;
                var_order.push_back(anchored_pose_ptr);
            }
            
            H_large.block<2, 6>(row_cnt, sub_var_col_idx.at(anchored_pose_ptr)) = H_fj_anch;
            
            if (sub_var_col_idx.find(lm_ptr) == sub_var_col_idx.end())
            {
                sub_var_col_idx[lm_ptr] = col_cnt;
                col_cnt += 3;
                var_order.push_back(lm_ptr);
            }
            
            H_large.block<2, 3>(row_cnt, sub_var_col_idx.at(lm_ptr)) = H_fj_pf;
            
            row_cnt += 2;
        }
        
        if (row_cnt == 0) return;
        
        /*
        std::cout << "[LandmarkUpdate]: row cnt = " << row_cnt << std::endl;
        std::cout << "[LandmarkUpdate]: col cnt = " << col_cnt << std::endl;
        */
        
        if (max_possible_rows > row_cnt || max_possible_cols > col_cnt)
        {
            H_large.conservativeResize(row_cnt, col_cnt);
            res_large.conservativeResize(row_cnt, 1);
        }
        
        StateManager::ekfUpdate(state, var_order, H_large, res_large, std::pow(this->_noise, 2)*Eigen::MatrixXd::Identity(res_large.rows(), res_large.rows()));
        
    }
    
    void LandmarkUpdate::changeLandmarkAnchor(std::shared_ptr<State> state,
                                              std::shared_ptr<MapServer> map_server)
    {
        double marg_time = state->nextMargTime();
        
        if (marg_time == INFINITY || state->_sw_camleft_poses.find(marg_time) == state->_sw_camleft_poses.end()) return;
        
        const std::shared_ptr<SE3> old_anchor = state->_sw_camleft_poses.at(marg_time);
        
        const double latest_sw = state->_sw_camleft_poses.rbegin()->first;
        
        const std::shared_ptr<SE3> new_anchor = state->_sw_camleft_poses.at(latest_sw);
        
        std::vector<int> ids_to_marg;
        
        for (auto& item : *map_server)
        {
            if (item.second->_ftype != FeatureInfo::FeatureType::SLAM)
                continue;
            
            if (item.second->_landmark->getAnchoredPose() == old_anchor)
            {
                const Eigen::Vector3d pf = item.second->_landmark->valuePosXyz();
                const Eigen::Vector3d body = new_anchor->valueLinearAsMat().transpose()*(pf - new_anchor->valueTrans());
                    
                if (body.z() <= 0)
                {
                    ids_to_marg.push_back(item.first);
                    continue;
                }
                    
                FeatureInfoManager::changeAnchoredPose(item.second, state, latest_sw);
                
                if (item.second->_landmark->getAnchoredPose() != new_anchor)
                    ids_to_marg.push_back(item.first);
            }
        }
        
        for (const int& id: ids_to_marg)
        {
            StateManager::margAnchoredLandmarkInState(state, id);
            map_server->erase(id);
        }
    }
    
    void LandmarkUpdate::initNewLandmarkMono(std::shared_ptr<State> state,
                                             std::shared_ptr<MapServer> map_server,
                                             std::shared_ptr<Triangulator> tri)
    {
        double marg_time = state->nextMargTime();
        
        const int vac_num_lm = state->_state_params._max_landmarks - state->_anchored_landmarks.size();
        
        if (marg_time == INFINITY || vac_num_lm <= 0) return;
        
        std::vector<int> ids_to_init;
        
        for (const auto& item : *map_server)
        {
            if (ids_to_init.size() >= vac_num_lm) break;
            
            if (item.second->_mono_obs.find(marg_time) == item.second->_mono_obs.end() || 
                item.second->_ftype == FeatureInfo::FeatureType::SLAM)
                continue;
            
            if (!FeatureInfoManager::triangulateFeatureInfoMono(item.second, tri, state))
                continue;
            
            ids_to_init.push_back(item.first);
        }
        
        if (ids_to_init.size() == 0) return;
        
        std::vector<std::shared_ptr<SE3>> sw_var_order;
        std::map<std::shared_ptr<SE3>, int> sw_index_map;
        std::vector<std::shared_ptr<Type>> sw_var_type;
        
        this->generateSwVarOrder(state, sw_var_order, sw_index_map, sw_var_type);
        
        for (const int& id : ids_to_init)
        {
            Eigen::VectorXd res_block;
            Eigen::MatrixXd Hx_block;
            Eigen::MatrixXd Hf_block;
            
            this->calcResJacobianSingleFeatAllMonoObs(map_server->at(id), state->_sw_camleft_poses, sw_var_order, sw_index_map, res_block, Hx_block, Hf_block);
            
            if (!StateManager::addVariableDelayed(state, map_server->at(id)->_landmark,
                                                  sw_var_type, Hx_block, Hf_block, res_block,
                                                  this->_noise, 0.95, true))
                continue;
            
            if (state->_anchored_landmarks.find(id) != state->_anchored_landmarks.end())
            {
                std::cout << "[LandmarkUpdate]: The id intended to add already in state!" << std::endl;
                std::exit(EXIT_FAILURE);
            }
            
            state->_anchored_landmarks[id] = map_server->at(id)->_landmark;
            
            map_server->at(id)->_ftype = FeatureInfo::FeatureType::SLAM;
        }
        
    }
    
    void LandmarkUpdate::calcResJacobianSingleFeatAllMonoObs(
        const std::shared_ptr<FeatureInfo> feature_info,
        const std::map<double, std::shared_ptr<SE3>>& sw_poses,
        const std::vector<std::shared_ptr<SE3>>& sw_var_order,
        const std::map<std::shared_ptr<SE3>, int>& sw_index_map,
        Eigen::VectorXd& res_block,
        Eigen::MatrixXd& Hx_block,
        Eigen::MatrixXd& Hf_block)
    {
        const int num_of_cols = 6*sw_var_order.size();
        const int num_of_rows = 2*feature_info->numOfMonoFrames();
        
        Eigen::VectorXd res_block_tmp = Eigen::VectorXd::Zero(num_of_rows);
        Eigen::MatrixXd H_block_tmp = Eigen::MatrixXd::Zero(num_of_rows, num_of_cols);
        Eigen::MatrixXd Hf_block_tmp = Eigen::MatrixXd::Zero(num_of_rows, 3);
        
        const Eigen::Vector3d pf_w = feature_info->_landmark->valuePosXyz();
        
        int row_cnt = 0;
        
        for (const auto& item : feature_info->_mono_obs)
        {
            const double& time_of_obs = item.first;
            
            if (sw_poses.find(time_of_obs) == sw_poses.end())
                continue;
            
            const std::shared_ptr<MonoMeas>& mono_obs_ptr = item.second;
            const std::shared_ptr<SE3>& pose_obs_ptr = sw_poses.at(time_of_obs);
            const std::shared_ptr<SE3> pose_anchor_ptr = feature_info->_landmark->getAnchoredPose();
            
            const Eigen::Matrix3d R_cm2w = pose_obs_ptr->valueLinearAsMat();
            const Eigen::Vector3d p_cm = pose_obs_ptr->valueTrans();
            
            const Eigen::Vector3d pf_cm = R_cm2w.transpose()*(pf_w-p_cm);
            
            Eigen::Matrix<double, 2, 3> H_proj = Eigen::Matrix<double, 2, 3>::Zero();
            H_proj(0, 0) = 1.0/pf_cm.z();
            H_proj(0, 2) = -pf_cm.x()/std::pow(pf_cm.z(), 2);
            H_proj(1, 1) = 1.0/pf_cm.z();
            H_proj(1, 2) = -pf_cm.y()/std::pow(pf_cm.z(), 2);
            
            Eigen::MatrixXd H_pf2x = Eigen::MatrixXd::Zero(3, num_of_cols);
            H_pf2x.block<3, 3>(0, sw_index_map.at(pose_obs_ptr)) = R_cm2w.transpose()*skew(pf_w);
            H_pf2x.block<3, 3>(0, sw_index_map.at(pose_anchor_ptr)) = -H_pf2x.block<3, 3>(0, sw_index_map.at(pose_obs_ptr));
            H_pf2x.block<3, 3>(0, sw_index_map.at(pose_obs_ptr)+3) = -R_cm2w.transpose();
            
            Eigen::Matrix3d H_pf2pf = R_cm2w.transpose();
            
            if (H_proj.hasNaN() || H_pf2x.hasNaN()) 
                continue;
            
            H_block_tmp.block(row_cnt, 0, 2, num_of_cols) = H_proj*H_pf2x;
            Hf_block_tmp.block(row_cnt, 0, 2, 3) = H_proj*H_pf2pf;
            
            res_block_tmp.block(row_cnt, 0, 2, 1) = mono_obs_ptr->asVec()-Eigen::Vector2d(pf_cm.x()/pf_cm.z(), pf_cm.y()/pf_cm.z());
            
            row_cnt += 2;
        }
        
        if (row_cnt < res_block.rows())
        {
            res_block_tmp.conservativeResize(row_cnt, 1);
            H_block_tmp.conservativeResize(row_cnt, num_of_cols);
            Hf_block_tmp.conservativeResize(row_cnt, 3);
        }
        
        res_block = res_block_tmp;
        Hx_block = H_block_tmp;
        Hf_block = Hf_block_tmp;
    }
    
    void LandmarkUpdate::generateSwVarOrder(const std::shared_ptr<State> state,
                                            std::vector<std::shared_ptr<SE3>>& sw_var_order,
                                            std::map<std::shared_ptr<SE3>, int>& sw_index,
                                            std::vector<std::shared_ptr<Type>>& sw_var_type)
    {
        sw_var_order.clear();
        sw_index.clear();
        sw_var_type.clear();
        
        int cnt = 0;
        for (const auto& item : state->_sw_camleft_poses)
        {
            sw_var_order.push_back(item.second);
            sw_var_type.push_back(item.second);
            sw_index[item.second] = 6*cnt;
            ++cnt;
        }
    }
    
    void LandmarkUpdate::calcResJacobianSingleLandmarkMono(
        const std::shared_ptr<FeatureInfo> feature_info,
        const std::shared_ptr<State> state,
        Eigen::Vector2d& res,
        Eigen::Matrix<double, 2, 9>& H_fj_epose,
        Eigen::Matrix<double, 2, 6>& H_fj_ext,
        Eigen::Matrix<double, 2, 6>& H_fj_anch,
        Eigen::Matrix<double, 2, 3>& H_fj_pf)
    {
        const Eigen::Vector3d pf_w = feature_info->_landmark->valuePosXyz();
        
        const Eigen::Matrix3d R_i2w_T = state->_extended_pose->valueLinearAsMat().transpose();
        const Eigen::Matrix3d R_cl2i_T = state->_camleft_imu_extrinsics->valueLinearAsMat().transpose();
        
        const Eigen::Vector3d p_i2w = state->_extended_pose->valueTrans1();
        const Eigen::Vector3d p_c2i = state->_camleft_imu_extrinsics->valueTrans();
        
        const Eigen::Vector3d pf_i = R_i2w_T*(pf_w-p_i2w);
        const Eigen::Vector3d pf_cl = R_cl2i_T*(pf_i-p_c2i);
        
        const double curr_time = state->_timestamp;
        
        if (feature_info->_mono_obs.find(curr_time) == feature_info->_mono_obs.end() ||
            feature_info->_ftype != FeatureInfo::FeatureType::SLAM)
        {
            std::cout << "[LandmarkUpdate]: Cannot calc curr slam feature mono res and jacobi!" << std::endl;
            std::exit(EXIT_FAILURE);
        }
        
        res = feature_info->_mono_obs.at(curr_time)->asVec() - Eigen::Vector2d(pf_cl.x()/pf_cl.z(), pf_cl.y()/pf_cl.z());
        
        Eigen::Matrix<double, 2, 3> H_proj = Eigen::Matrix<double, 2, 3>::Zero();
        H_proj(0, 0) = 1.0/pf_cl.z();
        H_proj(1, 1) = 1.0/pf_cl.z();
        H_proj(0, 2) = -pf_cl.x()/std::pow(pf_cl.z(), 2);
        H_proj(1, 2) = -pf_cl.y()/std::pow(pf_cl.z(), 2);
        
        Eigen::Matrix3d R_w2cl = R_cl2i_T*R_i2w_T;
        
        H_fj_epose.setZero();
        H_fj_epose.block<2, 3>(0, 0) = H_proj*R_w2cl*skew(pf_w);
        H_fj_epose.block<2, 3>(0, 3) = -H_proj*R_w2cl;
        
        H_fj_ext.setZero();
        H_fj_ext.block<2, 3>(0, 0) = H_proj*R_cl2i_T*skew(pf_i);
        H_fj_ext.block<2, 3>(0, 3) = -H_proj*R_cl2i_T;
        
        H_fj_anch.setZero();
        H_fj_anch.block<2, 3>(0, 0) = -H_proj*R_w2cl*skew(pf_w);
        
        H_fj_pf = H_proj*R_w2cl;
    }
    
    void LandmarkUpdate::calcResJacobianSingleLandmarkStereo(
        const std::shared_ptr<FeatureInfo> feature_info,
        const std::shared_ptr<State> state,
        Eigen::Vector4d& res,
        Eigen::Matrix<double, 4, 9>& H_fj_epose,
        Eigen::Matrix<double, 4, 6>& H_fj_ext,
        Eigen::Matrix<double, 4, 6>& H_fj_anch,
        Eigen::Matrix<double, 4, 3>& H_fj_pf)
    {
        const Eigen::Vector3d pf_w = feature_info->_landmark->valuePosXyz();
        
        const Eigen::Matrix3d R_i2w_T = state->_extended_pose->valueLinearAsMat().transpose();
        const Eigen::Matrix3d R_cl2i_T = state->_camleft_imu_extrinsics->valueLinearAsMat().transpose();
        
        const Eigen::Vector3d p_i2w = state->_extended_pose->valueTrans1();
        const Eigen::Vector3d p_c2i = state->_camleft_imu_extrinsics->valueTrans();
        
        const Eigen::Vector3d pf_i = R_i2w_T*(pf_w-p_i2w);
        const Eigen::Vector3d pf_cl = R_cl2i_T*(pf_i-p_c2i);
        
        const Eigen::Isometry3d& T_cl2cr = state->_state_params._T_cl2cr;
        const Eigen::Vector3d pf_cr = T_cl2cr*pf_cl;
        
        const double curr_time = state->_timestamp;
        
        if (feature_info->_stereo_obs.find(curr_time) == feature_info->_stereo_obs.end() ||
            feature_info->_ftype != FeatureInfo::FeatureType::SLAM)
        {
            std::cout << "[LandmarkUpdate]: Cannot calc curr slam feature stereo res and jacobi!" << std::endl;
            std::exit(EXIT_FAILURE);
        }
        
        res = feature_info->_stereo_obs.at(curr_time)->asVec() - Eigen::Vector4d(pf_cl.x()/pf_cl.z(), pf_cl.y()/pf_cl.z(), pf_cr.x()/pf_cr.z(), pf_cr.y()/pf_cr.z());
        
        Eigen::Matrix<double, 2, 3> H_proj = Eigen::Matrix<double, 2, 3>::Zero();
        H_proj(0, 0) = 1.0/pf_cl.z();
        H_proj(1, 1) = 1.0/pf_cl.z();
        H_proj(0, 2) = -pf_cl.x()/std::pow(pf_cl.z(), 2);
        H_proj(1, 2) = -pf_cl.y()/std::pow(pf_cl.z(), 2);
        
        Eigen::Matrix<double, 2, 3> H_proj_r = Eigen::Matrix<double, 2, 3>::Zero();
        H_proj_r(0, 0) = 1.0/pf_cr.z();
        H_proj_r(1, 1) = 1.0/pf_cr.z();
        H_proj_r(0, 2) = -pf_cr.x()/std::pow(pf_cr.z(), 2);
        H_proj_r(1, 2) = -pf_cr.y()/std::pow(pf_cr.z(), 2);
        
        Eigen::Matrix3d R_w2cl = R_cl2i_T*R_i2w_T;
        
        H_fj_epose.setZero();
        H_fj_epose.block<2, 3>(0, 0) = H_proj*R_w2cl*skew(pf_w);
        H_fj_epose.block<2, 3>(0, 3) = -H_proj*R_w2cl;
        H_fj_epose.block<2, 3>(2, 0) = H_proj_r*T_cl2cr.linear()*R_w2cl*skew(pf_w);
        H_fj_epose.block<2, 3>(2, 3) = -H_proj_r*T_cl2cr.linear()*R_w2cl;
        
        H_fj_ext.setZero();
        H_fj_ext.block<2, 3>(0, 0) = H_proj*R_cl2i_T*skew(pf_i);
        H_fj_ext.block<2, 3>(0, 3) = -H_proj*R_cl2i_T;
        H_fj_ext.block<2, 3>(2, 0) = H_proj_r*T_cl2cr.linear()*R_cl2i_T*skew(pf_i);
        H_fj_ext.block<2, 3>(2, 3) = -H_proj_r*T_cl2cr.linear()*R_cl2i_T;
        
        H_fj_anch.setZero();
        H_fj_anch.block<2, 3>(0, 0) = -H_proj*R_w2cl*skew(pf_w);
        H_fj_anch.block<2, 3>(2, 0) = -H_proj_r*T_cl2cr.linear()*skew(pf_w);
        
        H_fj_pf.setZero();
        H_fj_pf.block<2, 3>(0, 0) = H_proj*R_w2cl;
        H_fj_pf.block<2, 3>(2, 0) = H_proj_r*T_cl2cr.linear()*R_w2cl;
    }
    
    void LandmarkUpdate::updateLandmarkStereo(std::shared_ptr<State> state,
                                              std::shared_ptr<MapServer> map_server)
    {
        if (state->_anchored_landmarks.size() == 0) return;
        
        std::vector<std::shared_ptr<Type>> var_order;
        var_order.push_back(state->_extended_pose);
        var_order.push_back(state->_camleft_imu_extrinsics);
        
        std::map<std::shared_ptr<Type>, int> sub_var_col_idx;
        sub_var_col_idx[state->_extended_pose] = 0;
        sub_var_col_idx[state->_camleft_imu_extrinsics] = 9;
        
        const int max_possible_rows = 4*state->_anchored_landmarks.size();
        const int max_possible_cols = 15 + 6*state->_sw_camleft_poses.size() + 3*state->_anchored_landmarks.size();
        
        Eigen::MatrixXd H_large(max_possible_rows, max_possible_cols);
        Eigen::VectorXd res_large(max_possible_rows);
        H_large.setZero();
        res_large.setZero();
        
        int row_cnt = 0;
        int col_cnt = 15;
        
        for (const auto& item : state->_anchored_landmarks)
        {
            const int& id = item.first;
            if (map_server->find(id) == map_server->end())
            {
                std::cout << "[LandmarkUpdate]: Landmark in state not in map server!" << std::endl;
                std::exit(EXIT_FAILURE);
            }
            else if (map_server->at(id)->_ftype != FeatureInfo::FeatureType::SLAM)
            {
                std::cout << "[LandmarkUpdate]: Landmark in state not marked SLAM type in map server!" << std::endl;
                std::exit(EXIT_FAILURE);
            }
            else if (map_server->at(id)->_stereo_obs.find(state->_timestamp) == map_server->at(id)->_stereo_obs.end())
            {
                std::cout << "[LandmarkUpdate]: Landmark in state not tracked to curr time! Should have been marged before!" << std::endl;
                std::exit(EXIT_FAILURE);
            }
            
            if (map_server->at(id)->_landmark != state->_anchored_landmarks.at(id))
            {
                std::cout << "[LandmarkUpdate]: Landmark ptr in state not the same as that in map server!" << std::endl;
                std::exit(EXIT_FAILURE);
            }
            
            std::shared_ptr<SE3> anchored_pose_ptr = item.second->getAnchoredPose();
            std::shared_ptr<AnchoredLandmark> lm_ptr = item.second;
            
            Eigen::Vector4d res;
            Eigen::Matrix<double, 4, 9> H_fj_epose;
            Eigen::Matrix<double, 4, 6> H_fj_ext;
            Eigen::Matrix<double, 4, 6> H_fj_anch;
            Eigen::Matrix<double, 4, 3> H_fj_pf;
            
            this->calcResJacobianSingleLandmarkStereo(map_server->at(id), state,
                                                      res, H_fj_epose,
                                                      H_fj_ext, H_fj_anch, H_fj_pf);
            
            std::vector<std::shared_ptr<Type>> var_order_chi2(4);
            var_order_chi2[0] = state->_extended_pose;
            var_order_chi2[1] = state->_camleft_imu_extrinsics;
            var_order_chi2[2] = anchored_pose_ptr;
            var_order_chi2[3] = lm_ptr;
            
            Eigen::MatrixXd H_chi2(4, 24);
            H_chi2.block<4, 9>(0, 0) = H_fj_epose;
            H_chi2.block<4, 6>(0, 9) = H_fj_ext;
            H_chi2.block<4, 6>(0, 15) = H_fj_anch;
            H_chi2.block<4, 3>(0, 21) = H_fj_pf;
            
            if (!testChiSquared(state, res, H_chi2, var_order_chi2, this->_noise))
                continue;
            
            res_large.block<4, 1>(row_cnt, 0) = res;
            
            H_large.block<4, 9>(row_cnt, sub_var_col_idx.at(state->_extended_pose)) = H_fj_epose;
            
            H_large.block<4, 6>(row_cnt, sub_var_col_idx.at(state->_camleft_imu_extrinsics)) = H_fj_ext;
            
            if (sub_var_col_idx.find(anchored_pose_ptr) == sub_var_col_idx.end())
            {
                sub_var_col_idx[anchored_pose_ptr] = col_cnt;
                col_cnt += 6;
                var_order.push_back(anchored_pose_ptr);
            }
            
            H_large.block<4, 6>(row_cnt, sub_var_col_idx.at(anchored_pose_ptr)) = H_fj_anch;
            
            if (sub_var_col_idx.find(lm_ptr) == sub_var_col_idx.end())
            {
                sub_var_col_idx[lm_ptr] = col_cnt;
                col_cnt += 3;
                var_order.push_back(lm_ptr);
            }
            
            H_large.block<4, 3>(row_cnt, sub_var_col_idx.at(lm_ptr)) = H_fj_pf;
            
            row_cnt += 4;
        }
        
        if (row_cnt == 0) return;
        
        if (max_possible_rows > row_cnt || max_possible_cols > col_cnt)
        {
            H_large.conservativeResize(row_cnt, col_cnt);
            res_large.conservativeResize(row_cnt, 1);
        }
        
        StateManager::ekfUpdate(state, var_order, H_large, res_large, std::pow(this->_noise, 2)*Eigen::MatrixXd::Identity(res_large.rows(), res_large.rows()));
    }
    
    void LandmarkUpdate::calcResJacobianSingleFeatAllStereoObs(
        const std::shared_ptr<FeatureInfo> feature_info,
        const std::map<double, std::shared_ptr<SE3>>& sw_poses,
        const std::vector<std::shared_ptr<SE3>>& sw_var_order,
        const std::map<std::shared_ptr<SE3>, int>& sw_index_map,
        const Eigen::Isometry3d& T_cl2cr,
        Eigen::VectorXd& res_block,
        Eigen::MatrixXd& Hx_block,
        Eigen::MatrixXd& Hf_block)
    {
        const int num_of_cols = 6*sw_var_order.size();
        const int num_of_rows = 4*feature_info->numOfStereoFrames();
        
        Eigen::VectorXd res_block_tmp = Eigen::VectorXd::Zero(num_of_rows);
        Eigen::MatrixXd H_block_tmp = Eigen::MatrixXd::Zero(num_of_rows, num_of_cols);
        Eigen::MatrixXd Hf_block_tmp = Eigen::MatrixXd::Zero(num_of_rows, 3);
        
        const Eigen::Vector3d pf_w = feature_info->_landmark->valuePosXyz();
        
        int row_cnt = 0;
        
        for (const auto& item : feature_info->_stereo_obs)
        {
            const double& time_of_obs = item.first;
            
            if (sw_poses.find(time_of_obs) == sw_poses.end())
                continue;
            
            const std::shared_ptr<StereoMeas>& stereo_obs_ptr = item.second;
            const std::shared_ptr<SE3>& pose_obs_ptr = sw_poses.at(time_of_obs);
            const std::shared_ptr<SE3> pose_anchor_ptr = feature_info->_landmark->getAnchoredPose();
            
            const Eigen::Matrix3d R_cm2w = pose_obs_ptr->valueLinearAsMat();
            const Eigen::Vector3d p_cm = pose_obs_ptr->valueTrans();
            
            const Eigen::Vector3d pf_cm = R_cm2w.transpose()*(pf_w-p_cm);
            const Eigen::Vector3d pf_cm_r = T_cl2cr*pf_cm;
            
            Eigen::Matrix<double, 2, 3> H_proj = Eigen::Matrix<double, 2, 3>::Zero();
            H_proj(0, 0) = 1.0/pf_cm.z();
            H_proj(0, 2) = -pf_cm.x()/std::pow(pf_cm.z(), 2);
            H_proj(1, 1) = 1.0/pf_cm.z();
            H_proj(1, 2) = -pf_cm.y()/std::pow(pf_cm.z(), 2);
            
            Eigen::Matrix<double, 2, 3> H_proj_r = Eigen::Matrix<double, 2, 3>::Zero();
            H_proj_r(0, 0) = 1.0/pf_cm_r.z();
            H_proj_r(1, 1) = 1.0/pf_cm_r.z();
            H_proj_r(0, 2) = -pf_cm_r.x()/std::pow(pf_cm_r.z(), 2);
            H_proj_r(1, 2) = -pf_cm_r.y()/std::pow(pf_cm_r.z(), 2);
            
            Eigen::MatrixXd H_pf2x = Eigen::MatrixXd::Zero(3, num_of_cols);
            H_pf2x.block<3, 3>(0, sw_index_map.at(pose_obs_ptr)) = R_cm2w.transpose()*skew(pf_w);
            H_pf2x.block<3, 3>(0, sw_index_map.at(pose_anchor_ptr)) = -H_pf2x.block<3, 3>(0, sw_index_map.at(pose_obs_ptr));
            H_pf2x.block<3, 3>(0, sw_index_map.at(pose_obs_ptr)+3) = -R_cm2w.transpose();
            
            Eigen::Matrix3d H_pf2pf = R_cm2w.transpose();
            
            if (H_proj.hasNaN() || H_pf2x.hasNaN()) 
                continue;
            
            H_block_tmp.block(row_cnt, 0, 2, num_of_cols) = H_proj*H_pf2x;
            H_block_tmp.block(row_cnt+2, 0, 2, num_of_cols) = H_proj_r*T_cl2cr.linear()*H_pf2x;
            
            Hf_block_tmp.block(row_cnt, 0, 2, 3) = H_proj*H_pf2pf;
            Hf_block_tmp.block(row_cnt+2, 0, 2, 3) = H_proj_r*T_cl2cr.linear()*H_pf2pf;
            
            res_block_tmp.block(row_cnt, 0, 4, 1) = stereo_obs_ptr->asVec()-Eigen::Vector4d(pf_cm.x()/pf_cm.z(), pf_cm.y()/pf_cm.z(), pf_cm_r.x()/pf_cm_r.z(), pf_cm_r.y()/pf_cm_r.z());
            
            row_cnt += 4;
        }
        
        if (row_cnt < res_block.rows())
        {
            res_block_tmp.conservativeResize(row_cnt, 1);
            H_block_tmp.conservativeResize(row_cnt, num_of_cols);
            Hf_block_tmp.conservativeResize(row_cnt, 3);
        }
        
        res_block = res_block_tmp;
        Hx_block = H_block_tmp;
        Hf_block = Hf_block_tmp;
    }
    
    void LandmarkUpdate::initNewLandmarkStereo(std::shared_ptr<State> state,
                                               std::shared_ptr<MapServer> map_server,
                                               std::shared_ptr<Triangulator> tri)
    {
        double marg_time = state->nextMargTime();
        
        const int vac_num_lm = state->_state_params._max_landmarks - state->_anchored_landmarks.size();
        
        if (marg_time == INFINITY || vac_num_lm <= 0) return;
        
        std::vector<int> ids_to_init;
        
        for (const auto& item : *map_server)
        {
            if (ids_to_init.size() >= vac_num_lm) break;
            
            if (item.second->_stereo_obs.find(marg_time) == item.second->_stereo_obs.end() || 
                item.second->_ftype == FeatureInfo::FeatureType::SLAM)
                continue;
            
            if (!FeatureInfoManager::triangulateFeatureInfoStereo(item.second, tri, state))
                continue;
            
            ids_to_init.push_back(item.first);
        }
        
        if (ids_to_init.size() == 0) return;
        
        std::vector<std::shared_ptr<SE3>> sw_var_order;
        std::map<std::shared_ptr<SE3>, int> sw_index_map;
        std::vector<std::shared_ptr<Type>> sw_var_type;
        
        this->generateSwVarOrder(state, sw_var_order, sw_index_map, sw_var_type);
        
        for (const int& id : ids_to_init)
        {
            Eigen::VectorXd res_block;
            Eigen::MatrixXd Hx_block;
            Eigen::MatrixXd Hf_block;
            
            this->calcResJacobianSingleFeatAllStereoObs(map_server->at(id),
                                                        state->_sw_camleft_poses,
                                                        sw_var_order,
                                                        sw_index_map,
                                                        state->_state_params._T_cl2cr,
                                                        res_block, Hx_block, Hf_block);
            
            if (!StateManager::addVariableDelayed(state, map_server->at(id)->_landmark,
                sw_var_type, Hx_block, Hf_block, res_block,
                this->_noise, 0.95, true))
                continue;
            
            if (state->_anchored_landmarks.find(id) != state->_anchored_landmarks.end())
            {
                std::cout << "[LandmarkUpdate]: The id intended to add already in state!" << std::endl;
                std::exit(EXIT_FAILURE);
            }
            
            state->_anchored_landmarks[id] = map_server->at(id)->_landmark;
            
            map_server->at(id)->_ftype = FeatureInfo::FeatureType::SLAM;
        }
    }
    
}
