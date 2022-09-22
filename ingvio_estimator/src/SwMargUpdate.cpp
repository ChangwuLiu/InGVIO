#include <Eigen/SVD>

#include <Eigen/SparseCore>
#include <Eigen/QR>
#include <Eigen/SPQRSupport>

#include "IngvioParams.h"

#include "StateManager.h"

#include "MapServer.h"
#include "MapServerManager.h"

#include "Triangulator.h"

#include "SwMargUpdate.h"

#include "TicToc.h"

namespace ingvio
{
    void SwMargUpdate::updateStateMono(std::shared_ptr<State> state,
                                       std::shared_ptr<MapServer> map_server,
                                       std::shared_ptr<Triangulator> tri)
    {
        double marg_time = state->nextMargTime();
        
        if (marg_time == INFINITY) return;
        
        std::vector<int> update_ids;
        
        
        for (const auto& item : *map_server)
        {
            if (item.second->_mono_obs.find(marg_time) == item.second->_mono_obs.end())
                continue;
            
            if (item.second->_ftype == FeatureInfo::FeatureType::MSCKF &&
                FeatureInfoManager::triangulateFeatureInfoMono(item.second, tri, state))
                update_ids.push_back(item.first);
            
        }
        
        if (update_ids.size() == 0) return;
        
        std::vector<std::shared_ptr<SE3>> sw_var_order;
        std::map<std::shared_ptr<SE3>, int> sw_index_map;
        std::vector<std::shared_ptr<Type>> sw_var_type;
        
        this->generateSwVarOrder(state, sw_var_order, sw_index_map, sw_var_type);
        
        std::vector<double> selected_timestamps;
        this->selectSwTimestamps(state->_sw_camleft_poses, marg_time, selected_timestamps);
        
        const int num_of_cols = 6*sw_var_order.size();
        
        int row_cnt = 0;
        for (int i = 0; i < update_ids.size(); ++i)
            row_cnt += 2*selected_timestamps.size()-3;
        
        Eigen::VectorXd res_large(row_cnt);
        Eigen::MatrixXd H_large(row_cnt, num_of_cols);
        res_large.setZero();
        H_large.setZero();
        
        row_cnt = 0;
        
        for (int i = 0; i < update_ids.size(); ++i)
        {
            Eigen::VectorXd res_block;
            Eigen::MatrixXd H_block;
            
            this->calcResJacobianSingleFeatSelectedMonoObs(map_server->at(update_ids[i]),
                                                           state->_sw_camleft_poses,
                                                           sw_var_order, sw_index_map,
                                                           selected_timestamps,
                                                           res_block, H_block);
            
            if (!testChiSquared(state, res_block, H_block, sw_var_type, this->_noise))
                continue;
            
            H_large.block(row_cnt, 0, H_block.rows(), num_of_cols) = H_block;
            res_large.block(row_cnt, 0, res_block.rows(), 1) = res_block;
            
            row_cnt += res_block.rows();
        }
        
        if (row_cnt < H_large.rows())
        {
            H_large.conservativeResize(row_cnt, num_of_cols);
            res_large.conservativeResize(row_cnt, 1);
        }
        
        Eigen::MatrixXd H_thin;
        Eigen::VectorXd res_thin;
        
        if (H_large.rows() > H_large.cols())
        {
            Eigen::SparseMatrix<double> H_sparse = H_large.sparseView();
            
            Eigen::SPQR<Eigen::SparseMatrix<double>> spqr_helper;
            spqr_helper.setSPQROrdering(SPQR_ORDERING_NATURAL);
            spqr_helper.compute(H_sparse);
            
            Eigen::MatrixXd H_temp;
            Eigen::VectorXd r_temp;
            
            (spqr_helper.matrixQ().transpose()*H_large).evalTo(H_temp);
            (spqr_helper.matrixQ().transpose()*res_large).evalTo(r_temp);
            
            H_thin = H_temp.topRows(num_of_cols);
            res_thin = r_temp.head(num_of_cols);                 
        }
        else
        {
            H_thin = H_large;
            res_thin = res_large;
        }
        
        StateManager::ekfUpdate(state, sw_var_type, H_thin, res_thin, std::pow(this->_noise, 2)*Eigen::MatrixXd::Identity(res_thin.rows(), res_thin.rows()));
    }
    
    void SwMargUpdate::cleanMonoObsAtMargTime(std::shared_ptr<State> state,
                                              std::shared_ptr<MapServer> map_server)
    {
        double marg_time = state->nextMargTime();
        
        if (marg_time == INFINITY) return;
        
        std::vector<int> all_ids;
        for (const auto& item : *map_server)
            all_ids.push_back(item.first);
        
        std::vector<int> ids_to_clean;
        for (int i = 0; i < all_ids.size(); ++i)
        {
            if (map_server->at(all_ids[i])->_mono_obs.find(marg_time) != map_server->at(all_ids[i])->_mono_obs.end())
                map_server->at(all_ids[i])->_mono_obs.erase(marg_time);
            
            if (map_server->at(all_ids[i])->_mono_obs.size() == 0)
                ids_to_clean.push_back(all_ids[i]);
        }
        
        for (int id : ids_to_clean)
            map_server->erase(id);
    }
    
    void SwMargUpdate::updateStateStereo(std::shared_ptr<State> state,
                                         std::shared_ptr<MapServer> map_server,
                                         std::shared_ptr<Triangulator> tri)
    {
        double marg_time = state->nextMargTime();
        
        if (marg_time == INFINITY) return;
        
        std::vector<int> update_ids;
        
        for (const auto& item : *map_server)
        {
            if (item.second->_stereo_obs.find(marg_time) == item.second->_stereo_obs.end())
                continue;
            
            if (item.second->_ftype == FeatureInfo::FeatureType::MSCKF &&
                FeatureInfoManager::triangulateFeatureInfoStereo(item.second, tri, state) &&
                item.second->numOfStereoFrames() >= 3)
                update_ids.push_back(item.first);

        }
        
        if (update_ids.size() == 0) return;
        
        std::vector<std::shared_ptr<SE3>> sw_var_order;
        std::map<std::shared_ptr<SE3>, int> sw_index_map;
        std::vector<std::shared_ptr<Type>> sw_var_type;
        
        this->generateSwVarOrder(state, sw_var_order, sw_index_map, sw_var_type);
        
        std::vector<double> selected_timestamps;
        this->selectSwTimestamps(state->_sw_camleft_poses, marg_time, selected_timestamps);
        
        const int num_of_cols = 6*sw_var_order.size();
        
        int row_cnt = 0;
        for (int i = 0; i < update_ids.size(); ++i)
            row_cnt += 4*selected_timestamps.size()-3;
        
        Eigen::VectorXd res_large(row_cnt);
        Eigen::MatrixXd H_large(row_cnt, num_of_cols);
        res_large.setZero();
        H_large.setZero();
        
        row_cnt = 0;
        
        for (int i = 0; i < update_ids.size(); ++i)
        {
            Eigen::VectorXd res_block;
            Eigen::MatrixXd H_block;
            
            this->calcResJacobianSingleFeatSelectedStereoObs(map_server->at(update_ids[i]),
                                                            state->_sw_camleft_poses,
                                                            sw_var_order, sw_index_map,
                                                            selected_timestamps,
                                                            state->_state_params._T_cl2cr,
                                                            res_block, H_block);
            
            if (!testChiSquared(state, res_block, H_block, sw_var_type, this->_noise))
                continue;
            
            H_large.block(row_cnt, 0, H_block.rows(), num_of_cols) = H_block;
            res_large.block(row_cnt, 0, res_block.rows(), 1) = res_block;
            
            row_cnt += res_block.rows();
        }
        
        if (row_cnt < H_large.rows())
        {
            H_large.conservativeResize(row_cnt, num_of_cols);
            res_large.conservativeResize(row_cnt, 1);
        }
        
        Eigen::MatrixXd H_thin;
        Eigen::VectorXd res_thin;
        
        if (H_large.rows() > H_large.cols())
        {
            Eigen::SparseMatrix<double> H_sparse = H_large.sparseView();
            
            Eigen::SPQR<Eigen::SparseMatrix<double>> spqr_helper;
            spqr_helper.setSPQROrdering(SPQR_ORDERING_NATURAL);
            spqr_helper.compute(H_sparse);
            
            Eigen::MatrixXd H_temp;
            Eigen::VectorXd r_temp;
            
            (spqr_helper.matrixQ().transpose()*H_large).evalTo(H_temp);
            (spqr_helper.matrixQ().transpose()*res_large).evalTo(r_temp);
            
            H_thin = H_temp.topRows(num_of_cols);
            res_thin = r_temp.head(num_of_cols);                 
        }
        else
        {
            H_thin = H_large;
            res_thin = res_large;
        }
        
        StateManager::ekfUpdate(state, sw_var_type, H_thin, res_thin, std::pow(this->_noise, 2)*Eigen::MatrixXd::Identity(res_thin.rows(), res_thin.rows()));
    }
    
    void SwMargUpdate::changeMSCKFAnchor(std::shared_ptr<State> state,
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
            if (item.second->_ftype != FeatureInfo::FeatureType::MSCKF)
                continue;
                
            if (item.second->_landmark->getAnchoredPose() == old_anchor)
            {
                
                if (item.second->_isTri)
                {
                    const Eigen::Vector3d pf = item.second->_landmark->valuePosXyz();
                    const Eigen::Vector3d body = new_anchor->valueLinearAsMat().transpose()*(pf - new_anchor->valueTrans());
                    
                    if (body.z() <= 0)
                    {
                        ids_to_marg.push_back(item.first);
                        continue;
                    }

                    item.second->_landmark->resetAnchoredPose(new_anchor, true);
                }
                else
                    ids_to_marg.push_back(item.first);
            }
        }
        
        for (const int& id: ids_to_marg)
            map_server->erase(id);
    }
    
    void SwMargUpdate::margSwPose(std::shared_ptr<State> state)
    {
        double marg_time = state->nextMargTime();
        
        if (marg_time == INFINITY) return;
        
        StateManager::margSlidingWindowPose(state, marg_time);
    }
    
    void SwMargUpdate::cleanStereoObsAtMargTime(std::shared_ptr<State> state,
                                  std::shared_ptr<MapServer> map_server)
    {
        double marg_time = state->nextMargTime();
        
        if (marg_time == INFINITY) return;
        
        std::vector<int> all_ids;
        for (const auto& item : *map_server)
            all_ids.push_back(item.first);
        
        std::vector<int> ids_to_clean;
        for (int i = 0; i < all_ids.size(); ++i)
        {
            if (map_server->at(all_ids[i])->_stereo_obs.find(marg_time) != map_server->at(all_ids[i])->_stereo_obs.end())
                map_server->at(all_ids[i])->_stereo_obs.erase(marg_time);
            
            if (map_server->at(all_ids[i])->_stereo_obs.size() == 0)
                ids_to_clean.push_back(all_ids[i]);
        }
        
        for (int id : ids_to_clean)
            map_server->erase(id);
    }
    
    void SwMargUpdate::generateSwVarOrder(const std::shared_ptr<State> state,
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
    
    void SwMargUpdate::selectSwTimestamps(
        const std::map<double, std::shared_ptr<SE3>>& sw_poses,
        const double& marg_time,
        std::vector<double>& selected_timestamps)
    {
        selected_timestamps.clear();
        
        if (marg_time == INFINITY || sw_poses.find(marg_time) == sw_poses.end()) return;
        
        int cnt;
        for (const auto& item : sw_poses)
        {
            if (item.first < marg_time)
                continue;
            else if (item.first == marg_time)
            {
                cnt = 1;
                selected_timestamps.push_back(item.first);
                continue;
            }
            
            if (cnt % _frame_select_interval == 0) selected_timestamps.push_back(item.first);
            
            ++cnt;
        }
    
    }
    
    void SwMargUpdate::calcResJacobianSingleFeatSelectedMonoObs(
        const std::shared_ptr<FeatureInfo> feature_info,
        const std::map<double, std::shared_ptr<SE3>>& sw_poses,
        const std::vector<std::shared_ptr<SE3>>& sw_var_order,
        const std::map<std::shared_ptr<SE3>, int>& sw_index_map,
        const std::vector<double>& selected_timestamps,
        Eigen::VectorXd& res_block,
        Eigen::MatrixXd& H_block)
    {
        const int num_of_cols = 6*sw_var_order.size();
        const int num_of_rows = 2*selected_timestamps.size();
        
        Eigen::VectorXd res_block_tmp = Eigen::VectorXd::Zero(num_of_rows);
        Eigen::MatrixXd H_block_tmp = Eigen::MatrixXd::Zero(num_of_rows, num_of_cols);
        Eigen::MatrixXd Hf_block_tmp = Eigen::MatrixXd::Zero(num_of_rows, 3);
        
        const Eigen::Vector3d pf_w = feature_info->_landmark->valuePosXyz();
        
        int row_cnt = 0;
        
        for (const auto& time_of_obs : selected_timestamps)
        {
            if (sw_poses.find(time_of_obs) == sw_poses.end() || feature_info->_mono_obs.find(time_of_obs) == feature_info->_mono_obs.end())
                continue;
            
            const std::shared_ptr<MonoMeas>& mono_obs_ptr = feature_info->_mono_obs.at(time_of_obs);
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
        
        Eigen::JacobiSVD<Eigen::MatrixXd> svd_helper(Hf_block_tmp, Eigen::ComputeFullU | Eigen::ComputeThinV);
        Eigen::MatrixXd V = svd_helper.matrixU().rightCols(row_cnt-3);
        
        H_block = V.transpose()*H_block_tmp;
        res_block = V.transpose()*res_block_tmp;
    }
    
    void SwMargUpdate::calcResJacobianSingleFeatSelectedStereoObs(
        const std::shared_ptr<FeatureInfo> feature_info, 
        const std::map<double, std::shared_ptr<SE3>>& sw_poses,
        const std::vector<std::shared_ptr<SE3>>& sw_var_order,
        const std::map<std::shared_ptr<SE3>, int>& sw_index_map,
        const std::vector<double>& selected_timestamps,
        const Eigen::Isometry3d& T_cl2cr,
        Eigen::VectorXd& res_block,
        Eigen::MatrixXd& H_block)
    {
        const Eigen::Matrix3d& R_cl2cr = T_cl2cr.linear();
        
        const int num_of_cols = 6*sw_var_order.size();
        const int num_of_rows = 4*selected_timestamps.size();
        
        Eigen::VectorXd res_block_tmp = Eigen::VectorXd::Zero(num_of_rows);
        Eigen::MatrixXd H_block_tmp = Eigen::MatrixXd::Zero(num_of_rows, num_of_cols);
        Eigen::MatrixXd Hf_block_tmp = Eigen::MatrixXd::Zero(num_of_rows, 3);
        
        const Eigen::Vector3d pf_w = feature_info->_landmark->valuePosXyz();
        
        int row_cnt = 0;
        
        for (const auto& time_of_obs : selected_timestamps)
        {
            if (sw_poses.find(time_of_obs) == sw_poses.end() || feature_info->_stereo_obs.find(time_of_obs) == feature_info->_stereo_obs.end())
                continue;
            
            const std::shared_ptr<StereoMeas>& stereo_obs_ptr = feature_info->_stereo_obs.at(time_of_obs);
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
            
            const Eigen::Vector3d pf_cm_r = T_cl2cr*pf_cm;
            
            Eigen::Matrix<double, 2, 3> H_proj_r = Eigen::Matrix<double, 2, 3>::Zero();
            H_proj_r(0, 0) = 1.0/pf_cm_r.z();
            H_proj_r(0, 2) = -pf_cm_r.x()/std::pow(pf_cm_r.z(), 2);
            H_proj_r(1, 1) = 1.0/pf_cm_r.z();
            H_proj_r(1, 2) = -pf_cm_r.y()/std::pow(pf_cm_r.z(), 2);
            
            Eigen::MatrixXd H_pf2x_r = R_cl2cr*H_pf2x;
            Eigen::MatrixXd H_pf2pf_r = R_cl2cr*H_pf2x;
            
            if (H_proj.hasNaN() || H_pf2x.hasNaN()) 
                continue;
            
            H_block_tmp.block(row_cnt, 0, 2, num_of_cols) = H_proj*H_pf2x;
            Hf_block_tmp.block(row_cnt, 0, 2, 3) = H_proj*H_pf2pf;
            
            H_block_tmp.block(row_cnt+2, 0, 2, num_of_cols) = H_proj_r*H_pf2x_r;
            Hf_block_tmp.block(row_cnt+2, 0, 2, 3) = H_proj_r*H_pf2pf_r;
            
            res_block_tmp.block(row_cnt, 0, 4, 1) = stereo_obs_ptr->asVec()-Eigen::Vector4d(pf_cm.x()/pf_cm.z(), pf_cm.y()/pf_cm.z(), pf_cm_r.x()/pf_cm_r.z(), pf_cm_r.y()/pf_cm_r.z());
            
            row_cnt += 4;
        }
        
        if (row_cnt < res_block.rows())
        {
            res_block_tmp.conservativeResize(row_cnt, 1);
            H_block_tmp.conservativeResize(row_cnt, num_of_cols);
            Hf_block_tmp.conservativeResize(row_cnt, 3);
        }
        
        Eigen::JacobiSVD<Eigen::MatrixXd> svd_helper(Hf_block_tmp, Eigen::ComputeFullU | Eigen::ComputeThinV);
        Eigen::MatrixXd V = svd_helper.matrixU().rightCols(row_cnt-3);
        
        H_block = V.transpose()*H_block_tmp;
        res_block = V.transpose()*res_block_tmp;
    }
    
    void SwMargUpdate::removeMonoMSCKFinMargPose(std::shared_ptr<State> state,
                                             std::shared_ptr<MapServer> map_server)
    {
        const double marg_time = state->nextMargTime();
        
        if (marg_time == INFINITY) return;
        
        std::vector<int> ids_to_marg;
        
        for (const auto& item : *map_server)
            if (item.second->_mono_obs.find(marg_time) != item.second->_mono_obs.end() &&
                item.second->_ftype == FeatureInfo::FeatureType::MSCKF)
                ids_to_marg.push_back(item.first);
        
        for (const int& id : ids_to_marg)
            map_server->erase(id);
    }
    
    void SwMargUpdate::removeStereoMSCKFinMargPose(std::shared_ptr<State> state,
                                                 std::shared_ptr<MapServer> map_server)
    {
        const double marg_time = state->nextMargTime();
        
        if (marg_time == INFINITY) return;
        
        std::vector<int> ids_to_marg;
        
        for (const auto& item : *map_server)
            if (item.second->_stereo_obs.find(marg_time) != item.second->_stereo_obs.end() && 
                item.second->_ftype == FeatureInfo::FeatureType::MSCKF)
                ids_to_marg.push_back(item.first);
            
            for (const int& id : ids_to_marg)
                map_server->erase(id);
    }
}
