/**  This File is part of InGVIO, an invariant filter for mono/stereo visual-
 *    inertial-raw GNSS navigation. 
 *    
 *    Copyright (C) 2022  Changwu Liu (cwliu529@163.com,
 *                                     lcw18@mails.tsinghua.edu.cn (valid until 2023))
 *    
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *    
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *    
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <Eigen/SVD>

#include <Eigen/SparseCore>
#include <Eigen/QR>
#include <Eigen/SPQRSupport>

#include <unordered_set>

#include "State.h"
#include "StateManager.h"

#include "MapServer.h"
#include "MapServerManager.h"

#include "Triangulator.h"

#include "KeyframeUpdate.h"

namespace ingvio
{
    int KeyframeUpdate::_select_cnt = 0;
    
    void KeyframeUpdate::getMargKfs(const std::shared_ptr<State> state,
                                    std::vector<double>& marg_kfs)
    {
        if (state->_sw_camleft_poses.size() < _max_sw_poses || _max_sw_poses < 3)
        {
            marg_kfs.clear();
            return;
        }
        
        if (state->_timestamp == _timestamp && _kfs.size() > 0)
        {
            assert(_kfs.size() == 2);
            marg_kfs = _kfs;
            
            return;
        }
        
        if (state->_sw_camleft_poses.size() > _max_sw_poses)
        {
            std::cout << "[KeyframeUpdate]: Current sw poses larger than max size!" << std::endl;
            std::exit(EXIT_FAILURE);
        }
        
        _timestamp = state->_timestamp;
        _kfs.clear();
        
        const int rem = _max_sw_poses - 2;
        
        const int idx1 = 2 + KeyframeUpdate::_select_cnt;
        ++KeyframeUpdate::_select_cnt;
        
        KeyframeUpdate::_select_cnt = KeyframeUpdate::_select_cnt % rem;
        
        auto item1 = state->_sw_camleft_poses.rbegin();
        for (int i = 0; i < idx1; ++i)
            ++item1;
        
        auto item2 = state->_sw_camleft_poses.rbegin();
        ++item2;
        
        _kfs.push_back(item1->first);
        _kfs.push_back(item2->first);
        
        /*
        auto item2 = state->_sw_camleft_poses.rbegin();
        for (int i = 0; i < 2; ++i)
            ++item2;
        
        double time1;
        
        if (KeyframeUpdate::_select_cnt == 0)
        {
            auto item1 = state->_sw_camleft_poses.begin();
            for (int i = 0; i < 2; ++i)
                ++item1;
            time1 = item1->first;
        }
        else
        {
            auto item1 = state->_sw_camleft_poses.rbegin();
            for (int i = 0; i < 3; ++i)
                ++item1;
            time1 = item1->first;
        }
        
        _kfs.push_back(time1);
        _kfs.push_back(item2->first);
        
        ++KeyframeUpdate::_select_cnt;
        KeyframeUpdate::_select_cnt = KeyframeUpdate::_select_cnt % 2;
        */
        
        marg_kfs = _kfs;
    }
    
    void KeyframeUpdate::margSwPose(std::shared_ptr<State> state)
    {
        std::vector<double> marg_kfs;
        
        this->getMargKfs(state, marg_kfs);
        
        if (marg_kfs.size() == 0)
            return;
        
        for (const double& marg_time : marg_kfs)
            StateManager::margSlidingWindowPose(state, marg_time);
    }
    
    void KeyframeUpdate::generateSwVarOrder(const std::shared_ptr<State> state,
                                            const std::vector<double>& selected_timestamps,
                                            std::vector<std::shared_ptr<SE3>>& sw_var_order,
                                            std::map<std::shared_ptr<SE3>, int>& sw_index,
                                            std::vector<std::shared_ptr<Type>>& sw_var_type)
    {
        sw_var_order.clear();
        sw_index.clear();
        sw_var_type.clear();
        
        int cnt = 0;
        
        const auto& sw_poses = state->_sw_camleft_poses;
        
        for (const double& ts : selected_timestamps)
        {
            if (sw_poses.find(ts) == sw_poses.end())
            {
                std::cout << "[KeyframeUpdate]: selected timestamp not in sw!" << std::endl;
                std::exit(EXIT_FAILURE);
            }
            
            sw_var_order.push_back(sw_poses.at(ts));
            sw_var_type.push_back(sw_poses.at(ts));
            sw_index[sw_poses.at(ts)] = 6*cnt;
            ++cnt;
        }
    }
    
    std::shared_ptr<SE3> KeyframeUpdate::calcResJacobianSingleFeatSelectedMonoObs(
        const std::shared_ptr<FeatureInfo> feature_info,
        const std::map<double, std::shared_ptr<SE3>>& sw_poses,
        const std::vector<std::shared_ptr<SE3>>& sw_var_order,
        const std::map<std::shared_ptr<SE3>, int>& sw_index_map,
        const std::vector<double>& selected_timestamps,
        Eigen::VectorXd& res_block,
        Eigen::MatrixXd& H_block,
        Eigen::MatrixXd& H_anchor_block)
    {
        const int num_of_cols = 6*sw_var_order.size();
        const int num_of_rows = 2*selected_timestamps.size();
        
        Eigen::VectorXd res_block_tmp = Eigen::VectorXd::Zero(num_of_rows);
        Eigen::MatrixXd H_block_tmp = Eigen::MatrixXd::Zero(num_of_rows, num_of_cols);
        Eigen::MatrixXd H_anchor_block_tmp = Eigen::MatrixXd::Zero(num_of_rows, 6);
        Eigen::MatrixXd Hf_block_tmp = Eigen::MatrixXd::Zero(num_of_rows, 3);
        
        const Eigen::Vector3d pf_w = feature_info->_landmark->valuePosXyz();
        const std::shared_ptr<SE3> pose_anchor_ptr = feature_info->_landmark->getAnchoredPose();
        
        int row_cnt = 0;
        
        for (const auto& time_of_obs : selected_timestamps)
        {
            if (sw_poses.find(time_of_obs) == sw_poses.end() || feature_info->_mono_obs.find(time_of_obs) == feature_info->_mono_obs.end())
                continue;
            
            const std::shared_ptr<MonoMeas>& mono_obs_ptr = feature_info->_mono_obs.at(time_of_obs);
            const std::shared_ptr<SE3>& pose_obs_ptr = sw_poses.at(time_of_obs);
            
            
            const Eigen::Matrix3d R_cm2w = pose_obs_ptr->valueLinearAsMat();
            const Eigen::Vector3d p_cm = pose_obs_ptr->valueTrans();
            
            const Eigen::Vector3d pf_cm = R_cm2w.transpose()*(pf_w-p_cm);
            
            Eigen::Matrix<double, 2, 3> H_proj = Eigen::Matrix<double, 2, 3>::Zero();
            H_proj(0, 0) = 1.0/pf_cm.z();
            H_proj(0, 2) = -pf_cm.x()/std::pow(pf_cm.z(), 2);
            H_proj(1, 1) = 1.0/pf_cm.z();
            H_proj(1, 2) = -pf_cm.y()/std::pow(pf_cm.z(), 2);
            
            Eigen::MatrixXd H_pf2x = Eigen::MatrixXd::Zero(3, num_of_cols);
            Eigen::Matrix<double, 3, 6> H_pf2anchor = Eigen::Matrix<double, 3, 6>::Zero();
            
            if (pose_obs_ptr != pose_anchor_ptr)
            {
                H_pf2x.block<3, 3>(0, sw_index_map.at(pose_obs_ptr)) = R_cm2w.transpose()*skew(pf_w);
        
                H_pf2anchor.block<3, 3>(0, 0) = -H_pf2x.block<3, 3>(0, sw_index_map.at(pose_obs_ptr));
            }
            
            H_pf2x.block<3, 3>(0, sw_index_map.at(pose_obs_ptr)+3) = -R_cm2w.transpose();
            Eigen::Matrix3d H_pf2pf = R_cm2w.transpose();
            
            if (H_proj.hasNaN() || H_pf2x.hasNaN()) 
                continue;
            
            H_block_tmp.block(row_cnt, 0, 2, num_of_cols) = H_proj*H_pf2x;
            
            H_anchor_block_tmp.block(row_cnt, 0, 2, 6) = H_proj*H_pf2anchor;
            
            Hf_block_tmp.block(row_cnt, 0, 2, 3) = H_proj*H_pf2pf;
            
            res_block_tmp.block(row_cnt, 0, 2, 1) = mono_obs_ptr->asVec()-Eigen::Vector2d(pf_cm.x()/pf_cm.z(), pf_cm.y()/pf_cm.z());
            
            row_cnt += 2;
        }
        
        if (row_cnt < res_block.rows())
        {
            res_block_tmp.conservativeResize(row_cnt, 1);
            H_block_tmp.conservativeResize(row_cnt, num_of_cols);
            H_anchor_block_tmp.conservativeResize(row_cnt, 6);
            Hf_block_tmp.conservativeResize(row_cnt, 3);
        }
        
        if (row_cnt <= 3)
            std::cout << "[KeyframeUpdate]: Warning! in calc one feat selected jacobian, num of rows <= 3 !" << std::endl;
        
        Eigen::JacobiSVD<Eigen::MatrixXd> svd_helper(Hf_block_tmp, Eigen::ComputeFullU | Eigen::ComputeThinV);
        Eigen::MatrixXd V = svd_helper.matrixU().rightCols(row_cnt-3);
        
        H_block = V.transpose()*H_block_tmp;
        res_block = V.transpose()*res_block_tmp;
        H_anchor_block = V.transpose()*H_anchor_block_tmp;
        
        return pose_anchor_ptr;
    }
    
    void KeyframeUpdate::cleanMonoObsAtMargTime(std::shared_ptr<State> state,
                                                std::shared_ptr<MapServer> map_server)
    {
        std::vector<double> marg_kfs;
        
        this->getMargKfs(state, marg_kfs);
        
        if (marg_kfs.size() == 0)
            return;
        
        std::vector<int> all_ids;
        for (const auto& item : *map_server)
            all_ids.push_back(item.first);
        
        std::vector<int> ids_to_clean;
        for (const int& id : all_ids)
            for (const double& marg_time : marg_kfs)
            {
                if (map_server->at(id)->_mono_obs.find(marg_time) != map_server->at(id)->_mono_obs.end())
                    map_server->at(id)->_mono_obs.erase(marg_time);
            
                if (map_server->at(id)->_mono_obs.size() == 0)
                    ids_to_clean.push_back(id);
            }
        
        for (int id : ids_to_clean)
            map_server->erase(id);
    }
    
    void KeyframeUpdate::changeMSCKFAnchor(std::shared_ptr<State> state,
                                           std::shared_ptr<MapServer> map_server)
    {
        std::vector<double> marg_kfs;
        
        this->getMargKfs(state, marg_kfs);
        
        if (marg_kfs.size() == 0)
            return;
        
        std::unordered_set<std::shared_ptr<SE3>> old_anchor_set;
        
        for (const double& marg_time : marg_kfs)
            old_anchor_set.insert(state->_sw_camleft_poses.at(marg_time));
               
        const std::shared_ptr<SE3> new_anchor = state->_sw_camleft_poses.rbegin()->second;
        
        std::vector<int> ids_to_marg;
        
        for (auto& item : *map_server)
        {
            if (item.second->_ftype != FeatureInfo::FeatureType::MSCKF)
                continue;
            
            if (old_anchor_set.find(item.second->_landmark->getAnchoredPose()) !=
                old_anchor_set.end())
            {
                
                if (item.second->_isTri)
                {
                    const Eigen::Vector3d pf = item.second->_landmark->valuePosXyz();
                    const Eigen::Vector3d body = new_anchor->valueLinearAsMat().transpose()*(pf - new_anchor->valueTrans());
                    
                    if (body.z() <= 0.3)
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
    
    std::shared_ptr<SE3> KeyframeUpdate::calcResJacobianSingleFeatSelectedStereoObs(
        const std::shared_ptr<FeatureInfo> feature_info, 
        const std::map<double, std::shared_ptr<SE3>>& sw_poses,
        const std::vector<std::shared_ptr<SE3>>& sw_var_order,
        const std::map<std::shared_ptr<SE3>, int>& sw_index_map,
        const std::vector<double>& selected_timestamps,
        const Eigen::Isometry3d& T_cl2cr,
        Eigen::VectorXd& res_block,
        Eigen::MatrixXd& H_block,
        Eigen::MatrixXd& H_anchor_block)
    {
        
        const Eigen::Matrix3d& R_cl2cr = T_cl2cr.linear();
        
        const int num_of_cols = 6*sw_var_order.size();
        const int num_of_rows = 4*selected_timestamps.size();
        
        Eigen::VectorXd res_block_tmp = Eigen::VectorXd::Zero(num_of_rows);
        Eigen::MatrixXd H_block_tmp = Eigen::MatrixXd::Zero(num_of_rows, num_of_cols);
        Eigen::MatrixXd H_anchor_block_tmp = Eigen::MatrixXd::Zero(num_of_rows, 6);
        Eigen::MatrixXd Hf_block_tmp = Eigen::MatrixXd::Zero(num_of_rows, 3);
        
        const Eigen::Vector3d pf_w = feature_info->_landmark->valuePosXyz();
        const std::shared_ptr<SE3> pose_anchor_ptr = feature_info->_landmark->getAnchoredPose();
        
        int row_cnt = 0;
        
        for (const auto& time_of_obs : selected_timestamps)
        {
            if (sw_poses.find(time_of_obs) == sw_poses.end() || feature_info->_stereo_obs.find(time_of_obs) == feature_info->_stereo_obs.end())
                continue;
            
            const std::shared_ptr<StereoMeas>& stereo_obs_ptr = feature_info->_stereo_obs.at(time_of_obs);
            const std::shared_ptr<SE3>& pose_obs_ptr = sw_poses.at(time_of_obs);
            
            const Eigen::Matrix3d R_cm2w = pose_obs_ptr->valueLinearAsMat();
            const Eigen::Vector3d p_cm = pose_obs_ptr->valueTrans();
            
            const Eigen::Vector3d pf_cm = R_cm2w.transpose()*(pf_w-p_cm);
            
            Eigen::Matrix<double, 2, 3> H_proj = Eigen::Matrix<double, 2, 3>::Zero();
            H_proj(0, 0) = 1.0/pf_cm.z();
            H_proj(0, 2) = -pf_cm.x()/std::pow(pf_cm.z(), 2);
            H_proj(1, 1) = 1.0/pf_cm.z();
            H_proj(1, 2) = -pf_cm.y()/std::pow(pf_cm.z(), 2);
            
            const Eigen::Vector3d pf_cm_r = T_cl2cr*pf_cm;
            
            Eigen::Matrix<double, 2, 3> H_proj_r = Eigen::Matrix<double, 2, 3>::Zero();
            H_proj_r(0, 0) = 1.0/pf_cm_r.z();
            H_proj_r(0, 2) = -pf_cm_r.x()/std::pow(pf_cm_r.z(), 2);
            H_proj_r(1, 1) = 1.0/pf_cm_r.z();
            H_proj_r(1, 2) = -pf_cm_r.y()/std::pow(pf_cm_r.z(), 2);
            
            Eigen::MatrixXd H_pf2x = Eigen::MatrixXd::Zero(3, num_of_cols);
            Eigen::Matrix<double, 3, 6> H_pf2anchor = Eigen::Matrix<double, 3, 6>::Zero();
            
            if (pose_anchor_ptr != pose_obs_ptr)
            {
                H_pf2x.block<3, 3>(0, sw_index_map.at(pose_obs_ptr)) = R_cm2w.transpose()*skew(pf_w);
            
                H_pf2anchor.block<3, 3>(0, 0) = -H_pf2x.block<3, 3>(0, sw_index_map.at(pose_obs_ptr));
            }
            
            H_pf2x.block<3, 3>(0, sw_index_map.at(pose_obs_ptr)+3) = -R_cm2w.transpose();
            Eigen::Matrix3d H_pf2pf = R_cm2w.transpose();
            
            if (H_proj.hasNaN() || H_pf2x.hasNaN()) 
                continue;
            
            H_block_tmp.block(row_cnt, 0, 2, num_of_cols) = H_proj*H_pf2x;
            
            H_anchor_block_tmp.block(row_cnt, 0, 2, 6) = H_proj*H_pf2anchor;
            
            Hf_block_tmp.block(row_cnt, 0, 2, 3) = H_proj*H_pf2pf;
            
            H_block_tmp.block(row_cnt+2, 0, 2, num_of_cols) = H_proj_r*R_cl2cr*H_pf2x;
            
            H_anchor_block_tmp.block(row_cnt+2, 0, 2, 6) = H_proj_r*R_cl2cr*H_pf2anchor;
            
            Hf_block_tmp.block(row_cnt+2, 0, 2, 3) = H_proj_r*R_cl2cr*H_pf2pf;
            
            res_block_tmp.block(row_cnt, 0, 4, 1) = stereo_obs_ptr->asVec()-Eigen::Vector4d(pf_cm.x()/pf_cm.z(), pf_cm.y()/pf_cm.z(), pf_cm_r.x()/pf_cm_r.z(), pf_cm_r.y()/pf_cm_r.z());
            
            row_cnt += 4;
        }
        
        if (row_cnt < res_block.rows())
        {
            res_block_tmp.conservativeResize(row_cnt, 1);
            H_block_tmp.conservativeResize(row_cnt, num_of_cols);
            H_anchor_block_tmp.conservativeResize(row_cnt, 6);
            Hf_block_tmp.conservativeResize(row_cnt, 3);
        }
        
        if (row_cnt <= 3)
            std::cout << "[KeyframeUpdate]: Warning! in calc one feat selected jacobian, num of rows <= 3 !" << std::endl;
        
        Eigen::JacobiSVD<Eigen::MatrixXd> svd_helper(Hf_block_tmp, Eigen::ComputeFullU | Eigen::ComputeThinV);
        Eigen::MatrixXd V = svd_helper.matrixU().rightCols(row_cnt-3);
        
        H_block = V.transpose()*H_block_tmp;
        H_anchor_block = V.transpose()*H_anchor_block_tmp;
        res_block = V.transpose()*res_block_tmp;
        
        return pose_anchor_ptr;
    }
    
    void KeyframeUpdate::updateStateMono(std::shared_ptr<State> state,
                                         std::shared_ptr<MapServer> map_server,
                                         std::shared_ptr<Triangulator> tri)
    {
        std::vector<double> selected_timestamps;
        
        this->getMargKfs(state, selected_timestamps);
        
        if (selected_timestamps.size() == 0)
            return;
        
        std::vector<std::shared_ptr<SE3>> sw_var_order;
        std::map<std::shared_ptr<SE3>, int> sw_index_map;
        std::vector<std::shared_ptr<Type>> sw_var_type;
        
        this->generateSwVarOrder(state, selected_timestamps, sw_var_order,
                                 sw_index_map, sw_var_type);
        
        std::vector<int> update_ids;
        
        for (const auto& item : *map_server)
        {
            const auto& feat_info_ptr = item.second;
            
            if (feat_info_ptr->_ftype != FeatureInfo::FeatureType::MSCKF)
                continue;
            
            bool flag = false;
            const auto& mono_obs = item.second->_mono_obs;
            
            for (const double& ts : selected_timestamps)
                if (mono_obs.find(ts) == mono_obs.end())
                {
                    flag = true;
                    break;
                }
                
            if (flag) continue;
                
            if (FeatureInfoManager::triangulateFeatureInfoMono(feat_info_ptr, tri, state))
                update_ids.push_back(item.first);
        }
        
        if (update_ids.size() == 0) return;
        
        const int max_possible_cols = 6*state->_sw_camleft_poses.size();
        
        const int max_possible_rows = update_ids.size()*(2*selected_timestamps.size()-3);
        
        Eigen::VectorXd res_large(max_possible_rows);
        Eigen::MatrixXd H_large(max_possible_rows, max_possible_cols);
        res_large.setZero();
        H_large.setZero();
        
        int row_cnt = 0;
        int col_cnt = 6*sw_var_type.size();
        
        int feats_cnt = 0;
        
        for (int i = 0; i < update_ids.size(); ++i)
        {
            Eigen::VectorXd res_block;
            Eigen::MatrixXd H_block;
            Eigen::MatrixXd H_anchor_block;
            
            const auto& anchor_pose_ptr = this->calcResJacobianSingleFeatSelectedMonoObs(
                map_server->at(update_ids[i]), state->_sw_camleft_poses,
                sw_var_order, sw_index_map, selected_timestamps,
                res_block, H_block, H_anchor_block);
            
            bool flag = false;
            
            if (sw_index_map.find(anchor_pose_ptr) == sw_index_map.end())
            {
                flag = true;
                
                sw_var_order.push_back(anchor_pose_ptr);
                sw_var_type.push_back(anchor_pose_ptr);
                sw_index_map[anchor_pose_ptr] = col_cnt;
                
                col_cnt += anchor_pose_ptr->size();
                
                H_block.conservativeResize(H_block.rows(), H_block.cols() + 6);
            }
            
            H_block.block(0, sw_index_map.at(anchor_pose_ptr), H_block.rows(), 6) = H_anchor_block;
            
            if (!testChiSquared(state, res_block, H_block, 
                                sw_var_type, this->_noise, 2))
            {
                if (flag)
                {
                    sw_var_order.pop_back();
                    sw_var_type.pop_back();
                    sw_index_map.erase(anchor_pose_ptr);
                    
                    col_cnt -= anchor_pose_ptr->size();
                }
                
                continue;
            }
            
            H_large.block(row_cnt, 0, H_block.rows(), col_cnt) = H_block;
            res_large.block(row_cnt, 0, res_block.rows(), 1) = res_block;
            
            row_cnt += res_block.rows();
            
            ++feats_cnt;
        }
        
        if (row_cnt < max_possible_rows || col_cnt < max_possible_cols)
        {
            H_large.conservativeResize(row_cnt, col_cnt);
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
            
            H_thin = H_temp.topRows(col_cnt);
            res_thin = r_temp.head(col_cnt);                 
        }
        else
        {
            H_thin = H_large;
            res_thin = res_large;
        }
        
        /*
         std::cout << "[KeyframeUpdate]: Num of mono feats used in selected pose = " << feats_cnt << std::endl;                                                 *
         */
        
        StateManager::ekfUpdate(state, sw_var_type, H_thin, res_thin, std::pow(this->_noise, 2)*Eigen::MatrixXd::Identity(res_thin.rows(), res_thin.rows()));
    }
    
    void KeyframeUpdate::updateStateStereo(std::shared_ptr<State> state,
                                           std::shared_ptr<MapServer> map_server,
                                           std::shared_ptr<Triangulator> tri)
    {
        std::vector<double> selected_timestamps;
        
        this->getMargKfs(state, selected_timestamps);
        
        if (selected_timestamps.size() == 0)
            return;
        
        std::vector<std::shared_ptr<SE3>> sw_var_order;
        std::map<std::shared_ptr<SE3>, int> sw_index_map;
        std::vector<std::shared_ptr<Type>> sw_var_type;
        
        this->generateSwVarOrder(state, selected_timestamps, sw_var_order,
                                 sw_index_map, sw_var_type);
        
        std::vector<int> update_ids;
        
        for (const auto& item : *map_server)
        {
            const auto& feat_info_ptr = item.second;
            
            if (feat_info_ptr->_ftype != FeatureInfo::FeatureType::MSCKF)
                continue;
            
            bool flag = false;
            const auto& stereo_obs = item.second->_stereo_obs;
            
            for (const double& ts : selected_timestamps)
                if (stereo_obs.find(ts) == stereo_obs.end())
                {
                    flag = true;
                    break;
                }
                
            if (flag) continue;
                
            if (FeatureInfoManager::triangulateFeatureInfoStereo(feat_info_ptr, tri, state))
                update_ids.push_back(item.first);
        }
        
        if (update_ids.size() == 0) return;
        
        const int max_possible_cols = 6*state->_sw_camleft_poses.size();
        
        const int max_possible_rows = update_ids.size()*(4*selected_timestamps.size()-3);
        
        Eigen::VectorXd res_large(max_possible_rows);
        Eigen::MatrixXd H_large(max_possible_rows, max_possible_cols);
        res_large.setZero();
        H_large.setZero();
        
        int row_cnt = 0;
        int col_cnt = 6*sw_var_type.size();
        
        int feats_cnt = 0;
        
        for (int i = 0; i < update_ids.size(); ++i)
        {
            Eigen::VectorXd res_block;
            Eigen::MatrixXd H_block;
            Eigen::MatrixXd H_anchor_block;
            
            const auto& anchor_pose_ptr = this->calcResJacobianSingleFeatSelectedStereoObs(
                map_server->at(update_ids[i]), state->_sw_camleft_poses,
                sw_var_order, sw_index_map, selected_timestamps,
                state->_state_params._T_cl2cr,
                res_block, H_block, H_anchor_block);
            
            bool flag = false;
            
            if (sw_index_map.find(anchor_pose_ptr) == sw_index_map.end())
            {
                flag = true;
                
                sw_var_order.push_back(anchor_pose_ptr);
                sw_var_type.push_back(anchor_pose_ptr);
                sw_index_map[anchor_pose_ptr] = col_cnt;
                
                col_cnt += anchor_pose_ptr->size();
                
                H_block.conservativeResize(H_block.rows(), H_block.cols() + 6);
            }
            
            H_block.block(0, sw_index_map.at(anchor_pose_ptr), H_block.rows(), 6) = H_anchor_block;
            
            if (!testChiSquared(state, res_block, H_block, sw_var_type, 
                this->_noise, 2))
            {
                if (flag)
                {
                    sw_var_order.pop_back();
                    sw_var_type.pop_back();
                    sw_index_map.erase(anchor_pose_ptr);
                    
                    col_cnt -= anchor_pose_ptr->size();
                }
                
                continue;
            }
            
            H_large.block(row_cnt, 0, H_block.rows(), col_cnt) = H_block;
            res_large.block(row_cnt, 0, res_block.rows(), 1) = res_block;
            
            row_cnt += res_block.rows();
            
            ++feats_cnt;
        }
        
        if (row_cnt < max_possible_rows || col_cnt < max_possible_cols)
        {
            H_large.conservativeResize(row_cnt, col_cnt);
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
            
            H_thin = H_temp.topRows(col_cnt);
            res_thin = r_temp.head(col_cnt);                 
        }
        else
        {
            H_thin = H_large;
            res_thin = res_large;
        }
        
        /*
         std::cout << "[KeyframeUpdate]: Num of stereo feats used in selected pose = " << feats_cnt << std::endl;                                              
         */
        
        StateManager::ekfUpdate(state, sw_var_type, H_thin, res_thin, std::pow(this->_noise, 2)*Eigen::MatrixXd::Identity(res_thin.rows(), res_thin.rows()));
    }
    
    void KeyframeUpdate::cleanStereoObsAtMargTime(std::shared_ptr<State> state,
                                                  std::shared_ptr<MapServer> map_server)
    {
        std::vector<double> marg_kfs;
        
        this->getMargKfs(state, marg_kfs);
        
        std::vector<int> all_ids;
        for (const auto& item : *map_server)
            all_ids.push_back(item.first);
        
        std::vector<int> ids_to_clean;
        for (const int& id : all_ids)
            for (const double& marg_time : marg_kfs)
            {
                if (map_server->at(id)->_stereo_obs.find(marg_time) != map_server->at(id)->_stereo_obs.end())
                    map_server->at(id)->_stereo_obs.erase(marg_time);
                
                if (map_server->at(id)->_stereo_obs.size() == 0)
                    ids_to_clean.push_back(id);
            }
            
        for (int id : ids_to_clean)
                map_server->erase(id);
    }
}
