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

#include <iostream>
#include <map>
#include <unordered_map>
#include <memory>
#include <cmath>

#include "State.h"
#include "StateManager.h"

#include "MapServer.h"
#include "MapServerManager.h"

#include "Triangulator.h"

namespace ingvio
{
    std::shared_ptr<MonoMeas> MonoMeasManager::convertFromMsg(
        const feature_tracker::MonoMeas::ConstPtr& mono_msg)
    {
        std::shared_ptr<MonoMeas> mono_meas = std::make_shared<MonoMeas>();
        
        mono_meas->setFromMonoMsg(mono_msg);
        
        return mono_meas;
    }
    
    std::shared_ptr<MonoMeas> MonoMeasManager::convertFromMsg(
        const feature_tracker::MonoMeas& mono_msg)
    {
        std::shared_ptr<MonoMeas> mono_meas = std::make_shared<MonoMeas>();
        
        mono_meas->setFromMonoMsg(mono_msg);
        
        return mono_meas;
    }
    
    std::shared_ptr<MonoMeas> MonoMeasManager::random(int id)
    {
        std::shared_ptr<MonoMeas> mono_meas = std::make_shared<MonoMeas>();
        
        Eigen::Vector2d vec = Eigen::Vector2d::Random();
        
        mono_meas->_id = id;
        mono_meas->_u0 = vec.x();
        mono_meas->_v0 = vec.y();
        
        return mono_meas;
    }
    
    std::shared_ptr<StereoMeas> StereoMeasManager::convertFromMsg(
        const feature_tracker::StereoMeas::ConstPtr& stereo_msg)
    {
        std::shared_ptr<StereoMeas> stereo_meas = std::make_shared<StereoMeas>();
        
        stereo_meas->setFromStereoMsg(stereo_msg);
        
        return stereo_meas;
    }
    
    std::shared_ptr<StereoMeas> StereoMeasManager::convertFromMsg(
        const feature_tracker::StereoMeas& stereo_msg)
    {
        std::shared_ptr<StereoMeas> stereo_meas = std::make_shared<StereoMeas>();
        
        stereo_meas->setFromStereoMsg(stereo_msg);
        
        return stereo_meas;
    }
    
    std::shared_ptr<StereoMeas> StereoMeasManager::random(int id)
    {
        std::shared_ptr<StereoMeas> stereo_meas = std::make_shared<StereoMeas>();
        
        Eigen::Vector4d vec = Eigen::Vector4d::Random();
        
        stereo_meas->_id = id;
        stereo_meas->_u0 = vec.x();
        stereo_meas->_v0 = vec.y();
        stereo_meas->_u1 = vec.z();
        stereo_meas->_v1 = vec.w();
        
        return stereo_meas;
    }
    
    void FeatureInfoManager::collectMonoMeas(std::shared_ptr<FeatureInfo> feature_info,
                                             std::shared_ptr<State> state,
                                             std::shared_ptr<MonoMeas> mono_meas)
    {
        const double& timestamp = state->_timestamp;
        
        if (state->_sw_camleft_poses.find(timestamp) == state->_sw_camleft_poses.end())
        {
            std::cout << "[FeatureInfoManager]: Meas timestamp not in sw!" << std::endl;
            assert(false);
        }
        
        if (feature_info->_mono_obs.size() == 0)
        {
            feature_info->_mono_obs[timestamp] = mono_meas;
            feature_info->_id = mono_meas->_id;
            
            feature_info->_ftype = FeatureInfo::FeatureType::MSCKF;
            feature_info->_isToMarg = false;
            feature_info->_isTri = false;
            
            feature_info->_landmark->resetAnchoredPose(state->_sw_camleft_poses.at(timestamp));
        }
        else
        {
            if (feature_info->hasMonoObsAt(timestamp))
            {
                std::cout << "[FeatureInfoManager]: Meas timestamp already in mono obs, skip adding! " << std::endl;
                return;
            }
            
            if (feature_info->_ftype == FeatureInfo::FeatureType::SLAM)
                feature_info->_mono_obs.clear();
                
            feature_info->_mono_obs[timestamp] = mono_meas;
            
            assert(feature_info->_id == mono_meas->_id);
            
            feature_info->_isToMarg = false;
        }
    }
    
    void FeatureInfoManager::collectStereoMeas(std::shared_ptr<FeatureInfo> feature_info,
                                               std::shared_ptr<State> state,
                                               std::shared_ptr<StereoMeas> stereo_meas)
    {
        const double& timestamp = state->_timestamp;
        
        if (state->_sw_camleft_poses.find(timestamp) == state->_sw_camleft_poses.end())
        {
            std::cout << "[FeatureInfoManager]: Meas timestamp not in sw!" << std::endl;
            assert(false);
        }
        
        if (feature_info->_stereo_obs.size() == 0)
        {
            feature_info->_stereo_obs[timestamp] = stereo_meas;
            feature_info->_id = stereo_meas->_id;
            
            feature_info->_ftype = FeatureInfo::FeatureType::MSCKF;
            feature_info->_isToMarg = false;
            feature_info->_isTri = false;
            
            feature_info->_landmark->resetAnchoredPose(state->_sw_camleft_poses.at(timestamp));
        }
        else
        {
            if (feature_info->hasStereoObsAt(timestamp))
            {
                std::cout << "[FeatureInfoManager]: Meas timestamp already in stereo obs, skip adding! " << std::endl;
                return;
            }
            
            if (feature_info->_ftype == FeatureInfo::FeatureType::SLAM)
                feature_info->_stereo_obs.clear();
            
            feature_info->_stereo_obs[timestamp] = stereo_meas;
            
            assert(feature_info->_id == stereo_meas->_id);
            
            feature_info->_isToMarg = false;
        }
    }
    
    void MapServerManager::collectMonoMeas(std::shared_ptr<MapServer> map_server,
                                           std::shared_ptr<State> state,
                                           const feature_tracker::MonoFrame::ConstPtr& mono_frame_msg)
    {
        assert(mono_frame_msg->header.stamp.toSec() == state->_timestamp);
        
        for (const auto& item : mono_frame_msg->mono_features)
        {
            if (map_server->find(item.id) == map_server->end())
                map_server->insert(std::make_pair(item.id, std::make_shared<FeatureInfo>()));
                
            FeatureInfoManager::collectMonoMeas(map_server->at(item.id), state, MonoMeasManager::convertFromMsg(item));
        }
    }
    
    void MapServerManager::collectStereoMeas(std::shared_ptr<MapServer> map_server,
                                             std::shared_ptr<State> state,
                                             const feature_tracker::StereoFrame::ConstPtr& stereo_frame_msg)
    {
        assert(stereo_frame_msg->header.stamp.toSec() == state->_timestamp);
        
        for (const auto& item : stereo_frame_msg->stereo_features)
        {
            if (map_server->find(item.id) == map_server->end())
                map_server->insert(std::make_pair(item.id, std::make_shared<FeatureInfo>()));
            
            FeatureInfoManager::collectStereoMeas(map_server->at(item.id), state, StereoMeasManager::convertFromMsg(item));
        }
    }
    
    void MapServerManager::markMargMonoFeatures(std::shared_ptr<MapServer> map_server,
                                                std::shared_ptr<State> state)
    {
        const double& curr_timestamp = state->_timestamp;
        
        std::vector<int> marg_ids;
        
        for (auto& item : *map_server)
            if (!item.second->hasMonoObsAt(curr_timestamp))
            {
                item.second->_isToMarg = true;
                    
                if (item.second->_ftype == FeatureInfo::FeatureType::SLAM)
                {
                    assert(item.second->_id == item.first);
                    marg_ids.push_back(item.second->_id);
                }
            }
            
        for (int i = 0; i < marg_ids.size(); ++i)
        {
            StateManager::margAnchoredLandmarkInState(state, marg_ids[i]);
                
            map_server->erase(marg_ids[i]);
        }
    }
    
    void MapServerManager::markMargStereoFeatures(std::shared_ptr<MapServer> map_server,
                                                  std::shared_ptr<State> state)
    {
        const double& curr_timestamp = state->_timestamp;
        
        std::vector<int> marg_ids;
        
        for (auto& item : *map_server)
        {
            if (!item.second->hasStereoObsAt(curr_timestamp))
            {
                item.second->_isToMarg = true;
                    
                if (item.second->_ftype == FeatureInfo::FeatureType::SLAM)
                {
                    assert(item.second->_id == item.first);
                    marg_ids.push_back(item.second->_id);
                }
            }
        }
            
        for (int i = 0; i < marg_ids.size(); ++i)
        {
            StateManager::margAnchoredLandmarkInState(state, marg_ids[i]);
                
            map_server->erase(marg_ids[i]);
        }
    }
    
    bool FeatureInfoManager::triangulateFeatureInfoMono(
        std::shared_ptr<FeatureInfo> feature_info,
        const std::shared_ptr<Triangulator> tri,
        const std::shared_ptr<State> state)
    {
        Eigen::Vector3d pf;
        
        bool flag = tri->triangulateMonoObs(feature_info->_mono_obs, state->_sw_camleft_poses, pf);
        
        if (flag && !pf.hasNaN())
        {
            ++feature_info->_numOfTri;
            
            Eigen::Vector3d body_anchor = feature_info->_landmark->getAnchoredPose()->copyValueAsIso().inverse()*pf;
            
            if (body_anchor.z() <= 0) return false;
            
            if (!feature_info->_isTri)
            {
                feature_info->_landmark->setFejPosXyz(pf);
                feature_info->_landmark->setValuePosXyz(pf);
                feature_info->_isTri = true;
                
                return true;
            }
                
            feature_info->_landmark->setValuePosXyz(pf);
                
            return true;
        }
        else
            return false;
    }
    
    bool FeatureInfoManager::triangulateFeatureInfoStereo(
        std::shared_ptr<FeatureInfo> feature_info,
        const std::shared_ptr<Triangulator> tri,
        const std::shared_ptr<State> state)
    {
        Eigen::Vector3d pf;
        
        bool flag = tri->triangulateStereoObs(feature_info->_stereo_obs, state->_sw_camleft_poses, state->_state_params._T_cl2cr, pf);
        
        if (flag && !pf.hasNaN())
        {
            ++feature_info->_numOfTri;
            
            Eigen::Vector3d body_anchor = feature_info->_landmark->getAnchoredPose()->copyValueAsIso().inverse()*pf;
            
            if (body_anchor.z() <= 0) return false;
            
            if (!feature_info->_isTri)
            {
                feature_info->_landmark->setFejPosXyz(pf);
                feature_info->_landmark->setValuePosXyz(pf);
                feature_info->_isTri = true;
                
                return true;
            }
            
            feature_info->_landmark->setValuePosXyz(pf);
            
            return true;
        }
        else
            return false;
    }
    
    void FeatureInfoManager::changeAnchoredPose(std::shared_ptr<FeatureInfo> feature_info,
                                                std::shared_ptr<State> state,
                                                double target_sw_timestamp)
    {
        if (state->_sw_camleft_poses.size() < 2) return;
        if (state->_sw_camleft_poses.find(target_sw_timestamp) == state->_sw_camleft_poses.end() || state->_anchored_landmarks.find(feature_info->getId()) == state->_anchored_landmarks.end())
            return;
        
        if (feature_info->_ftype != FeatureInfo::FeatureType::SLAM) return;
        
        bool notFoundcurr = true;
        for (const auto& item : state->_sw_camleft_poses)
            if (item.second == feature_info->anchor())
            {
                notFoundcurr = false;
                break;
            }
        
        if (notFoundcurr || state->_anchored_landmarks.at(feature_info->getId()) != feature_info->landmark()) return;

        std::vector<std::shared_ptr<Type>> var_order;
        var_order.push_back(feature_info->anchor());
        var_order.push_back(state->_sw_camleft_poses.at(target_sw_timestamp));
        var_order.push_back(state->_anchored_landmarks.at(feature_info->getId()));
        
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 15);
        const Eigen::Vector3d& pf = feature_info->landmark()->valuePosXyz();
        
        H.block<3, 3>(0, 0) = -skew(pf);
        H.block<3, 3>(0, 6) = skew(pf);
        H.block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
        
        StateManager::replaceVarLinear(state, feature_info->_landmark, var_order, H);
        
        feature_info->_landmark->resetAnchoredPose(state->_sw_camleft_poses.at(target_sw_timestamp), true);
        
    }
    
    void FeatureInfoManager::changeAnchoredPose(std::shared_ptr<FeatureInfo> feature_info,
                                                std::shared_ptr<State> state)
    {
        if (state->_sw_camleft_poses.size() < 2) return;
        
        FeatureInfoManager::changeAnchoredPose(feature_info, state, state->_sw_camleft_poses.rbegin()->first);
    }
    
    void MapServerManager::mapStatistics(const std::shared_ptr<MapServer> map_server)
    {
        int num_of_msckf = 0;
        int num_of_slam = 0;
        int num_of_err = 0;
        
        for (const auto& item : *map_server)
        {
            if (item.second->_ftype ==  FeatureInfo::FeatureType::SLAM)
                ++num_of_slam;
            else if (item.second->_ftype == FeatureInfo::FeatureType::MSCKF)
                ++num_of_msckf;
            
            if (item.second->_mono_obs.size() == 0 && item.second->_stereo_obs.size() == 0)
                ++num_of_err;
        }
        
        std::cout << "[MapServerManager]: Num of msckf feats = " << num_of_msckf << " Num of slam feats = " << num_of_slam << std::endl;
        
        if (num_of_err > 0)
            std::cout << "[MapServerManager]: Warning!! " << num_of_err << " feats in map server is null without obs!" << std::endl;
    }
    
    void MapServerManager::checkMapStateConsistent(const std::shared_ptr<MapServer> map_server,
                                                   const std::shared_ptr<State> state)
    {
        std::vector<int> ids_slam;
        
        for (const auto& item : *map_server)
            if (item.second->_ftype == FeatureInfo::FeatureType::SLAM)
            {
                const int& id = item.first;
                
                ids_slam.push_back(id);
                
                if (state->_anchored_landmarks.find(id) == state->_anchored_landmarks.end())
                    std::cout << "[MapServerManager]: Map server slam feat id = " << id << " not in the state! " << std::endl;
                else
                {
                    if (state->_anchored_landmarks.at(id) != item.second->_landmark)
                        std::cout << "[MapServerManager]: Map server slam feat id = " << id << " lm ptr not the same with that in state! " << std::endl;
                    
                    if (state->_anchored_landmarks.at(id)->getAnchoredPose() != item.second->_landmark->getAnchoredPose())
                        std::cout << "[MapServerManager]: Map server slam feat id = " << id << " anchor pose not the same with that of in state! " << std::endl;
                }
            }
        
        if (ids_slam.size() != state->_anchored_landmarks.size())
            std::cout << "[MapServerManager]: Some state lm are no longer in map server!" << std::endl;
    }
    
    void MapServerManager::checkAnchorStatus(const std::shared_ptr<MapServer> map_server,
                                             const std::shared_ptr<State> state)
    {
        std::unordered_map<std::shared_ptr<SE3>, double> sw_poses;
        for (const auto& item : state->_sw_camleft_poses)
            sw_poses.insert(std::make_pair(item.second, item.first));
        
        for (const auto& item : *map_server)
            if (sw_poses.find(item.second->_landmark->getAnchoredPose()) == sw_poses.end())
            {
                std::cout << "[MapServerManager]: Anchor of Feat id = " << item.first << " not in the sw! " << std::endl;
            }
    }
    
    void MapServerManager::eraseInvalidFeatures(std::shared_ptr<MapServer> map_server,
                                                std::shared_ptr<State> state)
    {
        std::vector<int> ids_to_remove;
        
        for (const auto& item : *map_server)
        {
            const int& id = item.first;
            const auto& feat_info_ptr = item.second;
            
            if (!feat_info_ptr->_isTri) continue;
            
            if (feat_info_ptr->_landmark->getAnchoredPose() == nullptr)
            {
                ids_to_remove.push_back(id);
                continue;
            }
            
            const auto anchor_ptr = feat_info_ptr->_landmark->getAnchoredPose();
            
            Eigen::Vector3d body = anchor_ptr->copyValueAsIso().inverse()*feat_info_ptr->_landmark->valuePosXyz();
            
            if (body.z() <= 0.2) 
                ids_to_remove.push_back(id);
        }
        
        if (ids_to_remove.size() > 0)
            std::cout << "[MapServerManager]: Num of feats have negative depth in anchor pose = " << ids_to_remove.size() << std::endl;
        
        for (const int& id : ids_to_remove)
        {
            if (map_server->at(id)->_ftype == FeatureInfo::FeatureType::SLAM)
                StateManager::margAnchoredLandmarkInState(state, id);
            
            map_server->erase(id);
        }
    }
}

