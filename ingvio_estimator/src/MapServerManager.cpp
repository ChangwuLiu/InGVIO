#include <iostream>
#include <map>
#include <memory>
#include <cmath>

#include "State.h"
#include "StateManager.h"

#include "MapServer.h"
#include "MapServerManager.h"

namespace ingvio
{
    std::shared_ptr<MonoMeas> MonoMeasManager::convertFromMsg(const feature_tracker::MonoMeas::ConstPtr& mono_msg)
    {
        std::shared_ptr<MonoMeas> mono_meas = std::make_shared<MonoMeas>();
        
        mono_meas->setFromMonoMsg(mono_msg);
        
        return mono_meas;
    }
    
    std::shared_ptr<MonoMeas> MonoMeasManager::convertFromMsg(const feature_tracker::MonoMeas& mono_msg)
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
    
    std::shared_ptr<StereoMeas> StereoMeasManager::convertFromMsg(const feature_tracker::StereoMeas::ConstPtr& stereo_msg)
    {
        std::shared_ptr<StereoMeas> stereo_meas = std::make_shared<StereoMeas>();
        
        stereo_meas->setFromStereoMsg(stereo_msg);
        
        return stereo_meas;
    }
    
    std::shared_ptr<StereoMeas> StereoMeasManager::convertFromMsg(const feature_tracker::StereoMeas& stereo_msg)
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
    
    void FeatureInfoManager::collectMonoMeas(std::shared_ptr<FeatureInfo> feature_info, std::shared_ptr<State> state, std::shared_ptr<MonoMeas> mono_meas)
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
    
    void FeatureInfoManager::collectStereoMeas(std::shared_ptr<FeatureInfo> feature_info, std::shared_ptr<State> state, std::shared_ptr<StereoMeas> stereo_meas)
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
    
    void MapServerManager::collectMonoMeas(std::shared_ptr<MapServer> map_server, std::shared_ptr<State> state, const feature_tracker::MonoFrame::ConstPtr& mono_frame_msg)
    {
        assert(mono_frame_msg->header.stamp.toSec() == state->_timestamp);
        
        for (const auto& item : mono_frame_msg->mono_features)
        {
            if (map_server->_feats.find(item.id) == map_server->_feats.end())
                map_server->_feats[item.id] = std::make_shared<FeatureInfo>();
                
            FeatureInfoManager::collectMonoMeas(map_server->_feats.at(item.id), state, MonoMeasManager::convertFromMsg(item));
        }
    }
    
    void MapServerManager::collectStereoMeas(std::shared_ptr<MapServer> map_server, std::shared_ptr<State> state, const feature_tracker::StereoFrame::ConstPtr& stereo_frame_msg)
    {
        assert(stereo_frame_msg->header.stamp.toSec() == state->_timestamp);
        
        for (const auto& item : stereo_frame_msg->stereo_features)
        {
            if (map_server->_feats.find(item.id) == map_server->_feats.end())
                map_server->_feats[item.id] = std::make_shared<FeatureInfo>();
            
            FeatureInfoManager::collectStereoMeas(map_server->_feats.at(item.id), state, StereoMeasManager::convertFromMsg(item));
        }
    }
    
    void MapServerManager::markMargFeatures(std::shared_ptr<MapServer> map_server, std::shared_ptr<State> state, bool isStereo)
    {
        const double& curr_timestamp = state->_timestamp;
        
        std::vector<int> marg_ids;
        
        for (auto& item : map_server->_feats)
            if (isStereo)
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
            else
            {
                if (!item.second->hasMonoObsAt(curr_timestamp))
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
                
                map_server->_feats.erase(marg_ids[i]);
            }
    }
}

