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

#pragma once

#include <feature_tracker/MonoFrame.h>
#include <feature_tracker/MonoMeas.h>

#include <feature_tracker/StereoFrame.h>
#include <feature_tracker/StereoMeas.h>

#include <memory>

namespace ingvio
{
    struct MonoMeas;
    struct StereoMeas;
    class FeatureInfo;
    
    class State;
    class Triangulator;
    
    class MonoMeasManager
    {
    private:
        MonoMeasManager() {}
        MonoMeasManager(const MonoMeasManager&) {}
        MonoMeasManager operator=(const MonoMeasManager&) = delete;
        
    public:
        static std::shared_ptr<MonoMeas> convertFromMsg(
            const feature_tracker::MonoMeas::ConstPtr& mono_msg);
        
        static std::shared_ptr<MonoMeas> convertFromMsg(
            const feature_tracker::MonoMeas& mono_msg);
        
        static std::shared_ptr<MonoMeas> random(int id);
    };
    
    class StereoMeasManager
    {
    private:
        StereoMeasManager() {}
        StereoMeasManager(const StereoMeasManager&) {}
        StereoMeasManager operator=(const StereoMeasManager&) = delete;
   
    public:
        static std::shared_ptr<StereoMeas> convertFromMsg(const feature_tracker::StereoMeas::ConstPtr& stereo_msg);
        
        static std::shared_ptr<StereoMeas> convertFromMsg(
            const feature_tracker::StereoMeas& stereo_msg);
        
        static std::shared_ptr<StereoMeas> random(int id);
    };
    
    class FeatureInfoManager
    {
    private:
        FeatureInfoManager () {}
        FeatureInfoManager(const FeatureInfoManager&) {}
        FeatureInfoManager operator=(const FeatureInfoManager&) = delete;
    
    public:
        static void collectMonoMeas(std::shared_ptr<FeatureInfo> feature_info,
                                    std::shared_ptr<State> state,
                                    std::shared_ptr<MonoMeas> mono_meas);
        
        static void collectStereoMeas(std::shared_ptr<FeatureInfo> feature_info,
                                      std::shared_ptr<State> state,
                                      std::shared_ptr<StereoMeas> stereo_meas);
        
        static bool triangulateFeatureInfoMono(std::shared_ptr<FeatureInfo> feature_info,
                                               const std::shared_ptr<Triangulator> tri,
                                               const std::shared_ptr<State> state);
        
        static bool triangulateFeatureInfoStereo(std::shared_ptr<FeatureInfo> feature_info,
                                                 const std::shared_ptr<Triangulator> tri,
                                                 const std::shared_ptr<State> state);
        
        static void changeAnchoredPose(std::shared_ptr<FeatureInfo> feature_info,
                                       std::shared_ptr<State> state,
                                       double target_sw_timestamp);
        
        static void changeAnchoredPose(std::shared_ptr<FeatureInfo> feature_info,
                                       std::shared_ptr<State> state);
    };
    
    class MapServerManager
    {
    private:
        MapServerManager () {}
        MapServerManager(const MapServerManager&) {}
        MapServerManager operator=(const MapServerManager&) = delete;
        
    public:     
        
        static void collectMonoMeas(std::shared_ptr<MapServer> map_server,
                                    std::shared_ptr<State> state,
                                    const feature_tracker::MonoFrame::ConstPtr& mono_frame_msg);
        
        static void collectStereoMeas(std::shared_ptr<MapServer> map_server,
                                      std::shared_ptr<State> state,
                                      const feature_tracker::StereoFrame::ConstPtr& stereo_frame_msg);
        
        static void markMargMonoFeatures(std::shared_ptr<MapServer> map_server,
                                         std::shared_ptr<State> state);
        
        static void markMargStereoFeatures(std::shared_ptr<MapServer> map_server,
                                           std::shared_ptr<State> state);
        
        static void mapStatistics(const std::shared_ptr<MapServer> map_server);
        
        static void checkMapStateConsistent(const std::shared_ptr<MapServer> map_server,
                                            const std::shared_ptr<State> state);
        
        static void checkAnchorStatus(const std::shared_ptr<MapServer> map_server,
                                      const std::shared_ptr<State> state);
        
        static void eraseInvalidFeatures(std::shared_ptr<MapServer> map_server,
                                         std::shared_ptr<State> state);
    };
}
