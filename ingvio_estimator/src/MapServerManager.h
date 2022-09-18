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
        
    };
}
