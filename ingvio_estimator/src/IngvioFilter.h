#pragma once

#include <memory>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>

#include <feature_tracker/MonoFrame.h>
#include <feature_tracker/StereoFrame.h>

#include "IngvioParams.h"
#include "MapServer.h"

namespace ingvio
{
    class State;
    class ImuPropagator;
    class Triangulator;
    class RemoveLostUpdate;
    class SwMargUpdate;
    
    class IngvioFilter
    {
    public:
        IngvioFilter(ros::NodeHandle& n) : _nh(n) {}
        
        ~IngvioFilter() = default;
        
        IngvioFilter(const IngvioFilter&) = delete;
        
        IngvioFilter operator=(const IngvioFilter&) = delete;
        
        void initIO();
        
    protected:
        
        IngvioParams _filter_params;
        
        ros::NodeHandle _nh;
        
        ros::Subscriber _sub_mono_frame, _sub_stereo_frame, _sub_imu;
        ros::Publisher _odom_w_pub, _path_w_pub;
        tf::TransformBroadcaster _tf_pub;
        
        nav_msgs::Path _path_w_msg;
        
        bool _hasImageCome = false;
        
        bool _hasInitState = false;
        
        std::shared_ptr<State> _state;
        
        std::shared_ptr<ImuPropagator> _imu_propa;
        
        std::shared_ptr<Triangulator> _tri;
        
        std::shared_ptr<MapServer> _map_server;
        
        std::shared_ptr<RemoveLostUpdate> _remove_lost_update;
        
        std::shared_ptr<SwMargUpdate> _sw_marg_update;
        
        void callbackMonoFrame(const feature_tracker::MonoFrameConstPtr& mono_frame_ptr);
        
        void callbackStereoFrame(const feature_tracker::StereoFrameConstPtr& stereo_frame_ptr);
        
        void callbackIMU(sensor_msgs::Imu::ConstPtr imu_msg);
        
        void visualize(const std_msgs::Header& header);
    };
    
    typedef std::shared_ptr<IngvioFilter> IngvioFilterPtr;
}
