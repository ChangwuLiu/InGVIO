#pragma once

#include <memory>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Path.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#include <gnss_comm/gnss_ros.hpp>

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
    class LandmarkUpdate;
    class GnssData;
    class GnssSync;
    class GvioAligner;
    class SppMeas;
    
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
        
        ros::Subscriber _sub_ephem, _sub_glo_ephem, _sub_gnss_meas, _sub_iono_params, _sub_rtk_gt;
        
        ros::Publisher _path_spp_pub, _path_gt_pub;
        
        nav_msgs::Path _path_w_msg, _path_spp_msg, _path_gt_msg;
        
        bool _hasImageCome = false;
        
        bool _hasInitState = false;
        
        std::shared_ptr<State> _state;
        
        std::shared_ptr<ImuPropagator> _imu_propa;
        
        std::shared_ptr<Triangulator> _tri;
        
        std::shared_ptr<MapServer> _map_server;
        
        std::shared_ptr<RemoveLostUpdate> _remove_lost_update;
        
        std::shared_ptr<SwMargUpdate> _sw_marg_update;
        
        std::shared_ptr<LandmarkUpdate> _landmark_update;
        
        std::shared_ptr<GnssData> _gnss_data;
        
        std::shared_ptr<GnssSync> _gnss_sync;
        
        std::shared_ptr<GvioAligner> _gvio_aligner;
        
        void callbackMonoFrame(const feature_tracker::MonoFrameConstPtr& mono_frame_ptr);
        
        void callbackStereoFrame(const feature_tracker::StereoFrameConstPtr& stereo_frame_ptr);
        
        void callbackIMU(sensor_msgs::Imu::ConstPtr imu_msg);
        
        void callbackEphem(const gnss_comm::GnssEphemMsgConstPtr& ephem_msg);
        
        void callbackGloEphem(const gnss_comm::GnssGloEphemMsgConstPtr& glo_ephem_msg);
        
        void callbackIonoParams(const gnss_comm::StampedFloat64ArrayConstPtr& iono_msg);
        
        void callbackGnssMeas(const gnss_comm::GnssMeasMsgConstPtr& gnss_meas_msg);
        
        void callbackRtkGroundTruth(const sensor_msgs::NavSatFix::ConstPtr& nav_sat_msg);
        
        void visualize(const std_msgs::Header& header);
        
        void visualizeSpp(const std_msgs::Header& header, const SppMeas& spp_meas);
    };
    
    typedef std::shared_ptr<IngvioFilter> IngvioFilterPtr;
}
