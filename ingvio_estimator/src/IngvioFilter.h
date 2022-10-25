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
    class KeyframeUpdate;
    class LandmarkUpdate;
    class GnssData;
    class GnssSync;
    class GvioAligner;
    struct SppMeas;
    class GnssUpdate;
    
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
        
        ros::Publisher _odom_spp_pub, _odom_gt_pub;
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
        
        std::shared_ptr<KeyframeUpdate> _keyframe_update;
        
        std::shared_ptr<LandmarkUpdate> _landmark_update;
        
        std::shared_ptr<GnssData> _gnss_data;
        
        std::shared_ptr<GnssSync> _gnss_sync;
        
        std::shared_ptr<GvioAligner> _gvio_aligner;
        
        std::shared_ptr<GnssUpdate> _gnss_update;
        
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
