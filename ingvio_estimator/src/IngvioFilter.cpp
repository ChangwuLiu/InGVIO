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

#include <nav_msgs/Odometry.h>
#include <tf_conversions/tf_eigen.h>

#include "IngvioFilter.h"

#include "State.h"
#include "StateManager.h"

#include "ImuPropagator.h"

#include "Triangulator.h"

#include "MapServerManager.h"

#include "RemoveLostUpdate.h"
#include "SwMargUpdate.h"
#include "KeyframeUpdate.h"
#include "LandmarkUpdate.h"

#include "TicToc.h"
#include "Color.h"

#include "GnssData.h"
#include "GnssSync.h"
#include "GvioAligner.h"
#include "GnssUpdate.h"

namespace ingvio
{
    void IngvioFilter::initIO()
    {
        _filter_params.readParams(_nh);
        
        if (_filter_params._cam_nums == 2)
        {
            _sub_stereo_frame = _nh.subscribe(_filter_params._feature_topic, 100, &IngvioFilter::callbackStereoFrame, this);
        }
        else if (_filter_params._cam_nums == 1)
        {
            _sub_mono_frame = _nh.subscribe(_filter_params._feature_topic, 100, &IngvioFilter::callbackMonoFrame, this);
        }
        else
        {
            std::cout << "[IngvioParams]: Cam num " << _filter_params._cam_nums << " not supported! Init as mono config!" << std::endl;
            
            _filter_params._cam_nums = 1;
            
            _sub_mono_frame = _nh.subscribe(_filter_params._feature_topic, 100, &IngvioFilter::callbackMonoFrame, this);
        }
        
        _sub_imu = _nh.subscribe(_filter_params._imu_topic, 500, &IngvioFilter::callbackIMU, this);
        
        _odom_w_pub = _nh.advertise<nav_msgs::Odometry>("pose_w", 5);
        _path_w_pub = _nh.advertise<nav_msgs::Path>("path_w", 1);
        
        _state = std::make_shared<State>(_filter_params);
        
        _imu_propa = std::make_shared<ImuPropagator>(_filter_params);
        
        _tri = std::make_shared<Triangulator>(_filter_params);
        
        _map_server = std::make_shared<MapServer>();
        
        _remove_lost_update = std::make_shared<RemoveLostUpdate>(_filter_params);
        
        _keyframe_update = std::make_shared<KeyframeUpdate>(_filter_params);
        
        _sw_marg_update = std::make_shared<SwMargUpdate>(_filter_params);
        
        _landmark_update = std::make_shared<LandmarkUpdate>(_filter_params);
        
        _gnss_data = std::make_shared<GnssData>();
        
        _gnss_sync = std::make_shared<GnssSync>(_filter_params);
        
        _gvio_aligner = std::make_shared<GvioAligner>(_filter_params);
        
        _gnss_update = std::make_shared<GnssUpdate>(_filter_params);
        
        if (_filter_params._enable_gnss)
        {
            _sub_ephem = _nh.subscribe(_filter_params._gnss_ephem_topic, 100, &IngvioFilter::callbackEphem, this);
            
            _sub_glo_ephem = _nh.subscribe(_filter_params._gnss_glo_ephem_topic, 100, &IngvioFilter::callbackGloEphem, this);
            
            _sub_gnss_meas = _nh.subscribe(_filter_params._gnss_meas_topic, 100, &IngvioFilter::callbackGnssMeas, this);
            
            _sub_iono_params = _nh.subscribe(_filter_params._gnss_iono_params_topic, 100, &IngvioFilter::callbackIonoParams, this);
            
            _sub_rtk_gt = _nh.subscribe(_filter_params._rtk_gt_topic, 50, &IngvioFilter::callbackRtkGroundTruth, this);
            
            _odom_spp_pub = _nh.advertise<nav_msgs::Odometry>("pose_spp", 5);
            
            _path_spp_pub = _nh.advertise<nav_msgs::Path>("path_spp", 1);
      
            _odom_gt_pub = _nh.advertise<nav_msgs::Odometry>("pose_gt", 5);
            
            _path_gt_pub = _nh.advertise<nav_msgs::Path>("path_gt", 1);
        }
        
        _filter_params.printParams();
    }
    
    void IngvioFilter::callbackMonoFrame(const feature_tracker::MonoFrameConstPtr& mono_frame_ptr)
    {
        if (_filter_params._enable_gnss && !_gnss_sync->isSync())
            return;
        
        if (!_hasImageCome)
        {
            _hasImageCome = true;
            return;
        }
        
        if (!_hasInitState) return;
        
        const double target_time = mono_frame_ptr->header.stamp.toSec();
        
        if (_state->_timestamp >= target_time) return;
    
        TicToc timer_mono;
        
        _imu_propa->propagateAugmentAtEnd(_state, target_time);
        
        if (_state->_timestamp < target_time) return;
        
        MapServerManager::collectMonoMeas(_map_server, _state, mono_frame_ptr);
        
        _remove_lost_update->updateStateMono(_state, _map_server, _tri);
        
        if (_filter_params._is_key_frame)
        {
            _keyframe_update->updateStateMono(_state, _map_server, _tri);
            
            if (_filter_params._max_lm_feats > 0)
            {
                _landmark_update->updateLandmarkMono(_state, _map_server);
                
                _landmark_update->initNewLandmarkMono(_state, _map_server, 
                                                      _tri, _filter_params._max_sw_clones);
            }
            
            _keyframe_update->cleanMonoObsAtMargTime(_state, _map_server);
            
            _keyframe_update->changeMSCKFAnchor(_state, _map_server);
            
            if (_filter_params._max_lm_feats > 0)
            {
                std::vector<double> marg_kfs;
                _keyframe_update->getMargKfs(_state, marg_kfs);
                
                _landmark_update->changeLandmarkAnchor(_state, _map_server, marg_kfs);
            }
            
            _keyframe_update->margSwPose(_state);
        }
        else
        {
            _sw_marg_update->updateStateMono(_state, _map_server, _tri);
            
            if (_filter_params._max_lm_feats > 0)
            {
                _landmark_update->updateLandmarkMono(_state, _map_server);
                
                _landmark_update->initNewLandmarkMono(_state, _map_server, 
                                                      _tri, _filter_params._max_sw_clones);
            }
            
            _sw_marg_update->cleanMonoObsAtMargTime(_state, _map_server);
            
            _sw_marg_update->changeMSCKFAnchor(_state, _map_server);
            
            if (_filter_params._max_lm_feats > 0)
                _landmark_update->changeLandmarkAnchor(_state, _map_server);
            
            _sw_marg_update->margSwPose(_state);
        }

        MapServerManager::eraseInvalidFeatures(_map_server, _state);
        
        if (_filter_params._enable_gnss)
        {
            GnssMeas gnss_meas;
            SppMeas spp_meas;
            
            bool flag = false;
            
            if (_gnss_sync->getSppAt(mono_frame_ptr->header, spp_meas))
            {
                flag = true;
                visualizeSpp(mono_frame_ptr->header, spp_meas);
            }
            
            if (_gnss_sync->getGnssMeasAt(mono_frame_ptr->header, gnss_meas))
            {
                if (flag && !_gvio_aligner->isAlign())
                    _gvio_aligner->batchAlign(gnss_meas, _state->_extended_pose, _gnss_data->latest_gnss_iono_params);
                
                if (_gvio_aligner->isAlign())
                {
                    _gnss_update->checkYofStatus(_state, _gvio_aligner);
                    
                    // _gnss_update->removeUntrackedSys(_state, gnss_meas);
                    
                    _gnss_update->updateTrackedSys(_state, gnss_meas, _gvio_aligner,
                                                   _gnss_data->latest_gnss_iono_params);
                    
                    if (flag)
                        _gnss_update->addNewTrackedSys(_state, gnss_meas, spp_meas,
                                                       _gvio_aligner,
                                                       _gnss_data->latest_gnss_iono_params);
                }
            }
        }
        
        visualize(mono_frame_ptr->header);
        
        double time_mono = timer_mono.toc();
        
        static double total_time = 0.0;
        static int total_cnt = 0;
        
        ++total_cnt;
        total_time += time_mono;
        
        if (time_mono < 50.0)
            std::cout << color::setGreen << "[IngvioFilter]: One loop mono callback: " << total_time/total_cnt << " (ms) " << color::resetColor << std::endl;
        else
            std::cout << color::setRed << "[IngvioFilter]: One loop mono callback: " << total_time/total_cnt << " (ms) " << color::resetColor << std::endl;
    }
    
    void IngvioFilter::callbackStereoFrame(const feature_tracker::StereoFrameConstPtr& stereo_frame_ptr)
    {
        if (_filter_params._enable_gnss && !_gnss_sync->isSync())
            return;
        
        if (!_hasImageCome)
        {
            _hasImageCome = true;
            return;
        }
        
        if (!_hasInitState) return;
        
        const double target_time = stereo_frame_ptr->header.stamp.toSec();
        
        if (_state->_timestamp >= target_time) return;
        
        TicToc timer_stereo;
        
        _imu_propa->propagateAugmentAtEnd(_state, target_time);
        
        if (_state->_timestamp < target_time) return;
        
        MapServerManager::collectStereoMeas(_map_server, _state, stereo_frame_ptr);
        
        _remove_lost_update->updateStateStereo(_state, _map_server, _tri);
        
        if (_filter_params._is_key_frame)
        {
            _keyframe_update->updateStateStereo(_state, _map_server, _tri);
            
            if (_filter_params._max_lm_feats > 0)
            {
                _landmark_update->updateLandmarkStereo(_state, _map_server);
                
                _landmark_update->initNewLandmarkStereo(_state, _map_server, 
                                                      _tri, _filter_params._max_sw_clones);
            }
            
            _keyframe_update->cleanStereoObsAtMargTime(_state, _map_server);
            
            _keyframe_update->changeMSCKFAnchor(_state, _map_server);
            
            if (_filter_params._max_lm_feats > 0)
            {
                std::vector<double> marg_kfs;
                _keyframe_update->getMargKfs(_state, marg_kfs);
                
                _landmark_update->changeLandmarkAnchor(_state, _map_server, marg_kfs);
            }
            
            _keyframe_update->margSwPose(_state);
        }
        else
        {
            _sw_marg_update->updateStateStereo(_state, _map_server, _tri);
            
            if (_filter_params._max_lm_feats > 0)
            {
                _landmark_update->updateLandmarkStereo(_state, _map_server);
                
                _landmark_update->initNewLandmarkStereo(_state, _map_server, 
                                                        _tri, _filter_params._max_sw_clones);
            }
            
            _sw_marg_update->cleanStereoObsAtMargTime(_state, _map_server);
            
            _sw_marg_update->changeMSCKFAnchor(_state, _map_server);
            
            if (_filter_params._max_lm_feats > 0)
                _landmark_update->changeLandmarkAnchor(_state, _map_server);
            
            _sw_marg_update->margSwPose(_state);
        }
                
        MapServerManager::eraseInvalidFeatures(_map_server, _state);
        
        if (_filter_params._enable_gnss)
        {
            GnssMeas gnss_meas;
            SppMeas spp_meas;
            
            bool flag = false;
            
            if (_gnss_sync->getSppAt(stereo_frame_ptr->header, spp_meas))
            {
                flag = true;
                visualizeSpp(stereo_frame_ptr->header, spp_meas);
            }
            
            if (_gnss_sync->getGnssMeasAt(stereo_frame_ptr->header, gnss_meas))
            {
                if (flag && !_gvio_aligner->isAlign())
                    _gvio_aligner->batchAlign(gnss_meas, _state->_extended_pose, _gnss_data->latest_gnss_iono_params);
                
                if (_gvio_aligner->isAlign())
                {
                    _gnss_update->checkYofStatus(_state, _gvio_aligner);
                    
                    // _gnss_update->removeUntrackedSys(_state, gnss_meas);
                    
                    _gnss_update->updateTrackedSys(_state, gnss_meas, _gvio_aligner,
                                                   _gnss_data->latest_gnss_iono_params);
                    
                    if (flag)
                        _gnss_update->addNewTrackedSys(_state, gnss_meas, spp_meas,
                                                       _gvio_aligner,
                                                       _gnss_data->latest_gnss_iono_params);
                }
            }
        }
        
        visualize(stereo_frame_ptr->header);
        
        double time_stereo = timer_stereo.toc();
        
        static double total_time = 0.0;
        static int total_cnt = 0;
        
        ++total_cnt;
        total_time += time_stereo;
        
        if (time_stereo < 50.0)
            std::cout << color::setGreen << "[IngvioFilter]: One loop stereo callback: " << total_time/total_cnt << " (ms) " << color::resetColor << std::endl;
        else
            std::cout << color::setRed << "[IngvioFilter]: One loop stereo callback: " << total_time/total_cnt << " (ms) " << color::resetColor << std::endl;
        
    }
    
    void IngvioFilter::callbackIMU(sensor_msgs::Imu::ConstPtr imu_msg)
    {
        double aux_time = ros::Time::now().toSec();
        
        if (_filter_params._enable_gnss)
        {
            _gnss_sync->storeTimePair(aux_time, imu_msg->header.stamp.toSec());
            
            if (!_gnss_sync->isSync())
                return;
        }
        
        if (!_hasImageCome) return;
        
        _imu_propa->storeImu(imu_msg);
        
        if (_imu_propa->isInit() && !_hasInitState)
        {
            Eigen::Quaterniond init_quat;
            
            if (_imu_propa->getInitQuat(init_quat))
            {
                _state->initStateAndCov(imu_msg->header.stamp.toSec(), init_quat);
                _hasInitState = true;
            }
        }
    }
    
    void IngvioFilter::visualize(const std_msgs::Header& header)
    {
        ros::Time time = header.stamp;
        
        Eigen::Isometry3d T_i2w = Eigen::Isometry3d::Identity();
        T_i2w.linear() = _state->_extended_pose->valueLinearAsMat();
        T_i2w.translation() = _state->_extended_pose->valueTrans1();
        
        Eigen::Vector3d vel = _state->_extended_pose->valueTrans2();
        
        if (T_i2w.linear().hasNaN() || T_i2w.translation().hasNaN() || vel.hasNaN())
            return;
        
        tf::Transform T_i2w_tf;
        tf::transformEigenToTF(T_i2w, T_i2w_tf);
        _tf_pub.sendTransform(tf::StampedTransform(T_i2w_tf, time, "world", "uav"));
        
        nav_msgs::Odometry odom_w_msg;
        odom_w_msg.header.stamp = time;
        odom_w_msg.header.frame_id = "world";
        odom_w_msg.child_frame_id = "uav";
        
        tf::poseEigenToMsg(T_i2w, odom_w_msg.pose.pose);
        tf::vectorEigenToMsg(vel, odom_w_msg.twist.twist.linear);
        
        // ROS_INFO("Velocity vx = %f vy = %f vz = %f", vel(0), vel(1), vel(2));
        // ROS_INFO("Relative global position x = %f y = %f z = %f", odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z);
        
        _odom_w_pub.publish(odom_w_msg);
        
        geometry_msgs::PoseStamped pose_w;
        pose_w.header.stamp = time;
        pose_w.header.frame_id = "world";
        pose_w.pose.position.x = odom_w_msg.pose.pose.position.x;
        pose_w.pose.position.y = odom_w_msg.pose.pose.position.y;
        pose_w.pose.position.z = odom_w_msg.pose.pose.position.z;
        pose_w.pose.orientation.x = odom_w_msg.pose.pose.orientation.x;
        pose_w.pose.orientation.y = odom_w_msg.pose.pose.orientation.y;
        pose_w.pose.orientation.z = odom_w_msg.pose.pose.orientation.z;
        pose_w.pose.orientation.w = odom_w_msg.pose.pose.orientation.w;
        
        _path_w_msg.header.stamp = time;
        _path_w_msg.header.frame_id = "world";
        _path_w_msg.poses.push_back(pose_w);
        _path_w_pub.publish(_path_w_msg);
    }
    
    void IngvioFilter::visualizeSpp(const std_msgs::Header& header, const SppMeas& spp_meas)
    {
        if (!_filter_params._enable_gnss || !_gnss_sync->isSync() || !_gvio_aligner->isAlign())
            return;
        
        Eigen::Vector3d pos_spp_w = _gvio_aligner->getTecef2w()*spp_meas.posSpp.block<3, 1>(0, 0);
        
        if (pos_spp_w.hasNaN())
            return;
    
        geometry_msgs::PoseStamped pose_spp;
        pose_spp.header.stamp = header.stamp;
        pose_spp.header.frame_id = "world";
        pose_spp.pose.position.x = pos_spp_w.x();
        pose_spp.pose.position.y = pos_spp_w.y();
        pose_spp.pose.position.z = pos_spp_w.z();
        pose_spp.pose.orientation.x = 0.0;
        pose_spp.pose.orientation.y = 0.0;
        pose_spp.pose.orientation.z = 0.0;
        pose_spp.pose.orientation.w = 1.0;
        
        _path_spp_msg.header.stamp = pose_spp.header.stamp;
        _path_spp_msg.header.frame_id = "world";
        _path_spp_msg.poses.push_back(pose_spp);
        _path_spp_pub.publish(_path_spp_msg);
        
        Eigen::Isometry3d T_i2w = Eigen::Isometry3d::Identity();
        T_i2w.translation() = pos_spp_w;
        
        Eigen::Vector3d vel_spp_w = _gvio_aligner->getRecef2enu()*spp_meas.velSpp.block<3, 1>(0, 0);
        if (vel_spp_w.hasNaN())
            return;
        
        nav_msgs::Odometry odom_spp_msg;
        odom_spp_msg.header.stamp = header.stamp;
        odom_spp_msg.header.frame_id = "world";
        odom_spp_msg.child_frame_id = "spp";
        
        tf::poseEigenToMsg(T_i2w, odom_spp_msg.pose.pose);
        tf::vectorEigenToMsg(vel_spp_w, odom_spp_msg.twist.twist.linear);
        
        _odom_spp_pub.publish(odom_spp_msg);
    }
}
