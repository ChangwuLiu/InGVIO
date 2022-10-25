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

#include <gnss_comm/gnss_spp.hpp>

#include "IngvioFilter.h"

#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>

#include "GnssData.h"
#include "GnssSync.h"
#include "GvioAligner.h"

namespace ingvio
{
    void IngvioFilter::callbackRtkGroundTruth(
        const sensor_msgs::NavSatFix::ConstPtr& nav_sat_msg)
    {
        if (!_filter_params._enable_gnss || !_gnss_sync->isSync()) return;
        
        
        if (_gvio_aligner->isAlign() &&
            nav_sat_msg->status.status >= sensor_msgs::NavSatStatus::STATUS_FIX)
        {
            Eigen::Vector3d gt_geo, gt_ecef;
            gt_geo.x() = nav_sat_msg->latitude;
            gt_geo.y() = nav_sat_msg->longitude;
            gt_geo.z() = nav_sat_msg->altitude;
            
            gt_ecef = gnss_comm::geo2ecef(gt_geo);
            
            Eigen::Vector3d gt_w = _gvio_aligner->getTecef2w()*gt_ecef;
            
            if (gt_w.hasNaN())
                return;
            
            geometry_msgs::PoseStamped pose_gt;
            pose_gt.header.stamp = ros::Time().fromSec(nav_sat_msg->header.stamp.toSec() + _gnss_sync->getUnsyncTime());
            pose_gt.header.frame_id = "world";
            pose_gt.pose.position.x = gt_w.x();
            pose_gt.pose.position.y = gt_w.y();
            pose_gt.pose.position.z = gt_w.z();
            pose_gt.pose.orientation.x = 0.0;
            pose_gt.pose.orientation.y = 0.0;
            pose_gt.pose.orientation.z = 0.0;
            pose_gt.pose.orientation.w = 0.0;
            
            _path_gt_msg.header.stamp = pose_gt.header.stamp;
            _path_gt_msg.header.frame_id = "world";
            _path_gt_msg.poses.push_back(pose_gt);
            _path_gt_pub.publish(_path_gt_msg);
            
            Eigen::Isometry3d T_i2w = Eigen::Isometry3d::Identity();
            T_i2w.translation() = gt_w;
            
            nav_msgs::Odometry odom_gt_msg;
            odom_gt_msg.header.stamp = pose_gt.header.stamp;
            odom_gt_msg.header.frame_id = "world";
            odom_gt_msg.child_frame_id = "gt";
            
            tf::poseEigenToMsg(T_i2w, odom_gt_msg.pose.pose);
            tf::vectorEigenToMsg(Eigen::Vector3d::Zero(), odom_gt_msg.twist.twist.linear);
            
            _odom_gt_pub.publish(odom_gt_msg);
        }
    }
    
    void IngvioFilter::callbackEphem(const gnss_comm::GnssEphemMsgConstPtr& ephem_msg)
    {
        if (!_filter_params._enable_gnss) return;
        
        gnss_comm::EphemPtr ephem = gnss_comm::msg2ephem(ephem_msg);
        
        _gnss_data->inputEphem(ephem);
    }
    
    void IngvioFilter::callbackGloEphem(const gnss_comm::GnssGloEphemMsgConstPtr& glo_ephem_msg)
    {
        if (!_filter_params._enable_gnss) return;
        
        gnss_comm::GloEphemPtr glo_ephem = gnss_comm::msg2glo_ephem(glo_ephem_msg);
        
        _gnss_data->inputEphem(glo_ephem);
    }
    
    void IngvioFilter::callbackIonoParams(const gnss_comm::StampedFloat64ArrayConstPtr& iono_msg)
    {
        if (!_filter_params._enable_gnss) return;
        
        std::vector<double> iono_params;
        std::copy(iono_msg->data.begin(), iono_msg->data.end(), std::back_inserter(iono_params));
        
        if (iono_params.size() != 8) return;
        
        _gnss_data->latest_gnss_iono_params.clear();
        _gnss_data->latest_gnss_iono_params.assign(iono_params.begin(), iono_params.end());
        
        if (!_gnss_data->_isIono) _gnss_data->_isIono = true;
    }
    
    void IngvioFilter::callbackGnssMeas(const gnss_comm::GnssMeasMsgConstPtr& gnss_meas_msg)
    {
        double aux_time = ros::Time::now().toSec();
        
        if (!_filter_params._enable_gnss || !_gnss_data->_isIono)
            return;
        
        std::vector<gnss_comm::ObsPtr> gnss_meas = gnss_comm::msg2meas(gnss_meas_msg);
        
        if (gnss_meas.size() <= 0) return;
        
        _gnss_sync->storeTimePair(aux_time, gnss_meas[0]->time);
        
        if (!_gnss_sync->isSync()) return;
        
        std::vector<gnss_comm::ObsPtr> valid_meas;
        std::vector<gnss_comm::EphemBasePtr> valid_ephems;
        
        for (auto obs : gnss_meas)
        {
            uint32_t sys = gnss_comm::satsys(obs->sat, NULL);
            
            if (sys != SYS_GPS && sys != SYS_GLO && sys != SYS_GAL && sys != SYS_BDS)
                continue;
            
            if (_gnss_data->sat2ephem.count(obs->sat) == 0)
                continue;
            
            if (obs->freqs.empty()) continue;
            
            int freq_idx = -1;
            gnss_comm::L1_freq(obs, &freq_idx);
            
            if (freq_idx < 0) continue;
            
            double obs_time = gnss_comm::time2sec(obs->time);
            
            std::map<double, size_t> time2index = _gnss_data->sat2time_index.at(obs->sat);
            
            double ephem_time = EPH_VALID_SECONDS;
            size_t ephem_index = -1;
            
            for (auto ti : time2index)
            {
                if (std::fabs(ti.first - obs_time) < ephem_time)
                {
                    ephem_time = std::fabs(ti.first - obs_time);
                    ephem_index = ti.second;
                }
            }
            
            if (ephem_time >= EPH_VALID_SECONDS || ephem_index == -1)
                continue;
            
            const gnss_comm::EphemBasePtr& best_ephem = _gnss_data->sat2ephem.at(obs->sat).at(ephem_index);
            
            if (obs->psr_std[freq_idx] > _filter_params._gnss_psr_std_thres || 
                obs->psr_std[freq_idx] > _filter_params._gnss_dopp_std_thres)
            {
                _gnss_data->sat_track_status[obs->sat] = 0;
                continue;
            }
            else
            {
                if (_gnss_data->sat_track_status.count(obs->sat) == 0)
                    _gnss_data->sat_track_status[obs->sat] = 0;
                
                ++_gnss_data->sat_track_status.at(obs->sat);
            }
            
            if (_gnss_data->sat_track_status[obs->sat] < _filter_params._gnss_track_num_thres)
                continue;
            
            valid_meas.push_back(obs);
            valid_ephems.push_back(best_ephem);
        }
        
        if (valid_meas.size() <= 0) 
            return;
        else
            _gnss_sync->bufferGnssMeas(valid_meas, valid_ephems);
        
        if (valid_meas.size() >= 4)
        {
            Eigen::Matrix<double, 7, 1> pos_raw_spp = gnss_comm::psr_pos(valid_meas,
                valid_ephems, _gnss_data->latest_gnss_iono_params);
            
            if (pos_raw_spp.hasNaN() ||
                pos_raw_spp.block<3, 1>(0, 0).norm() < 1e-03)
                return;
            
            Eigen::Vector4d vel_raw_spp;
            Eigen::Vector3d pos_ecef = pos_raw_spp.block<3, 1>(0, 0);
            
            vel_raw_spp = gnss_comm::dopp_vel(valid_meas, valid_ephems, pos_ecef);
            
            if (vel_raw_spp.hasNaN())
                return;
            
            _gnss_sync->bufferSppMeas(valid_meas[0]->time, pos_raw_spp, vel_raw_spp);
        }
    }
    
}
