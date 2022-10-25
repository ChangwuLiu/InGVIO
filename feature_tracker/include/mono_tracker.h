/**   This File is part of Feature Tracker
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
 *    
 *    A few functions of the mono tracker mimic the realization from GVINS
 *    <https://github.com/HKUST-Aerial-Robotics/GVINS>
 *    GVINS is under GPLv3 License.
 */
 

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

#include <iostream>
#include <string>
#include <vector>
#include <map>

#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "tic_toc.hpp"

#include "camera_factory.h"
#include "MeiCamera.h"
#include "PinholeCamera.h"

#include "mono_parameters.h"

#include <feature_tracker/MonoMeas.h>
#include <feature_tracker/MonoFrame.h>

namespace feature_tracker
{
    class MonoTracker
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        MonoTracker(ros::NodeHandle& n) : nh(n) {}
        ~MonoTracker() {}
        MonoTracker(const MonoTracker&) = delete;
        MonoTracker operator=(const MonoTracker&) = delete;
        
        void init();
        
    private:
        ros::NodeHandle nh;
        ros::Publisher pub_mono_features, pub_match;
        ros::Subscriber sub_img;
        void img_callback(const sensor_msgs::ImageConstPtr& img_msg);
        
        MonoParameters param;
        camera_model::CameraPtr m_camera;        
        
        bool pub_this_frame;
        bool first_image_flag;
        double first_image_time, last_image_time;
        int pub_count;
        
        static int n_id;
        
        double cur_time, prev_time;
        cv::Mat mask;
        cv::Mat prev_img, cur_img, forw_img;
        
        std::vector<cv::Point2f> prev_pts, cur_pts, forw_pts;
        std::vector<cv::Point2f> prev_un_pts, cur_un_pts;
        std::vector<cv::Point2f> pts_velocity;
        std::vector<cv::Point2f> n_pts;
        
        std::vector<int> ids;
        std::vector<int> track_cnt;
        
        std::map<int, cv::Point2f> cur_un_pts_map;
        std::map<int, cv::Point2f> prev_un_pts_map;
        
        
        void readImage(const cv::Mat& _img, double _cur_time);
        
        void rejectWithF();
        
        void setMask();
        
        void addPoints();
        
        void undistortedPoints();
        
        void updateID();
        
        
        bool inBorder(const cv::Point2f& pt);
        
        template <typename T1, typename T2 = uchar>
        void reduceVector(std::vector<T1>& v, std::vector<T2> status);
        
        void readIntrinsicParameter(const std::string& calib_file);
    };
    
    typedef boost::shared_ptr<MonoTracker> MonoTrackerPtr; 
    
    template <typename T1, typename T2>
    void MonoTracker::reduceVector(std::vector<T1>& v, std::vector<T2> status)
    {
        int j = 0;
        for (int i = 0; i < int(v.size()); ++i)
            if (status[i])
                v[j++] = v[i];
            
        v.resize(j);
    }
}
