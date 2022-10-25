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
 */

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

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

#include "random_pair.hpp"
#include "stereo_parameters.h"

#include "feature_tracker/StereoMeas.h"
#include "feature_tracker/StereoFrame.h"

namespace feature_tracker
{
    class StereoTracker
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        StereoTracker(ros::NodeHandle& n) : nh(n) {}
        ~StereoTracker() {}
        StereoTracker(const StereoTracker&) = delete;
        StereoTracker operator=(const StereoTracker&) = delete;
        
        void init();
        
    private:
        ros::NodeHandle nh;
        ros::Publisher pub_stereo_feature, pub_match;
        
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> stereoSyncPolicy;
        boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> cam_left_sub_;
        boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> cam_right_sub_;
        boost::shared_ptr<message_filters::Synchronizer<stereoSyncPolicy>> stereo_sub_;
        
        StereoParameters param;
        camera_model::CameraPtr m_camera_left, m_camera_right;
        
        cv::Mat mask;
        bool first_image_flag;
        double first_image_time, last_image_time;
        bool pub_this_frame;
        unsigned long long int pub_count;
        
        double cur_time, prev_time;
        cv::Mat prev_left_img, cur_left_img, forw_left_img;
        cv::Mat prev_right_img, cur_right_img, forw_right_img;
        
        std::vector<cv::Point2f> prev_left_pts, cur_left_pts, forw_left_pts;
        std::vector<cv::Point2f> prev_right_pts, cur_right_pts, forw_right_pts;
        
        std::vector<cv::Point2f> prev_left_un_pts, cur_left_un_pts;
        std::vector<cv::Point2f> prev_right_un_pts, cur_right_un_pts;
        
        std::vector<unsigned long long int> ids;
        std::vector<unsigned int> track_cnt;
        
        std::vector<cv::Point2f> n_left_pts, n_right_pts;
        
        static unsigned long long int n_id;
        
        
        void stereo_callback(const sensor_msgs::Image::ConstPtr& cam_left_msg, const sensor_msgs::Image::ConstPtr& cam_right_msg);
        
        void readIntrinsicParameters();
        
        void processStereoImage(const cv::Mat& _left_img, const cv::Mat& _right_img, double _cur_time);
        
        bool inBorder(const cv::Point2f& pt);
        
        void markStereoMatchOutlier(const std::vector<cv::Point2f>& left_pts, const std::vector<cv::Point2f>& right_pts, std::vector<uchar>& status);
        
        void rejectWithTwinRANSAC();
        
        void setMask();
        
        void addPoints();
        
        void undistortedPoints();
        
        void updateID();
        
        static void publishStereoImg(const cv::Mat& left_img, const cv::Mat& right_img, const std::vector<cv::Point2f>& left_pts, const std::vector<cv::Point2f>& right_pts, const int row, const int col, bool pair_line = true);
        
        void predictHomography(const std::vector<cv::Point2f>& left_pts, std::vector<cv::Point2f>& right_pts);
        
        template <typename T1, typename T2 = uchar>
        void reduceVector(std::vector<T1>& v, std::vector<T2> status);
        
        template <typename T1, typename T2 = uchar>
        void reduceTwinVector(std::vector<T1>& v1, std::vector<T1>& v2, std::vector<T2> status);
        
        template <typename T1, typename T2 = uchar>
        void reduceQuadVector(std::vector<T1>& v1, std::vector<T1>& v2, std::vector<T1>& v3, std::vector<T1>& v4, std::vector<T2> status);
    };
    
    typedef boost::shared_ptr<StereoTracker> StereoTrackerPtr;
    
    template <typename T1, typename T2>
    void StereoTracker::reduceVector(std::vector<T1>& v, std::vector<T2> status)
    {
        int j = 0;
        for (int i = 0; i < int(v.size()); ++i)
            if (status[i])
                v[j++] = v[i];
            
        v.resize(j);
    }    
    
    template <typename T1, typename T2>
    void StereoTracker::reduceTwinVector(std::vector<T1>& v1, std::vector<T1>& v2, std::vector<T2> status)
    {
        if (v1.size() != v2.size())
            ROS_WARN("[Stereo Tracker]: Reduce twin vectors, size: %i and %i are not equal!", int(v1.size()), int(v2.size()));
        
        int j = 0;
        for (int i = 0; i < int(v1.size()); ++i)
            if (status[i])
            {
                v1[j] = v1[i];
                v2[j] = v2[i];
                ++j;
            }
            
        v1.resize(j);
        v2.resize(j);        
    }
    
    template <typename T1, typename T2>
    void StereoTracker::reduceQuadVector(std::vector<T1>& v1, std::vector<T1>& v2, std::vector<T1>& v3, std::vector<T1>& v4, std::vector<T2> status)
    {
        if (!(v1.size() == v2.size() && v2.size() == v3.size() && v3.size() == v4.size()))
            ROS_WARN("[Stereo Tracker]: Reduce quad vectors, size: %i, %i, %i and %i  are not equal!", int(v1.size()), int(v2.size()), int(v3.size()), int(v4.size())); 
        
        int j = 0;
        for (int i = 0; i < int(v1.size()); ++i)
            if (status[i])
            {
                v1[j] = v1[i];
                v2[j] = v2[i];
                v3[j] = v3[i];
                v4[j] = v4[i];
                ++j;
            }
            
        v1.resize(j);
        v2.resize(j);         
        v3.resize(j);
        v4.resize(j);
    }
}
