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

#include "stereo_parameters.h"
#include <opencv2/core/eigen.hpp>

namespace feature_tracker
{
    StereoParameters::StereoParameters() : row(480), col(752), image_left_topic("/cam0/image_raw"), image_right_topic("/cam1/image_raw"), cam_left_path("~"), cam_right_path("~"), cam_left_name("camera_left"), cam_right_name("camera_right"), max_cnt(150), min_dist(30), window_size(20), freq(0), show_track(1), equalize(1), epipolar_thres(25.0), F_threshold(1.5), print_track_info(0),  T_cl2i(Eigen::Isometry3d::Identity()), T_cr2i(Eigen::Isometry3d::Identity())
    {}
    
    void StereoParameters::readParameters(ros::NodeHandle& n)
    {
        std::string config_file;
        config_file = readParam<std::string>(n, "config_file_left");
        cam_left_path = config_file;
        config_file = readParam<std::string>(n, "config_file_right");
        cam_right_path = config_file;
        
        cv::FileStorage fs_left(cam_left_path, cv::FileStorage::READ);
        if (!fs_left.isOpened())
        {
            ROS_ERROR("ERROR: Wrong path to stereo left config yaml files!");
            n.shutdown();
        }
        
        row = fs_left["image_height"];
        col = fs_left["image_width"];
        fs_left["cam_left_topic"] >> image_left_topic;
        fs_left["cam_right_topic"] >> image_right_topic;
        fs_left["camera_name"] >> cam_left_name;
        max_cnt = fs_left["max_cnt"];
        min_dist = fs_left["min_dist"];
        window_size = fs_left["window_size"];
        freq = fs_left["freq"];
        
        if (freq == 0) freq = 100;
        
        show_track = fs_left["show_track"];
        
        show_timer = fs_left["show_timer"];
        timer_warning_thres = fs_left["timer_warning_thres"];
        
        equalize = fs_left["equalize"];
        
        epipolar_thres = static_cast<double>(fs_left["epipolar_thres"]);
        F_threshold = static_cast<double>(fs_left["F_threshold"]);
        
        print_track_info = fs_left["print_track_info"];
        
        cv::Mat cv_R1, cv_T1;
        fs_left["extrinsicRotation"] >> cv_R1;
        fs_left["extrinsicTranslation"] >> cv_T1;
        Eigen::Matrix3d eigen_R1;
        Eigen::Vector3d eigen_T1;
        cv::cv2eigen(cv_R1, eigen_R1);
        cv::cv2eigen(cv_T1, eigen_T1);
        T_cl2i.linear() = eigen_R1;
        T_cl2i.translation() = eigen_T1;
        
        fs_left.release();
        
        cv::FileStorage fs_right(cam_right_path, cv::FileStorage::READ);
        if (!fs_right.isOpened())
        {
            ROS_ERROR("ERROR: Wrong path to stereo right config yaml files!");
            n.shutdown();            
        }
        
        fs_right["camera_name"] >> cam_right_name;
        
        cv::Mat cv_R2, cv_T2;
        fs_right["extrinsicRotation"] >> cv_R2;
        fs_right["extrinsicTranslation"] >> cv_T2;
        Eigen::Matrix3d eigen_R2;
        Eigen::Vector3d eigen_T2;
        cv::cv2eigen(cv_R2, eigen_R2);
        cv::cv2eigen(cv_T2, eigen_T2);
        T_cr2i.linear() = eigen_R2;
        T_cr2i.translation() = eigen_T2;
        
        fs_right.release();
    }
    
}
