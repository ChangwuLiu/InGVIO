#pragma once

#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace feature_tracker
{
    class StereoParameters
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        StereoParameters();
        
        int row, col;
        
        std::string image_left_topic;
        std::string image_right_topic;
        
        std::string cam_left_path;
        std::string cam_right_path;
        
        std::string cam_left_name;
        std::string cam_right_name;
        
        int max_cnt;
        int min_dist;
        
        int window_size;
        int freq;
        
        int show_track;
        int equalize;
        
        double epipolar_thres;
        double F_threshold;
        
        int print_track_info;
        
        Eigen::Isometry3d T_cl2i, T_cr2i;
        
        void readParameters(ros::NodeHandle& n);
        
        template <typename T>
        static T readParam(ros::NodeHandle& n, std::string name);
    };
    
    template <typename T>
    T StereoParameters::readParam(ros::NodeHandle& n, std::string name)
    {
        T ans;
        if (n.getParam(name, ans))
        {
            ROS_INFO_STREAM("Loaded " << name << ": " << ans);    
        }
        else
        {
            ROS_ERROR_STREAM("Failed to load: " << name);
            n.shutdown();
        }
        
        return ans;
    }
    
}
