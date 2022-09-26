#pragma once

#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>

namespace feature_tracker
{
    class MonoParameters
    {
    public:
        MonoParameters();
        
        int row, col;
        int focal_length;
        
        std::string image_topic;
        std::string imu_topic;
        std::string cam_name;
        std::string cam_path;
        
        int max_cnt;
        int min_dist;
        
        int window_size;
        int freq;
        
        double f_threshold;
        
        int show_track;
        int equalize;
        double timer_warning_thres;
        int show_timer;
        
        void readParameters(ros::NodeHandle& n);
        
        template <typename T>
        static T readParam(ros::NodeHandle& n, std::string name);
    };
    
    template <typename T>
    T MonoParameters::readParam(ros::NodeHandle& n, std::string name)
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

