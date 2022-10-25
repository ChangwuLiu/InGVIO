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

