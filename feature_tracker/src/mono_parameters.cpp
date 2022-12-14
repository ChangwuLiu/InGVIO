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
 
#include "mono_parameters.h"

namespace feature_tracker
{
    MonoParameters::MonoParameters() : row(480), col(752), focal_length(460), image_topic("/cam1/image_raw"), imu_topic("/imu0"), cam_name("camera"), cam_path("~"), max_cnt(150), min_dist(30), window_size(20), freq(0), f_threshold(1.0), show_track(1), equalize(1)
    {}
    
    void MonoParameters::readParameters(ros::NodeHandle& n)
    {
        std::string config_file;
        config_file = readParam<std::string>(n, "config_file");
        
        cv::FileStorage fs(config_file, cv::FileStorage::READ);
        if (!fs.isOpened())
        {
            ROS_ERROR("ERROR: Wrong path to mono config yaml files!");
            n.shutdown();
        }
        
        cam_path = config_file;
        
        row = fs["image_height"];
        col = fs["image_width"];
        focal_length = 460;
        fs["image_topic"] >> image_topic;
        fs["imu_topic"] >> imu_topic;
        fs["camera_name"] >> cam_name;
        max_cnt = fs["max_cnt"];
        min_dist = fs["min_dist"];
        
        window_size = 20;
        
        freq = fs["freq"];
        if (freq == 0) freq = 100;
        
        f_threshold = fs["F_threshold"];
        show_track = fs["show_track"];
        show_timer = fs["show_timer"];
        timer_warning_thres = fs["timer_warning_thres"];
        equalize = fs["equalize"];
        
        fs.release();
    }
}
