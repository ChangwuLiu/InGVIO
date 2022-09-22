#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>

#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>

namespace ingvio
{
    class IngvioParams
    {
    public:
        IngvioParams() : _T_cl2i(Eigen::Isometry3d::Identity()), _T_cr2i(Eigen::Isometry3d::Identity()) {}
        
        IngvioParams(const IngvioParams&) = delete;
        
        IngvioParams operator=(const IngvioParams&) = delete;
        
        ~IngvioParams() = default;
        
        std::string _config_path;
        
        int _cam_nums;
        std::string _cam_left_config_path;
        std::string _cam_right_config_path;
        
        Eigen::Isometry3d _T_cl2i, _T_cr2i;
        
        std::string _feature_topic;
        std::string _imu_topic;
        
        int _max_sw_clones;
        int _max_lm_feats;
        int _enable_gnss;
        
        double _noise_g;
        double _noise_a;
        double _noise_bg;
        double _noise_ba;
        double _noise_clockbias;
        double _noise_cb_rw;
        
        double _init_cov_rot;
        double _init_cov_pos;
        double _init_cov_vel;
        double _init_cov_bg;
        double _init_cov_ba;
        double _init_cov_ext_rot;
        double _init_cov_ext_pos;
        double _init_cov_rcv_clockbias;
        double _init_cov_rcv_clockbias_randomwalk;
        double _init_cov_yof;
        
        double _init_gravity;
        int _max_imu_buffer_size;
        int _init_imu_buffer_sp;
        
        double _trans_thres;
        double _huber_epsilon;
        double _conv_precision;
        double _init_damping;
        int _outer_loop_max_iter;
        int _inner_loop_max_iter;
        double _max_depth;
        double _min_depth;
        double _max_baseline_ratio;
        
        int _chi2_max_dof;
        double _chi2_thres;
        double _visual_noise;
        
        int _frame_select_interval;
        
        void readParams(ros::NodeHandle& nh);
        void readParams(const std::string& config_path);
        
        void printParams();
        
        template <typename T>
        static T readParams(ros::NodeHandle& n, std::string name);
    };
    
    template <typename T>
    T IngvioParams::readParams(ros::NodeHandle& n, std::string name)
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
