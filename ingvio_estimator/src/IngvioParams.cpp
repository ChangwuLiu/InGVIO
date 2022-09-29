#include "IngvioParams.h"

#include <opencv2/core/eigen.hpp>

namespace ingvio
{
    void IngvioParams::readParams(ros::NodeHandle& nh)
    {
        _config_path = readParams<std::string>(nh, "config_file");
        this->readParams(_config_path);
    }
    
    void IngvioParams::readParams(const std::string& config_path)
    {
        cv::FileStorage fs(config_path, cv::FileStorage::READ);
        
        if (!fs.isOpened())
            ROS_ERROR("[IngvioParams]: Wrong path to config file!");
        
        _cam_nums = fs["cam_nums"];
        if (_cam_nums >= 3)
        {
            ROS_WARN("[IngvioParams]: Camera number not available! Switch to mono!");
            _cam_nums = 1;
        }
        
        fs["cam_left_file_path"] >> _cam_left_config_path;
        
        if (_cam_nums == 2)
            fs["cam_right_file_path"] >> _cam_right_config_path;
        
        fs["feature_topic"] >> _feature_topic;
        fs["imu_topic"] >> _imu_topic;
        
        _max_sw_clones = fs["max_sliding_window_poses"];
        
        _max_lm_feats = fs["max_landmark_features"];
        
        _enable_gnss = fs["enable_gnss"];
        
        _noise_g = fs["noise_gyro"];
        _noise_a = fs["noise_accel"];
        _noise_bg = fs["noise_bias_gyro"];
        _noise_ba = fs["noise_bias_accel"];
        
        _init_cov_rot = fs["init_cov_rot"];
        _init_cov_pos = fs["init_cov_pos"];
        _init_cov_vel = fs["init_cov_vel"];
        _init_cov_bg = fs["init_cov_bg"];
        _init_cov_ba = fs["init_cov_ba"];
        _init_cov_ext_rot = fs["init_cov_ext_rot"];
        _init_cov_ext_pos = fs["init_cov_ext_pos"];
    
        _init_gravity = fs["gravity_norm"];
        _max_imu_buffer_size = fs["max_imu_buffer_size"];
        _init_imu_buffer_sp = fs["init_imu_buffer_sp"];
        
        _trans_thres = fs["trans_thres"];
        _huber_epsilon = fs["huber_epsilon"];
        _conv_precision = fs["conv_precision"];
        _init_damping = fs["init_damping"];
        _outer_loop_max_iter = fs["outer_loop_max_iter"];
        _inner_loop_max_iter = fs["inner_loop_max_iter"];
        _max_depth = fs["max_depth"];
        _min_depth = fs["min_depth"];
        _max_baseline_ratio = fs["max_baseline_ratio"];
        
        _chi2_max_dof = fs["chi2_max_dof"];
        _chi2_thres = fs["chi2_thres"];
        _visual_noise = fs["visual_noise"];
        _frame_select_interval = fs["frame_select_interval"];
        
        if (_enable_gnss)
        {
            _noise_clockbias = fs["noise_rcv_clockbias"];
            _noise_cb_rw = fs["noise_rcv_clockbias_randomwalk"];
            
            _init_cov_rcv_clockbias = fs["init_cov_rcv_clockbias"];
            _init_cov_rcv_clockbias_randomwalk = fs["init_cov_rcv_clockbias_randomwalk"];
            _init_cov_yof = fs["init_cov_yof"];
            
            fs["gnss_ephem_topic"] >> _gnss_ephem_topic;
            fs["gnss_glo_ephem_topic"] >> _gnss_glo_ephem_topic;
            fs["gnss_meas_topic"] >> _gnss_meas_topic;
            fs["gnss_iono_params_topic"] >> _gnss_iono_params_topic;
            fs["rtk_gt_topic"] >> _rtk_gt_topic;
            
            _gnss_elevation_thres = fs["gnss_elevation_thres"];
            _gnss_psr_std_thres = fs["gnss_psr_std_thres"];
            _gnss_dopp_std_thres = fs["gnss_dopp_std_thres"];
            _gnss_track_num_thres = fs["gnss_track_num_thres"];
            
            _use_fix_time_offset = fs["use_fix_time_offset"];
            
            if (_use_fix_time_offset)
                _gnss_local_offset = fs["gnss_local_offset"];
        }
        
        
        fs.release();
        
        cv::FileStorage fs_cam_left(_cam_left_config_path, cv::FileStorage::READ);
        if (!fs_cam_left.isOpened())
            ROS_ERROR("[IngvioParams]: Wrong path to left cam config file!");
        
        cv::Mat cv_R1, cv_T1;
        
        fs_cam_left["extrinsicRotation"] >> cv_R1;
        fs_cam_left["extrinsicTranslation"] >> cv_T1;
        Eigen::Matrix3d eigen_R1;
        Eigen::Vector3d eigen_T1;
        cv::cv2eigen(cv_R1, eigen_R1);
        cv::cv2eigen(cv_T1, eigen_T1);
        
        _T_cl2i.linear() = eigen_R1;
        _T_cl2i.translation() = eigen_T1;
        
        fs_cam_left.release();
        
        if (_cam_nums == 2)
        {
            cv::FileStorage fs_cam_right(_cam_right_config_path, cv::FileStorage::READ);
            
            if (!fs_cam_right.isOpened())
                ROS_ERROR("[IngvioParams]: Wrong path to right cam config file!");
            
            cv::Mat cv_R2, cv_T2;
            
            fs_cam_right["extrinsicRotation"] >> cv_R2;
            fs_cam_right["extrinsicTranslation"] >> cv_T2;
            Eigen::Matrix3d eigen_R2;
            Eigen::Vector3d eigen_T2;
            cv::cv2eigen(cv_R2, eigen_R2);
            cv::cv2eigen(cv_T2, eigen_T2);
            
            _T_cr2i.linear() = eigen_R2;
            _T_cr2i.translation() = eigen_T2;
            
            fs_cam_right.release();
        }
        
    }
    
    void IngvioParams::printParams()
    {
        std::cout << "===== Ingvio Parameter List =====" << std::endl;
        
        std::cout << "Number of CAMS: " << _cam_nums << std::endl;
        
        std::cout << "InEKF_config_path: " << _config_path << std::endl;
        
        std::cout << "cam_left_file_path: " << _cam_left_config_path << std::endl;
        
        if (_cam_nums == 2)
            std::cout << "cam_right_file_path: " << _cam_right_config_path << std::endl;
        
        std::cout << "feature_topic: " << _feature_topic << std::endl;
        
        std::cout << "imu_topic: " << _imu_topic << std::endl;
        
        std::cout << "max_sliding_window_poses: " << _max_sw_clones << std::endl;
        
        std::cout << "max_landmark_features: " << _max_lm_feats << std::endl;
        
        std::cout << "enable_gnss: " << _enable_gnss << std::endl;
        
        std::cout << "noise_gyro: " << _noise_g << std::endl;
        
        std::cout << "noise_accel: " << _noise_a << std::endl;
        
        std::cout << "noise_bias_gyro: " << _noise_bg << std::endl;
        
        std::cout << "noise_bias_accel: " << _noise_ba << std::endl;
        
        if (_enable_gnss)
        {
            std::cout << "noise_rcv_clockbias: " << _noise_clockbias << std::endl;
            std::cout << "noise_rcv_clockbias_randomwalk: " << _noise_cb_rw << std::endl;
        }
        
        std::cout << "init_cov_rot: " << _init_cov_rot << std::endl;
        
        std::cout << "init_cov_pos: " << _init_cov_pos << std::endl;
        
        std::cout << "init_cov_vel: " << _init_cov_vel << std::endl;
        
        std::cout << "init_cov_bg: " << _init_cov_bg << std::endl;
        
        std::cout << "init_cov_ba: " << _init_cov_ba << std::endl;
        
        std::cout << "init_cov_ext_rot: " << _init_cov_ext_rot << std::endl;
        
        std::cout << "init_cov_ext_pos: " << _init_cov_ext_pos << std::endl;
        
        std::cout << "gravity_norm: " << _init_gravity << std::endl;

        std::cout << "max_imu_buffer_size: " << _max_imu_buffer_size << std::endl;
        
        std::cout << "init_imu_buffer_sp: " << _init_imu_buffer_sp << std::endl;

        std::cout << "trans_thres: " << _trans_thres << std::endl;
        
        std::cout << "huber_epsilon: " << _huber_epsilon << std::endl;
        
        std::cout << "conv_precision: " << _conv_precision << std::endl;
        
        std::cout << "init_damping: " << _init_damping << std::endl;
        
        std::cout << "outer_loop_max_iter: " << _outer_loop_max_iter << std::endl;
        
        std::cout << "inner_loop_max_iter: " << _inner_loop_max_iter << std::endl;
        
        std::cout << "max depth: " << _max_depth << std::endl;
        
        std::cout << "min_depth: " << _min_depth << std::endl;
        
        std::cout << "max_baseline_ratio: " << _max_baseline_ratio << std::endl;

        std::cout << "chi2_max_dof: " << _chi2_max_dof << std::endl;
         
        std::cout << "chi2_thres: " << _chi2_thres << std::endl;
        
        std::cout << "visual_noise: " << _visual_noise << std::endl;
        
        std::cout << "frame_select_interval: " <<  _frame_select_interval << std::endl;
        
        if (_enable_gnss)
        {
            std::cout << "init_cov_rcv_clockbias: " << _init_cov_rcv_clockbias << std::endl;
            
            std::cout << "init_cov_rcv_clockbias_randomwalk: " << _init_cov_rcv_clockbias_randomwalk << std::endl;
        
            std::cout << "init_cov_yof: " << _init_cov_yof << std::endl;
            
            std::cout << "gnss_ephem_topic: " << _gnss_ephem_topic << std::endl;
            
            std::cout << "gnss_glo_ephem_topic: " << _gnss_glo_ephem_topic << std::endl;
            
            std::cout << "gnss_meas_topic: " << _gnss_meas_topic << std::endl;
            
            std::cout << "gnss_iono_params_topic: " << _gnss_iono_params_topic << std::endl;
            
            std::cout << "rtk_gt_topic: " << _rtk_gt_topic << std::endl; 
            
            std::cout << "gnss_elevation_thres: " << _gnss_elevation_thres << std::endl;
            
            std::cout << "gnss_psr_std_thres: " << _gnss_psr_std_thres << std::endl;
            
            std::cout << "gnss_dopp_std_thres: " << _gnss_dopp_std_thres << std::endl;
            
            std::cout << "gnss_track_num_thres: " << _gnss_track_num_thres << std::endl;
            
            std::cout << "use_fix_time_offset: " << _use_fix_time_offset << std::endl;
            
            if (_use_fix_time_offset)
                std::cout << "gnss_local_offset: " << _gnss_local_offset << std::endl;
        }
        
        std::cout << "===============================" << std::endl;
        
    }
}
