#include "State.h"

namespace ingvio
{
    StateParams::StateParams(const IngvioParams& filter_params)
    {
        _cam_nums = filter_params._cam_nums;
        _max_sw_poses = filter_params._max_sw_clones;
        _max_landmarks = filter_params._max_lm_feats;
        
        _T_cl2cr = filter_params._T_cl2i*filter_params._T_cr2i.inverse();
        
        _enable_gnss = static_cast<bool>(filter_params._enable_gnss);
        
        _noise_a = filter_params._noise_a;
        _noise_g = filter_params._noise_g;
        _noise_ba = filter_params._noise_ba;
        _noise_bg = filter_params._noise_bg;
        
        if (_enable_gnss)
        {
            _noise_clockbias = filter_params._noise_clockbias;
            _noise_clockbias = filter_params._noise_cb_rw;
        }
    }
    
    State::State(const IngvioParams& filter_params) : _state_params(filter_params)
    {
        int idx = 0;
        
        _extended_pose = std::make_shared<SE23>();
        _extended_pose->set_cov_idx(idx);
        _err_variables.push_back(_extended_pose);
        idx += _extended_pose->size();
        
        _bg = std::make_shared<Vec3>();
        _bg->set_cov_idx(idx);
        _err_variables.push_back(_bg);
        idx += _bg->size();
        
        _ba = std::make_shared<Vec3>();
        _ba->set_cov_idx(idx);
        _err_variables.push_back(_ba);
        idx += _ba->size();
        
        _camleft_imu_extrinsics = std::make_shared<SE3>();
        _camleft_imu_extrinsics->set_cov_idx(idx);
        _err_variables.push_back(_camleft_imu_extrinsics);
        idx += _camleft_imu_extrinsics->size();
        
        _gnss.clear();
        _sw_camleft_poses.clear();
        _anchored_landmarks.clear();
        
        _cov = std::pow(1e-03, 2)*Eigen::MatrixXd::Identity(idx, idx);
        
        _camleft_imu_extrinsics->setValueByIso(filter_params._T_cl2i);
    }
    
    State::State() : _state_params()
    {
        int idx = 0;
        
        _extended_pose = std::make_shared<SE23>();
        _extended_pose->set_cov_idx(idx);
        _err_variables.push_back(_extended_pose);
        idx += _extended_pose->size();
        
        _bg = std::make_shared<Vec3>();
        _bg->set_cov_idx(idx);
        _err_variables.push_back(_bg);
        idx += _bg->size();
        
        _ba = std::make_shared<Vec3>();
        _ba->set_cov_idx(idx);
        _err_variables.push_back(_ba);
        idx += _ba->size();
        
        _camleft_imu_extrinsics = std::make_shared<SE3>();
        _camleft_imu_extrinsics->set_cov_idx(idx);
        _err_variables.push_back(_camleft_imu_extrinsics);
        idx += _camleft_imu_extrinsics->size();
        
        _gnss.clear();
        _sw_camleft_poses.clear();
        _anchored_landmarks.clear();
        
        _cov = std::pow(1e-03, 2)*Eigen::MatrixXd::Identity(idx, idx);
        
        _camleft_imu_extrinsics->setValueByIso(Eigen::Isometry3d::Identity());
    }
    
}
