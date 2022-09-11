#include "State.h"

namespace ingvio
{
    StateParams::StateParams(const IngvioParams& filter_params)
    {
        _cam_nums = filter_params._cam_nums;
        _max_sw_poses = filter_params._max_sw_clones;
        _max_landmarks = filter_params._max_lm_feats;
        
        _T_cl2cr = filter_params._T_cl2i*filter_params._T_cr2i.inverse();
        _T_cl2i = filter_params._T_cl2i;
        
        _enable_gnss = static_cast<bool>(filter_params._enable_gnss);
        
        _noise_a = filter_params._noise_a;
        _noise_g = filter_params._noise_g;
        _noise_ba = filter_params._noise_ba;
        _noise_bg = filter_params._noise_bg;
        
        _init_cov_rot = filter_params._init_cov_rot;
        _init_cov_pos = filter_params._init_cov_pos;
        _init_cov_vel = filter_params._init_cov_vel;
        _init_cov_bg = filter_params._init_cov_bg;
        _init_cov_ba = filter_params._init_cov_ba;
        _init_cov_ext_rot = filter_params._init_cov_ext_rot;
        _init_cov_ext_pos = filter_params._init_cov_ext_pos;
        
        if (_enable_gnss)
        {
            _noise_clockbias = filter_params._noise_clockbias;
            _noise_clockbias = filter_params._noise_cb_rw;
            
            _init_cov_rcv_clockbias = filter_params._init_cov_rcv_clockbias;
            _init_cov_rcv_clockbias_randomwalk = filter_params._init_cov_rcv_clockbias_randomwalk;
            _init_cov_yof = filter_params._init_cov_yof;
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
    
    void State::initStateAndCov(double init_timestamp, const Eigen::Quaterniond& init_quat_i2w, const Eigen::Vector3d& init_pos, const Eigen::Vector3d& init_vel, const Eigen::Vector3d& init_bg, const Eigen::Vector3d& init_ba)
    {
        _timestamp = init_timestamp;
        
        for (int i = _extended_pose->idx(); i < _extended_pose->idx()+3; ++i)
            _cov(i, i) = std::pow(_state_params._init_cov_rot, 2.0);
        
        for (int i = _extended_pose->idx()+3; i < _extended_pose->idx()+6; ++i)
            _cov(i, i) = std::pow(_state_params._init_cov_pos, 2.0);
        
        for (int i = _extended_pose->idx()+6; i < _extended_pose->idx()+9; ++i)
            _cov(i, i) = std::pow(_state_params._init_cov_vel, 2.0);
        
        for (int i = _bg->idx(); i < _bg->idx()+_bg->size(); ++i)
            _cov(i, i) = std::pow(_state_params._init_cov_bg, 2.0);
        
        for (int i = _ba->idx(); i < _ba->idx()+_ba->size(); ++i)
            _cov(i, i) = std::pow(_state_params._init_cov_ba, 2.0);
        
        for (int i = _camleft_imu_extrinsics->idx(); i < _camleft_imu_extrinsics->idx()+3; ++i)
            _cov(i, i) = std::pow(_state_params._init_cov_ext_rot, 2.0);
        
        for (int i = _camleft_imu_extrinsics->idx()+3; i < _camleft_imu_extrinsics->idx()+6; ++i)
            _cov(i, i) = std::pow(_state_params._init_cov_ext_pos, 2.0);
        
        this->_extended_pose->setValueLinearByQuat(init_quat_i2w);
        
        this->_extended_pose->setValueTrans1(init_pos);
        
        this->_extended_pose->setValueTrans2(init_vel);
        
        this->_bg->setValue(init_bg);
        
        this->_ba->setValue(init_ba);
        
        this->_camleft_imu_extrinsics->setValueByIso(_state_params._T_cl2i);
    }
    
    void State::initStateAndCov(double init_timestamp, const Eigen::Quaterniond& init_quat_i2w, const Eigen::Vector3d& init_pos)
    {
        this->initStateAndCov(init_timestamp, init_quat_i2w, init_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    }
    
    void State::initStateAndCov(double init_timestamp, const Eigen::Quaterniond& init_quat_i2w)
    {
        this->initStateAndCov(init_timestamp, init_quat_i2w, Eigen::Vector3d::Zero());
    }
    
    double distance(std::shared_ptr<State> state1, std::shared_ptr<State> state2)
    {
        if (state1->_timestamp != state2->_timestamp)
            std::cout << "[State]: Timestamp not the same when calc distance!" << std::endl; 
        
        double dist = 0.0;
        
        dist += (state1->_extended_pose->valueLinearAsMat()-state2->_extended_pose->valueLinearAsMat()).norm();
        
        dist += (state1->_extended_pose->valueTrans1()-state2->_extended_pose->valueTrans1()).norm();
        
        dist += (state1->_extended_pose->valueTrans2()-state2->_extended_pose->valueTrans2()).norm();
        
        dist += (state1->_bg->value()-state2->_bg->value()).norm();
        
        dist += (state1->_ba->value()-state2->_ba->value()).norm();
        
        dist += (state1->_camleft_imu_extrinsics->valueLinearAsMat()-state2->_camleft_imu_extrinsics->valueLinearAsMat()).norm();
        
        dist += (state1->_camleft_imu_extrinsics->valueTrans()-state2->_camleft_imu_extrinsics->valueTrans()).norm();
        
        for (const auto& item : state1->_gnss)
        {
            assert(state2->_gnss.find(item.first) != state2->_gnss.end());
            
            dist += std::fabs(item.second->value() - state2->_gnss.at(item.first)->value());
        }
        
        for (const auto& item : state1->_anchored_landmarks)
        {
            assert(state2->_anchored_landmarks.find(item.first) != state2->_anchored_landmarks.end());
            
            dist += (item.second->valuePosXyz()-state2->_anchored_landmarks.at(item.first)->valuePosXyz()).norm();
        }
        
        for (const auto& item : state2->_sw_camleft_poses)
        {
            assert(state2->_sw_camleft_poses.find(item.first) != state2->_sw_camleft_poses.end());
            
            dist += (item.second->valueLinearAsMat()-state2->_sw_camleft_poses.at(item.first)->valueLinearAsMat()).norm();
            
            dist += (item.second->valueTrans()-state2->_sw_camleft_poses.at(item.first)->valueTrans()).norm();
        }
        
        return  dist;
    }

}
