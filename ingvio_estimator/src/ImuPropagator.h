#pragma once

#include <memory>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <vector>

#include <sensor_msgs/Imu.h>

#include <eigen_conversions/eigen_msg.h>

#include "IngvioParams.h"
#include "AuxGammaFunc.h"

namespace ingvio
{
    class State;
    
    class ImuCtrl
    {
    public:
        ImuCtrl() : _timestamp(-1), _accel_raw(Eigen::Vector3d::Zero()), _gyro_raw(Eigen::Vector3d::Zero()) {}
        
        ImuCtrl(const sensor_msgs::Imu::ConstPtr& imu_msg)
        {
            _timestamp = imu_msg->header.stamp.toSec();
            
            tf::vectorMsgToEigen(imu_msg->angular_velocity, _gyro_raw);
            tf::vectorMsgToEigen(imu_msg->linear_acceleration, _accel_raw);
        }
        
        double _timestamp;
        
        Eigen::Vector3d _accel_raw;
        Eigen::Vector3d _gyro_raw;
        
        ImuCtrl operator+(const ImuCtrl& other_imu_ctrl)
        {
            ImuCtrl result;
            
            result._accel_raw = this->_accel_raw + other_imu_ctrl._accel_raw;
            result._gyro_raw = this->_gyro_raw + other_imu_ctrl._gyro_raw;
            
            return result;
        }
        
        ImuCtrl& operator=(const ImuCtrl& other_imu_ctrl)
        {
            this->_timestamp = other_imu_ctrl._timestamp;
            this->_accel_raw = other_imu_ctrl._accel_raw;
            this->_gyro_raw = other_imu_ctrl._gyro_raw;
            
            return *this;
        }
        
        void setRandom()
        {
            _accel_raw.setRandom();
            _gyro_raw.setRandom();
        }
    };
    
    class ImuPropagator
    {
    public:
        ImuPropagator() : _has_gravity_set(true) 
        {
            _init_imu_buffer_sp = -1;
            _init_gravity = 9.8;
            
            _gravity = Eigen::Vector3d(0.0, 0.0, -9.8);
            _quat_init.setIdentity();
        }
        
        ImuPropagator(const IngvioParams& filter_params) : _has_gravity_set(false), _init_gravity(filter_params._init_gravity), _max_imu_buffer_size(filter_params._max_imu_buffer_size), _init_imu_buffer_sp(filter_params._init_imu_buffer_sp)
        {
            if (_init_imu_buffer_sp < 0)
            {
                _has_gravity_set = true;
                
                _gravity = Eigen::Vector3d(0.0, 0.0, -_init_gravity);
                
                _quat_init.setIdentity();
            }
        }
        
        void reset(bool isResetGravity = false)
        {
            _has_gravity_set = !isResetGravity;
            _imu_ctrl_buffer.clear();
            
            if (_init_imu_buffer_sp < 0)
            {
                _has_gravity_set = true;
                
                _gravity = Eigen::Vector3d(0.0, 0.0, -_init_gravity);
                
                _quat_init.setIdentity();
            }
        }
        
        void storeImu(const ImuCtrl& imu_ctrl);
        
        void storeImu(const sensor_msgs::Imu::ConstPtr& imu_msg)
        {
            ImuCtrl tmp(imu_msg);
            this->storeImu(tmp);
        }
        
        bool getInitQuat(Eigen::Quaterniond& quat_init) const
        { 
            if (!_has_gravity_set)
            {
                quat_init.setIdentity();
                return false;
            }
            else
            {
                quat_init = _quat_init;
                return true;
            }
        }
        
        bool getAvgQuat(Eigen::Quaterniond& quat_avg, int num_ctrls = 1);
        
        const Eigen::Vector3d& getGravity() const
        {
            return _gravity;
        }
        
        const bool& isInit() const
        { return _has_gravity_set; }
        
        void stateAndCovTransition(std::shared_ptr<State> state, const ImuCtrl& imu_ctrl, double dt, Eigen::Matrix<double, 15, 15>& Phi, Eigen::Matrix<double, 15, 12>& G, bool isAnalytic = true);
        
        void propagateUntil(std::shared_ptr<State> state, double t_end, bool isAnalytic = true);
        
        void propagateAugmentAtEnd(std::shared_ptr<State> state, double t_end, bool isAnalytic = true);
        
        void propagateToExpectedPoseAndAugment(std::shared_ptr<State> state, double t_end, const Eigen::Isometry3d& T_i2w);
        
    protected:
        
        bool _has_gravity_set; 
        
        double _init_gravity;
        int _max_imu_buffer_size;
        int _init_imu_buffer_sp;
        
        Eigen::Vector3d _gravity;
        
        Eigen::Quaterniond _quat_init;
        
        std::vector<ImuCtrl> _imu_ctrl_buffer;
    };
}
