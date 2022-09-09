#include <algorithm>

#include "ImuPropagator.h"

namespace ingvio
{
    void ImuPropagator::storeImu(const ImuCtrl& imu_ctrl)
    {
        if (_imu_ctrl_buffer.size() > _max_imu_buffer_size)
        {
            std::cout << "[ImuPropagator]: Exceeding imu max buffer size, throw curr imu ctrl!" << std::endl;
            return;
        }
        else
            _imu_ctrl_buffer.push_back(imu_ctrl);
        
        if (!_has_gravity_set)
        {
            if (_imu_ctrl_buffer.size() < _init_imu_buffer_sp) return;
            
            std::cout << "[ImuPropagator]: Start init gravity norm ..." << std::endl;
            
            Eigen::Vector3d sum_gravity = Eigen::Vector3d::Zero();
            
            for (const auto& item : _imu_ctrl_buffer)
                sum_gravity += item._accel_raw;
            
            sum_gravity /= _imu_ctrl_buffer.size();
            
            if (std::fabs(sum_gravity.norm()-_init_gravity)/_init_gravity > 0.02)
            {
                std::cout << "[ImuPropagator]: Keep Camera STEADY!! Reinit gravity ..." << std::endl;
                
                _imu_ctrl_buffer.clear();
                _has_gravity_set = false;
            }
            else
            {
                _gravity = Eigen::Vector3d(0.0, 0.0, -sum_gravity.norm());
                
                _quat_init = Eigen::Quaterniond::FromTwoVectors(-sum_gravity, _gravity);
                
                _has_gravity_set = true;
                
                std::cout << "[ImuPropagator]: Gravity in body init: gx = " << -sum_gravity.x() << " gy = " << -sum_gravity.y() << " gz = " << -sum_gravity.z() << std::endl;
            }
        }
    }
    
    bool ImuPropagator::getAvgQuat(Eigen::Quaterniond& quat_avg, int num_ctrls)
    {
        if (_imu_ctrl_buffer.size() == 0 || num_ctrls <= 0)
        {
            quat_avg.setIdentity();
            return false;
        }
        
        Eigen::Vector3d sum_sf = Eigen::Vector3d::Zero();
        
        int idx = 0;
        
        for (int i = _imu_ctrl_buffer.size()-1; i >= std::max(0, (int)_imu_ctrl_buffer.size()-num_ctrls); --i)
        {
            sum_sf += _imu_ctrl_buffer[i]._accel_raw;
            ++idx;
        }
        
        sum_sf /= idx;
        sum_sf.normalize();
        
        quat_avg = Eigen::Quaterniond::FromTwoVectors(-sum_sf, Eigen::Vector3d(0, 0, -1));
        
        return true;
    }
}
