#include <algorithm>

#include "StateManager.h"
#include "State.h"
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
        
        if (!_has_gravity_set && _init_imu_buffer_sp > 0)
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
    
    void ImuPropagator::stateAndCovTransition(std::shared_ptr<State> state,
                                              const ImuCtrl& imu_ctrl,
                                              double dt,
                                              Eigen::Matrix<double, 15, 15>& Phi,
                                              Eigen::Matrix<double, 15, 12>& G,
                                              bool isAnalytic)
    {        
        Phi.setIdentity();
        G.setZero();
        
        Eigen::Matrix3d R_hat = state->_extended_pose->valueLinearAsMat();
        Eigen::Vector3d p_hat = state->_extended_pose->valueTrans1();
        Eigen::Vector3d v_hat = state->_extended_pose->valueTrans2();
        
        G.block<3, 3>(0, 0) = R_hat;
        G.block<3, 3>(3, 0) = skew(p_hat)*R_hat;
        G.block<3, 3>(6, 0) = skew(v_hat)*R_hat;
        G.block<3, 3>(6, 3) = R_hat;
        G.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity();
        G.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity();
        
        if (isAnalytic)
        {
            const Eigen::Vector3d gyro_unbiased = imu_ctrl._gyro_raw - state->_bg->value();
            const Eigen::Vector3d acc_unbiased = imu_ctrl._accel_raw - state->_ba->value();
            
            state->_timestamp += dt;
            
            Eigen::Matrix3d Gamma0 = GammaFunc(dt*gyro_unbiased, 0);
            Eigen::Matrix3d Gamma1 = GammaFunc(dt*gyro_unbiased, 1);
            Eigen::Matrix3d Gamma2 = GammaFunc(dt*gyro_unbiased, 2);
            
            Eigen::Matrix3d R_hat_new = R_hat*Gamma0;
            state->_extended_pose->setValueLinearByMat(R_hat_new);
            
            Eigen::Vector3d v_hat_new = v_hat + _gravity*dt + R_hat*Gamma1*acc_unbiased*dt;
            state->_extended_pose->setValueTrans2(v_hat_new);
            
            Eigen::Vector3d p_hat_new = p_hat + v_hat*dt + 0.5*_gravity*std::pow(dt, 2) + R_hat*Gamma2*acc_unbiased*std::pow(dt, 2);
            state->_extended_pose->setValueTrans1(p_hat_new);
            
            if (state->_state_params._enable_gnss)
                for (int i = 0; i < 4; ++i)
                    if (state->_gnss.find(static_cast<State::GNSSType>(i)) != state->_gnss.end() && state->_gnss.find(State::GNSSType::FS) != state->_gnss.end())
                    {
                        double clockbias_hat = state->_gnss.at(static_cast<State::GNSSType>(i))->value();
                        
                        double clockbias_hat_new = clockbias_hat + dt*state->_gnss.at(State::GNSSType::FS)->value();
                        
                        state->_gnss.at(static_cast<State::GNSSType>(i))->setValue(clockbias_hat_new);
                    }
            
            Phi.block<3, 3>(3, 0) = 0.5*skew(_gravity)*std::pow(dt, 2.0);
            Phi.block<3, 3>(3, 6) = dt*Eigen::Matrix3d::Identity();
            Phi.block<3, 3>(6, 0) = skew(_gravity)*dt;
            
            Phi.block<3, 3>(0, 9) = -R_hat*Gamma1*dt;
            Phi.block<3, 3>(6, 12) = Phi.block<3, 3>(0, 9);
            
            Phi.block<3, 3>(3, 12) = -R_hat*Gamma2*std::pow(dt, 2);
            
            Phi.block<3, 3>(6, 9) = skew(v_hat_new)*Phi.block<3, 3>(0, 9) + R_hat*Psi1Func(gyro_unbiased, acc_unbiased, dt);
            
            Phi.block<3, 3>(3, 9) = skew(p_hat_new)*Phi.block<3, 3>(0, 9) + R_hat*Psi2Func(gyro_unbiased, acc_unbiased, dt);
        }
        else
        {
            const Eigen::Vector3d gyro_unbiased = imu_ctrl._gyro_raw - state->_bg->value();
            const Eigen::Vector3d acc_unbiased = imu_ctrl._accel_raw - state->_ba->value();
            
            state->_timestamp += dt;
            
            Eigen::Quaterniond q_hat = state->_extended_pose->valueLinearAsQuat();
            
            Eigen::Vector3d delta_angle = dt*gyro_unbiased;
            
            Eigen::Quaterniond q_hat_dt2 = q_hat*Eigen::AngleAxisd(delta_angle.norm()/2, delta_angle.normalized());
            
            Eigen::Quaterniond q_hat_dt = q_hat*Eigen::AngleAxisd(delta_angle.norm(), delta_angle.normalized());
            
            // k1 = f(tk, xk)
            Eigen::Vector3d k1_v = R_hat*acc_unbiased + _gravity;
            Eigen::Vector3d k1_p = v_hat;
            
            // k2 = f(tk+dt/2, xk+k1*dt/2)
            Eigen::Vector3d k2_v = q_hat_dt2.toRotationMatrix()*acc_unbiased + _gravity;
            Eigen::Vector3d k2_p = v_hat + k1_v*dt/2.0;
            
            // k3 = f(tk+dt/2, xk+k2*dt/2)
            Eigen::Vector3d k3_v = q_hat_dt2.toRotationMatrix()*acc_unbiased + _gravity;
            Eigen::Vector3d k3_p = v_hat + k2_v*dt/2.0;
            
            // k4 = f(tk+dt, xk+k3*dt)
            Eigen::Vector3d k4_v = q_hat_dt.toRotationMatrix()*acc_unbiased + _gravity;
            Eigen::Vector3d k4_p = v_hat + k3_v*dt;
            
            // update p and v nominal states using RK4
            Eigen::Vector3d v_hat_new = v_hat + dt/6.0*(k1_v + 2.0*k2_v + 2.0*k3_v + k4_v);
            Eigen::Vector3d p_hat_new = p_hat + dt/6.0*(k1_p + 2.0*k2_p + 2.0*k3_p + k4_p);
            
            state->_extended_pose->setValueLinearByQuat(q_hat_dt);
            
            state->_extended_pose->setValueTrans1(p_hat_new);
            
            state->_extended_pose->setValueTrans2(v_hat_new);
            
            if (state->_state_params._enable_gnss)
                for (int i = 0; i < 4; ++i)
                    if (state->_gnss.find(static_cast<State::GNSSType>(i)) != state->_gnss.end() && state->_gnss.find(State::GNSSType::FS) != state->_gnss.end())
                    {
                        double clockbias_hat = state->_gnss.at(static_cast<State::GNSSType>(i))->value();
                        
                        double clockbias_hat_new = clockbias_hat + dt*state->_gnss.at(State::GNSSType::FS)->value();
                        
                        state->_gnss.at(static_cast<State::GNSSType>(i))->setValue(clockbias_hat_new);
                    }
            
            Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Zero();
            
            F.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity();
            F.block<3, 3>(6, 0) = skew(_gravity);
            
            F.block<3, 3>(0, 9) = -R_hat;
            F.block<3, 3>(3, 9) = -skew(p_hat)*R_hat;
            F.block<3, 3>(6, 9) = -skew(v_hat)*R_hat;
            F.block<3, 3>(6, 12) = -R_hat;
            
            Eigen::Matrix<double, 15, 15> F2 = F*F/2.0;
            Eigen::Matrix<double, 15, 15> F3 = F2*F/3.0;
            
            Phi = Eigen::Matrix<double, 15, 15>::Identity() + F*dt + F2*dt*dt + F3*std::pow(dt, 3.0);
        }
    }
    
    void ImuPropagator::propagateUntil(std::shared_ptr<State> state,
                                       double t_end,
                                       bool isAnalytic)
    {
        if (!_has_gravity_set || t_end <= state->_timestamp) return;
        
        if (_imu_ctrl_buffer.size() == 0) return;
        
        if (_imu_ctrl_buffer[0]._timestamp > t_end) return;
        
        int propa_cnt = 0;
        
        ImuCtrl last_imu_ctrl = _imu_ctrl_buffer[_imu_ctrl_buffer.size()-1];
        
        for (int i = 0; i < _imu_ctrl_buffer.size(); ++i)
        {
            double ctrl_time = _imu_ctrl_buffer[i]._timestamp;
            
            if (ctrl_time < state->_timestamp)
            {
                ++propa_cnt;
                continue;
            }
            
            if (ctrl_time > t_end) break;
            
            ++propa_cnt;
            
            double dt = ctrl_time - state->_timestamp;
                    
            if (dt < 1e-6) continue;
            
            Eigen::Matrix<double, 15, 15> Phi;
            Eigen::Matrix<double, 15, 12> G;
            
            last_imu_ctrl = _imu_ctrl_buffer[i];
            
            this->stateAndCovTransition(state, _imu_ctrl_buffer[i], dt, Phi, G, isAnalytic);
            
            StateManager::propagateStateCov(state, Phi, G, dt);
        }
        
        if (state->_timestamp < t_end)
        {
            double dt_last = t_end - state->_timestamp;
            
            if (dt_last > 1e-06)
            {
                Eigen::Matrix<double, 15, 15> Phi;
                Eigen::Matrix<double, 15, 12> G;
                
                this->stateAndCovTransition(state, last_imu_ctrl, dt_last, Phi, G, isAnalytic);
                
                StateManager::propagateStateCov(state, Phi, G, dt_last);
            }
            else
                state->_timestamp = t_end;
        }
        
        _imu_ctrl_buffer.erase(_imu_ctrl_buffer.begin(), _imu_ctrl_buffer.begin()+propa_cnt);
    }
    
    void ImuPropagator::propagateAugmentAtEnd(std::shared_ptr<State> state,
                                              double t_end,
                                              bool isAnalytic)
    {
        if (!_has_gravity_set) return;
        
        this->propagateUntil(state, t_end, isAnalytic);
        
        if (state->_timestamp < t_end)
        {
            std::cout << "[ImuPropagator]: Cannot propa to t_end due to no imu ctrl!" << std::endl;
            return;
        }
        else if (state->_timestamp > t_end)
        {
            std::cout << "[IMUPropagator]: Cannot propa because t_end < curr state time!" << std::endl;
            return;
        }
        
        StateManager::augmentSlidingWindowPose(state);
    }
    
    void ImuPropagator::propagateToExpectedPoseAndAugment(std::shared_ptr<State> state,
                                                          double t_end,
                                                          const Eigen::Isometry3d& T_i2w)
    {
        if (!_has_gravity_set) return;
        
        state->_extended_pose->setValueLinearByMat(T_i2w.linear());
        state->_extended_pose->setValueTrans1(T_i2w.translation());
        state->_extended_pose->setValueTrans2(Eigen::Vector3d::Zero());
        
        state->_bg->setIdentity();
        state->_ba->setIdentity();
        
        state->_camleft_imu_extrinsics->setIdentity();
        
        state->_timestamp = t_end;
        
        StateManager::augmentSlidingWindowPose(state);
    }
}
