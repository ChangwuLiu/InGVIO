/**  This File is part of InGVIO, an invariant filter for mono/stereo visual-
 *    inertial-raw GNSS navigation. 
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
 */

#include <gtest/gtest.h>

#include "IngvioParams.h"
#include "ImuPropagator.h"

#include "State.h"
#include "StateManager.h"

#include "TicToc.h"

using namespace ingvio;

namespace ingvio_test
{
    class TestPropagator : public virtual ::testing::Test
    {
    public:
        TestPropagator() : _N(300)
        {
            sf = Eigen::Vector3d::Random().normalized();
            sf *= 9.75;
        }
        
        virtual ~TestPropagator() {}
    protected:
        int _N;
        
        IngvioParams filter_params;
        
        Eigen::Vector3d sf;

    };
    
    TEST_F(TestPropagator, initGravity)
    {
        filter_params.readParams("/home/lcw/VIO/ws_ingvio/src/config/sportsfield/ingvio_stereo.yaml");
        filter_params.printParams();
        
        ImuPropagator ip2;
        
        ImuPropagator ip1(filter_params);
        
        for (int i = 0; i < _N; ++i)
        {
            ImuCtrl imu_ctrl;
            imu_ctrl._accel_raw = sf;
            imu_ctrl._gyro_raw.setZero();
            
            imu_ctrl._timestamp = 0.01*i;
            
            ip1.storeImu(imu_ctrl);
            ip2.storeImu(imu_ctrl);
        }
        
        auto isMatNear = [](const Eigen::MatrixXd& mat1, const Eigen::MatrixXd& mat2)
        {
            if ((mat1-mat2).norm() < 1e-08)
                return true;
            else
                return false;
        };
        
        auto isQuatNear = [](const Eigen::Quaterniond& quat1, const Eigen::Quaterniond& quat2)
        {
            if ((quat1.toRotationMatrix()-quat2.toRotationMatrix()).norm() < 1e-08)
                return true;
            else
                return false;
        };
        
        ASSERT_TRUE(isMatNear(Eigen::Vector3d(0, 0, -sf.norm()), ip1.getGravity()));
        ASSERT_TRUE(isMatNear(Eigen::Vector3d(0, 0, -9.8), ip2.getGravity()));
        
        Eigen::Quaterniond init_quat1, init_quat2;
        ASSERT_TRUE(ip1.getInitQuat(init_quat1));
        ASSERT_TRUE(ip2.getInitQuat(init_quat2));
        
        ASSERT_TRUE(isQuatNear(init_quat2, Eigen::Quaterniond::Identity()));
        
        Eigen::Quaterniond quat_ref = Eigen::Quaterniond::FromTwoVectors(-sf, Eigen::Vector3d(0, 0, -sf.norm()));
        
        ASSERT_TRUE(isQuatNear(init_quat1, quat_ref));
        
        Eigen::Quaterniond tmp_quat1, tmp_quat2;
        for (int i = 1; i < 400; i += 20)
        {
            ASSERT_TRUE(ip1.getAvgQuat(tmp_quat1, i));
            ASSERT_TRUE(ip2.getAvgQuat(tmp_quat2, i));
            ASSERT_TRUE(isQuatNear(quat_ref, tmp_quat1));
            ASSERT_TRUE(isQuatNear(quat_ref, tmp_quat2));
        }
    }
    
    TEST_F(TestPropagator, oneStepProp)
    {
        filter_params.readParams("/home/lcw/VIO/ws_ingvio/src/config/sportsfield/ingvio_stereo.yaml");
        
        filter_params._init_imu_buffer_sp = -1;
        
        ImuPropagator ip(filter_params);
        
        ImuCtrl imu_ctrl;
        imu_ctrl.setRandom();
        
        std::shared_ptr<State> state1, state2;
        
        auto reset = [](std::shared_ptr<State>& state1, std::shared_ptr<State>& state2, const IngvioParams& filter_params)
        {
            state1.reset(new State(filter_params));
            state2.reset(new State(filter_params));
            
            state1->_timestamp = 1.0;
            state2->_timestamp = 1.0;
            
            state1->_extended_pose->setRandom();
            state2->_extended_pose = state1->_extended_pose->clone();
            
            state1->_bg->setRandom();
            state2->_bg = state1->_bg->clone();
            
            state1->_ba->setRandom();
            state2->_ba = state1->_ba->clone();
            
            StateManager::addGNSSVariable(state1, State::GNSSType::GPS, 5000, 9.0);
            StateManager::addGNSSVariable(state1, State::GNSSType::FS, 20, 2.0);
            
            StateManager::addGNSSVariable(state2, State::GNSSType::GPS, 5000, 9.0);
            StateManager::addGNSSVariable(state2, State::GNSSType::FS, 20, 2.0);
        };
        
        reset(state1, state2, filter_params);
        ASSERT_NEAR(distance(state1, state2), 0.0, 1e-12);
        
        double err1 = INFINITY;
        double err2 = INFINITY;
        
        Eigen::Matrix<double, 15, 15> Phi1, Phi2;
        Eigen::Matrix<double, 15, 12> G1, G2;
        
        double t_analytic = 0.0;
        double t_numerical = 0.0;
        
        for (int i = 0; i < 5; ++i)
        {
            reset(state1, state2, filter_params);
            
            double dt = std::pow(10.0, -i);
            
            TicToc analytic;
            ip.stateAndCovTransition(state1, imu_ctrl, dt, Phi1, G1, true);
            t_analytic += analytic.toc();
            
            TicToc numerical;
            ip.stateAndCovTransition(state2, imu_ctrl, dt, Phi2, G2, false);
            t_numerical += numerical.toc();
            
            double err_state = distance(state1, state2);
            double err_Phi = (Phi1-Phi2).norm();
            
            ASSERT_TRUE(err_state < err1);
            ASSERT_TRUE(err_Phi < err2);
            
            err1 = err_state;
            err2 = err_Phi;
        }
        
        std::cout << "Analytic one step prop: " << t_analytic << " (ms)" << std::endl;
        std::cout << "Numerical one step prop: " << t_numerical << " (ms)" << std::endl;
    }
    
    TEST_F(TestPropagator, propaUntil)
    {
        filter_params.readParams("/home/lcw/VIO/ws_ingvio/src/config/sportsfield/ingvio_stereo.yaml");
        filter_params._init_imu_buffer_sp = -1;
        
        std::shared_ptr<State> state = std::make_shared<State>(filter_params);
        
        ImuPropagator ip(filter_params);
        
        ASSERT_TRUE(ip.isInit());
        
        state->initStateAndCov(0.0, Eigen::Quaterniond::Identity());
        
        for (int i = -20; i <= 100; ++i)
        {
            ImuCtrl imu_ctrl;
            imu_ctrl.setRandom();
            imu_ctrl._timestamp = 0.1*i;
            
            ip.storeImu(imu_ctrl);
        }
        
        ip.propagateUntil(state, 1.0);
        ASSERT_EQ(state->_timestamp, 1.0);
        
        ip.propagateUntil(state, 8.35);
        ASSERT_EQ(state->_timestamp, 8.35);
        
        ip.propagateUntil(state, 7.1);
        ASSERT_EQ(state->_timestamp, 8.35);
        
        ip.propagateUntil(state, 9.5);
        ASSERT_EQ(state->_timestamp, 9.5);
        
        ip.propagateUntil(state, 10.5);
        ASSERT_EQ(state->_timestamp, 10.5);
        
        ip.propagateUntil(state, 11.0);
        ASSERT_EQ(state->_timestamp, 10.5);
    }
    
    TEST_F(TestPropagator, propagateAugment)
    {
        filter_params.readParams("/home/lcw/VIO/ws_ingvio/src/config/sportsfield/ingvio_stereo.yaml");
        filter_params._init_imu_buffer_sp = -1;
        
        std::shared_ptr<State> state = std::make_shared<State>(filter_params);
        
        ImuPropagator ip(filter_params);
        
        ASSERT_TRUE(ip.isInit());
        
        state->initStateAndCov(0.0, Eigen::Quaterniond::Identity());
        
        for (int i = -20; i <= 100; ++i)
        {
            ImuCtrl imu_ctrl;
            imu_ctrl.setRandom();
            imu_ctrl._timestamp = 0.1*i;
            
            ip.storeImu(imu_ctrl);
        }
        
        for (int i = 0; i < 15; ++i)
            ip.propagateAugmentAtEnd(state, (i+1)*0.5);
        
        ASSERT_EQ(state->_timestamp, 7.5);
        ASSERT_EQ(state->curr_cov_size(), 21 + 6*15);
    }
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
