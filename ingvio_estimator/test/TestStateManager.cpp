#include <gtest/gtest.h>
#include <Eigen/Geometry>

#include "State.h"
#include "StateManager.h"

using namespace ingvio;

namespace ingvio_test
{
    TEST(testState, BasicFuncs)
    {
        for (int i = 0; i < 20; ++i)
        {
            Eigen::Vector3d vec;
            vec.setRandom();
            
            ASSERT_TRUE(skew(vec) == -skew(vec).transpose());
            ASSERT_TRUE(vee(skew(vec)) == vec);
            
            Eigen::Matrix3d rot_mat = Eigen::AngleAxisd(vec.norm(), vec.normalized()).toRotationMatrix();
            
            ASSERT_NEAR((GammaFunc(vec)-rot_mat).norm(), 0.0, 1e-08);
        }
        
        ASSERT_NEAR((GammaFunc(Eigen::Vector3d::Zero(), 1)-Eigen::Matrix3d::Identity()).norm(), 0.0, 1e-08);
        
        ASSERT_NEAR((GammaFunc(Eigen::Vector3d::Zero(), 2)-0.5*Eigen::Matrix3d::Identity()).norm(), 0.0, 1e-08);
        
        ASSERT_NEAR((GammaFunc(Eigen::Vector3d::Zero(), 3)-1.0/6.0*Eigen::Matrix3d::Identity()).norm(), 0.0, 1e-08);
    }

    TEST(testState, StateAddMargProp)
    {
        IngvioParams filter_params;
        
        filter_params.readParams("/home/lcw/VIO/ws_ingvio/src/config/sportsfield/ingvio_stereo.yaml");
        filter_params.printParams();
        
        std::shared_ptr<State> state = std::make_shared<State>(filter_params);
        
        ASSERT_TRUE(StateManager::checkStateContinuity(state));
        ASSERT_EQ(state->curr_cov_size(), 21);
        
        ASSERT_NEAR((state->_camleft_imu_extrinsics->valueLinearAsMat()-filter_params._T_cl2i.linear()).norm(), 0.0, 1e-08);
        
        ASSERT_NEAR((state->_camleft_imu_extrinsics->valueTrans()-filter_params._T_cl2i.translation()).norm(), 0.0, 1e-08);
        
        StateManager::addGNSSVariable(state, State::GNSSType::GPS, 20.0, 4.0);
        ASSERT_TRUE(state->curr_cov_size() == 22);
        ASSERT_TRUE(state->curr_err_variable_size() == 5);
        
        StateManager::addGNSSVariable(state, State::GNSSType::BDS, 16.0, 4.0);
        ASSERT_TRUE(state->curr_cov_size() == 23);
        ASSERT_TRUE(state->curr_err_variable_size() == 6);
        
        StateManager::addGNSSVariable(state, State::GNSSType::YOF, 123.0, 1.0);
        ASSERT_TRUE(state->curr_cov_size() == 24);
        ASSERT_TRUE(state->curr_err_variable_size() == 7);
        
        StateManager::addGNSSVariable(state, State::GNSSType::FS, 2.0, 1.0);
        ASSERT_TRUE(state->curr_cov_size() == 25);
        ASSERT_TRUE(state->curr_err_variable_size() == 8);
        
        StateManager::margGNSSVariable(state, State::GNSSType::GPS);
        ASSERT_TRUE(state->curr_cov_size() == 24);
        ASSERT_TRUE(state->curr_err_variable_size() == 7);
        
        StateManager::addGNSSVariable(state, State::GNSSType::GLO, 3.0, 6.0);
        ASSERT_TRUE(state->curr_cov_size() == 25);
        ASSERT_TRUE(state->curr_err_variable_size() == 8);
        
        Eigen::MatrixXd cov = StateManager::getFullCov(state);
        const Eigen::Matrix<double, 15, 15> Phi_imu = Eigen::Matrix<double, 15, 15>::Random();
        const Eigen::Matrix<double, 15, 12> G_imu = Eigen::Matrix<double, 15, 12>::Random();
        const double dt = 1.5;
        
        Eigen::Matrix<double, 12, 12> Q_imu;
        Q_imu.setIdentity();
        
        Q_imu.middleCols<3>(0) *= std::pow(state->_state_params._noise_g, 2.0);
        Q_imu.middleCols<3>(3) *= std::pow(state->_state_params._noise_a, 2.0);
        Q_imu.middleCols<3>(6) *= std::pow(state->_state_params._noise_bg, 2.0);
        Q_imu.middleCols<3>(9) *= std::pow(state->_state_params._noise_ba, 2.0);
        
        Eigen::Matrix2d Q_gnss;
        Q_gnss.setZero();
        Q_gnss(0, 0) = std::pow(state->_state_params._noise_clockbias, 2.0);
        Q_gnss(1, 1) = std::pow(state->_state_params._noise_cb_rw, 2.0);
        
        Eigen::Matrix<double, 14, 14> Q = Eigen::Matrix<double, 14, 14>::Zero();
        Q.block<12, 12>(0, 0) = Q_imu;
        Q.block<2, 2>(12, 12) = Q_gnss;
        
        Eigen::Matrix4d Phi_gnss = Eigen::Matrix4d::Identity();
        Phi_gnss(0, 2) = dt;
        Phi_gnss(3, 2) = dt;
        
        Eigen::Matrix<double, 25, 25> Phi = Eigen::Matrix<double, 25, 25>::Identity();
        Phi.block<15, 15>(0, 0) = Phi_imu;
        Phi.block<4, 4>(21, 21) = Phi_gnss;
        
        Eigen::Matrix<double, 25, 14> G = Eigen::Matrix<double, 25, 14>::Zero();
        G.block<15, 12>(0, 0) = G_imu;
        Eigen::Matrix<double, 4, 2> G_gnss = Eigen::Matrix<double, 4, 2>::Zero();
        G_gnss(0, 0) = 1;
        G_gnss(2, 1) = 1;
        G_gnss(3, 0) = 1;
        G.block<4, 2>(21, 12) = G_gnss;
        
        Eigen::MatrixXd cov_result = Phi*cov*Phi.transpose() + dt*Phi*G*Q*G.transpose()*Phi.transpose();
        
        StateManager::propagateStateCov(state, Phi_imu, G_imu, dt);
        
        ASSERT_NEAR((StateManager::getFullCov(state)-cov_result).norm(), 0.0, 1e-10);
        
        // std::cout << "propagateStateCov = " << StateManager::getFullCov(state) << std::endl;
        // std::cout << "=====================" << std::endl;
        // std::cout << "calculated ref cov = " << cov_result << std::endl;
        
        std::vector<std::shared_ptr<Type>> small_var;
        small_var.push_back(state->_extended_pose);

        small_var.push_back(state->_ba);
    
        small_var.push_back(state->_gnss.at(State::GNSSType::YOF));
        
        small_var.push_back(state->_gnss.at(State::GNSSType::GLO));
        
        Eigen::MatrixXd small_cov = StateManager::getMarginalCov(state, small_var);
    
        ASSERT_NEAR((small_cov.block<9, 9>(0, 0) - StateManager::getFullCov(state).block<9, 9>(0, 0)).norm(), 0.0, 1e-06);
        ASSERT_NEAR((small_cov.block<3, 3>(9, 9) - StateManager::getFullCov(state).block<3, 3>(12, 12)).norm(), 0.0, 1e-06);
    }

    class StateUpdateTest : public virtual ::testing::Test
    {
    protected:
        StateUpdateTest()
        {
            filter_params.readParams("/home/lcw/VIO/ws_ingvio/src/config/sportsfield/ingvio_stereo.yaml");
            
            state = std::make_shared<State>(filter_params);
            
            state->_extended_pose->setRandom();
            state->_bg->setRandom();
            state->_ba->setRandom();
            
            state->_camleft_imu_extrinsics->setRandom();
            
            StateManager::addGNSSVariable(state, State::GNSSType::GPS, 20.0, 4.0);
            
            StateManager::addGNSSVariable(state, State::GNSSType::YOF, 123.0, 1.0);
            
            StateManager::addGNSSVariable(state, State::GNSSType::FS, 2.0, 1.0);
            
            StateManager::addGNSSVariable(state, State::GNSSType::BDS, 16.0, 4.0);
            
            Phi_imu.setRandom();
            
            G_imu.setRandom();
        }
        
        virtual ~StateUpdateTest() {}
        
        IngvioParams filter_params;
        std::shared_ptr<State> state;
        
        Eigen::Matrix<double, 15, 15> Phi_imu;
        Eigen::Matrix<double, 15, 12> G_imu;
    };
    
    TEST_F(StateUpdateTest, augmentPose)
    {
        auto constructJ = [](const Eigen::Matrix3d& C_i2w)
        {
            Eigen::Matrix<double, 6, 21> tmp;
            tmp.setZero();
            
            tmp.block<3, 3>(0, 0).setIdentity();
            tmp.block<3, 3>(3, 3).setIdentity();
            
            tmp.block<3, 3>(0, 15) = C_i2w;
            tmp.block<3, 3>(3, 18) = C_i2w;
            
            return tmp;
        };
        
        auto constructLargeJ = [&](int orig_row, const Eigen::Matrix3d& C_i2w)
        {
            Eigen::MatrixXd tmp(orig_row+6, orig_row);
            tmp.setZero();
            
            tmp.block(0, 0, orig_row, orig_row) = Eigen::MatrixXd::Identity(orig_row, orig_row);
            tmp.block<6, 21>(orig_row, 0) = constructJ(C_i2w);
            
            return tmp;
        };
        
        this->state->_timestamp = 1.0;
        
        StateManager::propagateStateCov(this->state, this->Phi_imu, this->G_imu, 1.5);
        
        state->_timestamp = 2.5;
        
        Eigen::MatrixXd cov1 = StateManager::getFullCov(state);
        Eigen::MatrixXd largeJ1 = constructLargeJ(cov1.rows(), this->state->_extended_pose->valueLinearAsMat()); 
        
        StateManager::augmentSlidingWindowPose(state);
        
        ASSERT_NEAR((StateManager::getFullCov(state)-largeJ1*cov1*largeJ1.transpose()).norm(), 0.0, 1e-08);
        
        ASSERT_EQ(state->curr_cov_size(), cov1.rows() + 6);
        
        ASSERT_NEAR((state->_extended_pose->valueLinearAsMat()*state->_camleft_imu_extrinsics->valueLinearAsMat()-state->_sw_camleft_poses[2.5]->valueLinearAsMat()).norm(), 0.0, 1e-08);
        ASSERT_NEAR((state->_extended_pose->valueTrans1()+state->_extended_pose->valueLinearAsMat()*state->_camleft_imu_extrinsics->valueTrans()-state->_sw_camleft_poses[2.5]->valueTrans()).norm(), 0.0, 1e-08);
        
        StateManager::propagateStateCov(this->state, this->Phi_imu, this->G_imu, 0.5);
        
        state->_timestamp = 3.0;
        
        Eigen::MatrixXd cov2 = StateManager::getFullCov(state);
        Eigen::MatrixXd largeJ2 = constructLargeJ(cov2.rows(), this->state->_extended_pose->valueLinearAsMat());
        
        StateManager::augmentSlidingWindowPose(state);
        
        ASSERT_NEAR((StateManager::getFullCov(state)-largeJ2*cov2*largeJ2.transpose()).norm(), 0.0, 1e-08);
        
        ASSERT_EQ(state->curr_cov_size(), cov2.rows() + 6);
        
        ASSERT_NEAR((state->_extended_pose->valueLinearAsMat()*state->_camleft_imu_extrinsics->valueLinearAsMat()-state->_sw_camleft_poses[3.0]->valueLinearAsMat()).norm(), 0.0, 1e-08);
        ASSERT_NEAR((state->_extended_pose->valueTrans1()+state->_extended_pose->valueLinearAsMat()*state->_camleft_imu_extrinsics->valueTrans()-state->_sw_camleft_poses[3.0]->valueTrans()).norm(), 0.0, 1e-08);
    }

}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
