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
    
    TEST_F(StateUpdateTest, stateBoxPlus)
    {
        
        Eigen::Vector3d x1, x2, x3;
        x1.setRandom();
        x1.z() = std::fabs(x1.z());
        x1.head<2>().normalize();
        AnchoredLandmark::switchRepBear2Xyz(x1, x2);
        AnchoredLandmark::switchRepXyz2Bear(x2, x3);
        ASSERT_NEAR(x1.z()-x3.z(), 0.0, 1e-08);
        ASSERT_NEAR(std::fabs(x1.y()), std::fabs(x3.y()), 1e-08);
        ASSERT_TRUE(std::fabs(x1.x()-x3.x()) < 1e-08 || std::fabs(std::fabs(x1.x()-x3.x())-M_PI) < 1e-08);
        
        x1.setRandom();
        x1.z() = std::fabs(x1.z());
        AnchoredLandmark::switchRepXyz2Bear(x1, x2);
        AnchoredLandmark::switchRepBear2Xyz(x2, x3);
        ASSERT_NEAR((x1-x3).norm(), 0.0, 1e-08);
        
        x1.setRandom();
        x1.z() = std::fabs(x1.z());
        AnchoredLandmark::switchRepXyz2Invd(x1, x2);
        AnchoredLandmark::switchRepInvd2Xyz(x2, x3);
        ASSERT_NEAR((x1-x3).norm(), 0.0, 1e-08);
        
        x1.setRandom();
        x1.z() = std::fabs(x1.z());
        AnchoredLandmark::switchRepInvd2Xyz(x1, x2);
        AnchoredLandmark::switchRepXyz2Invd(x2, x3);
        ASSERT_NEAR((x1-x3).norm(), 0.0, 1e-08);
        
        this->state->_timestamp = 1.0;
        
        StateManager::propagateStateCov(this->state, this->Phi_imu, this->G_imu, 1.5);
        
        state->_timestamp = 2.5;
        
        StateManager::augmentSlidingWindowPose(state);
        
        std::shared_ptr<AnchoredLandmark> lm1(new AnchoredLandmark());
        
        lm1->resetAnchoredPose(state->_sw_camleft_poses.at(2.5));
        lm1->setRepType(AnchoredLandmark::LmRep::BEARING);
        
        Eigen::Vector3d tmp = Eigen::Vector3d::Random();
        tmp.z() = std::fabs(tmp.z());
        tmp.topRows<2>().normalize();
        lm1->setValuePosRep(tmp);
        
        StateManager::addAnchoredLandmarkInState(state, lm1, 5, 10.0*Eigen::Matrix3d::Identity());
        
        Eigen::Vector3d bodyXYZ;
        bodyXYZ.z() = 1.0/tmp.z()*std::cos(tmp.y());
        bodyXYZ.x() = 1.0/tmp.z()*std::cos(tmp.x())*std::sin(tmp.y());
        bodyXYZ.y() = 1.0/tmp.z()*std::sin(tmp.x())*std::sin(tmp.y());
        
        Eigen::Vector3d worldXYZ = state->_sw_camleft_poses.at(2.5)->valueLinearAsMat()*bodyXYZ + state->_sw_camleft_poses.at(2.5)->valueTrans();
        
        ASSERT_NEAR((worldXYZ-lm1->valuePosXyz()).norm(), 0.0, 1e-8);
        ASSERT_NEAR((worldXYZ-state->_anchored_landmarks.at(5)->valuePosXyz()).norm(), 0.0, 1e-08);
        
        StateManager::propagateStateCov(this->state, this->Phi_imu, this->G_imu, 0.5);
        
        state->_timestamp = 3.0;
        
        StateManager::augmentSlidingWindowPose(state);
        
        std::shared_ptr<AnchoredLandmark> lm2(new AnchoredLandmark());
        
        lm2->resetAnchoredPose(state->_sw_camleft_poses.at(3.0));
        lm2->setRepType(AnchoredLandmark::LmRep::INV_DEPTH);
        
        tmp = Eigen::Vector3d::Random();
        tmp.z() = std::fabs(tmp.z());
        lm2->setValuePosRep(tmp);
        
        StateManager::addAnchoredLandmarkInState(state, lm2, 6, 12.0*Eigen::Matrix3d::Identity());
        
        bodyXYZ.x() = tmp.x()/tmp.z();
        bodyXYZ.y() = tmp.y()/tmp.z();
        bodyXYZ.z() = 1.0/tmp.z();
        
        worldXYZ = state->_sw_camleft_poses.at(3.0)->copyValueAsIso()*bodyXYZ;
        ASSERT_NEAR((worldXYZ-lm2->valuePosXyz()).norm(), 0.0, 1e-08);
        ASSERT_NEAR((worldXYZ-state->_anchored_landmarks.at(6)->valuePosXyz()).norm(), 0.0, 1e-08);
        
        Eigen::Matrix3d R_hat = state->_extended_pose->valueLinearAsMat();
        Eigen::Vector3d p_hat = state->_extended_pose->valueTrans1();
        Eigen::Vector3d v_hat = state->_extended_pose->valueTrans2();

        Eigen::Vector3d bg_hat = state->_bg->value();
        Eigen::Vector3d ba_hat = state->_ba->value();

        Eigen::Matrix3d R_ext_hat = state->_camleft_imu_extrinsics->valueLinearAsMat();
        Eigen::Vector3d p_ext_hat = state->_camleft_imu_extrinsics->valueTrans();
        
        double t_gps_hat = state->_gnss.at(State::GNSSType::GPS)->value();
        double t_bds_hat = state->_gnss.at(State::GNSSType::BDS)->value();
        double fs_hat = state->_gnss.at(State::GNSSType::FS)->value();
        double yof_hat = state->_gnss.at(State::GNSSType::YOF)->value();
        
        Eigen::Matrix3d R_c1_hat = state->_sw_camleft_poses.at(2.5)->valueLinearAsMat();
        Eigen::Vector3d p_c1_hat = state->_sw_camleft_poses.at(2.5)->valueTrans();
        
        Eigen::Matrix3d R_c2_hat = state->_sw_camleft_poses.at(3.0)->valueLinearAsMat();
        Eigen::Vector3d p_c2_hat = state->_sw_camleft_poses.at(3.0)->valueTrans();
        
        Eigen::Vector3d pf1_hat = state->_anchored_landmarks.at(5)->valuePosXyz();
        Eigen::Vector3d pf2_hat = state->_anchored_landmarks.at(6)->valuePosXyz();
        
        const int rows = state->curr_cov_size();
        Eigen::VectorXd dx(rows);
        dx.setRandom();
        
        const Eigen::Vector3d& delta_theta = dx.block<3, 1>(0, 0);
        const Eigen::Vector3d& delta_p = dx.block<3, 1>(3, 0);
        const Eigen::Vector3d& delta_v = dx.block<3, 1>(6, 0);
       
        const Eigen::Vector3d& delta_bg = dx.block<3, 1>(9, 0);
        const Eigen::Vector3d& delta_ba = dx.block<3, 1>(12, 0);
        
        const Eigen::Vector3d& delta_theta_ext = dx.block<3, 1>(15, 0);
        const Eigen::Vector3d& delta_p_ext = dx.block<3, 1>(18, 0);
        
        const double& delta_t_gps = dx(21);
        const double& delta_yof = dx(22);
        const double& delta_fs = dx(23);
        const double& delta_t_bds = dx(24);
        
        const Eigen::Vector3d& delta_theta_c1 = dx.block<3, 1>(25, 0);
        const Eigen::Vector3d& delta_p_c1 = dx.block<3, 1>(28, 0);
        
        const Eigen::Vector3d& delta_pf1 = dx.block<3, 1>(31, 0);
        
        const Eigen::Vector3d& delta_theta_c2 = dx.block<3, 1>(34, 0);
        const Eigen::Vector3d& delta_p_c2 = dx.block<3, 1>(37, 0);
        
        const Eigen::Vector3d& delta_pf2 = dx.block<3, 1>(40, 0);
        
        Eigen::Matrix3d R = GammaFunc(delta_theta, 0)*R_hat;
        Eigen::Vector3d p = GammaFunc(delta_theta, 0)*p_hat + GammaFunc(delta_theta, 1)*delta_p;
        Eigen::Vector3d v = GammaFunc(delta_theta, 0)*v_hat + GammaFunc(delta_theta, 1)*delta_v;
        
        Eigen::Vector3d bg = bg_hat + delta_bg;
        Eigen::Vector3d ba = ba_hat + delta_ba;
        
        Eigen::Matrix3d R_ext = GammaFunc(delta_theta_ext, 0)*R_ext_hat;
        Eigen::Vector3d p_ext = GammaFunc(delta_theta_ext, 0)*p_ext_hat + GammaFunc(delta_theta_ext, 1)*delta_p_ext;
        
        double t_gps = t_gps_hat + delta_t_gps;
        double t_bds = t_bds_hat + delta_t_bds;
        double yof = yof_hat + delta_yof;
        double fs = fs_hat + delta_fs;
        
        Eigen::Matrix3d R_c1 = GammaFunc(delta_theta_c1, 0)*R_c1_hat;
        Eigen::Vector3d p_c1 = GammaFunc(delta_theta_c1, 0)*p_c1_hat + GammaFunc(delta_theta_c1, 1)*delta_p_c1;
        
        Eigen::Matrix3d R_c2 = GammaFunc(delta_theta_c2, 0)*R_c2_hat;
        Eigen::Vector3d p_c2 = GammaFunc(delta_theta_c2, 0)*p_c2_hat + GammaFunc(delta_theta_c2, 1)*delta_p_c2;
        
        Eigen::Vector3d pf1 = GammaFunc(delta_theta_c1, 0)*pf1_hat + GammaFunc(delta_theta_c1, 1)*delta_pf1;
        
        Eigen::Vector3d pf2 = GammaFunc(delta_theta_c2, 0)*pf2_hat + GammaFunc(delta_theta_c2, 1)*delta_pf2;
        
        StateManager::boxPlus(state, dx);
        
        auto isMatNear = [](const Eigen::MatrixXd& mat1, const Eigen::MatrixXd& mat2)
        {
            assert(mat1.rows() == mat2.rows());
            assert(mat1.cols() == mat2.cols());
            
            if ((mat1-mat2).norm() < 1e-08)
                return true;
            else return false;
        };
        
        ASSERT_TRUE(isMatNear(R, state->_extended_pose->valueLinearAsMat()));
        ASSERT_TRUE(isMatNear(p, state->_extended_pose->valueTrans1()));
        ASSERT_TRUE(isMatNear(v, state->_extended_pose->valueTrans2()));
        
        ASSERT_TRUE(isMatNear(bg, state->_bg->value()));
        ASSERT_TRUE(isMatNear(ba, state->_ba->value()));
        
        ASSERT_TRUE(isMatNear(R_ext, state->_camleft_imu_extrinsics->valueLinearAsMat()));
        ASSERT_TRUE(isMatNear(p_ext, state->_camleft_imu_extrinsics->valueTrans()));
        
        ASSERT_NEAR(t_gps, state->_gnss.at(State::GNSSType::GPS)->value(), 1e-08);
        ASSERT_NEAR(t_bds, state->_gnss.at(State::GNSSType::BDS)->value(), 1e-08);
        ASSERT_NEAR(yof, state->_gnss.at(State::GNSSType::YOF)->value(), 1e-08);
        ASSERT_NEAR(fs, state->_gnss.at(State::GNSSType::FS)->value(), 1e-08);
        
        ASSERT_TRUE(isMatNear(R_c1, state->_sw_camleft_poses.at(2.5)->valueLinearAsMat()));
        ASSERT_TRUE(isMatNear(p_c1, state->_sw_camleft_poses.at(2.5)->valueTrans()));
        
        ASSERT_TRUE(isMatNear(R_c2, state->_sw_camleft_poses.at(3.0)->valueLinearAsMat()));
        ASSERT_TRUE(isMatNear(p_c2, state->_sw_camleft_poses.at(3.0)->valueTrans()));
        
        ASSERT_TRUE(isMatNear(pf1, state->_anchored_landmarks.at(5)->valuePosXyz()));
        ASSERT_TRUE(isMatNear(pf2, state->_anchored_landmarks.at(6)->valuePosXyz()));
        
        pf1 += delta_pf1;
        pf2 += delta_pf2;
        
        state->_anchored_landmarks.at(5)->resetAnchoredPose();
        state->_anchored_landmarks.at(6)->resetAnchoredPose();
        
        StateManager::boxPlus(state, dx);
        
        ASSERT_TRUE(isMatNear(pf1, state->_anchored_landmarks.at(5)->valuePosXyz()));
        ASSERT_TRUE(isMatNear(pf2, state->_anchored_landmarks.at(6)->valuePosXyz()));
        
        StateManager::margAnchoredLandmarkInState(state, 5);
        StateManager::margAnchoredLandmarkInState(state, 6);
        StateManager::margSlidingWindowPose(state);
        StateManager::margSlidingWindowPose(state, 3.0);
        StateManager::margSlidingWindowPose(state, 2.5);
        
        ASSERT_TRUE(state->curr_cov_size() == 25);
        ASSERT_TRUE(StateManager::checkStateContinuity(state));
    }
    
    TEST_F(StateUpdateTest, stateCovUpdate)
    {
        this->state->_timestamp = 1.0;
        
        StateManager::propagateStateCov(this->state, this->Phi_imu, this->G_imu, 1.5);
        
        state->_timestamp = 2.5;
        
        StateManager::augmentSlidingWindowPose(state);
        
        std::shared_ptr<AnchoredLandmark> lm1(new AnchoredLandmark());
        
        lm1->resetAnchoredPose(state->_sw_camleft_poses.at(2.5));
        lm1->setRepType(AnchoredLandmark::LmRep::BEARING);
        
        Eigen::Vector3d tmp = Eigen::Vector3d::Random();
        tmp.z() = std::fabs(tmp.z());
        tmp.topRows<2>().normalize();
        lm1->setValuePosRep(tmp);
        
        StateManager::addAnchoredLandmarkInState(state, lm1, 5, 10.0*Eigen::Matrix3d::Identity());
        
        std::vector<std::shared_ptr<Type>> var_order;
        var_order.push_back(state->_extended_pose);
        var_order.push_back(state->_gnss.at(State::GNSSType::GPS));
        var_order.push_back(state->_gnss.at(State::GNSSType::BDS));
        var_order.push_back(state->_gnss.at(State::GNSSType::FS));
        
        int var_size = StateManager::calcSubVarSize(var_order);
        
        Eigen::VectorXd res(6);
        res.setRandom();
        
        Eigen::MatrixXd H(res.rows(), var_size);
        H.setZero();
        
        H.block<6, 3>(0, 0).setRandom();
        H.block<3, 3>(0, 3).setRandom();
        H.block<3, 3>(3, 6).setRandom();
        
        H(0, 9) = 1.0;
        H(1, 9) = 1.0;
        H(2, 10) = 1.0;
        
        H.block<3, 1>(3, 11).setOnes();
        
        Eigen::MatrixXd H_large(res.rows(), state->curr_cov_size());
        
        H_large.setZero();
        
        H_large.block<6, 9>(0, 0) = H.block<6, 9>(0, 0);
        
        H_large(0, state->_gnss.at(State::GNSSType::GPS)->idx()) = 1.0;
        H_large(1, state->_gnss.at(State::GNSSType::GPS)->idx()) = 1.0;
        H_large(2, state->_gnss.at(State::GNSSType::BDS)->idx()) = 1.0;
        
        H_large.block<3, 1>(3, state->_gnss.at(State::GNSSType::FS)->idx()).setOnes();
        
        Eigen::MatrixXd cov_orig = StateManager::getFullCov(state);
        
        auto isMatNear = [](const Eigen::MatrixXd& mat1, const Eigen::MatrixXd& mat2)
        {
            assert(mat1.rows() == mat2.rows());
            assert(mat1.cols() == mat2.cols());
            
            if ((mat1-mat2).norm() < 1e-08)
                return true;
            else return false;
        };
        
        Eigen::MatrixXd R = 0.5*Eigen::MatrixXd::Identity(res.rows(), res.rows());
        
        StateManager::ekfUpdate(state, var_order, H, res, R);
        
        Eigen::MatrixXd K = cov_orig*H_large.transpose()*(H_large*cov_orig*H_large.transpose() + R).inverse();
        
        Eigen::MatrixXd cov_ref = (Eigen::MatrixXd::Identity(cov_orig.rows(), cov_orig.cols())-K*H_large)*cov_orig;
        
        ASSERT_TRUE(isMatNear(cov_ref, StateManager::getFullCov(state)));
    }
    
    class AddDelayedTest : public virtual ::testing::Test
    {
    protected:
        AddDelayedTest()
        {
            filter_params.readParams("/home/lcw/VIO/ws_ingvio/src/config/sportsfield/ingvio_stereo.yaml");
            
            state = std::make_shared<State>(filter_params);
            
            state->_extended_pose->setRandom();
            state->_bg->setRandom();
            state->_ba->setRandom();
            
            state->_camleft_imu_extrinsics->setRandom();
            
            Phi_imu.setRandom();
            G_imu.setRandom();
            
            state->initStateAndCov(Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::Random().normalized(), Eigen::Vector3d::Random().normalized()));
            
            this->state->_timestamp = 1.0;
            StateManager::propagateStateCov(this->state, this->Phi_imu, this->G_imu, 1.5);
            state->_timestamp = 2.5;
        }
        
        virtual ~AddDelayedTest() {}
        
        IngvioParams filter_params;
        std::shared_ptr<State> state;
        
        Eigen::Matrix<double, 15, 15> Phi_imu;
        Eigen::Matrix<double, 15, 12> G_imu;
    
    };

    TEST_F(AddDelayedTest, addVarInv)
    {
        Eigen::VectorXd res(1);
        res = Eigen::Vector2d::Random().head<1>();
        
        std::vector<std::shared_ptr<Type>> sub_var_old;
        sub_var_old.push_back(state->_extended_pose);
        
        Eigen::MatrixXd H_old(1, state->_extended_pose->size());
        H_old.setRandom();
        
        Eigen::MatrixXd H_new(1, 1);
        H_new(0, 0) = 1.0;
        
        std::shared_ptr<Scalar> tgps = std::make_shared<Scalar>();
        tgps->setRandom();
        state->_gnss[State::GNSSType::GPS] = tgps;
        
        Eigen::MatrixXd cov_orig = StateManager::getFullCov(state);
        
        double noise_meas_iso = 2.0;
        
        StateManager::addVariableDelayedInvertible(state, tgps, sub_var_old, H_old, H_new, res, noise_meas_iso);
        
        ASSERT_TRUE(StateManager::checkStateContinuity(state));
        
        ASSERT_EQ(state->curr_cov_size(), cov_orig.cols()+1);
        
        ASSERT_EQ(state->_gnss[State::GNSSType::GPS]->idx(), cov_orig.rows());
        
        Eigen::MatrixXd cov_new = StateManager::getFullCov(state);
        
        Eigen::MatrixXd x1 = H_old*cov_orig.block<9, 9>(0, 0)*H_old.transpose();
        
        ASSERT_NEAR(cov_new(cov_new.rows()-1, cov_new.cols()-1), (x1(0, 0)+std::pow(noise_meas_iso, 2.0))/std::pow(H_new(0, 0), 2.0), 1e-8);
        
        auto isMatNear = [](const Eigen::MatrixXd& mat1, const Eigen::MatrixXd& mat2)
        {
            if ((mat1-mat2).norm() < 1e-08)
                return true;
            else
                return false;
        };
        
        Eigen::Matrix<double, 1, 21> H_old_large = Eigen::Matrix<double, 1, 21>::Zero();
        H_old_large.block<1, 9>(0, 0) = H_old.block<1, 9>(0, 0);
        
        ASSERT_TRUE(isMatNear(cov_new.block<21, 1>(0, 21), -cov_orig*H_old_large.transpose()/H_new(0, 0)));
    }
    
    TEST_F(AddDelayedTest, addVar)
    {
        Eigen::VectorXd res(2);
        res.setRandom();
        
        std::vector<std::shared_ptr<Type>> sub_var_old;
        sub_var_old.push_back(state->_extended_pose);
        
        Eigen::MatrixXd H_old(2, state->_extended_pose->size());
        H_old.setRandom();
        
        Eigen::MatrixXd H_new(2, 1);
        H_new(0, 0) = 1.0;
        H_new(1, 0) = 1.0;
        
        Eigen::MatrixXd H_old_copy = H_old;
        Eigen::MatrixXd H_new_copy = H_new;
        Eigen::VectorXd res_copy = res;
        
        std::shared_ptr<Scalar> tgps = std::make_shared<Scalar>();
        tgps->setRandom();
        state->_gnss[State::GNSSType::GPS] = tgps;
        
        Eigen::MatrixXd cov_orig = StateManager::getFullCov(state);
        
        double noise_meas_iso = 2.0;
        
        StateManager::addVariableDelayed(state, tgps, sub_var_old, H_old, H_new, res, noise_meas_iso, 1.0, false);
        
        ASSERT_TRUE(StateManager::checkStateContinuity(state));
        
        ASSERT_EQ(state->curr_cov_size(), cov_orig.cols()+1);
        
        ASSERT_EQ(state->_gnss[State::GNSSType::GPS]->idx(), cov_orig.rows());
        
        Eigen::MatrixXd cov_new = StateManager::getFullCov(state);
        
        Eigen::Matrix<double, 2, 2> Q_T;
        Q_T(0, 0) = std::sqrt(2.0)/2.0;
        Q_T(0, 1) = std::sqrt(2.0)/2.0;
        Q_T(1, 0) = -std::sqrt(2.0)/2.0;
        Q_T(1, 1) = std::sqrt(2.0)/2.0;
        
        res_copy = Q_T*res_copy;
        H_old_copy = Q_T*H_old_copy;
        H_new_copy = Q_T*H_new_copy;
        
        Eigen::MatrixXd x1 = H_old_copy.block<1, 9>(0, 0)*cov_orig.block<9, 9>(0, 0)*H_old_copy.block<1, 9>(0, 0).transpose();
    
        
        auto isMatNear = [](const Eigen::MatrixXd& mat1, const Eigen::MatrixXd& mat2)
        {
            if ((mat1-mat2).norm() < 1e-08)
                return true;
            else
                return false;
        };
        
        Eigen::MatrixXd cov_before_update(22, 22);
        cov_before_update.block<21, 21>(0, 0) = cov_orig;
        
        cov_before_update(21, 21) = (x1(0, 0)+std::pow(noise_meas_iso, 2.0))/std::pow(H_new_copy(0, 0), 2.0);
        
        Eigen::Matrix<double, 1, 21> H_old_copy_large = Eigen::Matrix<double, 1, 21>::Zero();
        H_old_copy_large.block<1, 9>(0, 0) = H_old_copy.block<1, 9>(0, 0);
        
        cov_before_update.block<21, 1>(0, 21) = -cov_orig*H_old_copy_large.transpose()/H_new_copy(0, 0);
        
        cov_before_update.block<1, 21>(21, 0) = cov_before_update.block<21, 1>(0, 21).transpose();
        
        Eigen::MatrixXd H_update = Eigen::MatrixXd::Zero(1, 22);
        H_update.block<1, 9>(0, 0) = H_old_copy.block<1, 9>(1, 0);
        
        Eigen::MatrixXd S = H_update*cov_before_update*H_update.transpose() + std::pow(noise_meas_iso, 2.0)*Eigen::MatrixXd::Identity(1, 1);
        
        Eigen::MatrixXd Kalman_gain = cov_before_update*H_update.transpose()*S.inverse();
        
        Eigen::MatrixXd cov_after_update = cov_before_update;
        cov_after_update -= Kalman_gain*H_update*cov_before_update;
        
        ASSERT_TRUE(isMatNear(cov_after_update, cov_new));
    }
    
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
