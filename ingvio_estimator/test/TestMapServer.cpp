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

#include <ros/ros.h>
#include <std_msgs/Header.h>

#include "MapServer.h"
#include "MapServerManager.h"

#include "State.h"
#include "StateManager.h"

#include "ImuPropagator.h"

#include <ros/ros.h>
#include <std_msgs/Header.h>

#include "GenerateNoise.h"
#include "Triangulator.h"

using namespace ingvio;

namespace ingvio_test
{
    class TestMapServer : public virtual ::testing::Test
    {
    public:
        TestMapServer ()
        {
            T_cl2cr.linear() = Eigen::Matrix3d::Identity();
            T_cl2cr.translation() = Eigen::Vector3d(0.001, -0.12, 0.003);
            
            filter_params.readParams("/home/lcw/VIO/ws_ingvio/src/config/sportsfield/ingvio_stereo.yaml");
            filter_params._init_imu_buffer_sp = -1;
            
            state = std::make_shared<State>(filter_params);
            state->_state_params._T_cl2cr = T_cl2cr;
            
            imu_propa = std::make_shared<ImuPropagator>(filter_params);
            
            state->initStateAndCov(0.0, Eigen::Quaterniond::Identity());
            
            map_server = std::make_shared<MapServer>();
        }
        
        virtual ~TestMapServer() = default;
    protected:
        
        Eigen::Isometry3d T_cl2cr;
        
        IngvioParams filter_params;
        
        std::shared_ptr<State> state;
        
        std::shared_ptr<ImuPropagator> imu_propa;
        
        std::shared_ptr<MapServer> map_server;
        
        feature_tracker::MonoFrameConstPtr generateRandomMonoFrame(const std::vector<int>& ids, double timestamp)
        {
            feature_tracker::MonoFramePtr mono_frame(new feature_tracker::MonoFrame());
            
            mono_frame->header.stamp = ros::Time(timestamp);
            
            for (int i = 0; i < ids.size(); ++i)
            {
                feature_tracker::MonoMeas::Ptr mono_meas(new feature_tracker::MonoMeas());
                
                mono_meas->id = ids[i];
                
                Eigen::Vector2d vec = Eigen::Vector2d::Random();
                mono_meas->u0 = vec.x();
                mono_meas->v0 = vec.y();
                
                mono_frame->mono_features.push_back(*mono_meas);
            }
            
            return mono_frame;
        }
        
        feature_tracker::MonoMeas::Ptr calcMonoMeas(int id, const Eigen::Vector3d& pf, const std::shared_ptr<SE3> pose)
        {
            feature_tracker::MonoMeas::Ptr mono_meas(new feature_tracker::MonoMeas());
            
            Eigen::Vector3d body = pose->copyValueAsIso().inverse()*pf;
            
            mono_meas->id = id;
            mono_meas->u0 = body.x()/body.z() + generateGaussRandom(0.0, 0.02);
            mono_meas->v0 = body.y()/body.z() + generateGaussRandom(0.0, 0.02);
            
            return mono_meas;
        }
        
        feature_tracker::MonoFrameConstPtr generateMonoFrame(double timestamp, const std::vector<int>& ids, const std::vector<Eigen::Vector3d>& pfs, const std::shared_ptr<SE3> pose)
        {
            feature_tracker::MonoFramePtr mono_frame(new feature_tracker::MonoFrame());
            
            mono_frame->header.stamp = ros::Time(timestamp);
            
            assert(ids.size() == pfs.size());
            
            for (int i = 0; i < ids.size(); ++i)
                mono_frame->mono_features.push_back(*(calcMonoMeas(ids[i], pfs[i], pose)));
            
            return mono_frame;
        }
        
        feature_tracker::StereoFrameConstPtr generateRandomStereoFrame(const std::vector<int>& ids, double timestamp)
        {
            feature_tracker::StereoFramePtr stereo_frame(new feature_tracker::StereoFrame());
            
            stereo_frame->header.stamp = ros::Time(timestamp);
            
            for (int i = 0; i < ids.size(); ++i)
            {
                feature_tracker::StereoMeas::Ptr stereo_meas(new feature_tracker::StereoMeas());
                
                stereo_meas->id = ids[i];
                
                Eigen::Vector4d vec = Eigen::Vector4d::Random();
                stereo_meas->u0 = vec.x();
                stereo_meas->v0 = vec.y();
                stereo_meas->u1 = vec.z();
                stereo_meas->v1 = vec.w();
                
                stereo_frame->stereo_features.push_back(*stereo_meas);
            }
            
            return stereo_frame;
        }
        
        feature_tracker::StereoMeas::Ptr calcStereoMeas(int id, const Eigen::Vector3d& pf, const std::shared_ptr<SE3> pose, const Eigen::Isometry3d& T_cl2cr)
        {
            feature_tracker::StereoMeas::Ptr stereo_meas(new feature_tracker::StereoMeas());
            
            Eigen::Vector3d body_left = pose->copyValueAsIso().inverse()*pf;
            Eigen::Vector3d body_right = T_cl2cr*pose->copyValueAsIso().inverse()*pf;
            
            stereo_meas->id = id;
            
            stereo_meas->u0 = body_left.x()/body_left.z() + generateGaussRandom(0.0, 0.02);
            stereo_meas->v0 = body_left.y()/body_left.z() + generateGaussRandom(0.0, 0.02);
            
            stereo_meas->u1 = body_right.x()/body_right.z() + generateGaussRandom(0.0, 0.02);
            stereo_meas->v1 = body_right.y()/body_right.z() + generateGaussRandom(0.0, 0.02);
            
            return stereo_meas;
        }
        
        feature_tracker::StereoFrameConstPtr generateStereoFrame(double timestamp, const std::vector<int>& ids, const std::vector<Eigen::Vector3d>& pfs, const std::shared_ptr<SE3> pose, const Eigen::Isometry3d& T_cl2cr)
        {
            feature_tracker::StereoFramePtr stereo_frame(new feature_tracker::StereoFrame());
            
            stereo_frame->header.stamp = ros::Time(timestamp);
            
            assert(ids.size() == pfs.size());
            
            for (int i = 0; i < ids.size(); ++i)
                stereo_frame->stereo_features.push_back(*(calcStereoMeas(ids[i], pfs[i], pose, T_cl2cr)));
            
            return stereo_frame;
        }
    };
    
    TEST_F(TestMapServer, collectFeatureAndMarg)
    {
        std::vector<int> id1(4);
        for (int i = 0; i < id1.size(); ++i)
            id1[i] = i+1;
        
        std::vector<int> id2(4);
        for (int i = 0; i < id2.size(); ++i)
            id2[i] = i+2;

        Eigen::Isometry3d T;
        Eigen::Vector3d rot = Eigen::Vector3d::Random();
        Eigen::Vector3d trans = Eigen::Vector3d::Random();
        
        T.linear() = Eigen::AngleAxisd(rot.norm(), rot.normalized()).toRotationMatrix();
        T.translation() = trans;
        
        imu_propa->propagateToExpectedPoseAndAugment(state, 2.0, T);
        
        MapServerManager::collectMonoMeas(map_server, state, generateRandomMonoFrame(id1, 2.0));
        
        ASSERT_EQ(map_server->size(), 4);
        
        for (int i = 1; i < 5; ++i)
        {
            ASSERT_EQ(map_server->at(i)->getId(), i);
            ASSERT_EQ(map_server->at(i)->getFeatureType(), FeatureInfo::FeatureType::MSCKF);
            ASSERT_EQ(map_server->at(i)->numOfMonoFrames(), 1);
            ASSERT_TRUE(map_server->at(i)->hasMonoObsAt(2.0));
        }
        
        imu_propa->propagateToExpectedPoseAndAugment(state, 4.0, T);
        
        MapServerManager::collectMonoMeas(map_server, state, generateRandomMonoFrame(id2, 4.0));
        
        ASSERT_EQ(map_server->size(), 5);
        
        for (int i = 1; i < 6; ++i)
        {
            ASSERT_EQ(map_server->at(i)->getId(), i);
            ASSERT_EQ(map_server->at(i)->getFeatureType(), FeatureInfo::FeatureType::MSCKF);
            if (i == 1 || i == 5)
                ASSERT_EQ(map_server->at(i)->numOfMonoFrames(), 1);
            else
                ASSERT_EQ(map_server->at(i)->numOfMonoFrames(), 2);
                
            if (i > 1)
                ASSERT_TRUE(map_server->at(i)->hasMonoObsAt(4.0));
            
            ASSERT_TRUE(!map_server->at(i)->isToMarg());
        }
        
        MapServerManager::markMargMonoFeatures(map_server, state);
        
        ASSERT_EQ(map_server->size(), 5);
        
        for (int i = 1; i < 6; ++i)
        {
            if (i == 1)
                ASSERT_TRUE(map_server->at(i)->isToMarg());
            else
                ASSERT_TRUE(!map_server->at(i)->isToMarg());
        }
        
        state.reset(new State(filter_params));
        
        state->initStateAndCov(0.0, Eigen::Quaterniond::Identity());
        
        imu_propa.reset(new ImuPropagator(filter_params));
        
        map_server.reset(new MapServer());
        
        imu_propa->propagateToExpectedPoseAndAugment(state, 2.0, T);
        
        MapServerManager::collectStereoMeas(map_server, state, generateRandomStereoFrame(id1, 2.0));
        
        ASSERT_EQ(map_server->size(), 4);
        
        for (int i = 1; i < 5; ++i)
        {
            ASSERT_EQ(map_server->at(i)->getId(), i);
            ASSERT_EQ(map_server->at(i)->getFeatureType(), FeatureInfo::FeatureType::MSCKF);
            ASSERT_EQ(map_server->at(i)->numOfStereoFrames(), 1);
            ASSERT_TRUE(map_server->at(i)->hasStereoObsAt(2.0));
        }
        
        imu_propa->propagateToExpectedPoseAndAugment(state, 4.0, T);
        
        MapServerManager::collectStereoMeas(map_server, state, generateRandomStereoFrame(id2, 4.0));
        
        ASSERT_EQ(map_server->size(), 5);
        
        for (int i = 1; i < 6; ++i)
        {
            ASSERT_EQ(map_server->at(i)->getId(), i);
            ASSERT_EQ(map_server->at(i)->getFeatureType(), FeatureInfo::FeatureType::MSCKF);
            if (i == 1 || i == 5)
                ASSERT_EQ(map_server->at(i)->numOfStereoFrames(), 1);
            else
                ASSERT_EQ(map_server->at(i)->numOfStereoFrames(), 2);
            
            if (i > 1)
                ASSERT_TRUE(map_server->at(i)->hasStereoObsAt(4.0));
            
            ASSERT_TRUE(!map_server->at(i)->isToMarg());
        }
        
        MapServerManager::markMargStereoFeatures(map_server, state);
        
        ASSERT_EQ(map_server->size(), 5);
        
        for (int i = 1; i < 6; ++i)
        {
            if (i == 1)
                ASSERT_TRUE(map_server->at(i)->isToMarg());
            else
                ASSERT_TRUE(!map_server->at(i)->isToMarg());
        }
        
        for (int i = 1; i < 6; ++i)
            if (i < 5)
                ASSERT_TRUE(map_server->at(i)->anchor() == static_cast<const std::shared_ptr<SE3>>(state->_sw_camleft_poses.at(2.0)));
            else
                ASSERT_TRUE((map_server->at(i)->anchor() == static_cast<const std::shared_ptr<SE3>>(state->_sw_camleft_poses.at(4.0))));
    }
    
    TEST_F(TestMapServer, featureInfoTriMono)
    {
        std::vector<int> ids1(2);
        ids1[0] = 1;
        ids1[1] = 2;
        
        std::vector<Eigen::Vector3d> pfs1(2);
        pfs1[0] = Eigen::Vector3d(1.0, 1.5, 2.0);
        pfs1[1] = Eigen::Vector3d(1.0, 2.0, 3.0);
        
        std::vector<int> ids2(2);
        ids2[0] = 2;
        ids2[1] = 3;
        
        std::vector<Eigen::Vector3d> pfs2(2);
        pfs2[0] = pfs1[1];
        pfs2[1] = Eigen::Vector3d(1.5, 2.0, 3.0);
        
        std::vector<double> time(9);
        for (int i = 0; i < time.size(); ++i)
            time[i] = 0.5*(i+1);
        
        std::vector<Eigen::Isometry3d> T(9);
        for (int i = 0; i < T.size(); ++i)
        {
            T[i].linear() = Eigen::AngleAxisd(generateGaussRandom(0.0, 0.1), Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
            T[i].translation() = Eigen::Vector3d(2*i-5.0, 2*i-6.0, 0.0);
        }
        
        std::vector<std::shared_ptr<SE3>> poses(9);
        for (int i = 0; i < 9; ++i)
        {
            poses[i] = std::make_shared<SE3>();
            poses[i]->setValueByIso(T[i]);
        }
        
        imu_propa->propagateToExpectedPoseAndAugment(state, time[0], T[0]);
        MapServerManager::collectMonoMeas(map_server, state, generateMonoFrame(time[0], ids1, pfs1, poses[0]));
        
        imu_propa->propagateToExpectedPoseAndAugment(state, time[1], T[1]);
        MapServerManager::collectMonoMeas(map_server, state, generateMonoFrame(time[1], ids1, pfs1, poses[1]));
        
        imu_propa->propagateToExpectedPoseAndAugment(state, time[2], T[2]);
        MapServerManager::collectMonoMeas(map_server, state, generateMonoFrame(time[2], ids1, pfs1, poses[2]));
        
        imu_propa->propagateToExpectedPoseAndAugment(state, time[3], T[3]);
        MapServerManager::collectMonoMeas(map_server, state, generateMonoFrame(time[3], ids1, pfs1, poses[3]));
        
        imu_propa->propagateToExpectedPoseAndAugment(state, time[4], T[4]);
        MapServerManager::collectMonoMeas(map_server, state, generateMonoFrame(time[4], ids2, pfs2, poses[4]));
        
        imu_propa->propagateToExpectedPoseAndAugment(state, time[5], T[5]);
        MapServerManager::collectMonoMeas(map_server, state, generateMonoFrame(time[5], ids2, pfs2, poses[5]));
        
        std::shared_ptr<Triangulator> tri(new Triangulator());
        
        Eigen::Vector3d result;
        bool flag;
        flag = FeatureInfoManager::triangulateFeatureInfoMono(map_server->at(1), tri, state);
        
        std::cout << "pf1 ref = " << pfs1[0].transpose() << std::endl;
        std::cout << "pf1 mono estimated = " << map_server->at(1)->landmark()->valuePosXyz().transpose() << std::endl;
        
        ASSERT_TRUE(map_server->at(1)->isTri() == flag);
        
        flag = FeatureInfoManager::triangulateFeatureInfoMono(map_server->at(2), tri, state);
        std::cout << "pf2 ref = " << pfs1[1].transpose() << std::endl;
        std::cout << "pf2 mono estimated = " << map_server->at(2)->landmark()->valuePosXyz().transpose() << std::endl;
        
        ASSERT_TRUE(map_server->at(2)->isTri() == flag);
        
        imu_propa->propagateToExpectedPoseAndAugment(state, time[6], T[6]);
        MapServerManager::collectMonoMeas(map_server, state, generateMonoFrame(time[6], ids2, pfs2, poses[6]));
        
        imu_propa->propagateToExpectedPoseAndAugment(state, time[7], T[7]);
        MapServerManager::collectMonoMeas(map_server, state, generateMonoFrame(time[7], ids2, pfs2, poses[7]));
        
        imu_propa->propagateToExpectedPoseAndAugment(state, time[8], T[8]);
        MapServerManager::collectMonoMeas(map_server, state, generateMonoFrame(time[8], ids2, pfs2, poses[8]));
        
        flag = FeatureInfoManager::triangulateFeatureInfoMono(map_server->at(3), tri, state);
        std::cout << "pf3 ref = " << pfs2[1].transpose() << std::endl;
        std::cout << "pf3 mono estimated = " << map_server->at(3)->landmark()->valuePosXyz().transpose() << std::endl;
        
        ASSERT_TRUE(map_server->at(3)->isTri() == flag);
    }
    
    TEST_F(TestMapServer, featureInfoTriStereo)
    {
        std::vector<int> ids1(2);
        ids1[0] = 1;
        ids1[1] = 2;
        
        std::vector<Eigen::Vector3d> pfs1(2);
        pfs1[0] = Eigen::Vector3d(1.0, 1.5, 2.0);
        pfs1[1] = Eigen::Vector3d(1.0, 2.0, 3.0);
        
        std::vector<int> ids2(2);
        ids2[0] = 2;
        ids2[1] = 3;
        
        std::vector<Eigen::Vector3d> pfs2(2);
        pfs2[0] = pfs1[1];
        pfs2[1] = Eigen::Vector3d(1.5, 2.0, 3.0);
        
        std::vector<double> time(9);
        for (int i = 0; i < time.size(); ++i)
            time[i] = 0.5*(i+1);
        
        std::vector<Eigen::Isometry3d> T(9);
        for (int i = 0; i < T.size(); ++i)
        {
            T[i].linear() = Eigen::AngleAxisd(generateGaussRandom(0.0, 0.1), Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
            T[i].translation() = Eigen::Vector3d(2*i-5.0, 2*i-6.0, 0.0);
        }
        
        std::vector<std::shared_ptr<SE3>> poses(9);
        for (int i = 0; i < 9; ++i)
        {
            poses[i] = std::make_shared<SE3>();
            poses[i]->setValueByIso(T[i]);
        }
        
        imu_propa->propagateToExpectedPoseAndAugment(state, time[0], T[0]);
        MapServerManager::collectStereoMeas(map_server, state, generateStereoFrame(time[0], ids1, pfs1, poses[0], T_cl2cr));
        
        imu_propa->propagateToExpectedPoseAndAugment(state, time[1], T[1]);
        MapServerManager::collectStereoMeas(map_server, state, generateStereoFrame(time[1], ids1, pfs1, poses[1], T_cl2cr));
        
        imu_propa->propagateToExpectedPoseAndAugment(state, time[2], T[2]);
        MapServerManager::collectStereoMeas(map_server, state, generateStereoFrame(time[2], ids1, pfs1, poses[2], T_cl2cr));
        
        imu_propa->propagateToExpectedPoseAndAugment(state, time[3], T[3]);
        MapServerManager::collectStereoMeas(map_server, state, generateStereoFrame(time[3], ids1, pfs1, poses[3], T_cl2cr));
        
        imu_propa->propagateToExpectedPoseAndAugment(state, time[4], T[4]);
        MapServerManager::collectStereoMeas(map_server, state, generateStereoFrame(time[4], ids2, pfs2, poses[4], T_cl2cr));
        
        imu_propa->propagateToExpectedPoseAndAugment(state, time[5], T[5]);
        MapServerManager::collectStereoMeas(map_server, state, generateStereoFrame(time[5], ids2, pfs2, poses[5], T_cl2cr));
        
        std::shared_ptr<Triangulator> tri(new Triangulator());
        
        Eigen::Vector3d result;
        bool flag;
        flag = FeatureInfoManager::triangulateFeatureInfoStereo(map_server->at(1), tri, state);
        
        std::cout << "pf1 ref = " << pfs1[0].transpose() << std::endl;
        std::cout << "pf1 stereo estimated = " << map_server->at(1)->landmark()->valuePosXyz().transpose() << std::endl;
        
        ASSERT_TRUE(map_server->at(1)->isTri() == flag);
        
        flag = FeatureInfoManager::triangulateFeatureInfoStereo(map_server->at(2), tri, state);
        std::cout << "pf2 ref = " << pfs1[1].transpose() << std::endl;
        std::cout << "pf2 stereo estimated = " << map_server->at(2)->landmark()->valuePosXyz().transpose() << std::endl;
        
        ASSERT_TRUE(map_server->at(2)->isTri() == flag);
        
        imu_propa->propagateToExpectedPoseAndAugment(state, time[6], T[6]);
        MapServerManager::collectStereoMeas(map_server, state, generateStereoFrame(time[6], ids2, pfs2, poses[6], T_cl2cr));
        
        imu_propa->propagateToExpectedPoseAndAugment(state, time[7], T[7]);
        MapServerManager::collectStereoMeas(map_server, state, generateStereoFrame(time[7], ids2, pfs2, poses[7], T_cl2cr));
        
        imu_propa->propagateToExpectedPoseAndAugment(state, time[8], T[8]);
        MapServerManager::collectStereoMeas(map_server, state, generateStereoFrame(time[8], ids2, pfs2, poses[8], T_cl2cr));
        
        flag = FeatureInfoManager::triangulateFeatureInfoStereo(map_server->at(3), tri, state);
        std::cout << "pf3 ref = " << pfs2[1].transpose() << std::endl;
        std::cout << "pf3 stereo estimated = " << map_server->at(3)->landmark()->valuePosXyz().transpose() << std::endl;
        
        ASSERT_TRUE(map_server->at(3)->isTri() == flag);
    }
    
    TEST_F(TestMapServer, changeAnchor)
    {
        for (int i = 0; i < 500; ++i)
        {
            ImuCtrl imu_ctrl;
            
            imu_ctrl._gyro_raw = Eigen::Vector3d(0.0, 0.0, 0.0);
            imu_ctrl._accel_raw = Eigen::Vector3d(0.0, 0.0, 9.8);
            imu_ctrl._timestamp = 0.01*i;
            
            imu_propa->storeImu(imu_ctrl);
        }
        
        std::vector<int> ids(1);
        ids[0] = 1;
        std::vector<Eigen::Vector3d> pfs(1);
        pfs[0] = Eigen::Vector3d(1.0, 1.0, 6.0);
        
        imu_propa->propagateAugmentAtEnd(state, 1.8);
        MapServerManager::collectMonoMeas(map_server, state, generateMonoFrame(1.8, ids, pfs, state->_sw_camleft_poses.at(1.8)));
        
        StateManager::addAnchoredLandmarkInState(state, map_server->at(1)->landmark(), map_server->at(1)->getId(), 2.21*Eigen::Matrix3d::Identity());
        
        imu_propa->propagateAugmentAtEnd(state, 3.6);
        MapServerManager::collectMonoMeas(map_server, state, generateMonoFrame(3.6, ids, pfs, state->_sw_camleft_poses.at(3.6)));
        
        map_server->at(1)->_ftype = FeatureInfo::FeatureType::SLAM;
        map_server->at(1)->_landmark->setValuePosXyz(pfs[0]);
        
        ASSERT_EQ(map_server->at(1)->getId(), 1);
        ASSERT_EQ(map_server->at(1)->isTri(), false);
        ASSERT_EQ(map_server->at(1)->isToMarg(), false);
        
        ASSERT_EQ(map_server->at(1)->_landmark->getAnchoredPose(), state->_sw_camleft_poses.at(1.8));
        
        ASSERT_EQ(map_server->at(1)->numOfMonoFrames(), 2);
        ASSERT_EQ(state->_anchored_landmarks.at(1)->getAnchoredPose(), state->_sw_camleft_poses.at(1.8));
        
        ASSERT_NEAR((state->_anchored_landmarks.at(1)->valuePosXyz()-pfs[0]).norm(), 0.0, 1e-06);
        ASSERT_NEAR((map_server->at(1)->_landmark->valuePosXyz()-pfs[0]).norm(), 0.0, 1e-06);
                
        ASSERT_TRUE(isSPD(StateManager::getFullCov(state)));
        
        Eigen::MatrixXd cov_orig = StateManager::getFullCov(state);
        Eigen::MatrixXd cov = StateManager::getFullCov(state);
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 36);
        
        // std::cout << "cov ref = " << cov << std::endl;
        
        H.block<3, 3>(0, 27) = Eigen::Matrix3d::Identity();
        H.block<3, 3>(0, 21) = -skew(pfs[0]);
        H.block<3, 3>(0, 30) = skew(pfs[0]);
        
        FeatureInfoManager::changeAnchoredPose(map_server->at(1), state, 3.6);
        
        ASSERT_EQ(map_server->at(1)->_landmark->getAnchoredPose(), state->_sw_camleft_poses.at(3.6));
        ASSERT_EQ(state->_anchored_landmarks.at(1)->getAnchoredPose(), state->_sw_camleft_poses.at(3.6));
        
        ASSERT_TRUE(isSPD(StateManager::getFullCov(state)));
        
        cov.block<36, 3>(0, 27) = cov_orig*H.transpose();
        cov.block<3, 36>(27, 0) = H*cov_orig;
        cov.block<3, 3>(27, 27) = H*cov_orig*H.transpose();
        
        // std::cout << "cov ref = " << cov << std::endl;
        // std::cout << "cov diff = " << cov - StateManager::getFullCov(state) << std::endl;
        
        ASSERT_NEAR((cov-StateManager::getFullCov(state)).norm(), 0.0, 1e-10);
    }
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
