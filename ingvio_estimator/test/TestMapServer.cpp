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

using namespace ingvio;

namespace ingvio_test
{
    class TestMapServer : public virtual ::testing::Test
    {
    public:
        TestMapServer ()
        {
            filter_params.readParams("/home/lcw/VIO/ws_ingvio/src/config/sportsfield/ingvio_stereo.yaml");
            filter_params._init_imu_buffer_sp = -1;
            
            state = std::make_shared<State>(filter_params);
            
            imu_propa = std::make_shared<ImuPropagator>(filter_params);
            
            state->initStateAndCov(0.0, Eigen::Quaterniond::Identity());
            
            map_server = std::make_shared<MapServer>();
        }
        
        virtual ~TestMapServer() = default;
    protected:
        
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
        
        ASSERT_EQ(map_server->_feats.size(), 4);
        
        for (int i = 1; i < 5; ++i)
        {
            ASSERT_EQ(map_server->_feats.at(i)->getId(), i);
            ASSERT_EQ(map_server->_feats.at(i)->getFeatureType(), FeatureInfo::FeatureType::MSCKF);
            ASSERT_EQ(map_server->_feats.at(i)->numOfMonoFrames(), 1);
            ASSERT_TRUE(map_server->_feats.at(i)->hasMonoObsAt(2.0));
        }
        
        imu_propa->propagateToExpectedPoseAndAugment(state, 4.0, T);
        
        MapServerManager::collectMonoMeas(map_server, state, generateRandomMonoFrame(id2, 4.0));
        
        ASSERT_EQ(map_server->_feats.size(), 5);
        
        for (int i = 1; i < 6; ++i)
        {
            ASSERT_EQ(map_server->_feats.at(i)->getId(), i);
            ASSERT_EQ(map_server->_feats.at(i)->getFeatureType(), FeatureInfo::FeatureType::MSCKF);
            if (i == 1 || i == 5)
                ASSERT_EQ(map_server->_feats.at(i)->numOfMonoFrames(), 1);
            else
                ASSERT_EQ(map_server->_feats.at(i)->numOfMonoFrames(), 2);
                
            if (i > 1)
                ASSERT_TRUE(map_server->_feats.at(i)->hasMonoObsAt(4.0));
            
            ASSERT_TRUE(!map_server->_feats.at(i)->isToMarg());
        }
        
        MapServerManager::markMargFeatures(map_server, state, false);
        
        ASSERT_EQ(map_server->_feats.size(), 5);
        
        for (int i = 1; i < 6; ++i)
        {
            if (i == 1)
                ASSERT_TRUE(map_server->_feats.at(i)->isToMarg());
            else
                ASSERT_TRUE(!map_server->_feats.at(i)->isToMarg());
        }
        
        state.reset(new State(filter_params));
        
        state->initStateAndCov(0.0, Eigen::Quaterniond::Identity());
        
        imu_propa.reset(new ImuPropagator(filter_params));
        
        map_server.reset(new MapServer());
        
        imu_propa->propagateToExpectedPoseAndAugment(state, 2.0, T);
        
        MapServerManager::collectStereoMeas(map_server, state, generateRandomStereoFrame(id1, 2.0));
        
        ASSERT_EQ(map_server->_feats.size(), 4);
        
        for (int i = 1; i < 5; ++i)
        {
            ASSERT_EQ(map_server->_feats.at(i)->getId(), i);
            ASSERT_EQ(map_server->_feats.at(i)->getFeatureType(), FeatureInfo::FeatureType::MSCKF);
            ASSERT_EQ(map_server->_feats.at(i)->numOfStereoFrames(), 1);
            ASSERT_TRUE(map_server->_feats.at(i)->hasStereoObsAt(2.0));
        }
        
        imu_propa->propagateToExpectedPoseAndAugment(state, 4.0, T);
        
        MapServerManager::collectStereoMeas(map_server, state, generateRandomStereoFrame(id2, 4.0));
        
        ASSERT_EQ(map_server->_feats.size(), 5);
        
        for (int i = 1; i < 6; ++i)
        {
            ASSERT_EQ(map_server->_feats.at(i)->getId(), i);
            ASSERT_EQ(map_server->_feats.at(i)->getFeatureType(), FeatureInfo::FeatureType::MSCKF);
            if (i == 1 || i == 5)
                ASSERT_EQ(map_server->_feats.at(i)->numOfStereoFrames(), 1);
            else
                ASSERT_EQ(map_server->_feats.at(i)->numOfStereoFrames(), 2);
            
            if (i > 1)
                ASSERT_TRUE(map_server->_feats.at(i)->hasStereoObsAt(4.0));
            
            ASSERT_TRUE(!map_server->_feats.at(i)->isToMarg());
        }
        
        MapServerManager::markMargFeatures(map_server, state);
        
        ASSERT_EQ(map_server->_feats.size(), 5);
        
        for (int i = 1; i < 6; ++i)
        {
            if (i == 1)
                ASSERT_TRUE(map_server->_feats.at(i)->isToMarg());
            else
                ASSERT_TRUE(!map_server->_feats.at(i)->isToMarg());
        }
        
        for (int i = 1; i < 6; ++i)
            if (i < 5)
                ASSERT_TRUE(map_server->_feats.at(i)->anchor() == static_cast<const std::shared_ptr<SE3>>(state->_sw_camleft_poses.at(2.0)));
            else
                ASSERT_TRUE((map_server->_feats.at(i)->anchor() == static_cast<const std::shared_ptr<SE3>>(state->_sw_camleft_poses.at(4.0))));
    }
    
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
