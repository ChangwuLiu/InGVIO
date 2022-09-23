
#include <nav_msgs/Odometry.h>
#include <tf_conversions/tf_eigen.h>

#include "IngvioFilter.h"

#include "State.h"
#include "StateManager.h"

#include "ImuPropagator.h"

#include "Triangulator.h"

#include "MapServerManager.h"

#include "RemoveLostUpdate.h"
#include "SwMargUpdate.h"
#include "LandmarkUpdate.h"

#include "TicToc.h"

namespace ingvio
{
    void IngvioFilter::initIO()
    {
        _filter_params.readParams(_nh);
        
        if (_filter_params._cam_nums == 2)
        {
            _sub_stereo_frame = _nh.subscribe(_filter_params._feature_topic, 100, &IngvioFilter::callbackStereoFrame, this);
        }
        else if (_filter_params._cam_nums == 1)
        {
            _sub_mono_frame = _nh.subscribe(_filter_params._feature_topic, 100, &IngvioFilter::callbackMonoFrame, this);
        }
        else
        {
            std::cout << "[IngvioParams]: Cam num " << _filter_params._cam_nums << " not supported! Init as mono config!" << std::endl;
            
            _filter_params._cam_nums = 1;
            
            _sub_mono_frame = _nh.subscribe(_filter_params._feature_topic, 100, &IngvioFilter::callbackMonoFrame, this);
        }
        
        _sub_imu = _nh.subscribe(_filter_params._imu_topic, 500, &IngvioFilter::callbackIMU, this);
        
        _odom_w_pub = _nh.advertise<nav_msgs::Odometry>("pose_w", 5);
        _path_w_pub = _nh.advertise<nav_msgs::Path>("path_w", 1);
        
        _state = std::make_shared<State>(_filter_params);
        
        _imu_propa = std::make_shared<ImuPropagator>(_filter_params);
        
        _tri = std::make_shared<Triangulator>(_filter_params);
        
        _map_server = std::make_shared<MapServer>();
        
        _remove_lost_update = std::make_shared<RemoveLostUpdate>(_filter_params);
        
        _sw_marg_update = std::make_shared<SwMargUpdate>(_filter_params);
        
        _landmark_update = std::make_shared<LandmarkUpdate>(_filter_params);
        
        _filter_params.printParams();
    }
    
    void IngvioFilter::callbackMonoFrame(const feature_tracker::MonoFrameConstPtr& mono_frame_ptr)
    {
        if (!_hasImageCome)
        {
            _hasImageCome = true;
            return;
        }
        
        if (!_hasInitState) return;
        
        const double target_time = mono_frame_ptr->header.stamp.toSec();
        
        if (_state->_timestamp >= target_time) return;
    
        TicToc timer_mono;
        
        _imu_propa->propagateAugmentAtEnd(_state, target_time);
        
        if (_state->_timestamp < target_time) return;
        
        MapServerManager::collectMonoMeas(_map_server, _state, mono_frame_ptr);
        
        _remove_lost_update->updateStateMono(_state, _map_server, _tri);
        
        _sw_marg_update->updateStateMono(_state, _map_server, _tri);
        
        _landmark_update->updateLandmarkMono(_state, _map_server);
        
        _landmark_update->initNewLandmarkMono(_state, _map_server, _tri);
        
        _sw_marg_update->cleanMonoObsAtMargTime(_state, _map_server);
        
        _sw_marg_update->changeMSCKFAnchor(_state, _map_server);
        
        _landmark_update->changeLandmarkAnchor(_state, _map_server);
        
        // _sw_marg_update->removeMonoMSCKFinMargPose(_state, _map_server);
        
        _sw_marg_update->margSwPose(_state);
        
        MapServerManager::eraseInvalidFeatures(_map_server, _state);
        
        // std::cout << "[IngvioFilter]: Marg sw update time = " << timer_marg_sw.toc() << " (ms) " << std::endl;
        
        /*
        MapServerManager::mapStatistics(_map_server);
        MapServerManager::checkMapStateConsistent(_map_server, _state);
        */
        MapServerManager::checkAnchorStatus(_map_server, _state);
        
        
        visualize(mono_frame_ptr->header);
        
        std::cout << "[IngvioFilter]: One loop mono callback: " << timer_mono.toc() << " (ms) " << std::endl;
    }
    
    void IngvioFilter::callbackStereoFrame(const feature_tracker::StereoFrameConstPtr& stereo_frame_ptr)
    {
        if (!_hasImageCome)
        {
            _hasImageCome = true;
            return;
        }
        
        if (!_hasInitState) return;
        
        const double target_time = stereo_frame_ptr->header.stamp.toSec();
        
        if (_state->_timestamp >= target_time) return;
        
        TicToc timer_stereo;
        
        _imu_propa->propagateAugmentAtEnd(_state, target_time);
        
        if (_state->_timestamp < target_time) return;
        
        MapServerManager::collectStereoMeas(_map_server, _state, stereo_frame_ptr);
        
        _remove_lost_update->updateStateStereo(_state, _map_server, _tri);
        
        /*
        std::cout << " ===========================================" << std::endl;
        std::cout << "[IngvioFilter]: Remove lost update time = " << timer_remove.toc() << " (ms) " << std::endl;
        */
        
        _sw_marg_update->updateStateStereo(_state, _map_server, _tri);
        
        _sw_marg_update->cleanStereoObsAtMargTime(_state, _map_server);
        
        _sw_marg_update->changeMSCKFAnchor(_state, _map_server);
        
        // _sw_marg_update->removeStereoMSCKFinMargPose(_state, _map_server);
        
        _sw_marg_update->margSwPose(_state);
        
        // std::cout << "[IngvioFilter]: Marg sw update time = " << timer_marg_sw.toc() << " (ms) " << std::endl;
        
        visualize(stereo_frame_ptr->header);
        
        std::cout << "[IngvioFilter]: One loop stereo callback: " << timer_stereo.toc() << " (ms) " << std::endl;
    }
    
    void IngvioFilter::callbackIMU(sensor_msgs::Imu::ConstPtr imu_msg)
    {
        if (!_hasImageCome) return;
        
        _imu_propa->storeImu(imu_msg);
        
        if (_imu_propa->isInit() && !_hasInitState)
        {
            Eigen::Quaterniond init_quat;
            
            if (_imu_propa->getInitQuat(init_quat))
            {
                _state->initStateAndCov(imu_msg->header.stamp.toSec(), init_quat);
                _hasInitState = true;
            }
        }
    }
    
    void IngvioFilter::visualize(const std_msgs::Header& header)
    {
        ros::Time time = header.stamp;
        
        Eigen::Isometry3d T_i2w = Eigen::Isometry3d::Identity();
        T_i2w.linear() = _state->_extended_pose->valueLinearAsMat();
        T_i2w.translation() = _state->_extended_pose->valueTrans1();
        
        Eigen::Vector3d vel = _state->_extended_pose->valueTrans2();
        
        tf::Transform T_i2w_tf;
        tf::transformEigenToTF(T_i2w, T_i2w_tf);
        _tf_pub.sendTransform(tf::StampedTransform(T_i2w_tf, time, "world", "uav"));
        
        nav_msgs::Odometry odom_w_msg;
        odom_w_msg.header.stamp = time;
        odom_w_msg.header.frame_id = "world";
        odom_w_msg.child_frame_id = "uav";
        
        tf::poseEigenToMsg(T_i2w, odom_w_msg.pose.pose);
        tf::vectorEigenToMsg(vel, odom_w_msg.twist.twist.linear);
        
        // ROS_INFO("Velocity vx = %f vy = %f vz = %f", vel(0), vel(1), vel(2));
        // ROS_INFO("Relative global position x = %f y = %f z = %f", odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z);
        
        _odom_w_pub.publish(odom_w_msg);
        
        geometry_msgs::PoseStamped pose_w;
        pose_w.header.stamp = time;
        pose_w.header.frame_id = "world";
        pose_w.pose.position.x = odom_w_msg.pose.pose.position.x;
        pose_w.pose.position.y = odom_w_msg.pose.pose.position.y;
        pose_w.pose.position.z = odom_w_msg.pose.pose.position.z;
        pose_w.pose.orientation.x = odom_w_msg.pose.pose.orientation.x;
        pose_w.pose.orientation.y = odom_w_msg.pose.pose.orientation.y;
        pose_w.pose.orientation.z = odom_w_msg.pose.pose.orientation.z;
        pose_w.pose.orientation.w = odom_w_msg.pose.pose.orientation.w;
        
        _path_w_msg.header.stamp = time;
        _path_w_msg.header.frame_id = "world";
        _path_w_msg.poses.push_back(pose_w);
        _path_w_pub.publish(_path_w_msg);
    }
}
