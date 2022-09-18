#pragma once

#include <map>
#include <memory>

#include "IngvioParams.h"

namespace ingvio
{
    struct MonoMeas;
    struct StereoMeas;
    
    class SE3;
    
    class Triangulator
    {
    public:
        Triangulator() {}
        Triangulator(const IngvioParams& filter_params)
        {
            _trans_thres = filter_params._trans_thres;
            _huber_epsilon = filter_params._huber_epsilon;
            _conv_precision = filter_params._conv_precision;
            _init_damping = filter_params._init_damping;
            _outer_loop_max_iter = filter_params._outer_loop_max_iter;
            _inner_loop_max_iter = filter_params._inner_loop_max_iter;
        }
        
        ~Triangulator() {}
        
        bool triangulateMonoObs(
            const std::map<double, std::shared_ptr<MonoMeas>>& mono_obs,
            const std::map<double, std::shared_ptr<SE3>, std::less<double>>& sw_poses_raw,
            Eigen::Vector3d& pf) const;
        
        bool triangulateStereoObs(
            const std::map<double, std::shared_ptr<StereoMeas>>& stereo_obs,
            const std::map<double, std::shared_ptr<SE3>, std::less<double>>& sw_poses_raw,
            const Eigen::Isometry3d& T_cl2cr,
            Eigen::Vector3d& pf) const;
        
    protected:
    
        double _trans_thres = 0.1;
        double _huber_epsilon = 0.01;
        double _conv_precision = 5e-7;
        double _init_damping = 1e-3;
        int _outer_loop_max_iter = 10;
        int _inner_loop_max_iter = 10;
        
        double findLongestTrans(
            const std::map<double, std::shared_ptr<SE3>, std::less<double>>& sw_poses,
            double& max_length) const;
        
        double initDepth(const Eigen::Vector2d& m1,
                         const Eigen::Vector2d& m2,
                         const std::shared_ptr<SE3> T12) const;
        
        void calcRelaSwPose(
            const std::map<double, std::shared_ptr<SE3>, std::less<double>>& sw_poses,
            std::map<double, std::shared_ptr<SE3>, std::less<double>>& rel_sw_poses) const;
        
        double calcUnitCost(const Eigen::Vector2d& meas,
                            const std::shared_ptr<SE3> rel_pose,
                            const Eigen::Vector3d& solution) const;
        
        double calcTotalCost(const std::map<double, std::shared_ptr<MonoMeas>>& mobs,
                             const std::map<double, std::shared_ptr<SE3>>& rel_poses,
                             const Eigen::Vector3d& solution) const;
        
        void calcResJacobian(const Eigen::Vector2d& meas,
                             const std::shared_ptr<SE3> rel_pose,
                             const Eigen::Vector3d& solution,
                             Eigen::Vector2d& res,
                             Eigen::Matrix<double, 2, 3>& J,
                             double& w) const;
        
        template<typename T>
        void filterCommonTimestamp(
            const std::map<double, std::shared_ptr<SE3>, std::less<double>>& poses,
            const std::map<double, std::shared_ptr<T>>& obs,
            std::map<double, std::shared_ptr<SE3>>& common_poses,
            std::map<double, std::shared_ptr<T>>& common_obs) const
        {
            common_obs.clear();
            common_poses.clear();
            
            for (const auto& item : obs)
            {
                if (poses.find(item.first) == poses.end()) continue;
                common_obs[item.first] = item.second;
                common_poses[item.first] = poses.at(item.first);
            }
        }
    };
}
