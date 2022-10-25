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
            _max_depth = filter_params._max_depth;
            _min_depth = filter_params._min_depth;
            _max_baseline_ratio = filter_params._max_baseline_ratio;
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
        
        double _max_depth = 60.0;
        double _min_depth = 0.2;
        double _max_baseline_ratio = 40.0;
        
        double findLongestTrans(
            const std::map<double, std::shared_ptr<SE3>, std::less<double>>& sw_poses,
            const std::map<double, std::shared_ptr<MonoMeas>>& mobs,
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
