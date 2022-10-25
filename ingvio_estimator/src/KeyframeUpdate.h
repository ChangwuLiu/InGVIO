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

#include "IngvioParams.h"
#include "MapServer.h"

#include "Update.h"

namespace ingvio
{
    class State;
    class Triangulator;
    
    class KeyframeUpdate : public UpdateBase
    {
    public:
        using UpdateBase::UpdateBase;
        
        KeyframeUpdate(const IngvioParams& filter_params) :
        UpdateBase(filter_params._chi2_max_dof, filter_params._chi2_thres),
        _noise(filter_params._visual_noise),
        _max_sw_poses(filter_params._max_sw_clones)
        {}
        
        KeyframeUpdate(const KeyframeUpdate&) = delete;
        KeyframeUpdate operator=(const KeyframeUpdate&) = delete;
        
        virtual ~KeyframeUpdate() {}
        
        void getMargKfs(const std::shared_ptr<State> state,
                        std::vector<double>& marg_kfs);
        
        void margSwPose(std::shared_ptr<State> state);
        
        void changeMSCKFAnchor(std::shared_ptr<State> state,
                               std::shared_ptr<MapServer> map_server);
        
        void updateStateMono(std::shared_ptr<State> state,
                             std::shared_ptr<MapServer> map_server,
                             std::shared_ptr<Triangulator> tri);
        
        void cleanMonoObsAtMargTime(std::shared_ptr<State> state,
                                    std::shared_ptr<MapServer> map_server);
        
        void updateStateStereo(std::shared_ptr<State> state,
                               std::shared_ptr<MapServer> map_server,
                               std::shared_ptr<Triangulator> tri);
        
        void cleanStereoObsAtMargTime(std::shared_ptr<State> state,
                                      std::shared_ptr<MapServer> map_server);
        
    protected:
        double _noise;
        int _max_sw_poses;
        
        double _timestamp = -1.0;
        std::vector<double> _kfs;
        
        static int _select_cnt;
        
        void generateSwVarOrder(const std::shared_ptr<State> state,
                                const std::vector<double>& selected_timestamps,
                                std::vector<std::shared_ptr<SE3>>& sw_var_order,
                                std::map<std::shared_ptr<SE3>, int>& sw_index,
                                std::vector<std::shared_ptr<Type>>& sw_var_type);
        
        std::shared_ptr<SE3> calcResJacobianSingleFeatSelectedMonoObs(
            const std::shared_ptr<FeatureInfo> feature_info,
            const std::map<double, std::shared_ptr<SE3>>& sw_poses,
            const std::vector<std::shared_ptr<SE3>>& sw_var_order,
            const std::map<std::shared_ptr<SE3>, int>& sw_index_map,
            const std::vector<double>& selected_timestamps,
            Eigen::VectorXd& res_block,
            Eigen::MatrixXd& H_block,
            Eigen::MatrixXd& H_anchor_block);
        
        std::shared_ptr<SE3> calcResJacobianSingleFeatSelectedStereoObs(
            const std::shared_ptr<FeatureInfo> feature_info, 
            const std::map<double, std::shared_ptr<SE3>>& sw_poses,
            const std::vector<std::shared_ptr<SE3>>& sw_var_order,
            const std::map<std::shared_ptr<SE3>, int>& sw_index_map,
            const std::vector<double>& selected_timestamps,
            const Eigen::Isometry3d& T_cl2cr,
            Eigen::VectorXd& res_block,
            Eigen::MatrixXd& H_block,
            Eigen::MatrixXd& H_anchor_block);
        
    };
}
