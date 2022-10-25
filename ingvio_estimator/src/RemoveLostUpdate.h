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

#include "MapServer.h"

#include "Update.h"

namespace ingvio
{
    class Triangulator;
    class IngvioParams;
    
    class RemoveLostUpdate : public UpdateBase
    {
    public:
        using UpdateBase::UpdateBase;
        
        RemoveLostUpdate(const IngvioParams& filter_params) : UpdateBase(filter_params._chi2_max_dof, filter_params._chi2_thres),
        _max_valid_ids(20),
        _noise(filter_params._visual_noise)
        {}
        
        RemoveLostUpdate(const RemoveLostUpdate&) = delete;
        RemoveLostUpdate operator=(const RemoveLostUpdate&) = delete;
        
        virtual ~RemoveLostUpdate() {}
        
        void updateStateMono(std::shared_ptr<State> state,
                             std::shared_ptr<MapServer> map_server,
                             std::shared_ptr<Triangulator> tri);
        
        void updateStateStereo(std::shared_ptr<State> state,
                               std::shared_ptr<MapServer> map_server,
                               std::shared_ptr<Triangulator> tri);
        
    protected:
        
        void calcResJacobianSingleFeatAllMonoObs(
            const std::shared_ptr<FeatureInfo> feature_info,
            const std::map<double, std::shared_ptr<SE3>>& sw_poses,
            std::vector<std::shared_ptr<Type>>& block_var_order,
            std::map<std::shared_ptr<Type>, int>& block_index_map,
            Eigen::VectorXd& res_block,
            Eigen::MatrixXd& H_block);
        
        void calcResJacobianSingleFeatAllStereoObs(
            const std::shared_ptr<FeatureInfo> feature_info, 
            const std::map<double, std::shared_ptr<SE3>>& sw_poses,
            const Eigen::Isometry3d& T_cl2cr,
            std::vector<std::shared_ptr<Type>>& block_var_order,
            std::map<std::shared_ptr<Type>, int>& block_index_map,
            Eigen::VectorXd& res_block,
            Eigen::MatrixXd& H_block);
        
        int _max_valid_ids;        
        double _noise;
    }; 
}
