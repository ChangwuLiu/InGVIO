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

#include <tuple>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "IngvioParams.h"

#include "GnssData.h"
#include "GnssSync.h"

namespace ingvio
{
    class SE23;
    
    class GvioAligner
    {
    public:
        
        GvioAligner(const IngvioParams& filter_params)
        {
            _batch_size = filter_params._gv_align_batch_size;
            _max_iter = filter_params._gv_align_max_iter;
            _conv_epsilon = filter_params._gv_align_conv_epsilon;
            _vel_thres = filter_params._gv_align_vel_thres;
        }
        
        ~GvioAligner() = default;

        GvioAligner(const GvioAligner&) = delete;
        GvioAligner operator=(const GvioAligner&) = delete;
        
        bool isAlign() const;
        
        Eigen::Isometry3d getTecef2w() const;
        
        Eigen::Isometry3d getTw2ecef() const;
        
        Eigen::Isometry3d getTecef2enu() const;
        
        Eigen::Isometry3d getTenu2ecef() const;

        double getYawOffset() const;
        
        Eigen::Matrix3d getRecef2enu() const;
        
        Eigen::Matrix3d getRenu2ecef() const;
        
        Eigen::Matrix3d getRw2enu() const;
        
        Eigen::Matrix3d getRenu2w() const;
        
        void batchAlign(const GnssMeas& gnss_meas, 
                        const std::shared_ptr<SE23> epose, 
                        const std::vector<double>& iono);
        
    protected:
        
        int _batch_size;
        int _max_iter;
        double _conv_epsilon;       
        double _vel_thres;
        
        bool _isAligned = false;
        
        Eigen::Isometry3d _T_enu2ecef;
        double _yaw_offset;
        
        typedef std::tuple<Eigen::Vector3d, Eigen::Vector3d, std::vector<gnss_comm::ObsPtr>,
        std::vector<gnss_comm::EphemBasePtr>> AlignBufferItem;
        
        enum AlignBufferIndex {posVIO, velVIO, obsGNSS, ephemGNSS};
        
        std::vector<AlignBufferItem> _align_buffer;
        std::vector<double> _iono_params;
        
        int _num_all_meas;
        
        std::vector<std::vector<gnss_comm::SatStatePtr>> _all_sat_states;
        
        bool coarseLocalization(Eigen::Matrix<double, 7, 1>& rough_anchor_ecef);
        
        bool yawAlignment(const Eigen::Vector3d& rough_anchor_ecef, 
                          double& yaw_offset,
                          double& rcv_ddt);
        
        bool anchorRefinement(const double& yaw_offset,
                              const double& rcv_ddt,
                              const Eigen::Matrix<double, 7, 1>& rough_anchor_ecef,
                              Eigen::Matrix<double, 7, 1>& refined_anchor_ecef);
    };
    
}
