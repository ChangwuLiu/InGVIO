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

#include <gnss_comm/gnss_spp.hpp>

#include "PoseState.h"

#include "Color.h"

#include "GvioAligner.h"

namespace ingvio
{
    bool GvioAligner::isAlign() const
    {  return _isAligned;  }
    
    Eigen::Isometry3d GvioAligner::getTecef2w() const
    {
        Eigen::Isometry3d T_enu2w = Eigen::Isometry3d::Identity();
        T_enu2w.linear() = this->getRenu2w();
        
        return T_enu2w*this->getTecef2enu();
    }
    
    Eigen::Isometry3d GvioAligner::getTw2ecef() const
    {
        Eigen::Isometry3d T_w2enu = Eigen::Isometry3d::Identity();
        T_w2enu.linear() = this->getRw2enu();
        
        return this->getTenu2ecef()*T_w2enu;
    }
    
    double GvioAligner::getYawOffset() const
    {
        return _yaw_offset;
    }
    
    Eigen::Isometry3d GvioAligner::getTecef2enu() const
    {
        return _T_enu2ecef.inverse();
    }
    
    Eigen::Isometry3d GvioAligner::getTenu2ecef() const
    {
        return _T_enu2ecef;
    }
    
    Eigen::Matrix3d GvioAligner::getRecef2enu() const
    {
        return _T_enu2ecef.linear().transpose();
    }
    
    Eigen::Matrix3d GvioAligner::getRenu2ecef() const
    {
        return _T_enu2ecef.linear();
    }
    
    Eigen::Matrix3d GvioAligner::getRw2enu() const
    {
        return Eigen::AngleAxisd(_yaw_offset, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    }
    
    Eigen::Matrix3d GvioAligner::getRenu2w() const
    {
        return this->getRw2enu().transpose();
    }
    
    void GvioAligner::batchAlign(const GnssMeas& gnss_meas, 
                                 const std::shared_ptr<SE23> epose, 
                                 const std::vector<double>& iono)
    {
        if (_isAligned) return;
        
        _iono_params.clear();
        std::copy(iono.begin(), iono.end(), std::back_inserter(_iono_params));
        
        if (_align_buffer.size() < _batch_size)
            _align_buffer.push_back(std::make_tuple(epose->valueTrans1(),
                                                    epose->valueTrans2(),
                                                    gnss_meas.first,
                                                    gnss_meas.second));
        else
        {
            Eigen::Vector2d hor_vel = Eigen::Vector2d::Zero();
            
            std::for_each(_align_buffer.begin(), _align_buffer.end(), 
                [&hor_vel](const AlignBufferItem& item)
                {
                    hor_vel += std::get<velVIO>(item).head<2>().cwiseAbs();
                }
            );
            
            hor_vel /= _align_buffer.size();
            
            if (hor_vel.norm() <= _vel_thres)
            {
                std::cout << color::setYellow << "[GvioAligner]: Horizontal velocity excitation not enough, waiting and restart ..." << color::resetColor << std::endl;
                
                _align_buffer.clear();
                _isAligned = false;
                
                _num_all_meas = 0;
                _all_sat_states.clear();
                
                return;
            }
            
            std::cout << color::setBlue << "[GvioAligner]: Start batch alignment ..." << color::resetColor << std::endl;
            
            _num_all_meas = 0;
            _all_sat_states.clear();
            
            for (const auto& align_item : _align_buffer)
            {
                _all_sat_states.push_back(
                    gnss_comm::sat_states(std::get<obsGNSS>(align_item), 
                                          std::get<ephemGNSS>(align_item)));
                
                _num_all_meas += std::get<obsGNSS>(align_item).size();
            }
            
            Eigen::Matrix<double, 7, 1> rough_anchor_ecef;
            rough_anchor_ecef.setZero();
            
            if (!coarseLocalization(rough_anchor_ecef))
            {
                _align_buffer.clear();
                _isAligned = false;
                
                _num_all_meas = 0;
                _all_sat_states.clear();
                
                return;
            }
            
            double yaw_offset = 0.0;
            double rcv_ddt = 0.0;
            
            if (!yawAlignment(rough_anchor_ecef.head<3>(), yaw_offset, rcv_ddt))
            {
                _align_buffer.clear();
                _isAligned = false;
                
                _num_all_meas = 0;
                _all_sat_states.clear();
                
                return;
            }
            
            Eigen::Matrix<double, 7, 1> refined_anchor_ecef;
            refined_anchor_ecef.setZero();
            
            if (!anchorRefinement(yaw_offset, rcv_ddt, rough_anchor_ecef, refined_anchor_ecef))
            {
                _align_buffer.clear();
                _isAligned = false;
                
                _num_all_meas = 0;
                _all_sat_states.clear();
                
                return;
            }
            
            _align_buffer.clear();
            _num_all_meas = 0;
            _all_sat_states.clear();
            _isAligned = true;
            
            Eigen::Matrix3d R_enu2ecef = gnss_comm::ecef2rotation(refined_anchor_ecef.head<3>());
            
            _T_enu2ecef.linear() = R_enu2ecef;
            _T_enu2ecef.translation() = refined_anchor_ecef.head<3>();
            
            _yaw_offset = yaw_offset;
            
            std::cout << color::setGreen << "[GvioAligner]: Yaw offset from north = " << yaw_offset*180.0/M_PI << " (deg)" << color::resetColor << std::endl;
            
            std::cout << color::setGreen << "[GvioAligner]: Refined anchor in ECEF = " << refined_anchor_ecef.head<3>().transpose() << " (m)" << color::resetColor << std::endl;
        }
    }
    
    bool GvioAligner::coarseLocalization(Eigen::Matrix<double, 7, 1>& rough_anchor_ecef)
    {
        rough_anchor_ecef.setZero();
        
        std::vector<gnss_comm::ObsPtr> accumObs;
        std::vector<gnss_comm::EphemBasePtr> accumEphems;
        
        for (const auto& align_item : _align_buffer)
        {
            std::copy(std::get<obsGNSS>(align_item).begin(),
                      std::get<obsGNSS>(align_item).end(),
                      std::back_inserter(accumObs));
            
            std::copy(std::get<ephemGNSS>(align_item).begin(),
                      std::get<ephemGNSS>(align_item).end(),
                      std::back_inserter(accumEphems));
        }
        
        Eigen::Matrix<double, 7, 1> xyzt = gnss_comm::psr_pos(accumObs, accumEphems, this->_iono_params);
        
        if (xyzt.hasNaN() || xyzt.topRows<3>().norm() < 1e-06)
        {
            std::cout << color::setRed << "[GvioAligner]: Coarse anchor localization failed!" << color::resetColor << std::endl;
            return false;
        }
        
        for (int i = 0; i < 4; ++i)
            if (std::fabs(xyzt(i+3, 0)) < 1.0)
                xyzt(i+3, 0) = 0.0;
            
        rough_anchor_ecef = xyzt;
        return true;
    }
    
    bool GvioAligner::yawAlignment(const Eigen::Vector3d& rough_anchor_ecef, 
                                   double& yaw_offset,
                                   double& rcv_ddt)
    {
        yaw_offset = 0.0;
        rcv_ddt = 0.0;
        
        double estYaw = 0.0;
        double estRcvDdt = 0.0;
        
        const Eigen::Matrix3d roughRenu2ecef = gnss_comm::ecef2rotation(rough_anchor_ecef);
        
        int iter = 0;
        
        double delta_yaw_norm = 1.0;
        
        while (iter <= _max_iter && delta_yaw_norm > _conv_epsilon)
        {
            Eigen::MatrixXd A(_num_all_meas, 2);
            A.setZero();
            A.col(1).setOnes();
            
            Eigen::VectorXd b(_num_all_meas);
            b.setZero();
            
            Eigen::Matrix3d Rw2enu(Eigen::AngleAxisd(estYaw, Eigen::Vector3d::UnitZ()));
            
            Eigen::Matrix3d dotC3;
            dotC3 << -std::sin(estYaw), -std::cos(estYaw), 0.0,
                      std::cos(estYaw), -std::sin(estYaw), 0.0,
                                   0.0,               0.0, 0.0;
                                   
            int row_cnt = 0;
            for (int i = 0; i < _align_buffer.size(); ++i)
            {
                auto& align_item = _align_buffer[i]; 
                
                Eigen::Vector4d vel_ddt_ecef;
                
                vel_ddt_ecef.head<3>() = roughRenu2ecef*Rw2enu*std::get<velVIO>(align_item);
                vel_ddt_ecef(3) = estRcvDdt;
                
                Eigen::VectorXd epochRes;
                Eigen::MatrixXd epochJ; 
                
                gnss_comm::dopp_res(vel_ddt_ecef, rough_anchor_ecef, 
                                    std::get<obsGNSS>(align_item), _all_sat_states[i],
                                    epochRes, epochJ);
                
                b.segment(row_cnt, std::get<obsGNSS>(align_item).size()) = epochRes;
                A.block(row_cnt, 0, std::get<obsGNSS>(align_item).size(), 1) = 
                epochJ.leftCols(3)*roughRenu2ecef*dotC3*std::get<velVIO>(align_item);
                
                row_cnt += std::get<obsGNSS>(align_item).size();
            }
            
            Eigen::VectorXd delta_yaw = -(A.transpose()*A).inverse()*A.transpose()*b;
            
            estYaw += delta_yaw(0);
            estRcvDdt += delta_yaw(1);
            
            delta_yaw_norm = delta_yaw.norm();
            
            ++iter;
        }
        
        if (iter > _max_iter)
        {
            std::cout << color::setRed << "[GvioAligner]: Yaw alignment reaches max iter, failed!" << color::resetColor << std::endl;
            
            return false;
        }
        
        yaw_offset = estYaw;
        if (yaw_offset > M_PI)
            yaw_offset -= std::floor(estYaw/(2.0*M_PI)+0.5)*(2.0*M_PI);
        else if (yaw_offset < -M_PI)
            yaw_offset -= std::ceil(estYaw/(2.0*M_PI)-0.5)*(2.0*M_PI);
        
        rcv_ddt = estRcvDdt;
        
        return true;
    }
    
    bool GvioAligner::anchorRefinement(const double& yaw_offset,
                                       const double& rcv_ddt,
                                       const Eigen::Matrix<double, 7, 1>& rough_anchor_ecef,
                                       Eigen::Matrix<double, 7, 1>& refined_anchor_ecef)
    {
        refined_anchor_ecef = rough_anchor_ecef;
        
        const Eigen::Matrix3d alignedRw2enu = Eigen::AngleAxisd(yaw_offset, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        
        auto getRefinedRw2ecef = [&alignedRw2enu](const Eigen::Matrix<double, 7, 1>& anchor_ecef)
        {
            return gnss_comm::ecef2rotation(anchor_ecef.head<3>())*alignedRw2enu;
        };
        
        std::vector<Eigen::Matrix<double, 7, 1>> spp(_align_buffer.size());
        
        for (int i = 0; i < _align_buffer.size(); ++i)
        {
            const auto& align_item = _align_buffer[i];
            
            Eigen::Matrix<double, 7, 1> xyzt = gnss_comm::psr_pos(
                std::get<obsGNSS>(align_item),
                std::get<ephemGNSS>(align_item),
                this->_iono_params);
            
            if (xyzt.hasNaN() || xyzt.head<3>().norm() < 1e-03)
            {
                std::cout << color::setRed << "[GvioAligner]: Anchor refinement failure due to unable to conduct SPP!" << color::resetColor << std::endl;
                
                return false;
            }
            
            spp[i] = xyzt;
        }
        
        int iter_refine = 0;
        
        while (iter_refine <= _max_iter)
        {
            Eigen::Vector3d anchor_pos = Eigen::Vector3d::Zero();
            
            for (int i = 0; i < _align_buffer.size(); ++i)
                anchor_pos += spp[i].head<3>() - getRefinedRw2ecef(refined_anchor_ecef)*std::get<posVIO>(_align_buffer[i]);
            
            anchor_pos /= _align_buffer.size();
            
            Eigen::Vector3d dx = anchor_pos - refined_anchor_ecef.head<3>();
            
            refined_anchor_ecef.head<3>() = anchor_pos;
            
            if (dx.norm() > _conv_epsilon)
                break;
            
            ++iter_refine;
        }
        
        if (iter_refine > _max_iter)
        {
            std::cout << color::setRed << "[GvioAligner]: Anchor refinement failure reaching max iter!" << color::resetColor << std::endl;
            
            return false;
        }
        
        refined_anchor_ecef.tail<4>() = spp.rbegin()->tail<4>();
        
        return true;
    }
}
