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

#include <iostream>
#include "PoseState.h"

namespace ingvio
{
    class AnchoredLandmark : public Type
    {
    public:
        enum LmRep {XYZ, INV_DEPTH, BEARING};
        
        AnchoredLandmark() : Type(3)
        {
            _pos_rep.setZero();
            _pos_rep_fej.setZero();
            
            _pos_xyz.setZero();
            _pos_xyz_fej.setZero();
            
            _landmark_param_type = INV_DEPTH;
            _has_rep_set = false;
            
            _anchored_pose.reset();
        }
        
        ~AnchoredLandmark() {}
        
        void update(const Eigen::VectorXd& dx) override;
        
        void setIdentity() override;
        
        void setRandom() override;
        
        void setRepType(const AnchoredLandmark::LmRep& other_rep_type);
        
        static void switchRepXyz2Invd(const Eigen::Vector3d& xyz, Eigen::Vector3d& invd);
        
        static void switchRepInvd2Xyz(const Eigen::Vector3d& invd, Eigen::Vector3d& xyz);
        
        static void switchRepXyz2Bear(const Eigen::Vector3d& xyz, Eigen::Vector3d& bear);
        
        static void switchRepBear2Xyz(const Eigen::Vector3d& bear, Eigen::Vector3d& xyz);
        
        void resetAnchoredPose(std::shared_ptr<SE3> new_anchored_pose = nullptr,
                               bool isUpdateRepValue = false);
        
        const std::shared_ptr<SE3> getAnchoredPose() const
        { return _anchored_pose; }
        
        const Eigen::Vector3d& valuePosRep() const
        { return _pos_rep; }
        
        const Eigen::Vector3d& valuePosXyz() const
        { return _pos_xyz; }
        
        const Eigen::Vector3d& fejPosRep() const
        { return _pos_rep_fej; }
        
        const Eigen::Vector3d& fejPosXyz() const
        { return _pos_xyz_fej; }
        
        const LmRep& checkLmRepType() const
        { return _landmark_param_type; }
        
        void setValuePosXyz(const Eigen::Vector3d& xyz_world);
        
        void setValuePosRep(const Eigen::Vector3d& pos_rep_body);
        
        void setFejPosXyz(const Eigen::Vector3d& xyz_world_fej);
        
        void setFejPosRep(const Eigen::Vector3d& pos_rep_body_fej);
        
        std::shared_ptr<AnchoredLandmark> clone();
        
    protected:
        LmRep _landmark_param_type;
        
        Eigen::Vector3d _pos_rep;
        Eigen::Vector3d _pos_rep_fej;
        
        Eigen::Vector3d _pos_xyz;
        Eigen::Vector3d _pos_xyz_fej;
        
        std::shared_ptr<SE3> _anchored_pose;
        
        bool _has_rep_set;
    };
}

