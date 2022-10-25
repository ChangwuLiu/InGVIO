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

#include <Eigen/Geometry>

#include "AuxGammaFunc.h"
#include "VecState.h"

namespace ingvio
{
    class SO3 : public Type
    {
    public:
        SO3() : Type(3)
        {
            _q.setIdentity();
            _q_fej.setIdentity();
            _rot.setIdentity();
            _rot_fej.setIdentity();
        }
        
        ~SO3() {}
        
        void update(const Eigen::VectorXd& dx) override;
        
        void setIdentity() override;
        
        void setRandom() override;
        
        const Eigen::Quaterniond& valueAsQuat() const
        { return _q; }
        
        const Eigen::Matrix3d& valueAsMat() const
        { return _rot; }
        
        const Eigen::Quaterniond& fejAsQuat() const
        { return _q_fej; }
        
        const Eigen::Matrix3d& fejAsMat() const
        { return _rot_fej; }
        
        void setValueByQuat(const Eigen::Quaterniond& other_quat);
        
        void setValueByMat(const Eigen::Matrix3d& other_mat);
        
        void setFejByQuat(const Eigen::Quaterniond& other_quat);
        
        void setFejByMat(const Eigen::Matrix3d& other_mat);
        
        std::shared_ptr<SO3> clone();
        
    protected:
        Eigen::Quaterniond _q;
        Eigen::Matrix3d _rot;
        
        Eigen::Quaterniond _q_fej;
        Eigen::Matrix3d _rot_fej;
    };
    
    class SE3 : public Type
    {
    public:
        SE3() : Type(6)
        {
            _q.setIdentity();
            _q_fej = _q;
            
            _rot.setIdentity();
            _rot_fej = _rot;
            
            _vec.setZero();
            _vec_fej = _vec;
        }
        
        ~SE3() {}
        
        void update(const Eigen::VectorXd& dx) override;
        
        void setIdentity() override;
        
        void setRandom() override;
        
        const Eigen::Quaterniond& valueLinearAsQuat() const
        { return _q; }
        
        const Eigen::Matrix3d& valueLinearAsMat() const
        { return _rot; }
        
        const Eigen::Vector3d& valueTrans() const
        { return _vec; }
        
        Eigen::Isometry3d copyValueAsIso() const;
        
        const Eigen::Quaterniond& fejLinearAsQuat() const
        { return _q_fej; }
        
        const Eigen::Matrix3d& fejLinearAsMat() const
        { return _rot_fej; }
        
        const Eigen::Vector3d& fejTrans() const
        { return _vec_fej; }
        
        Eigen::Isometry3d copyFejAsIso() const;
        
        void setValueLinearByQuat(const Eigen::Quaterniond& other_quat);
        
        void setValueLinearByMat(const Eigen::Matrix3d& other_mat);
        
        void setValueTrans(const Eigen::Vector3d& other_vec)
        {  _vec = other_vec;  }
        
        void setValueByIso(const Eigen::Isometry3d& other_iso);
        
        void setFejLinearByQuat(const Eigen::Quaterniond& other_quat);
        
        void setFejLinearByMat(const Eigen::Matrix3d& other_mat);
        
        void setFejTrans(const Eigen::Vector3d& other_vec)
        {  _vec_fej = other_vec;  }
        
        void setFejByIso(const Eigen::Isometry3d& other_iso);
        
        std::shared_ptr<SE3> clone();
        
    protected:
        Eigen::Quaterniond _q;
        Eigen::Matrix3d _rot;
        
        Eigen::Quaterniond _q_fej;
        Eigen::Matrix3d _rot_fej;
        
        Eigen::Vector3d _vec;
        Eigen::Vector3d _vec_fej;
    };
    
    class SE23 : public Type
    {
    public:
        SE23() : Type(9)
        {
            _q.setIdentity();
            _q_fej = _q;
            
            _rot.setIdentity();
            _rot_fej = _rot;
            
            _vec1.setZero();
            _vec1_fej = _vec1;
            
            _vec2.setZero();
            _vec2_fej = _vec2;
        }
        
        ~SE23() {}
        
        void update(const Eigen::VectorXd& dx) override;
        
        void setIdentity() override;
        
        void setRandom() override;
        
        const Eigen::Quaterniond& valueLinearAsQuat() const
        { return _q; }
        
        const Eigen::Matrix3d& valueLinearAsMat() const
        { return _rot; }
        
        const Eigen::Vector3d& valueTrans1() const
        { return _vec1; }
        
        const Eigen::Vector3d& valueTrans2() const
        { return _vec2; }
        
        Eigen::Isometry3d copyValueAsIso() const;
        
        const Eigen::Quaterniond& fejLinearAsQuat() const
        {  return _q_fej; }
        
        const Eigen::Matrix3d& fejLinearAsMat() const
        {  return _rot_fej; }
        
        const Eigen::Vector3d& fejTrans1() const
        {  return _vec1_fej; }
        
        const Eigen::Vector3d& fejTrans2() const
        {  return _vec2_fej; }
        
        Eigen::Isometry3d copyFejAsIso() const;
        
        void setValueLinearByQuat(const Eigen::Quaterniond& other_quat);
        
        void setValueLinearByMat(const Eigen::Matrix3d& other_mat);
        
        void setValueTrans1(const Eigen::Vector3d& other_vec)
        {  _vec1 = other_vec;  }
        
        void setValueTrans2(const Eigen::Vector3d& other_vec)
        {  _vec2 = other_vec;  }
        
        void setFejLinearByQuat(const Eigen::Quaterniond& other_quat);
        
        void setFejLinearByMat(const Eigen::Matrix3d& other_mat);
        
        void setFejTrans1(const Eigen::Vector3d& other_vec)
        {  _vec1_fej = other_vec;  }
        
        void setFejTrans2(const Eigen::Vector3d& other_vec)
        {  _vec2_fej = other_vec;  }
        
        std::shared_ptr<SE23> clone();
        
    protected:
        Eigen::Quaterniond _q;
        Eigen::Quaterniond _q_fej;
        
        Eigen::Matrix3d _rot;
        Eigen::Matrix3d _rot_fej;
        
        Eigen::Vector3d _vec1;
        Eigen::Vector3d _vec1_fej;
        
        Eigen::Vector3d _vec2;
        Eigen::Vector3d _vec2_fej;
    };
    
}
