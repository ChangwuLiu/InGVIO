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

#include <memory>
#include <Eigen/Core>

namespace ingvio
{
    // Acknowledgements:
    // The idea of type-based index system is inspired by the realization of OpenVINS
    // see https://github.com/rpng/open_vins
    
    class Type
    {
    public:
        Type(int size) { _size = size; }
        virtual ~Type(){}
        
        virtual void set_cov_idx(int new_cov_idx)
        {
            _idx = new_cov_idx;
        }
        
        int idx() const { return _idx; }
        int size() const {return _size; }
        
        virtual void update(const Eigen::VectorXd& dx) = 0;
        
        virtual void setIdentity() = 0;
        virtual void setRandom() = 0;
        
    protected:
        int _idx = -1;
        int _size = -1;
    };
    
    class Vec3 : public Type
    {
    public:
        Vec3() : Type(3)
        {
            _vec.setZero();
            _vec_fej.setZero();
        }
        
        ~Vec3() {}
        
        void update(const Eigen::VectorXd& dx) override;
        
        void setIdentity() override
        {
            _vec.setZero();
        }
        void setRandom() override
        {
            _vec.setRandom();
        }
        
        const Eigen::Vector3d& value() const 
        { return _vec; }
        
        const Eigen::Vector3d& fej() const 
        { return _vec_fej; }
        
        void setValue(const Eigen::Vector3d& other_vec)
        { _vec = other_vec; }
        
        void setFej(const Eigen::Vector3d& other_vec_fej) 
        { _vec_fej = other_vec_fej; }
        
        std::shared_ptr<Vec3> clone();
        
    protected:
        Eigen::Vector3d _vec;
        Eigen::Vector3d _vec_fej;
    };
    
    class Scalar : public Type
    {
    public:
        Scalar() : Type(1)
        {
            _scalar = 0.0;
            _scalar_fej = 0.0;
        }
        
        ~Scalar() {}
        
        void update(const Eigen::VectorXd& dx) override;
        
        void setIdentity() override
        {
            _scalar = 0.0;
        }
        
        void setRandom() override;
        
        const double& value() const
        { return _scalar; }
        
        const double& fej() const 
        { return _scalar_fej; }
        
        void setValue(const double& other_scalar) 
        { _scalar = other_scalar; }
        
        void setFej(const double& other_scalar_fej) 
        { _scalar_fej = other_scalar_fej; }
        
        std::shared_ptr<Scalar> clone();
        
    protected:
        double _scalar;
        double _scalar_fej;
    };
}
