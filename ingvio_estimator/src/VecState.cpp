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

#include "VecState.h"

namespace ingvio
{
    void Vec3::update(const Eigen::VectorXd& dx)
    {
        assert(dx.rows() >= this->idx() + this->size());
        _vec += dx.block<3, 1>(this->idx(), 0);
    }
    
    std::shared_ptr<Vec3> Vec3::clone()
    {
        auto tmp = std::shared_ptr<Vec3>(new Vec3());
        
        tmp->setValue(this->value());
        tmp->setFej(this->fej());
        
        return tmp;
    }
    
    void Scalar::update(const Eigen::VectorXd& dx)
    {
        assert(dx.rows() >= this->idx() + this->size());
        _scalar += dx(this->idx());
    }
    
    void Scalar::setRandom()
    {
        Eigen::Vector2d tmp;
        tmp.setRandom();
        _scalar = tmp(0);
    }
    
    std::shared_ptr<Scalar> Scalar::clone()
    {
        auto tmp = std::shared_ptr<Scalar>(new Scalar());
        
        tmp->setValue(this->value());
        tmp->setFej(this->fej());
        
        return tmp;
    }
}

