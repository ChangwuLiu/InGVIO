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

#include <Eigen/Core>

namespace ingvio
{
    extern Eigen::Matrix3d skew(const Eigen::Vector3d& vec);
    
    extern Eigen::Vector3d vee(const Eigen::Matrix3d& mat);
    
    extern Eigen::Matrix3d GammaFunc(const Eigen::Vector3d& vec, int m = 0);
    
    extern Eigen::Matrix3d Psi1Func(const Eigen::Vector3d& tilde_omega,
                                    const Eigen::Vector3d& tilde_acc,
                                    double dt);
    
    extern Eigen::Matrix3d Psi2Func(const Eigen::Vector3d& tilde_omega,
                                    const Eigen::Vector3d& tilde_acc,
                                    double dt);
}
