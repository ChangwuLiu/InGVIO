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

#include <boost/math/distributions/chi_squared.hpp>

#include <vector>
#include <map>
#include <Eigen/Dense>

namespace ingvio
{
    class Type;
    class State;
    
    class UpdateBase
    {
    public:
        UpdateBase(const int& max_dof, const double& thres) : _thres(thres)
        {
            this->setChiSquaredTable(max_dof, _thres);
        }
        
        UpdateBase(const double& thres) : _thres(thres)
        {
            this->setChiSquaredTable(150, _thres);
        }
        
        UpdateBase() : _thres(0.95)
        {
            this->setChiSquaredTable(150, _thres);
        }
        
        virtual ~UpdateBase() {}
        
        UpdateBase(const UpdateBase&) = delete;
        
    protected:
        
        double _thres;
        
        std::map<int, double> _chi_squared_table;
        
        void setChiSquaredTable(const int& max_dof, const double& thres);
        
        double whitenResidual(const std::shared_ptr<State> state,
                              const Eigen::VectorXd& res,
                              const Eigen::MatrixXd& H,
                              const std::vector<std::shared_ptr<Type>>& var_order,
                              double noise);
        
        double whitenResidual(const std::shared_ptr<State> state,
                              const Eigen::VectorXd& res,
                              const Eigen::MatrixXd& H,
                              const std::vector<std::shared_ptr<Type>>& var_order,
                              const Eigen::MatrixXd& R);
        
        virtual bool testChiSquared(const std::shared_ptr<State> state,
                                    const Eigen::VectorXd& res,
                                    const Eigen::MatrixXd& H,
                                    const std::vector<std::shared_ptr<Type>>& var_order,
                                    double noise);
        
        virtual bool testChiSquared(const std::shared_ptr<State> state,
                                    const Eigen::VectorXd& res,
                                    const Eigen::MatrixXd& H,
                                    const std::vector<std::shared_ptr<Type>>& var_order,
                                    double noise,
                                    int dof);
        
        virtual bool testChiSquared(const std::shared_ptr<State> state,
                                    const Eigen::VectorXd& res,
                                    const Eigen::MatrixXd& H,
                                    const std::vector<std::shared_ptr<Type>>& var_order,
                                    const Eigen::MatrixXd& R,
                                    int dof);
    };
}
