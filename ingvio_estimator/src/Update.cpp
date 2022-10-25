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

#include "StateManager.h"

#include "Update.h"

namespace ingvio
{
    void UpdateBase::setChiSquaredTable(const int& max_dof, const double& thres)
    {
        for (int i = 1; i <= max_dof; ++i)
        {
            boost::math::chi_squared chi_squared_dist(i);
            this->_chi_squared_table[i] = boost::math::quantile(chi_squared_dist, thres);
        }
    }
    
    double UpdateBase::whitenResidual(const std::shared_ptr<State> state,
                                      const Eigen::VectorXd& res,
                                      const Eigen::MatrixXd& H,
                                      const std::vector<std::shared_ptr<Type>>& var_order,
                                      double noise)
    {
        assert(StateManager::checkSubOrder(state, var_order));
        
        int idx = 0;
        for (int i = 0; i < var_order.size(); ++i)
            idx += var_order[i]->size();
        
        assert(res.rows() == H.rows());
        assert(idx == H.cols());
        
        Eigen::MatrixXd small_cov = StateManager::getMarginalCov(state, var_order);
        
        Eigen::MatrixXd S = H*small_cov*H.transpose() + std::pow(noise, 2.0)*Eigen::MatrixXd::Identity(H.rows(), H.rows());
        
        return res.transpose()*S.ldlt().solve(res);
    }
    
    double UpdateBase::whitenResidual(const std::shared_ptr<State> state,
                                      const Eigen::VectorXd& res,
                                      const Eigen::MatrixXd& H,
                                      const std::vector<std::shared_ptr<Type>>& var_order,
                                      const Eigen::MatrixXd& R)
    {
        assert(StateManager::checkSubOrder(state, var_order));
        assert(R.rows() == R.cols());
        
        int idx = 0;
        for (int i = 0; i < var_order.size(); ++i)
            idx += var_order[i]->size();
        
        assert(res.rows() == H.rows());
        assert(idx == H.cols());
        
        Eigen::MatrixXd small_cov = StateManager::getMarginalCov(state, var_order);
        
        Eigen::MatrixXd S = H*small_cov*H.transpose() + R;
        
        return res.transpose()*S.ldlt().solve(res);
    }
    
    bool UpdateBase::testChiSquared(const std::shared_ptr<State> state,
                                    const Eigen::VectorXd& res,
                                    const Eigen::MatrixXd& H,
                                    const std::vector<std::shared_ptr<Type>>& var_order,
                                    double noise)
    {
        double prob = this->whitenResidual(state, res, H, var_order, noise);
        
        int dof = res.rows();
        
        if (_chi_squared_table.find(dof) == _chi_squared_table.end())
            for (int i = _chi_squared_table.rbegin()->first+1; i <= dof; ++i)
            {
                boost::math::chi_squared chi_squared_dist(i);
                this->_chi_squared_table[i] = boost::math::quantile(chi_squared_dist, _thres);
            }
        
        if (prob < _chi_squared_table.at(dof))
            return true;
        else
            return false;
    }
    
    bool UpdateBase::testChiSquared(const std::shared_ptr<State> state,
                                    const Eigen::VectorXd& res,
                                    const Eigen::MatrixXd& H,
                                    const std::vector<std::shared_ptr<Type>>& var_order,
                                    double noise,
                                    int dof)
    {
        double prob = this->whitenResidual(state, res, H, var_order, noise);
        
        if (_chi_squared_table.find(dof) == _chi_squared_table.end())
            for (int i = _chi_squared_table.rbegin()->first+1; i <= dof; ++i)
            {
                boost::math::chi_squared chi_squared_dist(i);
                this->_chi_squared_table[i] = boost::math::quantile(chi_squared_dist, _thres);
            }
            
        if (prob < _chi_squared_table.at(dof))
            return true;
        else
            return false;
    }
    
    bool UpdateBase::testChiSquared(const std::shared_ptr<State> state,
                                    const Eigen::VectorXd& res,
                                    const Eigen::MatrixXd& H,
                                    const std::vector<std::shared_ptr<Type>>& var_order,
                                    const Eigen::MatrixXd& R,
                                    int dof)
    {
        if (dof <= 0) 
            return false;
        
        double prob = this->whitenResidual(state, res, H, var_order, R);
        
        if (_chi_squared_table.find(dof) == _chi_squared_table.end())
            for (int i = _chi_squared_table.rbegin()->first+1; i <= dof; ++i)
            {
                boost::math::chi_squared chi_squared_dist(i);
                this->_chi_squared_table[i] = boost::math::quantile(chi_squared_dist, _thres);
            }
            
        if (prob < _chi_squared_table.at(dof))
            return true;
        else
            return false;
    }
}
