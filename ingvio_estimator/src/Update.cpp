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
}
