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
    };
}
