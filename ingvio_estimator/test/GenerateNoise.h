#pragma once

#include <Eigen/Core>

namespace ingvio_test
{
    extern double generateGaussRandom(const double& mean, const double& std);
    
    extern bool isSPD(const Eigen::MatrixXd& cov);
}
