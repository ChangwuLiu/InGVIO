#include "GenerateNoise.h"

#include <cmath>

namespace ingvio_test
{
    double generateGaussRandom(const double& mean, const double& std)
    {
        double z1;
        do
        {
            int x1, x2;
            x1 = rand();
            x2 = rand();
            double y1 = static_cast<double>(x1 % 100)/100.0;
            double y2 = static_cast<double>(x2 % 100)/100.0;
            z1 = std::sqrt(-2.0*std::log(y1))*std::sin(2.0*M_PI*y2);
        }
        while (std::isnan(mean+z1*std) || std::isinf(mean+z1*std));
        return mean+z1*std;
    }
    
    bool isSPD(const Eigen::MatrixXd& cov)
    {
        assert(cov.rows() == cov.cols());
        
        Eigen::VectorXd diags = cov.diagonal();
        
        for (int i = 0; i < diags.rows(); ++i)
            if (diags(i, 0) < 0.0)
                return false;
            
        return true;
    }
}
