#include <assert.h>
#include <cmath>

#include "AuxGammaFunc.h"

namespace ingvio
{
    Eigen::Matrix3d skew(const Eigen::Vector3d& vec)
    {
        Eigen::Matrix3d result;
        result << 0.0, -vec.z(),  vec.y(),
        vec.z(),      0.0, -vec.x(),
        -vec.y(),  vec.x(),      0.0;
        return result;
    }
    
    Eigen::Vector3d vee(const Eigen::Matrix3d& mat)
    {
        Eigen::Vector3d result;
        result.z() = mat(1, 0);
        result.y() = mat(0, 2);
        result.x() = mat(2, 1);
        return result;
    }
    
    Eigen::Matrix3d GammaFunc(const Eigen::Vector3d& vec, int m)
    {
        assert(m >= 0 && m <= 3);
        
        Eigen::Matrix3d result;
        
        double theta = vec.norm();
        if (std::fabs(theta) < 1e-06)
        {
            double factor = 1.0;
            switch (m)
            {
                case 3:
                    factor = 1.0/6.0;
                    break;
                case 2:
                    factor = 0.5;
                    break;
                default:
                    break;
            }
            return factor*Eigen::Matrix3d::Identity();
        }
        
        Eigen::Vector3d n = vec.normalized();
        Eigen::Matrix3d n_cross = skew(n);
        Eigen::Matrix3d n_cross2 = n_cross*n_cross;
        
        double factor0, factor1, factor2;
        double sintheta = std::sin(theta);
        double costheta = std::cos(theta);
        
        switch (m)
        {
            case 1:
            {
                factor0 = 1.0;
                factor1 = (1.0 - costheta)/theta;
                factor2 = (theta - sintheta)/theta;
                break;
            }
            case 2:
            {
                factor0 = 0.5;
                factor1 = (theta-sintheta)/std::pow(theta, 2);
                factor2 = (std::pow(theta, 2)+2.0*costheta-2.0)/(2.0*std::pow(theta, 2));
                break;
            }
            case 3:
            {
                factor0 = 1.0/6.0;
                double theta3 = std::pow(theta, 3);
                factor1 = (std::pow(theta, 2)+2.0*costheta-2.0)/(2.0*theta3);
                factor2 = (theta3-6.0*theta+6.0*sintheta)/(6.0*theta3);
                break;
            }
            default:
            {
                factor0 = 1.0;
                factor1 = sintheta;
                factor2 = 1.0 - costheta;
                break;
            }
        }
        
        result = factor0*Eigen::Matrix3d::Identity() + factor1*n_cross + factor2*n_cross2;
        return result;
    }
}
