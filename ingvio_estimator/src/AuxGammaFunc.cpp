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
    
    Eigen::Matrix3d Psi1Func(const Eigen::Vector3d& tilde_omega,
                             const Eigen::Vector3d& tilde_acc,
                             double dt)
    {
        if ((tilde_omega*dt).norm() < 1e-08) return Eigen::Matrix3d::Zero();
        
        Eigen::Matrix3d M1 = skew(tilde_acc)*GammaFunc(-tilde_omega*dt, 2)*std::pow(dt, 2.0);
        
        Eigen::Matrix3d WA = skew(tilde_omega)*skew(tilde_acc);
        
        Eigen::Matrix3d WAW = WA*skew(tilde_omega);
        
        Eigen::Matrix3d WAW2 = WAW*skew(tilde_omega);
        
        Eigen::Matrix3d W2A = skew(tilde_omega)*WA;
        
        Eigen::Matrix3d W2AW = W2A*skew(tilde_omega);
        
        Eigen::Matrix3d W2AW2 = W2AW*skew(tilde_omega);
        
        double eta = tilde_omega.norm();
        
        double xi = eta*dt;
        double xi2 = std::pow(xi, 2.0);
        
        double sin_xi = std::sin(xi);
        double cos_xi = std::cos(xi);
        
        double sin_2xi = std::sin(2*xi);
        double cos_2xi = std::cos(2*xi);
        
        double eta3 = std::pow(eta, 3);
        double eta4 = eta*eta3;
        double eta5 = eta*eta4;
        double eta6 = eta*eta5;
        
        double c1 = (sin_xi-xi*cos_xi)/eta3;
        
        double c2 = (cos_2xi-4*cos_xi+3)/(4*eta4);
        
        double c3 = (4*sin_xi+sin_2xi-4*xi*cos_xi-2*xi)/(4*eta5);
        
        double c4 = (xi2-2*xi*sin_xi-2*cos_xi+2)/(2*eta4);
        
        double c5 = (6*xi-8*sin_xi+sin_2xi)/(4*eta5);
        
        double c6 = (2*xi2-4*xi*sin_xi-cos_2xi+1)/(4*eta6);
        
        Eigen::Matrix3d result = M1*(c1*WA + c2*WAW + c3*WAW2 + c4*W2A + c5*W2AW + c6*W2AW2);
        
        return result;
    }
    
    Eigen::Matrix3d Psi2Func(const Eigen::Vector3d& tilde_omega,
                             const Eigen::Vector3d& tilde_acc,
                             double dt)
    {
        if ((tilde_omega*dt).norm() < 1e-07)
            return Eigen::Matrix3d::Zero();
        
        Eigen::Matrix3d M1 = skew(tilde_acc)*GammaFunc(-tilde_omega*dt, 3)*std::pow(dt, 3);
        
        Eigen::Matrix3d WA = skew(tilde_omega)*skew(tilde_acc);
        
        Eigen::Matrix3d WAW = WA*skew(tilde_omega);
        
        Eigen::Matrix3d WAW2 = WAW*skew(tilde_omega);
        
        Eigen::Matrix3d W2A = skew(tilde_omega)*WA;
        
        Eigen::Matrix3d W2AW = W2A*skew(tilde_omega);
        
        Eigen::Matrix3d W2AW2 = W2AW*skew(tilde_omega);
        
        double eta = tilde_omega.norm();
        
        double xi = eta*dt;
        
        double xi2 = std::pow(xi, 2.0);
        double xi3 = xi*xi2;
        
        double sin_xi = std::sin(xi);
        double cos_xi = std::cos(xi);
        
        double sin_2xi = std::sin(2*xi);
        double cos_2xi = std::cos(2*xi);
        
        double eta3 = std::pow(eta, 3);
        double eta4 = eta*eta3;
        double eta5 = eta*eta4;
        double eta6 = eta*eta5;
        double eta7 = eta*eta6;
        
        double c1 = (xi*sin_xi+2*cos_xi-2)/eta4;
        
        double c2 = (6*xi-8*sin_xi+sin_2xi)/(8*eta5);
        
        double c3 = (2*xi2+8*xi*sin_xi+16*cos_xi+cos_2xi-17)/(8*eta6);
        
        double c4 = (xi3+6*xi-12*sin_xi+6*xi*cos_xi)/(6*eta5);
        
        double c5 = (6*xi2+16*cos_xi-cos_2xi-15)/(8*eta6);
        
        double c6 = (4*xi3+6*xi-24*sin_xi-3*sin_2xi+24*xi*cos_xi)/(24*eta7);
        
        Eigen::Matrix3d result;
        
        result = M1*(c1*WA + c2*WAW + c3*WAW2 + c4*W2A + c5*W2AW + c6*W2AW2);
        
        return result;
    }
}
