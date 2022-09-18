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
