#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "IngvioParams.h"

namespace ingvio
{
    class GvioAligner
    {
    public:
        
        GvioAligner(const IngvioParams& filter_params)
        {
            
        }
        
        ~GvioAligner() = default;

        GvioAligner(const GvioAligner&) = delete;
        GvioAligner operator=(const GvioAligner&) = delete;
        
        
        bool isAlign() const
        {  return _isAligned;  }
        
        const Eigen::Isometry3d& getTecef2w() const
        {  return T_ecef2w;  }
        
        Eigen::Isometry3d getTw2ecef() const
        {  return T_ecef2w.inverse();  }
        
        bool _isAligned = false;
        
        Eigen::Isometry3d T_ecef2w = Eigen::Isometry3d::Identity();
        
    };
    
}
