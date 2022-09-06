#pragma once

#include <map>
#include <unordered_map>
#include <memory>
#include <vector>
#include <functional>

#include "VecState.h"
#include "PoseState.h"
#include "AnchoredLandmark.h"

#include "IngvioParams.h"

namespace ingvio
{
    class StateParams
    {
    public:
        StateParams() = default;
        
        StateParams(const IngvioParams& filter_params);
        
        int _cam_nums = 2;
        int _max_sw_poses = 20;
        int _max_landmarks = 25;
        
        double _noise_g = 0.005;
        double _noise_a = 0.05;
        double _noise_bg = 0.001;
        double _noise_ba = 0.01;
        double _noise_clockbias = 2.0;
        double _noise_cb_rw = 0.2;
        
        bool _enable_gnss = true;
        
        Eigen::Isometry3d _T_cl2cr = Eigen::Isometry3d::Identity();
    };
    
    class State
    {
    public:
        enum GNSSType {GPS = 0, GLO, GAL, BDS, FS, YOF};
        
        State();
        State(const IngvioParams& filter_params);

        
        ~State() {}
        
        double nextMargTime()
        {
            double time = INFINITY;
            
            if (_sw_camleft_poses.size() > _state_params._max_sw_poses)
                for (const auto& item: _sw_camleft_poses)
                    if (item.first < time) time = item.first;
                
            return time;
        }
        
        int curr_cov_size() { return _cov.rows(); }
        int curr_err_variable_size() { return _err_variables.size();}
        
        double _timestamp = -1;
        
        StateParams _state_params;
        
        std::shared_ptr<SE23> _extended_pose;
        
        std::shared_ptr<Vec3> _bg;
        
        std::shared_ptr<Vec3> _ba;
        
        std::shared_ptr<SE3> _camleft_imu_extrinsics;
        
        std::unordered_map<GNSSType, std::shared_ptr<Scalar>> _gnss;
        
        std::unordered_map<int, std::shared_ptr<AnchoredLandmark>> _anchored_landmarks;
        
        std::map<double, std::shared_ptr<SE3>, std::less<double>> _sw_camleft_poses;
        
    private:
        friend class StateManager;
        
        Eigen::MatrixXd _cov;
        
        std::vector<std::shared_ptr<Type>> _err_variables;
    };
}


