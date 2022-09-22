#pragma once

#include "MapServer.h"

#include "Update.h"

namespace ingvio
{
    class Triangulator;
    class IngvioParams;
    
    class SwMargUpdate : public UpdateBase
    {
    public:
        using UpdateBase::UpdateBase;
        
        SwMargUpdate(const IngvioParams& filter_params) : 
        UpdateBase(filter_params._chi2_max_dof, filter_params._chi2_thres),
        _noise(filter_params._visual_noise),
        _frame_select_interval(filter_params._frame_select_interval)
        {}
        
        SwMargUpdate(const SwMargUpdate&) = delete;
        SwMargUpdate operator=(const SwMargUpdate&) = delete;
        
        virtual ~SwMargUpdate() {}
        
        void updateStateMono(std::shared_ptr<State> state,
                             std::shared_ptr<MapServer> map_server,
                             std::shared_ptr<Triangulator> tri);
        
        void cleanMonoObsAtMargTime(std::shared_ptr<State> state,
                                    std::shared_ptr<MapServer> map_server);
        
        void updateStateStereo(std::shared_ptr<State> state,
                               std::shared_ptr<MapServer> map_server,
                               std::shared_ptr<Triangulator> tri);
        
        void cleanStereoObsAtMargTime(std::shared_ptr<State> state,
                                      std::shared_ptr<MapServer> map_server);
        
        void changeMSCKFAnchor(std::shared_ptr<State> state,
                               std::shared_ptr<MapServer> map_server);
        
        void margSwPose(std::shared_ptr<State> state);
        
        void removeMonoMSCKFinMargPose(std::shared_ptr<State> state,
                                   std::shared_ptr<MapServer> map_server);
        
        void removeStereoMSCKFinMargPose(std::shared_ptr<State> state,
                                         std::shared_ptr<MapServer> map_server);
        
    protected:
        
        double _noise;
        
        int _frame_select_interval;
        
        void generateSwVarOrder(const std::shared_ptr<State> state,
                                std::vector<std::shared_ptr<SE3>>& sw_var_order,
                                std::map<std::shared_ptr<SE3>, int>& sw_index,
                                std::vector<std::shared_ptr<Type>>& sw_var_type);
        
        void calcResJacobianSingleFeatSelectedMonoObs(
            const std::shared_ptr<FeatureInfo> feature_info,
            const std::map<double, std::shared_ptr<SE3>>& sw_poses,
            const std::vector<std::shared_ptr<SE3>>& sw_var_order,
            const std::map<std::shared_ptr<SE3>, int>& sw_index_map,
            const std::vector<double>& selected_timestamps,
            Eigen::VectorXd& res_block,
            Eigen::MatrixXd& H_block);
        
        void calcResJacobianSingleFeatSelectedStereoObs(
            const std::shared_ptr<FeatureInfo> feature_info, 
            const std::map<double, std::shared_ptr<SE3>>& sw_poses,
            const std::vector<std::shared_ptr<SE3>>& sw_var_order,
            const std::map<std::shared_ptr<SE3>, int>& sw_index_map,
            const std::vector<double>& selected_timestamps,
            const Eigen::Isometry3d& T_cl2cr,
            Eigen::VectorXd& res_block,
            Eigen::MatrixXd& H_block);
        
        void selectSwTimestamps(const std::map<double, std::shared_ptr<SE3>>& sw_poses,
                                const double& marg_time,
                                std::vector<double>& selected_timestamps);
    };
}
