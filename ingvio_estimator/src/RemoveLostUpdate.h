#pragma once

#include "MapServer.h"

#include "Update.h"

namespace ingvio
{
    class Triangulator;
    class IngvioParams;
    
    class RemoveLostUpdate : public UpdateBase
    {
    public:
        using UpdateBase::UpdateBase;
        
        RemoveLostUpdate(const IngvioParams& filter_params) : UpdateBase(filter_params._chi2_max_dof, filter_params._chi2_thres), _noise(filter_params._visual_noise),
        _max_valid_ids(10)
        {}
        
        RemoveLostUpdate(const RemoveLostUpdate&) = delete;
        RemoveLostUpdate operator=(const RemoveLostUpdate&) = delete;
        
        virtual ~RemoveLostUpdate() {}
        
        void updateStateMono(std::shared_ptr<State> state,
                             std::shared_ptr<MapServer> map_server,
                             std::shared_ptr<Triangulator> tri);
        
        void updateStateStereo(std::shared_ptr<State> state,
                               std::shared_ptr<MapServer> map_server,
                               std::shared_ptr<Triangulator> tri);
        
    protected:
        
        void calcResJacobianSingleFeatAllMonoObs(
            const std::shared_ptr<FeatureInfo> feature_info,
            const std::map<double, std::shared_ptr<SE3>>& sw_poses,
            std::vector<std::shared_ptr<Type>>& block_var_order,
            std::map<std::shared_ptr<Type>, int>& block_index_map,
            Eigen::VectorXd& res_block,
            Eigen::MatrixXd& H_block);
        
        void calcResJacobianSingleFeatAllStereoObs(
            const std::shared_ptr<FeatureInfo> feature_info, 
            const std::map<double, std::shared_ptr<SE3>>& sw_poses,
            const std::vector<std::shared_ptr<SE3>>& sw_var_order,
            const std::map<std::shared_ptr<SE3>, int>& sw_index_map,
            const Eigen::Isometry3d& T_cl2cr,
            Eigen::VectorXd& res_block,
            Eigen::MatrixXd& H_block);
        
        void generateSwVarOrder(const std::shared_ptr<State> state,
                                std::vector<std::shared_ptr<SE3>>& sw_var_order,
                                std::map<std::shared_ptr<SE3>, int>& sw_index_map,
                                std::vector<std::shared_ptr<Type>>& sw_var_type);
        
        int _max_valid_ids;        
        double _noise;
    }; 
}
