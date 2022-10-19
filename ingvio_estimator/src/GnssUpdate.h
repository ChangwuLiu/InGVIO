#pragma once

#include <memory>
#include <unordered_set>

#include "State.h"
#include "Update.h"

#include "IngvioParams.h"

#include "GnssSync.h"

namespace ingvio
{
    class State;
    class GvioAligner;
    
    class GnssUpdate : public UpdateBase
    {
    public:
        using UpdateBase::UpdateBase;
        
        GnssUpdate(const IngvioParams& filter_params) :
        UpdateBase(filter_params._chi2_max_dof, filter_params._chi2_thres),
        _psr_noise_amp(filter_params._psr_noise_amp),
        _dopp_noise_amp(filter_params._dopp_noise_amp),
        _is_adjust_yof(filter_params._is_adjust_yof),
        _is_gnss_chi2_test(filter_params._is_gnss_chi2_test)
        {}
     
        virtual ~GnssUpdate() {}
        
        GnssUpdate(const GnssUpdate&) = delete;
        
        GnssUpdate operator=(const GnssUpdate&) = delete;
        
        void checkYofStatus(std::shared_ptr<State> state, 
                            std::shared_ptr<GvioAligner> gvio_aligner);
        
        void removeUntrackedSys(std::shared_ptr<State> state, 
                                const GnssMeas& gnss_meas);
        
        void updateTrackedSys(std::shared_ptr<State> state,
                              const GnssMeas& gnss_meas,
                              std::shared_ptr<GvioAligner> gvio_aligner,
                              const std::vector<double>& iono_params);
        
        void addNewTrackedSys(std::shared_ptr<State> state,
                              const GnssMeas& gnss_meas,
                              const SppMeas& spp_meas,
                              std::shared_ptr<GvioAligner> gvio_aligner,
                              const std::vector<double>& iono_params);
        
    protected:
        
        double _psr_noise_amp = 1.0;
        
        double _dopp_noise_amp = 1.0;
        
        int _is_adjust_yof = 0;
        
        int _is_gnss_chi2_test = 0;
        
        void getSysInGnssMeas(const GnssMeas& gnss_meas);
        
        void getSysInSppMeas(const SppMeas& spp_meas);
        
        void calcSysToAdd(const std::shared_ptr<State> state,
                          std::unordered_set<State::GNSSType>& sys_to_add);
        
        void getResJacobianOfSys(const State::GNSSType& gtype,
                                 const GnssMeas& gnss_meas,
                                 const std::vector<Eigen::Vector2d>& all_sv_azel,
                                 const Eigen::VectorXd& res_total,
                                 const Eigen::MatrixXd& H_total,
                                 Eigen::VectorXd& res_sys,
                                 Eigen::MatrixXd& batch_unit_rv2sv_T_sys,
                                 double& noise);
        
        std::unordered_set<State::GNSSType> _gnss_sys;
        std::unordered_set<State::GNSSType> _spp_sys;
        
    };
}
