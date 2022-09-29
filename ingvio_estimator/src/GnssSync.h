#pragma once

#include <queue>

#include <gnss_comm/gnss_ros.hpp>

#include "IngvioParams.h"

namespace ingvio
{
    typedef std::pair<std::vector<gnss_comm::ObsPtr>, 
                      std::vector<gnss_comm::EphemBasePtr>> GnssMeas;
    
    struct SppMeas
    {
        SppMeas()
        {
            posSpp.setZero();
            velSpp.setZero();
        }
        
        SppMeas(const gnss_comm::gtime_t& time,
                const Eigen::Matrix<double, 7, 1>& pos_spp,
                const Eigen::Vector4d& vel_spp) :
                gtime(time), posSpp(pos_spp), velSpp(vel_spp)
        {}
        
        ~SppMeas() = default;
        
        gnss_comm::gtime_t gtime;
        
        Eigen::Matrix<double, 7, 1> posSpp;
        
        Eigen::Vector4d velSpp;
    };
    
    class GnssSync
    {
    public:
        
        GnssSync(const IngvioParams& filter_params) : _unsync_thres(0.13)
        {
            _isSync = false;
            
            if (!filter_params._enable_gnss)
                return;
            
            if (filter_params._use_fix_time_offset)
            {
                _isSync = true;
                _gnss2local_time_offset = filter_params._gnss_local_offset;
            }
        }
        
        ~GnssSync() = default;
        
        GnssSync(const GnssSync&) = delete;
        
        GnssSync operator=(const GnssSync&) = delete;
        
        void bufferGnssMeas(const std::vector<gnss_comm::ObsPtr>& valid_meas,
                            const std::vector<gnss_comm::EphemBasePtr>& valid_ephems);
        
        void bufferSppMeas(const gnss_comm::gtime_t& gtime,
                           const Eigen::Matrix<double, 7, 1>& pos_spp,
                           const Eigen::Vector4d& vel_spp);
        
        bool isSync() const
        {  return _isSync;  }
        
        const double& getUnsyncTime() const
        {  return _gnss2local_time_offset;  }
        
        void storeTimePair(const double& curr_time, const gnss_comm::gtime_t& gnss_time);
        
        void storeTimePair(const double& curr_time, const double& header_time);
        
        bool getGnssMeasAt(const std_msgs::Header& header, GnssMeas& gnss_meas);
        
        bool getSppAt(const std_msgs::Header& header, SppMeas& spp_meas);
        
        void clearMeasBuffer();
        
        void clearMeasBufferUntil(const std_msgs::Header& header);
        
    protected:
        
        bool _isSync;
        
        double _gnss2local_time_offset;
        
        double _unsync_thres;
        
        std::queue<GnssMeas> _gnss_meas_buffer;
        
        std::queue<SppMeas> _spp_meas_buffer;
        
        std::map<double, gnss_comm::gtime_t> _rosidx_gnsstime;
        
        std::map<double, double> _rosidx_header;
        
        template<typename T>
        static void clear(std::queue<T>& qu)
        {
            std::queue<T> empty;
            std::swap(empty, qu);
        }
        
    };
}
