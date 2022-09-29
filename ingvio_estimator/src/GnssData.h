#pragma once

#include <gnss_comm/gnss_ros.hpp>

namespace ingvio
{
    class GnssData
    {
    public:
        
        GnssData() : _isIono(false)
        {}
        
        ~GnssData() = default;
        
        GnssData operator=(const GnssData&) = delete;
        
        std::vector<double> latest_gnss_iono_params;
        
        std::map<uint32_t, std::vector<gnss_comm::EphemBasePtr>> sat2ephem;
        
        std::map<uint32_t, std::map<double, size_t>> sat2time_index;
        
        std::map<uint32_t, uint32_t> sat_track_status;
        
        void inputEphem(gnss_comm::EphemBasePtr ephem_ptr);
        
        bool _isIono;
        
    private:
        
        GnssData(const GnssData&) {}
    };
}
