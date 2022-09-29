
#include <iomanip>

#include "GnssSync.h"

namespace ingvio
{
    void GnssSync::bufferGnssMeas(const std::vector<gnss_comm::ObsPtr>& valid_meas,
                                  const std::vector<gnss_comm::EphemBasePtr>& valid_ephems)
    {
        if (!this->isSync()) return;
        
        bool flag = false;
        while (_gnss_meas_buffer.size() > 100)
        {
            _gnss_meas_buffer.pop();
            
            if (!flag) flag = true;
        }
        
        if (flag)
            std::cout << "[GnssSync]: Throw previous Gnss Meas in buffer due to exceeding max size!" << std::endl;
        
        _gnss_meas_buffer.push(std::make_pair(valid_meas, valid_ephems));
    }
    
    void GnssSync::bufferSppMeas(const gnss_comm::gtime_t& gtime,
                                 const Eigen::Matrix<double, 7, 1>& pos_spp,
                                 const Eigen::Vector4d& vel_spp)
    {
        if (!this->isSync()) return;
        
        bool flag = false;
        while (_spp_meas_buffer.size() > 100)
        {
            _spp_meas_buffer.pop();
            
            if (!flag) flag = true;
        }
        
        if (flag)
            std::cout << "[GnssSync]: Throw previous Spp Meas in buffer due to exceeding max size!" << std::endl;
        
        _spp_meas_buffer.push(SppMeas(gtime, pos_spp, vel_spp));
    }
    
    void GnssSync::storeTimePair(const double& curr_time, const gnss_comm::gtime_t& gnss_time)
    {
        if (this->isSync()) return;
        
        if (_rosidx_gnsstime.find(curr_time) == _rosidx_gnsstime.end())
            _rosidx_gnsstime[curr_time] = gnss_time;
        
        if (_rosidx_gnsstime.size() >= 3 && _rosidx_header.size() >= 3)
        {
            double delta_time = INFINITY;
            
            double idx_g, idx_h;
            
            for (const auto& item_g : _rosidx_gnsstime)
                for (const auto& item_h : _rosidx_header)
                    if (std::fabs(item_g.first - item_h.first) < delta_time)
                    {
                        idx_g = item_g.first;
                        idx_h = item_h.first;
                        delta_time = std::fabs(idx_g - idx_h);
                    }
                    
            if (delta_time < _unsync_thres)
            {
                _gnss2local_time_offset = _rosidx_header.at(idx_h) - gnss_comm::time2sec(_rosidx_gnsstime.at(idx_g)) - (idx_h - idx_g);
                _isSync = true;
                
                std::cout << "[GnssSync]: gnss time to local time offset = " << std::setprecision(10) << _gnss2local_time_offset << " (s) " << std::endl;
            }
            
            _rosidx_gnsstime.clear();
            _rosidx_header.clear();
        }
    }
    
    void GnssSync::storeTimePair(const double& curr_time, const double& header_time)
    {
        if (this->isSync()) return;
        
        if (_rosidx_header.find(curr_time) == _rosidx_header.end())
            _rosidx_header[curr_time] = header_time;
        
        if (_rosidx_gnsstime.size() >= 3 && _rosidx_header.size() >= 3)
        {
            double delta_time = INFINITY;
            
            double idx_g, idx_h;
            
            for (const auto& item_g : _rosidx_gnsstime)
                for (const auto& item_h : _rosidx_header)
                    if (std::fabs(item_g.first - item_h.first) < delta_time)
                    {
                        idx_g = item_g.first;
                        idx_h = item_h.first;
                        delta_time = std::fabs(idx_g - idx_h);
                    }
                    
            if (delta_time < _unsync_thres)
            {
                _gnss2local_time_offset = _rosidx_header.at(idx_h) - gnss_comm::time2sec(_rosidx_gnsstime.at(idx_g)) - (idx_h - idx_g);
                _isSync = true;
                        
                std::cout << "[GnssSync]: gnss time to local time offset = " << std::setprecision(10) << _gnss2local_time_offset << " (s) " << std::endl;
            }
                    
            _rosidx_gnsstime.clear();
            _rosidx_header.clear();
        }
    }
    
    bool GnssSync::getGnssMeasAt(const std_msgs::Header& header, GnssMeas& gnss_meas)
    {
        if (!isSync()) return false;
        
        double target_time = header.stamp.toSec();
        
        bool flag = false;
        
        while (!_gnss_meas_buffer.empty())
        {
            auto& g_m = _gnss_meas_buffer.front();
            
            double g_time = gnss_comm::time2sec(g_m.first[0]->time) + _gnss2local_time_offset;
            
            if (g_time < target_time - _unsync_thres)
            {
                _gnss_meas_buffer.pop();
                continue;
            }
            
            if (g_time >= target_time + _unsync_thres)
                break;
            
            std::swap(gnss_meas, g_m);
            _gnss_meas_buffer.pop();
            flag = true;
            
            break;
        }    
            
        return flag;
    }
    
    bool GnssSync::getSppAt(const std_msgs::Header& header, SppMeas& spp_meas)
    {        
        if (!isSync()) return false;
        
        double target_time = header.stamp.toSec();
        
        bool flag = false;
        
        while (!_spp_meas_buffer.empty())
        {
            auto& s_m = _spp_meas_buffer.front();
            
            double g_time = gnss_comm::time2sec(s_m.gtime) + _gnss2local_time_offset;
            
            if (g_time < target_time - _unsync_thres)
            {
                _spp_meas_buffer.pop();
                continue;
            }
            
            if (g_time >= target_time + _unsync_thres)
                break;
            
            std::swap(spp_meas, s_m);
            _spp_meas_buffer.pop();
            flag = true;
            
            break;
        }    
        
        return flag;
    }
    
    void GnssSync::clearMeasBuffer()
    {
        this->clear<GnssMeas>(this->_gnss_meas_buffer);
        this->clear<SppMeas>(this->_spp_meas_buffer);
    }
    
    void GnssSync::clearMeasBufferUntil(const std_msgs::Header& header)
    {
        if (!isSync()) return;
        
        double time_end = header.stamp.toSec();
        
        while (!_gnss_meas_buffer.empty())
        {
            auto& gnss_meas = _gnss_meas_buffer.front();
            
            double meas_time = gnss_comm::time2sec(gnss_meas.first[0]->time) + _gnss2local_time_offset;
            
            if (meas_time > time_end) break;
            
            _gnss_meas_buffer.pop();
        }
        
        while (!_spp_meas_buffer.empty())
        {
            auto& spp_meas = _spp_meas_buffer.front();
            
            double meas_time = gnss_comm::time2sec(spp_meas.gtime) + _gnss2local_time_offset;
            
            if (meas_time > time_end) break;
            
            _spp_meas_buffer.pop();
        }
    }
}
