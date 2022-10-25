/**  This File is part of InGVIO, an invariant filter for mono/stereo visual-
 *    inertial-raw GNSS navigation. 
 *    
 *    Copyright (C) 2022  Changwu Liu (cwliu529@163.com,
 *                                     lcw18@mails.tsinghua.edu.cn (valid until 2023))
 *    
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *    
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *    
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

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
