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

#include "GnssData.h"

namespace ingvio
{
    void GnssData::inputEphem(gnss_comm::EphemBasePtr ephem_ptr)
    {
        double toe = gnss_comm::time2sec(ephem_ptr->toe);
        
        if (this->sat2time_index.count(ephem_ptr->sat) == 0 || 
            this->sat2time_index.at(ephem_ptr->sat).count(toe) == 0)
        {
            this->sat2ephem[ephem_ptr->sat].emplace_back(ephem_ptr);
            this->sat2time_index[ephem_ptr->sat].emplace(toe, sat2ephem.at(ephem_ptr->sat).size()-1);
        }
    }
}
