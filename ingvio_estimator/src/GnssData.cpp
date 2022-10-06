
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
