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

#include "MapServer.h"

namespace ingvio
{
    void MonoMeas::setFromMonoMsg(const feature_tracker::MonoMeas::ConstPtr& mono_msg)
    {
        _id = mono_msg->id;
        _u0 = mono_msg->u0;
        _v0 = mono_msg->v0;
    }
    
    void MonoMeas::setFromMonoMsg(const feature_tracker::MonoMeas& mono_msg)
    {
        _id = mono_msg.id;
        _u0 = mono_msg.u0;
        _v0 = mono_msg.v0;
    }
    
    void StereoMeas::setFromStereoMsg(const feature_tracker::StereoMeas::ConstPtr& stereo_msg)
    {
        _id = stereo_msg->id;
        _u0 = stereo_msg->u0;
        _v0 = stereo_msg->v0;
        _u1 = stereo_msg->u1;
        _v1 = stereo_msg->v1;
    }
    
    void StereoMeas::setFromStereoMsg(const feature_tracker::StereoMeas& stereo_msg)
    {
        _id = stereo_msg.id;
        _u0 = stereo_msg.u0;
        _v0 = stereo_msg.v0;
        _u1 = stereo_msg.u1;
        _v1 = stereo_msg.v1;
    }
    
    bool FeatureInfo::hasMonoObsAt(double timestamp) const
    {  
        return (_mono_obs.find(timestamp) != _mono_obs.end());
        
    }
    
    Eigen::Vector2d FeatureInfo::monoMeasAt(double timestamp) const
    {  
        if (hasMonoObsAt(timestamp))
            return _mono_obs.at(timestamp)->asVec();
        else 
            return Eigen::Vector2d::Zero();
    }
    
    bool FeatureInfo::hasStereoObsAt(double timestamp) const
    {  
        return (_stereo_obs.find(timestamp) != _stereo_obs.end());
    }
    
    Eigen::Vector4d FeatureInfo::stereoMeasAt(double timestamp) const
    {
        if (hasStereoObsAt(timestamp))
            return _stereo_obs.at(timestamp)->asVec();
        else
            return Eigen::Vector4d::Zero();
    }
}
