
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
