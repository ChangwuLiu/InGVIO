#pragma once

#include <feature_tracker/MonoFrame.h>
#include <feature_tracker/MonoMeas.h>

#include <feature_tracker/StereoFrame.h>
#include <feature_tracker/StereoMeas.h>

#include "AnchoredLandmark.h"

namespace ingvio
{
    struct MonoMeas
    {
        int _id;
        double _u0;
        double _v0;
        
        Eigen::Vector2d asVec() const
        {  return Eigen::Vector2d(_u0, _v0);  }
        
        void setFromMonoMsg(const feature_tracker::MonoMeas::ConstPtr& mono_msg);
        
        void setFromMonoMsg(const feature_tracker::MonoMeas& mono_msg);
    };
    
    struct StereoMeas
    {
        int _id;
        double _u0;
        double _v0;
        double _u1;
        double _v1;
        
        Eigen::Vector2d asLeftVec() const
        {  return Eigen::Vector2d(_u0, _v0);  }
        
        Eigen::Vector2d asRightVec() const
        {  return Eigen::Vector2d(_u1, _v1);  }
        
        Eigen::Vector4d asVec() const
        {  return Eigen::Vector4d(_u0, _v0, _u1, _v1);  }
        
        void setFromStereoMsg(const feature_tracker::StereoMeas::ConstPtr& stereo_msg);
        
        void setFromStereoMsg(const feature_tracker::StereoMeas& stereo_msg);
    };
    
    class FeatureInfo
    {
    public:
        enum FeatureType {MSCKF = 0, SLAM};
        
        FeatureInfo() : _id(-1), _ftype(MSCKF), _isToMarg(false), _isTri(false), _numOfTri(0) 
        {
            _mono_obs.clear();
            _stereo_obs.clear();
            
            _landmark = std::make_shared<AnchoredLandmark>();
        }
        
        ~FeatureInfo() = default;
                
        const int& getId() const
        {  return _id;  }

        const FeatureType& getFeatureType() const
        {  return _ftype;  }
        
        const bool& isToMarg() const
        {  return _isToMarg;  }
        
        const bool& isTri() const
        {  return _isTri;  }
        
        bool hasMonoObsAt(double timestamp) const;
        
        Eigen::Vector2d monoMeasAt(double timestamp) const;
        
        bool hasStereoObsAt(double timestamp) const;
        
        Eigen::Vector4d stereoMeasAt(double timestamp) const;
        
        int numOfMonoFrames() const
        {  return _mono_obs.size();  }
        
        int numOfStereoFrames() const
        {  return _stereo_obs.size();  }
        
        const std::shared_ptr<SE3> anchor() const
        {  return _landmark->getAnchoredPose();  }
        
        const std::shared_ptr<AnchoredLandmark> landmark() const
        {  return _landmark;  }
     
     /*
     protected:
        friend class FeatureInfoManager;
        friend class MapServerManager;
     */
     
        int _id;
        FeatureType _ftype;
        bool _isToMarg;
        bool _isTri;
        int _numOfTri;
        
        std::shared_ptr<AnchoredLandmark> _landmark;
        
        std::map<double, std::shared_ptr<MonoMeas>> _mono_obs;
        std::map<double, std::shared_ptr<StereoMeas>> _stereo_obs;
    };
    
    typedef std::map<int, std::shared_ptr<FeatureInfo>> MapServer;
}
