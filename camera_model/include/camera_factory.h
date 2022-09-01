#pragma once

#include "camera.h"

namespace camera_model
{
    class CameraFactory
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        CameraFactory();
        
        static boost::shared_ptr<CameraFactory> instance(void);
        
        CameraPtr generateCamera(Camera::ModelType modelType, const std::string& cameraName, cv::Size imageSize) const;
        
        CameraPtr generateCameraFromYamlFile(const std::string& filename);
        
    private:
        static boost::shared_ptr<CameraFactory> m_instance;
    };
    
}
