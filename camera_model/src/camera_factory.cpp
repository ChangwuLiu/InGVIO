#include "camera_factory.h"
#include "MeiCamera.h"
#include "PinholeCamera.h"

#include <boost/algorithm/string.hpp>

namespace camera_model
{
    boost::shared_ptr<CameraFactory> CameraFactory::m_instance;
    
    CameraFactory::CameraFactory() {}
    
    boost::shared_ptr<CameraFactory> CameraFactory::instance(void)
    {
        if (m_instance.get() == 0)
            m_instance.reset(new CameraFactory);
        
        return m_instance;
    }
    
    CameraPtr CameraFactory::generateCamera(Camera::ModelType modelType, const std::string& cameraName, cv::Size imageSize) const
    {
        switch (modelType)
        {
            case Camera::PINHOLE:
            {
                PinholeCameraPtr camera(new PinholeCamera);
                
                PinholeCamera::Parameters params = camera->getParameters();
                params.cameraName() = cameraName;
                params.imageWidth() = imageSize.width;
                params.imageHeight() = imageSize.height;
                
                camera->setParameters(params);
                return camera;
            }
            case Camera::MEI:
            default:
            {
                MeiCameraPtr camera(new MeiCamera);
                
                MeiCamera::Parameters params = camera->getParameters();
                params.cameraName() = cameraName;
                params.imageWidth() = imageSize.width;
                params.imageHeight() = imageSize.height;
                
                camera->setParameters(params);
                return camera;
            }
        }
    }
    
    CameraPtr CameraFactory::generateCameraFromYamlFile(const std::string& filename)
    {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        
        if (!fs.isOpened())
            return CameraPtr();
        
        Camera::ModelType modelType = Camera::MEI;
        if (!fs["model_type"].isNone())
        {
            std::string sModelType;
            fs["model_type"] >> sModelType;
            
            if (boost::iequals(sModelType, "MEI"))
            {
                modelType = Camera::MEI;
            }
            else if (boost::iequals(sModelType, "PINHOLE"))
            {
                modelType = Camera::PINHOLE;
            }
            else
            {
                std::cerr << "# ERROR: Unknown camera model type: " << sModelType << std::endl;
                return CameraPtr();
            }
        }
        
        switch (modelType)
        {
            case Camera::PINHOLE:
            {
                PinholeCameraPtr camera(new PinholeCamera);
                
                PinholeCamera::Parameters params = camera->getParameters();
                params.readFromYamlFile(filename);
                
                camera->setParameters(params);
                return camera;
            }
            case Camera::MEI:
            default:
            {
                MeiCameraPtr camera(new MeiCamera);
                
                MeiCamera::Parameters params = camera->getParameters();
                params.readFromYamlFile(filename);
                
                camera->setParameters(params);
                return camera;
            }
        }
        
        return CameraPtr();
    }
    
}
