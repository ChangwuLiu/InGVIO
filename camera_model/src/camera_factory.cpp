/**   This File is part of Camera Model
 *  
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
 *    
 *    Since some functions are adapted from GVINS <https://github.com/HKUST-Aerial-Robotics/GVINS>
 *    GVINS is under GPLv3 License.
 */

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
