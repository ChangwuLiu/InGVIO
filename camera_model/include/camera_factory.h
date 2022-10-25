/**   This File is part of Camera Model
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
