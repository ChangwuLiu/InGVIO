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

#include "camera.h"

namespace camera_model
{
    Camera::Parameters::Parameters(ModelType modelType): m_modelType(modelType), m_imageWidth(0), m_imageHeight(0)
    {
        switch (modelType)
        {
            case PINHOLE:
                m_nIntrinsics = 8;
                break;
            case MEI:
            default:
                m_nIntrinsics = 9;
        }
    }
    
    Camera::Parameters::Parameters(ModelType modelType, const std::string& cameraName, int w, int h): m_modelType(modelType), m_cameraName(cameraName), m_imageWidth(w),  m_imageHeight(h)
    {
        switch (modelType)
        {
            case PINHOLE:
                m_nIntrinsics = 8;
                break;
            case MEI:
            default:
                m_nIntrinsics = 9;
        }
    }
    
    Camera::ModelType& Camera::Parameters::modelType()
    {
        return m_modelType;
    }
    
    std::string& Camera::Parameters::cameraName()
    {
        return m_cameraName;
    }
    
    int& Camera::Parameters::imageWidth()
    {
        return m_imageWidth;
    }

    int& Camera::Parameters::imageHeight()
    {
        return m_imageHeight;
    }

    Camera::ModelType Camera::Parameters::modelType() const 
    {
        return m_modelType;
    }

    const std::string& Camera::Parameters::cameraName() const
    {
        return m_cameraName;
    }

    int Camera::Parameters::imageWidth() const
    {
        return m_imageWidth;
    }

    int Camera::Parameters::imageHeight() const
    {
        return m_imageHeight;
    }

    int Camera::Parameters::nIntrinsics() const
    {
        return m_nIntrinsics;
    }

    cv::Mat& Camera::mask()
    {
        return m_mask;
    }

    const cv::Mat& Camera::mask() const
    {
        return m_mask;
    }    
    
    double Camera::reprojectionDist(const Eigen::Vector3d& P1, const Eigen::Vector3d& P2) const
    {
        Eigen::Vector2d p1, p2;
        
        spaceToPlane(P1, p1);
        spaceToPlane(P2, p2);
        
        return (p1-p2).norm();
    }
    
    double Camera::reprojectionError(const Eigen::Vector3d& P, 
                                     const Eigen::Quaterniond& camera_q, 
                                     const Eigen::Vector3d& camera_t, 
                                     const Eigen::Vector2d& observed_p) const
    {
        Eigen::Vector3d P_cam = camera_q.toRotationMatrix() * P + camera_t;

        Eigen::Vector2d p;
        spaceToPlane(P_cam, p);

        return (p - observed_p).norm();
    }
}
