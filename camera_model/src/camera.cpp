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
