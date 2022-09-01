#pragma once

#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <opencv2/core/core.hpp>
#include <string>
#include <iostream>

namespace camera_model
{
    class Camera
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        enum ModelType
        {
            MEI,
            PINHOLE
        };
        
        class Parameters
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            
            Parameters(ModelType modelType);
            Parameters(ModelType modelType, const std::string& cameraName, int w, int h);
            
            ModelType& modelType();
            ModelType modelType() const;
            
            std::string& cameraName();
            const std::string& cameraName() const;
            
            int& imageWidth();
            int imageWidth() const;
            
            int& imageHeight();
            int imageHeight() const;
            
            int nIntrinsics() const;
            
            virtual bool readFromYamlFile(const std::string& filename) = 0;
            virtual void writeToYamlFile(const std::string& filename) const = 0;
            
        protected:
            ModelType m_modelType;
            int m_nIntrinsics;
            std::string m_cameraName;
            int m_imageWidth;
            int m_imageHeight;
        };
        
        
        virtual ModelType modelType() const  = 0;
        virtual const std::string& cameraName() const = 0;
        virtual int imageWidth() const = 0;
        virtual int imageHeight() const = 0;
        
        virtual cv::Mat& mask();
        virtual const cv::Mat& mask() const;
        
        virtual double getNormalPixel() const = 0;
        
        // from the image plane to the sphere
        virtual void liftSphere(const Eigen::Vector2d& p, Eigen::Vector3d& P) const = 0;
        
        // from the image plane to the projective space
        virtual void liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const = 0;
        
        // from 3D points to the image plane
        virtual void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p) const = 0;
        
        virtual void undistToPlane(const Eigen::Vector2d& p_u, Eigen::Vector2d& p) const = 0;
        
        virtual int parameterCount(void) const = 0;
        
        virtual void readParameters(const std::vector<double>& parameters) = 0;
        virtual void writeParameters(std::vector<double>& parameters) const = 0;
        virtual void writeParametersToYamlFile(const std::string& filename) const = 0;
        virtual std::string parametersToString() const = 0;
        
        double reprojectionDist(const Eigen::Vector3d& P1, const Eigen::Vector3d& P2) const;

        double reprojectionError(const Eigen::Vector3d& P,
                                 const Eigen::Quaterniond& camera_q,
                                 const Eigen::Vector3d& camera_t,
                                 const Eigen::Vector2d& observed_p) const;
                           
    protected:
        cv::Mat m_mask;
    };
    
    typedef boost::shared_ptr<Camera> CameraPtr;
    typedef boost::shared_ptr<const Camera> CameraConstPtr;
}
