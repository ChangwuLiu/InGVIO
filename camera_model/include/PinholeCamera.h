#pragma once

#include "camera.h"
#include <cmath>

namespace camera_model
{
    class PinholeCamera: public Camera
    {
    public:
        class Parameters: public Camera::Parameters
        {
        public:
            Parameters();
            Parameters(const std::string& cameraName, int w, int h, double k1, double k2, double p1, double p2, double fx, double fy, double cx, double cy);

            double& k1(void);
            double& k2(void);
            double& p1(void);
            double& p2(void);
            double& fx(void);
            double& fy(void);
            double& cx(void);
            double& cy(void);

            double xi(void) const;
            double k1(void) const;
            double k2(void) const;
            double p1(void) const;
            double p2(void) const;
            double fx(void) const;
            double fy(void) const;
            double cx(void) const;
            double cy(void) const;

            bool readFromYamlFile(const std::string& filename) override;
            void writeToYamlFile(const std::string& filename) const override;

            Parameters& operator=(const Parameters& other);
            friend std::ostream& operator<< (std::ostream& out, const Parameters& params);

        private:
            double m_k1;
            double m_k2;
            double m_p1;
            double m_p2;
            double m_fx;
            double m_fy;
            double m_cx;
            double m_cy;
        };
        
        PinholeCamera();
        
        PinholeCamera(const std::string& cameraName, int imageWidth, int imageHeight,  double k1, double k2, double p1, double p2, double fx, double fy, double cx, double cy);
        
        PinholeCamera(const Parameters& params);
        
        Camera::ModelType modelType(void) const override;
        const std::string& cameraName(void) const override;
        int imageWidth(void) const override;
        int imageHeight(void) const override;
        
        double getNormalPixel() const override;
        
        void liftSphere(const Eigen::Vector2d& p, Eigen::Vector3d& P) const override;

        void liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const override;

        void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p) const override;

        void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p, Eigen::Matrix<double,2,3>& J) const;
        
        void undistToPlane(const Eigen::Vector2d& p_u, Eigen::Vector2d& p) const override;
        
        template <typename T>
        static void spaceToPlane(const T* const params, const T* const q, const T* const t, const Eigen::Matrix<T, 3, 1>& P, Eigen::Matrix<T, 2, 1>& p);
        
        void distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u) const;
        void distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u, Eigen::Matrix2d& J) const;
        
        int parameterCount() const override;
        
        const Parameters& getParameters(void) const;

        void setParameters(const Parameters& parameters);  
        
        void readParameters(const std::vector<double>& parameterVec) override;

        void writeParameters(std::vector<double>& parameterVec) const override;

        void writeParametersToYamlFile(const std::string& filename) const override;

        std::string parametersToString(void) const override;
        
    private:
        Parameters mParameters;
        
        double m_inv_K11, m_inv_K13, m_inv_K22, m_inv_K23;
        bool m_noDistortion;
    };
    
    typedef boost::shared_ptr<PinholeCamera> PinholeCameraPtr;
    typedef boost::shared_ptr<const PinholeCamera> PinholeCameraConstPtr;
    
    template <typename T>
    void PinholeCamera::spaceToPlane(const T* const params, const T* const q, const T* const t, const Eigen::Matrix<T, 3, 1>& P, Eigen::Matrix<T, 2, 1>& p)
    {
        Eigen::Vector3d Pw((double)P(0), (double)P(1), (double)P(2));
        Eigen::Vector3d tw((double)t(0), (double)t(1), (double)t(2));
        Eigen::Quaterniond qw;
        
        qw.w() = (double) q[3];
        qw.x() = (double) q[0];
        qw.y() = (double) q[1];
        qw.z() = (double) q[2];
        qw.normalized();
        
        Eigen::Vector3d transP = qw.toRotationMatrix()*Pw + tw;
        T P_c[3];
        P_c[0] = T(transP(0));
        P_c[1] = T(transP(1));
        P_c[2] = T(transP(2));
        
        T k1 = params[0];
        T k2 = params[1];
        T p1 = params[2];
        T p2 = params[3];
        T fx = params[4];
        T fy = params[5];
        T alpha = T(0);
        T cx = params[6];
        T cy = params[7];

        T u = P_c[0]/P_c[2];
        T v = P_c[1]/P_c[2];

        T rho_sqr = u*u + v*v;
        T L = T(1.0) + k1*rho_sqr + k2*rho_sqr*rho_sqr;
        T du = T(2.0)*p1*u*v + p2*(rho_sqr + T(2.0)*u*u);
        T dv = p1*(rho_sqr + T(2.0)*v*v) + T(2.0)*p2*u*v;

        u = L*u + du;
        v = L*v + dv;
        p(0) = fx*(u + alpha*v) + cx;
        p(1) = fy*v + cy;
    }
    
}
