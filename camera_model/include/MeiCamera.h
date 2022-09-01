#pragma once

#include "camera.h"
#include <cmath>

namespace camera_model
{
    class MeiCamera: public Camera
    {
    public:
        class Parameters: public Camera::Parameters
        {
        public:
            Parameters();
            Parameters(const std::string& cameraName, int w, int h, double xi, double k1, double k2, double p1, double p2, double gamma1, double gamma2, double u0, double v0);

            double& xi();
            double& k1();
            double& k2();
            double& p1();
            double& p2();
            double& gamma1();
            double& gamma2();
            double& u0();
            double& v0();

            double xi() const;
            double k1() const;
            double k2() const;
            double p1() const;
            double p2() const;
            double gamma1() const;
            double gamma2() const;
            double u0() const;
            double v0() const;
            
            bool readFromYamlFile(const std::string& filename) override;
            void writeToYamlFile(const std::string& filename) const override;
            
            Parameters& operator= (const Parameters& other);
            friend std::ostream& operator<< (std::ostream& out, const Parameters& params);
            
        private:
            double m_xi;
            double m_k1;
            double m_k2;
            double m_p1;
            double m_p2;
            double m_gamma1;
            double m_gamma2;
            double m_u0;
            double m_v0;
        };
        
        MeiCamera();
        
        MeiCamera(const std::string& cameraName, int imageWidth, int imageHeight, double xi, double k1, double k2, double p1, double p2, double gamma1, double gamma2, double u0, double v0);
        
        MeiCamera(const Parameters& params);
        
        Camera::ModelType modelType(void) const override;

        const std::string& cameraName(void) const override;

        int imageWidth(void) const override;

        int imageHeight(void) const override;
        
        double getNormalPixel() const override;
       
        void liftSphere(const Eigen::Vector2d& p, Eigen::Vector3d& P) const override;
        
        void liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const override;
        
        void spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p) const override;
        
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
    
    typedef boost::shared_ptr<MeiCamera> MeiCameraPtr;
    typedef boost::shared_ptr<const MeiCamera> MeiCameraConstPtr;
    
    template <typename T>
    void MeiCamera::spaceToPlane(const T* const params, const T* const q, const T* const t, const Eigen::Matrix<T, 3, 1>& P, Eigen::Matrix<T, 2, 1>& p)
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
        
        T xi = params[0];
        T k1 = params[1];
        T k2 = params[2];
        T p1 = params[3];
        T p2 = params[4];

        T gamma1 = params[5];
        T gamma2 = params[6];
        T alpha = T(0);
        T u0 = params[7];
        T v0 = params[8];
        
        T len = std::sqrt(P_c[0]*P_c[0] + P_c[1]*P_c[1] + P_c[2]*P_c[2]);
        P_c[0] /= len;
        P_c[1] /= len;
        P_c[2] /= len;
        
        T u = P_c[0]/(P_c[2] + xi);
        T v = P_c[1]/(P_c[2] + xi);
        
        T rho_sqr = u*u + v*v;
        T L = T(1.0) + k1*rho_sqr + k2*rho_sqr*rho_sqr;
        T du = T(2.0)*p1*u*v + p2*(rho_sqr + T(2.0)*u*u);
        T dv = p1*(rho_sqr + T(2.0)*v*v) + T(2.0)*p2*u*v;

        u = L*u + du;
        v = L*v + dv;

        p(0) = gamma1*(u + alpha*v) + u0;
        p(1) = gamma2*v + v0;        
    }
}
