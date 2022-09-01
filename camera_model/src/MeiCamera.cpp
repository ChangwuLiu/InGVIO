#include "MeiCamera.h"
#include <iomanip>

namespace camera_model
{
    MeiCamera::Parameters::Parameters(): Camera::Parameters(MEI), m_xi(0.0), m_k1(0.0), m_k2(0.0), m_p1(0.0), m_p2(0.0), m_gamma1(0.0), m_gamma2(0.0), m_u0(0.0), m_v0(0.0)
    {}
    
    MeiCamera::Parameters::Parameters(const std::string& cameraName, int w, int h,  double xi, double k1, double k2, double p1, double p2, double gamma1, double gamma2, double u0, double v0) : Camera::Parameters(MEI, cameraName, w, h), m_xi(xi), m_k1(k1), m_k2(k2), m_p1(p1), m_p2(p2), m_gamma1(gamma1), m_gamma2(gamma2), m_u0(u0), m_v0(v0)
    {}
    
    double& MeiCamera::Parameters::xi() { return m_xi;}

    double& MeiCamera::Parameters::k1() { return m_k1;}

    double& MeiCamera::Parameters::k2() { return m_k2;}

    double& MeiCamera::Parameters::p1() { return m_p1;}

    double& MeiCamera::Parameters::p2() { return m_p2;}

    double& MeiCamera::Parameters::gamma1() { return m_gamma1;}

    double& MeiCamera::Parameters::gamma2() { return m_gamma2;}

    double& MeiCamera::Parameters::u0() { return m_u0;}

    double& MeiCamera::Parameters::v0() { return m_v0;}

    double MeiCamera::Parameters::xi() const { return m_xi;}

    double MeiCamera::Parameters::k1() const { return m_k1;}

    double MeiCamera::Parameters::k2() const { return m_k2;}

    double MeiCamera::Parameters::p1() const { return m_p1;}

    double MeiCamera::Parameters::p2() const { return m_p2;}

    double MeiCamera::Parameters::gamma1() const { return m_gamma1;}

    double MeiCamera::Parameters::gamma2() const { return m_gamma2;}

    double MeiCamera::Parameters::u0() const { return m_u0;}

    double MeiCamera::Parameters::v0() const { return m_v0;}

    bool MeiCamera::Parameters::readFromYamlFile(const std::string& filename)
    {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        
        if (!fs.isOpened()) return false;
        
        if (!fs["model_type"].isNone())
        {
            std::string sModelType;
            fs["model_type"] >> sModelType;
            
            if (sModelType.compare("MEI") != 0) return false;
        }
        
        m_modelType = MEI;
        fs["camera_name"] >> m_cameraName;
        m_imageWidth = static_cast<int>(fs["image_width"]);
        m_imageHeight = static_cast<int>(fs["image_height"]);

        cv::FileNode n = fs["mirror_parameters"];
        m_xi = static_cast<double>(n["xi"]);

        n = fs["distortion_parameters"];
        m_k1 = static_cast<double>(n["k1"]);
        m_k2 = static_cast<double>(n["k2"]);
        m_p1 = static_cast<double>(n["p1"]);
        m_p2 = static_cast<double>(n["p2"]);
        
        n = fs["projection_parameters"];
        m_gamma1 = static_cast<double>(n["gamma1"]);
        m_gamma2 = static_cast<double>(n["gamma2"]);
        m_u0 = static_cast<double>(n["u0"]);
        m_v0 = static_cast<double>(n["v0"]);
    
        return true;
    }
    
    void MeiCamera::Parameters::writeToYamlFile(const std::string& filename) const
    {
        cv::FileStorage fs(filename, cv::FileStorage::WRITE);

        fs << "model_type" << "MEI";
        fs << "camera_name" << m_cameraName;
        fs << "image_width" << m_imageWidth;
        fs << "image_height" << m_imageHeight;

        fs << "mirror_parameters";
        fs << "{" << "xi" << m_xi << "}";
    
        fs << "distortion_parameters";
        fs << "{" << "k1" << m_k1
                  << "k2" << m_k2
                  << "p1" << m_p1
                  << "p2" << m_p2 << "}";
        
        fs << "projection_parameters";
        fs << "{" << "gamma1" << m_gamma1
                  << "gamma2" << m_gamma2
                  << "u0" << m_u0
                  << "v0" << m_v0 << "}";

        fs.release();
    }
    
    MeiCamera::Parameters& MeiCamera::Parameters::operator=(const MeiCamera::Parameters& other)
    {
        if (this != &other)
        {
            m_modelType = other.m_modelType;
            m_cameraName = other.m_cameraName;
            m_imageWidth = other.m_imageWidth;
            m_imageHeight = other.m_imageHeight;
            m_xi = other.m_xi;
            m_k1 = other.m_k1;
            m_k2 = other.m_k2;
            m_p1 = other.m_p1;
            m_p2 = other.m_p2;
            m_gamma1 = other.m_gamma1;
            m_gamma2 = other.m_gamma2;
            m_u0 = other.m_u0;
            m_v0 = other.m_v0;
        }
    
        return *this;
    }
    
    std::ostream& operator<< (std::ostream& out, const MeiCamera::Parameters& params)
    {
        out << "Camera Parameters:" << std::endl;
        out << "    model_type " << "MEI" << std::endl;
        out << "   camera_name " << params.m_cameraName << std::endl;
        out << "   image_width " << params.m_imageWidth << std::endl;
        out << "  image_height " << params.m_imageHeight << std::endl;

        out << "Mirror Parameters" << std::endl;
        out << std::fixed << std::setprecision(10);
        out << "            xi " << params.m_xi << std::endl;

        out << "Distortion Parameters" << std::endl;
        out << "            k1 " << params.m_k1 << std::endl
            << "            k2 " << params.m_k2 << std::endl
            << "            p1 " << params.m_p1 << std::endl
            << "            p2 " << params.m_p2 << std::endl;

        out << "Projection Parameters" << std::endl;
        out << "        gamma1 " << params.m_gamma1 << std::endl
            << "        gamma2 " << params.m_gamma2 << std::endl
            << "            u0 " << params.m_u0 << std::endl
            << "            v0 " << params.m_v0 << std::endl;

        return out;
    }
    
    MeiCamera::MeiCamera(): m_inv_K11(1.0), m_inv_K13(0.0), m_inv_K22(1.0), m_inv_K23(0.0), m_noDistortion(true)
    {}
    
    MeiCamera::MeiCamera(const std::string& cameraName, int imageWidth, int imageHeight, double xi, double k1, double k2, double p1, double p2, double gamma1, double gamma2, double u0, double v0) : mParameters(cameraName, imageWidth, imageHeight, xi, k1, k2, p1, p2, gamma1, gamma2, u0, v0)
    {
        if ((mParameters.k1() == 0.0) && (mParameters.k2() == 0.0) && (mParameters.p1() == 0.0) && (mParameters.p2() == 0.0))
        {
            m_noDistortion = true;
        }
        else
        {
            m_noDistortion = false;
        }
        
        m_inv_K11 = 1.0 / mParameters.gamma1();
        m_inv_K13 = -mParameters.u0() / mParameters.gamma1();
        m_inv_K22 = 1.0 / mParameters.gamma2();
        m_inv_K23 = -mParameters.v0() / mParameters.gamma2();
    }
    
    MeiCamera::MeiCamera(const MeiCamera::Parameters& params) : mParameters(params)
    {
        if ((mParameters.k1() == 0.0) && (mParameters.k2() == 0.0) && (mParameters.p1() == 0.0) && (mParameters.p2() == 0.0))
        {
            m_noDistortion = true;
        }
        else
        {
            m_noDistortion = false;
        }
        
        m_inv_K11 = 1.0 / mParameters.gamma1();
        m_inv_K13 = -mParameters.u0() / mParameters.gamma1();
        m_inv_K22 = 1.0 / mParameters.gamma2();
        m_inv_K23 = -mParameters.v0() / mParameters.gamma2();
    }
    
    Camera::ModelType MeiCamera::modelType() const
    {
        return mParameters.modelType();
    }

    const std::string& MeiCamera::cameraName() const
    {
        return mParameters.cameraName();
    }

    int MeiCamera::imageWidth() const
    {
        return mParameters.imageWidth();
    }

    int MeiCamera::imageHeight() const
    {
        return mParameters.imageHeight();
    }
    
    double MeiCamera::getNormalPixel() const
    {
        return 2.0/(mParameters.gamma1() + mParameters.gamma2());
    }
    
    void MeiCamera::liftSphere(const Eigen::Vector2d& p, Eigen::Vector3d& P) const
    {
        double mx_d, my_d,mx2_d, mxy_d, my2_d, mx_u, my_u;
        double rho2_d, rho4_d, radDist_d, Dx_d, Dy_d, inv_denom_d;
        double lambda;
        
        mx_d = m_inv_K11 * p(0) + m_inv_K13;
        my_d = m_inv_K22 * p(1) + m_inv_K23;
        
        if (m_noDistortion)
        {
            mx_u = mx_d;
            my_u = my_d;
        }
        else
        {
            double k1 = mParameters.k1();
            double k2 = mParameters.k2();
            double p1 = mParameters.p1();
            double p2 = mParameters.p2();
            
            mx2_d = mx_d*mx_d;
            my2_d = my_d*my_d;
            mxy_d = mx_d*my_d;
            rho2_d = mx2_d+my2_d;
            rho4_d = rho2_d*rho2_d;
            radDist_d = k1*rho2_d+k2*rho4_d;
            Dx_d = mx_d*radDist_d + p2*(rho2_d+2*mx2_d) + 2*p1*mxy_d;
            Dy_d = my_d*radDist_d + p1*(rho2_d+2*my2_d) + 2*p2*mxy_d;
            inv_denom_d = 1/(1+4*k1*rho2_d+6*k2*rho4_d+8*p1*my_d+8*p2*mx_d);

            mx_u = mx_d - inv_denom_d*Dx_d;
            my_u = my_d - inv_denom_d*Dy_d;
        }
        
        double xi = mParameters.xi();
        if (xi == 1.0)
        {
            lambda = 2.0/(mx_u*mx_u + my_u*my_u + 1.0);
            P << lambda*mx_u, lambda*my_u, lambda - 1.0;
        }
        else
        {
            lambda = (xi + std::sqrt(1.0 + (1.0 - xi*xi)*(mx_u*mx_u + my_u*my_u))) / (1.0 + mx_u*mx_u + my_u*my_u);
            P << lambda*mx_u, lambda*my_u, lambda - xi;
        }
        
    }
    
    void MeiCamera::liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const
    {
        double mx_d, my_d,mx2_d, mxy_d, my2_d, mx_u, my_u;
        double rho2_d, rho4_d, radDist_d, Dx_d, Dy_d, inv_denom_d;

        mx_d = m_inv_K11 * p(0) + m_inv_K13;
        my_d = m_inv_K22 * p(1) + m_inv_K23;

        if (m_noDistortion)
        {
            mx_u = mx_d;
            my_u = my_d;
        }
        else
        {
            double k1 = mParameters.k1();
            double k2 = mParameters.k2();
            double p1 = mParameters.p1();
            double p2 = mParameters.p2();

            mx2_d = mx_d*mx_d;
            my2_d = my_d*my_d;
            mxy_d = mx_d*my_d;
            rho2_d = mx2_d+my2_d;
            rho4_d = rho2_d*rho2_d;
            radDist_d = k1*rho2_d+k2*rho4_d;
            Dx_d = mx_d*radDist_d + p2*(rho2_d+2*mx2_d) + 2*p1*mxy_d;
            Dy_d = my_d*radDist_d + p1*(rho2_d+2*my2_d) + 2*p2*mxy_d;
            inv_denom_d = 1/(1+4*k1*rho2_d+6*k2*rho4_d+8*p1*my_d+8*p2*mx_d);

            mx_u = mx_d - inv_denom_d*Dx_d;
            my_u = my_d - inv_denom_d*Dy_d;
        }

        double xi = mParameters.xi();
        if (xi == 1.0)
        {
            P << mx_u, my_u, (1.0 - mx_u*mx_u - my_u*my_u)/2.0;
        }
        else
        {
            rho2_d = mx_u*mx_u + my_u*my_u;
            P << mx_u, my_u, 1.0 - xi*(rho2_d + 1.0)/(xi + std::sqrt(1.0 + (1.0 - xi*xi)*rho2_d));
        }
        
        P(0) = P(0)/P(2);
        P(1) = P(1)/P(2);
        P(2) = 1.0;
    }
    
    void MeiCamera::spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p) const
    {
        Eigen::Vector2d p_u, p_d;

        double z = P(2) + mParameters.xi() * P.norm();
        p_u << P(0) / z, P(1) / z;

        if (m_noDistortion)
        {
            p_d = p_u;
        }
        else
        {
            Eigen::Vector2d d_u;
            distortion(p_u, d_u);
            p_d = p_u + d_u;
        }

        p << mParameters.gamma1()*p_d(0) + mParameters.u0(),
             mParameters.gamma2()*p_d(1) + mParameters.v0();
    }
    
    void MeiCamera::undistToPlane(const Eigen::Vector2d& p_u, Eigen::Vector2d& p) const
    {
        Eigen::Vector2d p_d;

        if (m_noDistortion)
        {
            p_d = p_u;
        }
        else
        {
            Eigen::Vector2d d_u;
            distortion(p_u, d_u);
            p_d = p_u + d_u;
        }

        p << mParameters.gamma1()*p_d(0) + mParameters.u0(),
             mParameters.gamma2()*p_d(1) + mParameters.v0();
    }
    
    void MeiCamera::distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u) const
    {
        double k1 = mParameters.k1();
        double k2 = mParameters.k2();
        double p1 = mParameters.p1();
        double p2 = mParameters.p2();
        
        double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;
        
        mx2_u = p_u(0)*p_u(0);
        my2_u = p_u(1)*p_u(1);
        mxy_u = p_u(0)*p_u(1);
        
        rho2_u = mx2_u + my2_u;
        rad_dist_u = k1*rho2_u + k2*rho2_u*rho2_u;
        
        d_u(0) = p_u(0)*rad_dist_u + 2.0*p1*mxy_u + p2*(rho2_u + 2.0*mx2_u);
        d_u(1) = p_u(1)*rad_dist_u + 2.0*p2*mxy_u + p1*(rho2_u + 2.0*my2_u);
    }
    
    void MeiCamera::distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u, Eigen::Matrix2d& J) const
    {
        double k1 = mParameters.k1();
        double k2 = mParameters.k2();
        double p1 = mParameters.p1();
        double p2 = mParameters.p2();
        
        double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;
        
        mx2_u = p_u(0)*p_u(0);
        my2_u = p_u(1)*p_u(1);
        mxy_u = p_u(0)*p_u(1);
        
        rho2_u = mx2_u + my2_u;
        rad_dist_u = k1*rho2_u + k2*rho2_u*rho2_u;
        
        d_u(0) = p_u(0)*rad_dist_u + 2.0*p1*mxy_u + p2*(rho2_u + 2.0*mx2_u);
        d_u(1) = p_u(1)*rad_dist_u + 2.0*p2*mxy_u + p1*(rho2_u + 2.0*my2_u);

        J(0, 0) = 1.0 + rad_dist_u + k1*2.0*mx2_u + k2*rho2_u*4.0*mx2_u + 2.0*p1*p_u(1) + 6.0*p2*p_u(0);
        J(0, 1) = k1*2.0*p_u(0)*p_u(1) + k2*4.0*rho2_u*p_u(0)*p_u(1) + p1*2.0*p_u(0) + 2.0*p2*p_u(1);       
        J(1, 0) = J(0, 1);
        J(1, 1) = 1.0 + rad_dist_u + k1*2.0*my2_u + k2*rho2_u*4.0*my2_u + 6.0*p1*p_u(1) + 2.0*p2*p_u(0);
    }
    
    int MeiCamera::parameterCount() const
    {
        return 9;
    }

    const MeiCamera::Parameters& MeiCamera::getParameters() const
    {
        return mParameters;
    }

    void MeiCamera::setParameters(const MeiCamera::Parameters& parameters)
    {
        mParameters = parameters;

        if ((mParameters.k1() == 0.0) && (mParameters.k2() == 0.0) && (mParameters.p1() == 0.0) && (mParameters.p2() == 0.0))
        {
            m_noDistortion = true;
        }
        else
        {
            m_noDistortion = false;
        }

        m_inv_K11 = 1.0 / mParameters.gamma1();
        m_inv_K13 = -mParameters.u0() / mParameters.gamma1();
        m_inv_K22 = 1.0 / mParameters.gamma2();
        m_inv_K23 = -mParameters.v0() / mParameters.gamma2();
    }
    
    void MeiCamera::readParameters(const std::vector<double>& parameterVec)
    {
        if ((int)parameterVec.size() != parameterCount())
            return;

        Parameters params = getParameters();

        params.xi() = parameterVec.at(0);
        params.k1() = parameterVec.at(1);
        params.k2() = parameterVec.at(2);
        params.p1() = parameterVec.at(3);
        params.p2() = parameterVec.at(4);
        params.gamma1() = parameterVec.at(5);
        params.gamma2() = parameterVec.at(6);
        params.u0() = parameterVec.at(7);
        params.v0() = parameterVec.at(8);

        setParameters(params);
    }
    
    void MeiCamera::writeParameters(std::vector<double>& parameterVec) const
    {
        parameterVec.resize(parameterCount());
        parameterVec.at(0) = mParameters.xi();
        parameterVec.at(1) = mParameters.k1();
        parameterVec.at(2) = mParameters.k2();
        parameterVec.at(3) = mParameters.p1();
        parameterVec.at(4) = mParameters.p2();
        parameterVec.at(5) = mParameters.gamma1();
        parameterVec.at(6) = mParameters.gamma2();
        parameterVec.at(7) = mParameters.u0();
        parameterVec.at(8) = mParameters.v0();
    }

    void MeiCamera::writeParametersToYamlFile(const std::string& filename) const
    {
        mParameters.writeToYamlFile(filename);
    }

    std::string MeiCamera::parametersToString() const
    {
        std::ostringstream oss;
        oss << mParameters;

        return oss.str();
    }

}
