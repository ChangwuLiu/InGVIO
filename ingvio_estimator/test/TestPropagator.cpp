#include <gtest/gtest.h>

#include "IngvioParams.h"
#include "ImuPropagator.h"

using namespace ingvio;

namespace ingvio_test
{
    class TestPropagator : public virtual ::testing::Test
    {
    public:
        TestPropagator() : _N(300)
        {
            sf = Eigen::Vector3d::Random().normalized();
            sf *= 9.75;
        }
        
        virtual ~TestPropagator() {}
    protected:
        int _N;
        
        IngvioParams filter_params;
        
        Eigen::Vector3d sf;
    };
    
    TEST_F(TestPropagator, initGravity)
    {
        filter_params.readParams("/home/lcw/VIO/ws_ingvio/src/config/sportsfield/ingvio_stereo.yaml");
        filter_params.printParams();
        
        ImuPropagator ip1;
        
        filter_params._init_imu_buffer_sp = -1;
        ImuPropagator ip2(filter_params);
        
        for (int i = 0; i < _N; ++i)
        {
            ImuCtrl imu_ctrl;
            imu_ctrl._accel_raw = sf;
            imu_ctrl._gyro_raw.setZero();
            
            imu_ctrl._timestamp = 0.01*i;
            
            ip1.storeImu(imu_ctrl);
            ip2.storeImu(imu_ctrl);
        }
        
        auto isMatNear = [](const Eigen::MatrixXd& mat1, const Eigen::MatrixXd& mat2)
        {
            if ((mat1-mat2).norm() < 1e-08)
                return true;
            else
                return false;
        };
        
        auto isQuatNear = [](const Eigen::Quaterniond& quat1, const Eigen::Quaterniond& quat2)
        {
            if ((quat1.toRotationMatrix()-quat2.toRotationMatrix()).norm() < 1e-08)
                return true;
            else
                return false;
        };
        
        ASSERT_TRUE(isMatNear(Eigen::Vector3d(0, 0, -sf.norm()), ip1.getGravity()));
        ASSERT_TRUE(isMatNear(Eigen::Vector3d(0, 0, -9.8), ip2.getGravity()));
        
        Eigen::Quaterniond init_quat1, init_quat2;
        ASSERT_TRUE(ip1.getInitQuat(init_quat1));
        ASSERT_TRUE(ip2.getInitQuat(init_quat2));
        
        ASSERT_TRUE(isQuatNear(init_quat2, Eigen::Quaterniond::Identity()));
        
        Eigen::Quaterniond quat_ref = Eigen::Quaterniond::FromTwoVectors(-sf, Eigen::Vector3d(0, 0, -sf.norm()));
        
        ASSERT_TRUE(isQuatNear(init_quat1, quat_ref));
        
        Eigen::Quaterniond tmp_quat1, tmp_quat2;
        for (int i = 1; i < 400; i += 20)
        {
            ASSERT_TRUE(ip1.getAvgQuat(tmp_quat1, i));
            ASSERT_TRUE(ip2.getAvgQuat(tmp_quat2, i));
            ASSERT_TRUE(isQuatNear(quat_ref, tmp_quat1));
            ASSERT_TRUE(isQuatNear(quat_ref, tmp_quat2));
        }
    }
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
