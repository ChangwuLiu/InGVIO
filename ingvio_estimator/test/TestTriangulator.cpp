/**  This File is part of InGVIO, an invariant filter for mono/stereo visual-
 *    inertial-raw GNSS navigation. 
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
 */

#include <gtest/gtest.h>

#include <cmath>
#include "MapServer.h"
#include "Triangulator.h"
#include "GenerateNoise.h"

using namespace ingvio;

namespace ingvio_test
{    
    class TestTriangulator : public virtual ::testing::Test
    {
    public:
        TestTriangulator() : pf(Eigen::Vector3d(1.0, 2.0, 3.0))
        {
            T_cl2cr.linear() = Eigen::Matrix3d::Identity();
            T_cl2cr.translation() = Eigen::Vector3d(0.001, -0.12, 0.003);
            
            for (int i = 0; i < 10; ++i)
            {
                sw_poses1[0.1*i] = std::make_shared<SE3>();
                
                sw_poses1[0.1*i]->setValueLinearByMat(Eigen::AngleAxisd(generateGaussRandom(0.0, 0.1), Eigen::Vector3d(0, 0, 1)).toRotationMatrix());
                
                sw_poses1[0.1*i]->setValueTrans(Eigen::Vector3d(2*i-9.0, 2*i-9.0, 0.0));
                
                mono_obs1[0.1*i] = calcMonoMeas(pf, sw_poses1.at(0.1*i));
                
                stereo_obs1[0.1*i] = calcStereoMeas(pf, sw_poses1.at(0.1*i));
            }
            
            sw_poses2[0.1] = std::make_shared<SE3>();
            sw_poses2[0.1]->setValueLinearByQuat(Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, 1)));
            sw_poses2[0.1]->setValueTrans(Eigen::Vector3d(0, 0, 0));
            mono_obs2[0.1] = calcMonoMeas(pf, sw_poses2.at(0.1));
            stereo_obs2[0.1] = calcStereoMeas(pf, sw_poses2.at(0.1));
            
            sw_poses2[0.2] = std::make_shared<SE3>();
            sw_poses2[0.2]->setValueLinearByQuat(Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, -1, 0)));
            sw_poses2[0.2]->setValueTrans(Eigen::Vector3d(0, 5, 0));
            mono_obs2[0.2] = calcMonoMeas(pf, sw_poses2.at(0.2));
            stereo_obs2[0.2] = calcStereoMeas(pf, sw_poses2.at(0.2));
            
            sw_poses2[0.3] = std::make_shared<SE3>();
            sw_poses2[0.3]->setValueLinearByQuat(Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 0, -1)));
            sw_poses2[0.3]->setValueTrans(Eigen::Vector3d(0, 0, 8));
            mono_obs2[0.3] = calcMonoMeas(pf, sw_poses2.at(0.3));
            stereo_obs2[0.3] = calcStereoMeas(pf, sw_poses2.at(0.3));
            
            sw_poses2[0.4] = std::make_shared<SE3>();
            sw_poses2[0.4]->setValueLinearByQuat(Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(0, 1, 0)));
            sw_poses2[0.4]->setValueTrans(Eigen::Vector3d(0, -6, 0));
            mono_obs2[0.4] = calcMonoMeas(pf, sw_poses2.at(0.4));
            stereo_obs2[0.4] = calcStereoMeas(pf, sw_poses2.at(0.4));
            
            sw_poses2[0.5] = std::make_shared<SE3>();
            sw_poses2[0.5]->setValueLinearByQuat(Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(-1, 0, 0)));
            sw_poses2[0.5]->setValueTrans(Eigen::Vector3d(5.5, 0, 0));
            mono_obs2[0.5] = calcMonoMeas(pf, sw_poses2.at(0.5));
            stereo_obs2[0.5] = calcStereoMeas(pf, sw_poses2.at(0.5));
            
            sw_poses2[0.6] = std::make_shared<SE3>();
            sw_poses2[0.6]->setValueLinearByQuat(Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0, 0, 1), Eigen::Vector3d(1, 0, 0)));
            sw_poses2[0.6]->setValueTrans(Eigen::Vector3d(-10, 0, 0));
            mono_obs2[0.6] = calcMonoMeas(pf, sw_poses2.at(0.6));
            stereo_obs2[0.6] = calcStereoMeas(pf, sw_poses2.at(0.6));
        }
        
        virtual ~TestTriangulator() {}
    protected:
        Eigen::Vector3d pf;
        
        Eigen::Isometry3d T_cl2cr;
        
        std::map<double, std::shared_ptr<SE3>> sw_poses1, sw_poses2;
        
        std::map<double, std::shared_ptr<MonoMeas>> mono_obs1, mono_obs2;
        
        std::map<double, std::shared_ptr<StereoMeas>> stereo_obs1, stereo_obs2;
        
        std::shared_ptr<MonoMeas> calcMonoMeas(const Eigen::Vector3d& pf, const std::shared_ptr<SE3> pose)
        {
            std::shared_ptr<MonoMeas> mono_meas(new MonoMeas());
            
            Eigen::Vector3d body = pose->copyValueAsIso().inverse()*pf;
            
            mono_meas->_u0 = body.x()/body.z() + generateGaussRandom(0.0, 0.02);
            mono_meas->_v0 = body.y()/body.z() + generateGaussRandom(0.0, 0.02);
            
            return mono_meas;
        }
        
        std::shared_ptr<StereoMeas> calcStereoMeas(const Eigen::Vector3d& pf, const std::shared_ptr<SE3> pose)
        {
            std::shared_ptr<StereoMeas> stereo_meas(new StereoMeas());
            
            Eigen::Vector3d body_left = pose->copyValueAsIso().inverse()*pf;
            Eigen::Vector3d body_right = T_cl2cr*pose->copyValueAsIso().inverse()*pf;
            
            stereo_meas->_u0 = body_left.x()/body_left.z() + generateGaussRandom(0.0, 0.02);
            stereo_meas->_v0 = body_left.y()/body_left.z() + generateGaussRandom(0.0, 0.02);
            
            stereo_meas->_u1 = body_right.x()/body_right.z() + generateGaussRandom(0.0, 0.02);
            stereo_meas->_v1 = body_right.y()/body_right.z() + generateGaussRandom(0.0, 0.02);
            
            // std::cout << " u0 = " << stereo_meas->_u0 << " v0 = " << stereo_meas->_v0 << " u1 = " << stereo_meas->_u1 << " v1 = " << stereo_meas->_v1 << std::endl;
            
            return stereo_meas;
        }
    };
    
    TEST_F(TestTriangulator, monoTriangulate)
    {
        std::shared_ptr<Triangulator> tri(new Triangulator());
        
        Eigen::Vector3d result;
        
        bool flag = tri->triangulateMonoObs(mono_obs1, sw_poses1, result);
        
        std::cout << "pf ref = " << pf.transpose() << std::endl;
        std::cout << "mono triangulation result = " << result.transpose() << std::endl;
        
        ASSERT_NEAR((pf-result).norm(), 0.0, 0.05);
        ASSERT_TRUE(flag);
        
        flag = tri->triangulateMonoObs(mono_obs2, sw_poses2, result);
        
        std::cout << "pf ref = " << pf.transpose() << std::endl;
        std::cout << "mono triangulation result = " << result.transpose() << std::endl;
        
        ASSERT_NEAR((pf-result).norm(), 0.0, 0.15);
        ASSERT_TRUE(flag);
    }
    
    TEST_F(TestTriangulator, stereoTriangulate)
    {
        std::shared_ptr<Triangulator> tri(new Triangulator());
        
        Eigen::Vector3d result;
        
        bool flag = tri->triangulateStereoObs(stereo_obs1, sw_poses1, T_cl2cr, result);
        
        std::cout << "pf ref = " << pf.transpose() << std::endl;
        std::cout << "stereo triangulation result = " << result.transpose() << std::endl;
        
        ASSERT_NEAR((pf-result).norm(), 0.0, 0.05);
        ASSERT_TRUE(flag);
        
        flag = tri->triangulateStereoObs(stereo_obs2, sw_poses2, T_cl2cr, result);
        
        std::cout << "pf ref = " << pf.transpose() << std::endl;
        std::cout << "stereo triangulation result = " << result.transpose() << std::endl;
        
        ASSERT_NEAR((pf-result).norm(), 0.0, 0.15);
        ASSERT_TRUE(flag);
    }
    
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
