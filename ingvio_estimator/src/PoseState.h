#pragma once

#include <Eigen/Geometry>

#include "AuxGammaFunc.h"
#include "VecState.h"

namespace ingvio
{
    class SO3 : public Type
    {
    public:
        SO3() : Type(3)
        {
            _q.setIdentity();
            _q_fej.setIdentity();
            _rot.setIdentity();
            _rot_fej.setIdentity();
        }
        
        ~SO3() {}
        
        void update(const Eigen::VectorXd& dx) override
        {
            assert(dx.rows() >= this->size() + this->idx());
            _rot = GammaFunc(dx.block<3, 1>(this->idx(), 0), 0)*_rot;
            _q = Eigen::Quaterniond(_rot);
        }
        
        void setIdentity() override
        {
            _q.setIdentity();
            _rot.setIdentity();
        }
        
        void setRandom() override
        {
            _q = Eigen::Quaterniond(Eigen::Vector4d::Random());
            _q.normalize();
            _rot = _q.toRotationMatrix();
        }
        
        const Eigen::Quaterniond& valueAsQuat() const
        { return _q; }
        
        const Eigen::Matrix3d& valueAsMat() const
        { return _rot; }
        
        const Eigen::Quaterniond& fejAsQuat() const
        { return _q_fej; }
        
        const Eigen::Matrix3d& fejAsMat() const
        { return _rot_fej; }
        
        void setValueByQuat(const Eigen::Quaterniond& other_quat)
        {
            _q = other_quat.normalized();
            _rot = _q.toRotationMatrix();
        }
        
        void setValueByMat(const Eigen::Matrix3d& other_mat)
        {
            _rot = other_mat;
            _q = Eigen::Quaterniond(_rot);
        }
        
        void setFejByQuat(const Eigen::Quaterniond& other_quat)
        {
            _q_fej = other_quat.normalized();
            _rot_fej = _q_fej.toRotationMatrix();
        }
        
        void setFejByMat(const Eigen::Matrix3d& other_mat)
        {
            _rot_fej = other_mat;
            _q_fej = Eigen::Quaterniond(_rot_fej);
        }
        
        std::shared_ptr<SO3> clone()
        {
            auto tmp = std::shared_ptr<SO3>(new SO3());
            
            tmp->setFejByQuat(this->fejAsQuat());
            tmp->setValueByQuat(this->valueAsQuat());
            
            return tmp;
        }
        
    protected:
        Eigen::Quaterniond _q;
        Eigen::Matrix3d _rot;
        
        Eigen::Quaterniond _q_fej;
        Eigen::Matrix3d _rot_fej;
    };
    
    class SE3 : public Type
    {
    public:
        SE3() : Type(6)
        {
            _q.setIdentity();
            _q_fej = _q;
            
            _rot.setIdentity();
            _rot_fej = _rot;
            
            _vec.setZero();
            _vec_fej = _vec;
        }
        
        ~SE3() {}
        
        void update(const Eigen::VectorXd& dx) override
        {
            assert(dx.rows() >= this->idx() + this->size());
            Eigen::Matrix3d Gamma0 = GammaFunc(dx.block<3, 1>(this->idx(), 0), 0);
            
            _rot = Gamma0*_rot;
            _q = Eigen::Quaterniond(_rot);
            
            _vec = Gamma0*_vec + GammaFunc(dx.block<3, 1>(this->idx(), 0), 1)*dx.block<3, 1>(this->idx()+3, 0);
        }
        
        void setIdentity() override
        {
            _q.setIdentity();
            _rot.setIdentity();
            _vec.setZero();
        }
        
        void setRandom() override
        {
            _q = Eigen::Quaterniond(Eigen::Vector4d::Random());
            _q.normalize();
            _rot = _q.toRotationMatrix();
            _vec.setRandom();
        }
        
        const Eigen::Quaterniond& valueLinearAsQuat() const
        { return _q; }
        
        const Eigen::Matrix3d& valueLinearAsMat() const
        { return _rot; }
        
        const Eigen::Vector3d& valueTrans() const
        { return _vec; }
        
        Eigen::Isometry3d copyValueAsIso() const
        {
            Eigen::Isometry3d result;
            result.setIdentity();
            result.linear() = _rot;
            result.translation() = _vec;
            
            return result;
        }
        
        const Eigen::Quaterniond& fejLinearAsQuat() const
        { return _q_fej; }
        
        const Eigen::Matrix3d& fejLinearAsMat() const
        { return _rot_fej; }
        
        const Eigen::Vector3d& fejTrans() const
        { return _vec_fej; }
        
        Eigen::Isometry3d copyFejAsIso() const
        {
            Eigen::Isometry3d result;
            result.setIdentity();
            result.linear() = _rot_fej;
            result.translation() = _vec_fej;
            
            return result;
        }
        
        void setValueLinearByQuat(const Eigen::Quaterniond& other_quat)
        {
            _q = other_quat.normalized();
            _rot = _q.toRotationMatrix();
        }
        
        void setValueLinearByMat(const Eigen::Matrix3d& other_mat)
        {
            _rot = other_mat;
            _q = Eigen::Quaterniond(_rot);
        }
        
        void setValueTrans(const Eigen::Vector3d& other_vec)
        {
            _vec = other_vec;
        }
        
        void setValueByIso(const Eigen::Isometry3d& other_iso)
        {
            this->setValueLinearByMat(other_iso.linear());
            this->setValueTrans(other_iso.translation());
        }
        
        void setFejLinearByQuat(const Eigen::Quaterniond& other_quat)
        {
            _q_fej = other_quat.normalized();
            _rot_fej = _q_fej.toRotationMatrix();
        }
        
        void setFejLinearByMat(const Eigen::Matrix3d& other_mat)
        {
            _rot_fej = other_mat;
            _q_fej = Eigen::Quaterniond(_rot_fej);
        }
        
        void setFejTrans(const Eigen::Vector3d& other_vec)
        {
            _vec_fej = other_vec;
        }
        
        void setFejByIso(const Eigen::Isometry3d& other_iso)
        {
            this->setFejLinearByMat(other_iso.linear());
            this->setFejTrans(other_iso.translation());
        }
        
        std::shared_ptr<SE3> clone()
        {
            auto tmp = std::shared_ptr<SE3>(new SE3());
            
            tmp->setValueLinearByQuat(this->valueLinearAsQuat());
            tmp->setValueTrans(this->valueTrans());
            
            tmp->setFejLinearByQuat(this->fejLinearAsQuat());
            tmp->setFejTrans(this->fejTrans());
            
            return tmp;
        }
        
    protected:
        Eigen::Quaterniond _q;
        Eigen::Matrix3d _rot;
        
        Eigen::Quaterniond _q_fej;
        Eigen::Matrix3d _rot_fej;
        
        Eigen::Vector3d _vec;
        Eigen::Vector3d _vec_fej;
    };
    
    class SE23 : public Type
    {
    public:
        SE23() : Type(9)
        {
            _q.setIdentity();
            _q_fej = _q;
            
            _rot.setIdentity();
            _rot_fej = _rot;
            
            _vec1.setZero();
            _vec1_fej = _vec1;
            
            _vec2.setZero();
            _vec2_fej = _vec2;
        }
        
        ~SE23() {}
        
        void update(const Eigen::VectorXd& dx) override
        {
            assert(dx.rows() >= this->idx() + this->size());
            
            Eigen::Matrix3d Gamma0 = GammaFunc(dx.block<3, 1>(this->idx(), 0), 0);
            Eigen::Matrix3d Gamma1 = GammaFunc(dx.block<3, 1>(this->idx(), 0), 1);
            
            _rot = Gamma0*_rot;
            _q = Eigen::Quaterniond(_rot);
            
            _vec1 = Gamma0*_vec1 + Gamma1*dx.block<3, 1>(this->idx()+3, 0);
            _vec2 = Gamma0*_vec2 + Gamma1*dx.block<3, 1>(this->idx()+6, 0);
        }
        
        void setIdentity() override
        {
            _q.setIdentity();
            _rot.setIdentity();
            _vec1.setZero();
            _vec2.setZero();
        }
        
        void setRandom() override
        {
            _q = Eigen::Quaterniond(Eigen::Vector4d::Random());
            _q.normalize();
            _rot = _q.toRotationMatrix();
            _vec1.setRandom();
            _vec2.setRandom();
        }
        
        const Eigen::Quaterniond& valueLinearAsQuat() const
        { return _q; }
        
        const Eigen::Matrix3d& valueLinearAsMat() const
        { return _rot; }
        
        const Eigen::Vector3d& valueTrans1() const
        { return _vec1; }
        
        const Eigen::Vector3d& valueTrans2() const
        { return _vec2; }
        
        Eigen::Isometry3d copyValueAsIso() const
        {
            Eigen::Isometry3d result;
            result.setIdentity();
            result.linear() = _rot;
            result.translation() = _vec1;
            
            return result;
        }
        
        const Eigen::Quaterniond& fejLinearAsQuat() const
        { return _q_fej; }
        
        const Eigen::Matrix3d& fejLinearAsMat() const
        { return _rot_fej; }
        
        const Eigen::Vector3d& fejTrans1() const
        { return _vec1_fej; }
        
        const Eigen::Vector3d& fejTrans2() const
        { return _vec2_fej; }
        
        Eigen::Isometry3d copyFejAsIso() const
        {
            Eigen::Isometry3d result;
            result.setIdentity();
            result.linear() = _rot_fej;
            result.translation() = _vec1_fej;
            
            return result;
        }
        
        void setValueLinearByQuat(const Eigen::Quaterniond& other_quat)
        {
            _q = other_quat.normalized();
            _rot = _q.toRotationMatrix();
        }
        
        void setValueLinearByMat(const Eigen::Matrix3d& other_mat)
        {
            _rot = other_mat;
            _q = Eigen::Quaterniond(_rot);
        }
        
        void setValueTrans1(const Eigen::Vector3d& other_vec)
        {
            _vec1 = other_vec;
        }
        
        void setValueTrans2(const Eigen::Vector3d& other_vec)
        {
            _vec2 = other_vec;
        }
        
        void setFejLinearByQuat(const Eigen::Quaterniond& other_quat)
        {
            _q_fej = other_quat.normalized();
            _rot_fej = _q_fej.toRotationMatrix();
        }
        
        void setFejLinearByMat(const Eigen::Matrix3d& other_mat)
        {
            _rot_fej = other_mat;
            _q_fej = Eigen::Quaterniond(_rot_fej);
        }
        
        void setFejTrans1(const Eigen::Vector3d& other_vec)
        {
            _vec1_fej = other_vec;
        }
        
        void setFejTrans2(const Eigen::Vector3d& other_vec)
        {
            _vec2_fej = other_vec;
        }
        
        std::shared_ptr<SE23> clone()
        {
            auto tmp = std::shared_ptr<SE23>(new SE23());
            
            tmp->setValueLinearByQuat(this->valueLinearAsQuat());
            tmp->setValueTrans1(this->valueTrans1());
            tmp->setValueTrans2(this->valueTrans2());
            
            tmp->setFejLinearByQuat(this->fejLinearAsQuat());
            tmp->setFejTrans1(this->fejTrans1());
            tmp->setFejTrans2(this->fejTrans2());
            
            return tmp;
        }
        
    protected:
        Eigen::Quaterniond _q;
        Eigen::Quaterniond _q_fej;
        
        Eigen::Matrix3d _rot;
        Eigen::Matrix3d _rot_fej;
        
        Eigen::Vector3d _vec1;
        Eigen::Vector3d _vec1_fej;
        
        Eigen::Vector3d _vec2;
        Eigen::Vector3d _vec2_fej;
    };
    
}
