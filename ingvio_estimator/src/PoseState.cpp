#include "PoseState.h"

namespace ingvio
{
    void SO3::update(const Eigen::VectorXd& dx)
    {
        assert(dx.rows() >= this->size() + this->idx());
        _rot = GammaFunc(dx.block<3, 1>(this->idx(), 0), 0)*_rot;
        _q = Eigen::Quaterniond(_rot);
    }
    
    void SO3::setIdentity()
    {
        _q.setIdentity();
        _rot.setIdentity();
    }
    
    void SO3::setRandom()
    {
        _q = Eigen::Quaterniond(Eigen::Vector4d::Random());
        _q.normalize();
        _rot = _q.toRotationMatrix();
    }
    
    void SO3::setValueByQuat(const Eigen::Quaterniond& other_quat)
    {
        _q = other_quat.normalized();
        _rot = _q.toRotationMatrix();
    }
    
    void SO3::setValueByMat(const Eigen::Matrix3d& other_mat)
    {
        _rot = other_mat;
        _q = Eigen::Quaterniond(_rot);
    }
    
    void SO3::setFejByQuat(const Eigen::Quaterniond& other_quat)
    {
        _q_fej = other_quat.normalized();
        _rot_fej = _q_fej.toRotationMatrix();
    }
    
    void SO3::setFejByMat(const Eigen::Matrix3d& other_mat)
    {
        _rot_fej = other_mat;
        _q_fej = Eigen::Quaterniond(_rot_fej);
    }
    
    std::shared_ptr<SO3> SO3::clone()
    {
        auto tmp = std::shared_ptr<SO3>(new SO3());
        
        tmp->setFejByQuat(this->fejAsQuat());
        tmp->setValueByQuat(this->valueAsQuat());
        
        return tmp;
    }
    
    void SE3::update(const Eigen::VectorXd& dx)
    {
        assert(dx.rows() >= this->idx() + this->size());
        Eigen::Matrix3d Gamma0 = GammaFunc(dx.block<3, 1>(this->idx(), 0), 0);
        
        _rot = Gamma0*_rot;
        _q = Eigen::Quaterniond(_rot);
        
        _vec = Gamma0*_vec + GammaFunc(dx.block<3, 1>(this->idx(), 0), 1)*dx.block<3, 1>(this->idx()+3, 0);
    }
    
    void SE3::setIdentity()
    {
        _q.setIdentity();
        _rot.setIdentity();
        _vec.setZero();
    }
    
    void SE3::setRandom()
    {
        _q = Eigen::Quaterniond(Eigen::Vector4d::Random());
        _q.normalize();
        _rot = _q.toRotationMatrix();
        _vec.setRandom();
    }
    
    Eigen::Isometry3d SE3::copyValueAsIso() const
    {
        Eigen::Isometry3d result;
        result.setIdentity();
        result.linear() = _rot;
        result.translation() = _vec;
        
        return result;
    }
    
    Eigen::Isometry3d SE3::copyFejAsIso() const
    {
        Eigen::Isometry3d result;
        result.setIdentity();
        result.linear() = _rot_fej;
        result.translation() = _vec_fej;
        
        return result;
    }
    
    void SE3::setValueLinearByQuat(const Eigen::Quaterniond& other_quat)
    {
        _q = other_quat.normalized();
        _rot = _q.toRotationMatrix();
    }
    
    void SE3::setValueLinearByMat(const Eigen::Matrix3d& other_mat)
    {
        _rot = other_mat;
        _q = Eigen::Quaterniond(_rot);
    }
    
    void SE3::setValueByIso(const Eigen::Isometry3d& other_iso)
    {
        this->setValueLinearByMat(other_iso.linear());
        this->setValueTrans(other_iso.translation());
    }
    
    void SE3::setFejLinearByQuat(const Eigen::Quaterniond& other_quat)
    {
        _q_fej = other_quat.normalized();
        _rot_fej = _q_fej.toRotationMatrix();
    }
    
    void SE3::setFejLinearByMat(const Eigen::Matrix3d& other_mat)
    {
        _rot_fej = other_mat;
        _q_fej = Eigen::Quaterniond(_rot_fej);
    }
    
    void SE3::setFejByIso(const Eigen::Isometry3d& other_iso)
    {
        this->setFejLinearByMat(other_iso.linear());
        this->setFejTrans(other_iso.translation());
    }
    
    std::shared_ptr<SE3> SE3::clone()
    {
        auto tmp = std::shared_ptr<SE3>(new SE3());
        
        tmp->setValueLinearByQuat(this->valueLinearAsQuat());
        tmp->setValueTrans(this->valueTrans());
        
        tmp->setFejLinearByQuat(this->fejLinearAsQuat());
        tmp->setFejTrans(this->fejTrans());
        
        return tmp;
    }
    
    void SE23::update(const Eigen::VectorXd& dx)
    {
        assert(dx.rows() >= this->idx() + this->size());
        
        Eigen::Matrix3d Gamma0 = GammaFunc(dx.block<3, 1>(this->idx(), 0), 0);
        Eigen::Matrix3d Gamma1 = GammaFunc(dx.block<3, 1>(this->idx(), 0), 1);
        
        _rot = Gamma0*_rot;
        _q = Eigen::Quaterniond(_rot);
        
        _vec1 = Gamma0*_vec1 + Gamma1*dx.block<3, 1>(this->idx()+3, 0);
        _vec2 = Gamma0*_vec2 + Gamma1*dx.block<3, 1>(this->idx()+6, 0);
    }
    
    void SE23::setIdentity()
    {
        _q.setIdentity();
        _rot.setIdentity();
        _vec1.setZero();
        _vec2.setZero();
    }
    
    void SE23::setRandom()
    {
        _q = Eigen::Quaterniond(Eigen::Vector4d::Random());
        _q.normalize();
        _rot = _q.toRotationMatrix();
        _vec1.setRandom();
        _vec2.setRandom();
    }
    
    Eigen::Isometry3d SE23::copyValueAsIso() const
    {
        Eigen::Isometry3d result;
        result.setIdentity();
        result.linear() = _rot;
        result.translation() = _vec1;
        
        return result;
    }
    
    Eigen::Isometry3d SE23::copyFejAsIso() const
    {
        Eigen::Isometry3d result;
        result.setIdentity();
        result.linear() = _rot_fej;
        result.translation() = _vec1_fej;
        
        return result;
    }
    
    void SE23::setValueLinearByQuat(const Eigen::Quaterniond& other_quat)
    {
        _q = other_quat.normalized();
        _rot = _q.toRotationMatrix();
    }
    
    void SE23::setValueLinearByMat(const Eigen::Matrix3d& other_mat)
    {
        _rot = other_mat;
        _q = Eigen::Quaterniond(_rot);
    }
    
    void SE23::setFejLinearByQuat(const Eigen::Quaterniond& other_quat)
    {
        _q_fej = other_quat.normalized();
        _rot_fej = _q_fej.toRotationMatrix();
    }
    
    void SE23::setFejLinearByMat(const Eigen::Matrix3d& other_mat)
    {
        _rot_fej = other_mat;
        _q_fej = Eigen::Quaterniond(_rot_fej);
    }
    
    std::shared_ptr<SE23> SE23::clone()
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
}
