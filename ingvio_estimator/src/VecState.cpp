#include "VecState.h"

namespace ingvio
{
    void Vec3::update(const Eigen::VectorXd& dx)
    {
        assert(dx.rows() >= this->idx() + this->size());
        _vec += dx.block<3, 1>(this->idx(), 0);
    }
    
    std::shared_ptr<Vec3> Vec3::clone()
    {
        auto tmp = std::shared_ptr<Vec3>(new Vec3());
        
        tmp->setValue(this->value());
        tmp->setFej(this->fej());
        
        return tmp;
    }
    
    void Scalar::update(const Eigen::VectorXd& dx)
    {
        assert(dx.rows() >= this->idx() + this->size());
        _scalar += dx(this->idx());
    }
    
    void Scalar::setRandom()
    {
        Eigen::Vector2d tmp;
        tmp.setRandom();
        _scalar = tmp(0);
    }
    
    std::shared_ptr<Scalar> Scalar::clone()
    {
        auto tmp = std::shared_ptr<Scalar>(new Scalar());
        
        tmp->setValue(this->value());
        tmp->setFej(this->fej());
        
        return tmp;
    }
}

