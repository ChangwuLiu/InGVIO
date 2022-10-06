#pragma once

#include <memory>
#include <Eigen/Core>

namespace ingvio
{
    class Type
    {
    public:
        Type(int size) { _size = size; }
        virtual ~Type(){}
        
        virtual void set_cov_idx(int new_cov_idx)
        {
            _idx = new_cov_idx;
        }
        
        int idx() const { return _idx; }
        int size() const {return _size; }
        
        virtual void update(const Eigen::VectorXd& dx) = 0;
        
        virtual void setIdentity() = 0;
        virtual void setRandom() = 0;
        
    protected:
        int _idx = -1;
        int _size = -1;
    };
    
    class Vec3 : public Type
    {
    public:
        Vec3() : Type(3)
        {
            _vec.setZero();
            _vec_fej.setZero();
        }
        
        ~Vec3() {}
        
        void update(const Eigen::VectorXd& dx) override;
        
        void setIdentity() override
        {
            _vec.setZero();
        }
        void setRandom() override
        {
            _vec.setRandom();
        }
        
        const Eigen::Vector3d& value() const 
        { return _vec; }
        
        const Eigen::Vector3d& fej() const 
        { return _vec_fej; }
        
        void setValue(const Eigen::Vector3d& other_vec)
        { _vec = other_vec; }
        
        void setFej(const Eigen::Vector3d& other_vec_fej) 
        { _vec_fej = other_vec_fej; }
        
        std::shared_ptr<Vec3> clone();
        
    protected:
        Eigen::Vector3d _vec;
        Eigen::Vector3d _vec_fej;
    };
    
    class Scalar : public Type
    {
    public:
        Scalar() : Type(1)
        {
            _scalar = 0.0;
            _scalar_fej = 0.0;
        }
        
        ~Scalar() {}
        
        void update(const Eigen::VectorXd& dx) override;
        
        void setIdentity() override
        {
            _scalar = 0.0;
        }
        
        void setRandom() override;
        
        const double& value() const
        { return _scalar; }
        
        const double& fej() const 
        { return _scalar_fej; }
        
        void setValue(const double& other_scalar) 
        { _scalar = other_scalar; }
        
        void setFej(const double& other_scalar_fej) 
        { _scalar_fej = other_scalar_fej; }
        
        std::shared_ptr<Scalar> clone();
        
    protected:
        double _scalar;
        double _scalar_fej;
    };
}
