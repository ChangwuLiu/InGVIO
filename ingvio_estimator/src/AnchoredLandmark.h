#pragma once

#include <iostream>
#include "PoseState.h"

namespace ingvio
{
    class AnchoredLandmark : public Type
    {
    public:
        enum LmRep {XYZ, INV_DEPTH, BEARING};
        
        AnchoredLandmark() : Type(3)
        {
            _pos_rep.setZero();
            _pos_rep_fej.setZero();
            
            _pos_xyz.setZero();
            _pos_xyz_fej.setZero();
            
            _landmark_param_type = INV_DEPTH;
            _has_rep_set = false;
            
            _anchored_pose.reset();
        }
        
        ~AnchoredLandmark() {}
        
        void update(const Eigen::VectorXd& dx) override
        {
            assert(dx.rows() >= this->idx() + this->size());
            const Eigen::Vector3d& delta_p = dx.block<3, 1>(this->idx(), 0);
            
            if (_anchored_pose != nullptr)
            {
                assert(_anchored_pose->idx() + _anchored_pose->size() <= dx.rows());
                const Eigen::Vector3d& delta_theta = dx.block<3, 1>(_anchored_pose->idx(), 0);
                this->setValuePosXyz(GammaFunc(delta_theta, 0)*this->valuePosXyz() + GammaFunc(delta_theta, 1)*delta_p);
            }
            else
            {
                std::cout << "[AnchoredLandmark]: Warning! Update without anchored pose!" << std::endl;
                this->setValuePosXyz(this->valuePosXyz() + delta_p);
            }
        }
        
        void setIdentity() override
        {
            _pos_rep.setZero();
            _pos_rep_fej.setZero();
            
            _pos_xyz.setZero();
            _pos_xyz_fej.setZero();
        }
        
        void setRandom() override
        {
            setValuePosXyz(Eigen::Vector3d::Random());
        }
        
        void setRepType(const AnchoredLandmark::LmRep& other_rep_type)
        {
            if (!_has_rep_set)
            {
                _landmark_param_type = other_rep_type;
                _has_rep_set = true;
            }
            else
                std::cout << "[AnchoredLandmark]: Representation type has been set once! Cannot change!" << std::endl;
        }
        
        static void switchRepXyz2Invd(const Eigen::Vector3d& xyz, Eigen::Vector3d& invd)
        {
            assert(xyz.z() != 0.0);
            if (xyz.z() < 0)
                std::cout << "[AnchoredLandmark]: Warning xyz.z() < 0 !" << std::endl;
            
            invd.z() = 1.0/xyz.z();
            invd.x() = xyz.x()*invd.z();
            invd.y() = xyz.y()*invd.z();
        }
        
        static void switchRepInvd2Xyz(const Eigen::Vector3d& invd, Eigen::Vector3d& xyz)
        {
            assert(invd.z() != 0.0);
            if (invd.z() < 0)
                std::cout << "[AnchoredLandmark]: Warning invd.z() < 0 !" << std::endl;
            
            xyz.z() = 1.0/invd.z();
            xyz.x() = invd.x()*xyz.z();
            xyz.y() = invd.y()*xyz.z();
        }
        
        static void switchRepXyz2Bear(const Eigen::Vector3d& xyz, Eigen::Vector3d& bear)
        {
            double r = xyz.norm();
            
            assert(r != 0.0);
            double rho = 1.0/r;
            bear.z() = rho;
            bear.x() = std::atan2(xyz.y(), xyz.x());
            bear.y() = std::acos(rho*xyz.z());
        }
        
        static void switchRepBear2Xyz(const Eigen::Vector3d& bear, Eigen::Vector3d& xyz)
        {
            assert(bear.z() != 0.0);
            if (bear.z() < 0)
                std::cout << "[AnchoredLandmark]: Warning bear.z() < 0 !" << std::endl;
            
            double r = 1.0/bear.z();
            xyz.z() = r*std::cos(bear.y());
            xyz.x() = r*std::cos(bear.x())*std::sin(bear.y());
            xyz.y() = r*std::sin(bear.x())*std::sin(bear.y());
        }
        
        void resetAnchoredPose(std::shared_ptr<SE3> new_anchored_pose = nullptr,
                               bool isUpdateRepValue = false)
        {
            if (new_anchored_pose != nullptr)
            {
                _anchored_pose = new_anchored_pose;
                
                if (isUpdateRepValue)
                {
                    this->setValuePosXyz(this->_pos_xyz);
                    
                    // TODO: The anchored pose may not take fej?
                    this->setFejPosXyz(this->_pos_xyz_fej);
                }
            }
            else
                _anchored_pose.reset();
        }
        
        const std::shared_ptr<SE3> getAnchoredPose() const
        { return _anchored_pose; }
        
        const Eigen::Vector3d& valuePosRep() const
        { return _pos_rep; }
        
        const Eigen::Vector3d& valuePosXyz() const
        { return _pos_xyz; }
        
        const Eigen::Vector3d& fejPosRep() const
        { return _pos_rep_fej; }
        
        const Eigen::Vector3d& fejPosXyz() const
        { return _pos_xyz_fej; }
        
        const LmRep& checkLmRepType() const
        { return _landmark_param_type; }
        
        void setValuePosXyz(const Eigen::Vector3d& xyz_world)
        {
            _pos_xyz = xyz_world;
            
            if (_anchored_pose != nullptr)
            {
                Eigen::Vector3d xyz_body = _anchored_pose->valueLinearAsMat().transpose()*(_pos_xyz - _anchored_pose->valueTrans());
                
                switch (_landmark_param_type)
                {
                    case XYZ:
                        _pos_rep = xyz_body;
                        break;
                    case INV_DEPTH:
                        switchRepXyz2Invd(xyz_body, _pos_rep);
                        break;
                    case BEARING:
                        switchRepXyz2Bear(xyz_body, _pos_rep);
                        break;
                    default:
                        assert(false);
                        break;
                }
            }
            else
                std::cout << "[AnchoredLandmark]: Cannot set body rep, no anchored pose!" << std::endl;
        }
        
        void setValuePosRep(const Eigen::Vector3d& pos_rep_body)
        {
            _pos_rep = pos_rep_body;
            
            Eigen::Vector3d xyz_body;
            
            switch (_landmark_param_type)
            {
                case XYZ:
                    xyz_body = _pos_rep;
                    break;
                case INV_DEPTH:
                    switchRepInvd2Xyz(_pos_rep, xyz_body);
                    break;
                case BEARING:
                    switchRepBear2Xyz(_pos_rep, xyz_body);
                    break;
                default:
                    assert(false);
                    break;
            }
            
            if (_anchored_pose != nullptr)
                _pos_xyz = _anchored_pose->valueLinearAsMat()*xyz_body + _anchored_pose->valueTrans();
            else
                std::cout << "[AnchoredLandmark]: Cannot set world xyz pos, no anchored pose!" << std::endl;
        }
        
        void setFejPosXyz(const Eigen::Vector3d& xyz_world_fej)
        {
            _pos_xyz_fej = xyz_world_fej;
            
            if (_anchored_pose != nullptr)
            {
                Eigen::Vector3d xyz_body_fej = _anchored_pose->fejLinearAsMat().transpose()*(_pos_xyz_fej - _anchored_pose->fejTrans());
                
                switch (_landmark_param_type)
                {
                    case XYZ:
                        _pos_rep_fej = xyz_body_fej;
                        break;
                    case INV_DEPTH:
                        switchRepXyz2Invd(xyz_body_fej, _pos_rep_fej);
                        break;
                    case BEARING:
                        switchRepXyz2Bear(xyz_body_fej, _pos_rep_fej);
                        break;
                    default:
                        assert(false);
                        break;
                }
            }
            else
                std::cout << "[AnchoredLandmark]: Cannot set body rep fej, no anchored pose!" << std::endl;
        }
        
        void setFejPosRep(const Eigen::Vector3d& pos_rep_body_fej)
        {
            _pos_rep_fej = pos_rep_body_fej;
            
            Eigen::Vector3d xyz_body_fej;
            
            switch (_landmark_param_type)
            {
                case XYZ:
                    xyz_body_fej = _pos_rep_fej;
                    break;
                case INV_DEPTH:
                    switchRepInvd2Xyz(_pos_rep_fej, xyz_body_fej);
                    break;
                case BEARING:
                    switchRepBear2Xyz(_pos_rep_fej, xyz_body_fej);
                    break;
                default:
                    assert(false);
                    break;
            }
            
            if (_anchored_pose != nullptr)
                _pos_xyz_fej = _anchored_pose->fejLinearAsMat()*xyz_body_fej + _anchored_pose->fejTrans();
            else
                std::cout << "[AnchoredLandmark]: Cannot set world xyz pos fej, no anchored pose!" << std::endl;
        }
        
        std::shared_ptr<AnchoredLandmark> clone()
        {
            std::shared_ptr<AnchoredLandmark> tmp(new AnchoredLandmark());
            
            tmp->setRepType(this->checkLmRepType());
            
            tmp->resetAnchoredPose(this->getAnchoredPose());
            
            tmp->setValuePosXyz(this->valuePosXyz());
            
            tmp->setFejPosXyz(this->fejPosXyz());
            
            return tmp;
        }
        
    protected:
        LmRep _landmark_param_type;
        
        Eigen::Vector3d _pos_rep;
        Eigen::Vector3d _pos_rep_fej;
        
        Eigen::Vector3d _pos_xyz;
        Eigen::Vector3d _pos_xyz_fej;
        
        std::shared_ptr<SE3> _anchored_pose;
        
        bool _has_rep_set;
    };
}

