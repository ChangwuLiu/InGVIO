#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>

#include "State.h"

namespace ingvio
{
    class State;
    
    class StateManager
    {
    public:
        StateManager() = delete;
        
        static bool checkStateContinuity(const std::shared_ptr<State> state);
        
        static void propagateStateCov(std::shared_ptr<State> state, const Eigen::Matrix<double, 15, 15>& Phi_imu, const Eigen::Matrix<double, 15, 12>& G_imu, double dt);
        
        static Eigen::MatrixXd getFullCov(std::shared_ptr<State> state);
        
        static Eigen::MatrixXd getMarginalCov(std::shared_ptr<State> state, const std::vector<std::shared_ptr<Type>>& small_variables);
        
        static void marginalize(std::shared_ptr<State> state, std::shared_ptr<Type> marg);
        
        static void addVariableIndependent(std::shared_ptr<State> state, std::shared_ptr<Type> new_state, const Eigen::MatrixXd& new_state_cov_block);
        
        static void addGNSSVariable(std::shared_ptr<State> state, const State::GNSSType& gtype, double value, double cov);
        
        static void margGNSSVariable(std::shared_ptr<State> state, const State::GNSSType& gtype);
        
        static void boxPlus(std::shared_ptr<State> state, const Eigen::VectorXd& dx);
        
        static void augmentSlidingWindowPose(std::shared_ptr<State> state);
    };
}
