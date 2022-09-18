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
        
        static void propagateStateCov(std::shared_ptr<State> state,
                                      const Eigen::Matrix<double, 15, 15>& Phi_imu,
                                      const Eigen::Matrix<double, 15, 12>& G_imu,
                                      double dt);
        
        static Eigen::MatrixXd getFullCov(std::shared_ptr<State> state);
        
        static Eigen::MatrixXd getMarginalCov(
            std::shared_ptr<State> state,
            const std::vector<std::shared_ptr<Type>>& small_variables);
        
        static void marginalize(std::shared_ptr<State> state, std::shared_ptr<Type> marg);
        
        static void addVariableIndependent(std::shared_ptr<State> state,
                                           std::shared_ptr<Type> new_state,
                                           const Eigen::MatrixXd& new_state_cov_block);
        
        static void addGNSSVariable(std::shared_ptr<State> state,
                                    const State::GNSSType& gtype,
                                    double value,
                                    double cov);
        
        static void margGNSSVariable(std::shared_ptr<State> state, const State::GNSSType& gtype);
        
        static void boxPlus(std::shared_ptr<State> state, const Eigen::VectorXd& dx);
        
        static void augmentSlidingWindowPose(std::shared_ptr<State> state);
        
        static void addAnchoredLandmarkInState(
            std::shared_ptr<State> state,
            std::shared_ptr<AnchoredLandmark> anchored_landmark,
            int lm_id,
            const Eigen::Matrix3d& cov);
        
        static void margSlidingWindowPose(std::shared_ptr<State> state, double marg_time);
        
        static void margSlidingWindowPose(std::shared_ptr<State> state);
        
        static void margAnchoredLandmarkInState(std::shared_ptr<State> state, int lm_id);
        
        static void ekfUpdate(std::shared_ptr<State> state,
                              const std::vector<std::shared_ptr<Type>>& var_order,
                              const Eigen::MatrixXd& H,
                              const Eigen::VectorXd& res,
                              const Eigen::MatrixXd& R);
        
        static bool checkSubOrder(std::shared_ptr<State> state,
                                  const std::vector<std::shared_ptr<Type>>& sub_order);
        
        static int calcSubVarSize(const std::vector<std::shared_ptr<Type>>& sub_var);
        
        static void addVariableDelayedInvertible(
            std::shared_ptr<State> state,
            std::shared_ptr<Type> var_new,
            const std::vector<std::shared_ptr<Type>>& var_old_order,
            const Eigen::MatrixXd& H_old,
            const Eigen::MatrixXd& H_new,
            const Eigen::VectorXd& res,
            double noise_iso_meas);
        
        static void addVariableDelayed(
            std::shared_ptr<State> state,
            std::shared_ptr<Type> var_new,
            const std::vector<std::shared_ptr<Type>>& var_old_order,
            Eigen::MatrixXd& H_old,
            Eigen::MatrixXd& H_new,
            Eigen::VectorXd& res,
            double noise_iso_meas,
            double chi2_mult_factor,
            bool do_chi2 = true);
        
        static void replaceVarLinear(
            std::shared_ptr<State> state,
            const std::shared_ptr<Type> target_var,
            const std::vector<std::shared_ptr<Type>>& dependence_order,
            const Eigen::MatrixXd& H);
    };
}
