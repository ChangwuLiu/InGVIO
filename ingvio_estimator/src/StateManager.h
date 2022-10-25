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
        
        // Acknowledgements: This function is inspired by the realization of OpenVINS
        // Delayed-Initialization
        // See https://github.com/rpng/open_vins
        
        static void ekfUpdate(std::shared_ptr<State> state,
                              const std::vector<std::shared_ptr<Type>>& var_order,
                              const Eigen::MatrixXd& H,
                              const Eigen::VectorXd& res,
                              const Eigen::MatrixXd& R);
        
        static bool checkSubOrder(std::shared_ptr<State> state,
                                  const std::vector<std::shared_ptr<Type>>& sub_order);
        
        static int calcSubVarSize(const std::vector<std::shared_ptr<Type>>& sub_var);
        
        // Acknowledgements: This function is inspired by the realization of OpenVINS
        // Delayed-Initialization
        // See https://github.com/rpng/open_vins
        
        static void addVariableDelayedInvertible(
            std::shared_ptr<State> state,
            std::shared_ptr<Type> var_new,
            const std::vector<std::shared_ptr<Type>>& var_old_order,
            const Eigen::MatrixXd& H_old,
            const Eigen::MatrixXd& H_new,
            const Eigen::VectorXd& res,
            double noise_iso_meas);
        
        // Acknowledgements: This function is inspired by the realization of OpenVINS
        // Delayed-Initialization
        // See https://github.com/rpng/open_vins
        
        static bool addVariableDelayed(
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
