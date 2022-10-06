#pragma once

#include <unordered_map>

#include <gnss_comm/gnss_ros.hpp>

#include "State.h"

namespace ingvio
{
    class GnssManager
    {
    public:
        GnssManager() = delete;
        
        GnssManager(const GnssManager&) = delete;
        GnssManager operator=(const GnssManager&) = delete;
        
        static const std::unordered_map<uint32_t, State::GNSSType> satsys_to_gnsstype;
        
        static const std::unordered_map<State::GNSSType, uint32_t> gnsstype_to_satsys;
        
        static const std::unordered_map<int, State::GNSSType> idx_to_gnsstype;
        
        static State::GNSSType convertSatsys2GnssType(const uint32_t& sat_sys);
        
        static Eigen::Matrix3d calcRw2enu(const double& yaw_offset);
        
        static Eigen::Isometry3d calcTw2enu(const double& yaw_offset);
        
        static Eigen::Matrix3d calcRw2enu(const std::shared_ptr<State> state);
        
        static Eigen::Isometry3d calcTw2enu(const std::shared_ptr<State> state);
        
        static Eigen::Vector4d getClockbiasVec(const std::shared_ptr<State> state);
        
        static bool checkGnssStates(const std::shared_ptr<State> state);
        
        static Eigen::Matrix3d dotRw2enu(const std::shared_ptr<State> state);
        
        static void printYOF(const std::shared_ptr<State> state);
    };
}
