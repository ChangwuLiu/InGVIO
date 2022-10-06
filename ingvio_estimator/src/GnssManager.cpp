
#include "GnssManager.h"

namespace ingvio
{
    const std::unordered_map<uint32_t, State::GNSSType> GnssManager::satsys_to_gnsstype = 
    {
        {SYS_GPS, State::GNSSType::GPS},
        {SYS_BDS, State::GNSSType::BDS},
        {SYS_GAL, State::GNSSType::GAL},
        {SYS_GLO, State::GNSSType::GLO}
    };
    
    const std::unordered_map<State::GNSSType, uint32_t> GnssManager::gnsstype_to_satsys = 
    {
        {State::GNSSType::GPS, SYS_GPS},
        {State::GNSSType::BDS, SYS_BDS},
        {State::GNSSType::GAL, SYS_GAL},
        {State::GNSSType::GLO, SYS_GLO}
    };
    
    const std::unordered_map<int, State::GNSSType> GnssManager::idx_to_gnsstype =
    {
        {0, State::GNSSType::GPS},
        {1, State::GNSSType::GLO},
        {2, State::GNSSType::GAL},
        {3, State::GNSSType::BDS}
    };
    
    State::GNSSType GnssManager::convertSatsys2GnssType(const uint32_t& sat_sys)
    {
        if (GnssManager::satsys_to_gnsstype.find(sat_sys) == GnssManager::satsys_to_gnsstype.end())
        {
            std::cout << "[GnssManager]: Sat sys not in index map, cannot convert to gnss type!" << std::endl;
            std::exit(EXIT_FAILURE);
        }
        
        return GnssManager::satsys_to_gnsstype.at(sat_sys);
    }
    
    Eigen::Matrix3d GnssManager::calcRw2enu(const double& yaw_offset)
    {
        return Eigen::AngleAxisd(yaw_offset, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    }
    
    Eigen::Isometry3d GnssManager::calcTw2enu(const double& yaw_offset)
    {
        Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
        result.linear() = GnssManager::calcRw2enu(yaw_offset);
        return result;
    }
    
    Eigen::Vector4d GnssManager::getClockbiasVec(const std::shared_ptr<State> state)
    {
        Eigen::Vector4d cb = Eigen::Vector4d::Zero();
        
        for (const auto& gnss_item : state->_gnss)
            if (gnss_item.first == State::YOF || gnss_item.first == State::FS)
                continue;
            else
                cb(gnss_comm::sys2idx.at(GnssManager::gnsstype_to_satsys.at(gnss_item.first)))
                = gnss_item.second->value();
            
        return cb;
    }
    
    bool GnssManager::checkGnssStates(const std::shared_ptr<State> state)
    {
        if (state->_gnss.find(State::GNSSType::YOF) == state->_gnss.end())
            return false;
        
        if (state->_gnss.find(State::GNSSType::FS) == state->_gnss.end())
            return false;
        
        for (int i = 0; i < 4; ++i)
            if (state->_gnss.find(static_cast<State::GNSSType>(i)) != state->_gnss.end())
                return true;
            
        return false;
    }
    
    Eigen::Matrix3d GnssManager::dotRw2enu(const std::shared_ptr<State> state)
    {
        /*
        if (state->_gnss.find(State::GNSSType::YOF) == state->_gnss.end())
            return Eigen::Matrix3d::Zero();
        
        const double yo = state->_gnss.at(State::GNSSType::YOF)->value();
        
        Eigen::Matrix3d result;
        result << -std::sin(yo), -std::cos(yo), 0.0,
                   std::cos(yo), -std::sin(yo), 0.0,
                            0.0,           0.0, 0.0;
                            
        return result;
        */
        return Eigen::Matrix3d::Zero();
    }
    
    Eigen::Matrix3d GnssManager::calcRw2enu(const std::shared_ptr<State> state)
    {
        if (state->_gnss.find(State::GNSSType::YOF) == state->_gnss.end())
            return Eigen::Matrix3d::Identity();
        
        const double yo = state->_gnss.at(State::GNSSType::YOF)->value();
        
        return GnssManager::calcRw2enu(yo);
    }
    
    Eigen::Isometry3d GnssManager::calcTw2enu(const std::shared_ptr<State> state)
    {
        if (state->_gnss.find(State::GNSSType::YOF) == state->_gnss.end())
            return Eigen::Isometry3d::Identity();
        
        const double yo = state->_gnss.at(State::GNSSType::YOF)->value();
        
        return GnssManager::calcTw2enu(yo);
    }
    
    void GnssManager::printYOF(const std::shared_ptr<State> state)
    {
        if (state->_gnss.find(State::GNSSType::YOF) == state->_gnss.end())
            return;
        
        std::cout << "[GnssManager]: Local to ENU yaw offset = " << state->_gnss.at(State::GNSSType::YOF)->value()*180.0/M_PI << " (deg)" << std::endl;
    }
}
