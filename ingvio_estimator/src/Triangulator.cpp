#include <Eigen/QR>

#include "Triangulator.h"

#include "MapServer.h"
#include "PoseState.h"

namespace ingvio
{
    double Triangulator::findLongestTrans(
        const std::map<double, std::shared_ptr<SE3>, std::less<double>>& sw_poses,
        const std::map<double, std::shared_ptr<MonoMeas>>& mobs,
        double& max_length) const
    {
        assert(sw_poses.size() >= 2);
        
        const auto& item_last = sw_poses.rbegin();
        const double timestamp_last = item_last->first;
        const std::shared_ptr<SE3> pose_last = item_last->second;
        
        Eigen::Vector3d unit_feat_last;
        unit_feat_last.head<2>() = mobs.at(timestamp_last)->asVec();
        unit_feat_last.z() = 1.0;
        unit_feat_last.normalize();
        
        const Eigen::Vector3d unit_feat_w = pose_last->valueLinearAsMat()*unit_feat_last;
        const Eigen::Matrix3d proj = Eigen::Matrix3d::Identity()-unit_feat_w*unit_feat_w.transpose();
        
        max_length = -INFINITY;
        double max_timestamp = timestamp_last;
        for (const auto& item : sw_poses)
        {
            if (item.first == timestamp_last) continue;
            
            const Eigen::Vector3d trans_vec = proj*(item.second->valueTrans()-pose_last->valueTrans());
            
            double trans = std::abs(trans_vec.norm());
            if (trans > max_length)
            {
                max_length = trans;
                max_timestamp = item.first;
            }
        }
        
        return max_timestamp;
    }
    
    void Triangulator::calcRelaSwPose(
        const std::map<double, std::shared_ptr<SE3>, std::less<double>>& sw_poses,
        std::map<double, std::shared_ptr<SE3>, std::less<double>>& rel_sw_poses) const
    {
        const std::shared_ptr<SE3> pose_last = sw_poses.rbegin()->second;
        const double timestamp_last = sw_poses.rbegin()->first;
        
        rel_sw_poses.clear();
        for (const auto& item : sw_poses)
        {
            rel_sw_poses[item.first] = std::make_shared<SE3>();
            
            if (item.first == timestamp_last)
            {
                rel_sw_poses[timestamp_last]->setIdentity();
                continue;
            }
            rel_sw_poses[item.first]->setValueByIso(item.second->copyValueAsIso().inverse()*pose_last->copyValueAsIso());
        }
    }
    
    double Triangulator::initDepth(const Eigen::Vector2d& m1,
                                   const Eigen::Vector2d& m2,
                                   const std::shared_ptr<SE3> T12) const
    {
        Eigen::Vector3d tilde_m1 = T12->valueLinearAsMat()*Eigen::Vector3d(m1.x(), m1.y(), 1.0);
        Eigen::Vector3d t = T12->valueTrans();
        
        Eigen::Vector2d A, b;
        
        A(0, 0) = tilde_m1.x() - m2.x()*tilde_m1.z();
        A(1, 0) = tilde_m1.y() - m2.y()*tilde_m1.z();
        
        b(0, 0) = m2.x()*t.z()-t.x();
        b(1, 0) = m2.y()*t.z()-t.y();
        
        return (A.transpose()*A).inverse()*A.transpose()*b;
    }
    
    double Triangulator::calcUnitCost(const Eigen::Vector2d& meas,
                                      const std::shared_ptr<SE3> rel_pose,
                                      const Eigen::Vector3d& solution) const
    {
        Eigen::Vector3d pf0;
        
        pf0.z() = 1.0/solution.z();
        pf0.x() = solution.x()*pf0.z();
        pf0.y() = solution.y()*pf0.z();
        
        Eigen::Vector3d pf = rel_pose->valueLinearAsMat()*pf0 + rel_pose->valueTrans();
        
        Eigen::Vector2d meas_hat;
        meas_hat.x() = pf.x()/pf.z();
        meas_hat.y() = pf.y()/pf.z();
        
        return (meas-meas_hat).squaredNorm();
    }
    
    double Triangulator::calcTotalCost(const std::map<double, std::shared_ptr<MonoMeas>>& mobs,
                                       const std::map<double, std::shared_ptr<SE3>>& rel_poses,
                                       const Eigen::Vector3d& solution) const
    {
        double total_cost = 0.0;
        
        for (const auto& item : mobs)
            total_cost += calcUnitCost(item.second->asVec(), rel_poses.at(item.first), solution);
        
        return total_cost;
    }
    
    void Triangulator::calcResJacobian(const Eigen::Vector2d& meas,
                                       const std::shared_ptr<SE3> rel_pose,
                                       const Eigen::Vector3d& solution,
                                       Eigen::Vector2d& res,
                                       Eigen::Matrix<double, 2, 3>& J,
                                       double& w) const
    {
        Eigen::Vector3d tilde_pf_hat = rel_pose->valueLinearAsMat()*Eigen::Vector3d(solution.x(), solution.y(), 1.0)+rel_pose->valueTrans()*solution.z();
        
        Eigen::Vector2d meas_hat;
        meas_hat.x() = tilde_pf_hat.x()/tilde_pf_hat.z();
        meas_hat.y() = tilde_pf_hat.y()/tilde_pf_hat.z();
        
        res = meas_hat - meas;
        
        Eigen::Matrix<double, 2, 3> W;
        W.setZero();
        W(0, 0) = 1.0/tilde_pf_hat.z(); 
        W(0, 2) = -tilde_pf_hat.x()/std::pow(tilde_pf_hat.z(), 2);
        W(1, 1) = 1.0/tilde_pf_hat.z();
        W(1, 2) = -tilde_pf_hat.y()/std::pow(tilde_pf_hat.z(), 2);
        
        Eigen::Matrix<double, 3, 3> U;
        U.rightCols<1>() = rel_pose->valueTrans();
        U.leftCols<2>() = rel_pose->valueLinearAsMat().leftCols<2>();
        
        J = W*U;
        
        double e = res.norm();
        if (e <= _huber_epsilon)
            w = 1.0;
        else
            w = std::sqrt(2.0*_huber_epsilon/e);
    }
    
    bool Triangulator::triangulateMonoObs(
        const std::map<double, std::shared_ptr<MonoMeas>>& mono_obs,
        const std::map<double, std::shared_ptr<SE3>, std::less<double>>& sw_poses_raw,
        Eigen::Vector3d& pf) const
    {
        std::map<double, std::shared_ptr<MonoMeas>> mobs;
        std::map<double, std::shared_ptr<SE3>, std::less<double>> sw_poses;
        
        this->filterCommonTimestamp<MonoMeas>(sw_poses_raw, mono_obs, sw_poses, mobs);
        
        if (mobs.size() <= 4)
        {
            pf.setZero();
            return false;
        }
        
        double max_trans;
        double max_timestamp = findLongestTrans(sw_poses, mobs, max_trans);
        
        if (max_trans < _trans_thres) return false;
            
        std::map<double, std::shared_ptr<SE3>> rel_sw_poses;
        calcRelaSwPose(sw_poses, rel_sw_poses);
        
        Eigen::Vector3d solution;
        const double timestamp_last = rel_sw_poses.rbegin()->first;
        const std::shared_ptr<SE3> pose_last = sw_poses.rbegin()->second;
        
        solution.x() = mobs[timestamp_last]->_u0;
        solution.y() = mobs[timestamp_last]->_v0;
        solution.z() = 1.0/initDepth(mobs[timestamp_last]->asVec(), mobs[max_timestamp]->asVec(), rel_sw_poses[max_timestamp]);
        
        /*
        std::cout << "init pose = " << (pose0->copyValueAsIso().inverse()*Eigen::Vector3d(solution.x()/solution.z(), solution.y()/solution.z(), 1.0/solution.z())).transpose() << std::endl;
        */
        
        double total_cost = calcTotalCost(mobs, rel_sw_poses, solution);
        
        double lambda = _init_damping;
        int inner_loop_cnt = 0;
        int outer_loop_cnt = 0;
        bool is_cost_reduced = false;
        double delta_norm = INFINITY;
        
        do
        {
            Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
            Eigen::Vector3d b = Eigen::Vector3d::Zero();
            
            for (const auto& item : rel_sw_poses)
            {
                Eigen::Matrix<double, 2, 3> J;
                Eigen::Vector2d res;
                double w;
                
                calcResJacobian(mobs.at(item.first)->asVec(), item.second, solution, res, J, w);
                
                if (w == 1.0)
                {
                    A += J.transpose()*J;
                    b -= J.transpose()*res;
                }
                else
                {
                    A += std::pow(w, 2)*J.transpose()*J;
                    b -= std::pow(w, 2)*J.transpose()*res;
                }
            }
            
            do
            {
                Eigen::Matrix3d Damper = lambda*Eigen::Matrix3d::Identity();
                
                Eigen::Vector3d delta = (A + Damper).ldlt().solve(b);
                
                Eigen::Vector3d new_solution = solution + delta;
                
                delta_norm = delta.norm();
                
                double new_total_cost = calcTotalCost(mobs, rel_sw_poses, new_solution);
                
                
                if (new_total_cost < total_cost)
                {
                    total_cost = new_total_cost;
                    solution = new_solution;
                    is_cost_reduced = true;
                    lambda = lambda/10.0 > 1e-10 ? lambda/10.0 : 1e-10;
                }
                else
                {
                    is_cost_reduced = false;
                    lambda = lambda*10 < 1e12 ? lambda*10 : 1e12;
                }
            }
            while (inner_loop_cnt++ < _inner_loop_max_iter && !is_cost_reduced);
            
            inner_loop_cnt = 0;
        }
        while (outer_loop_cnt++ < _outer_loop_max_iter && delta_norm > _conv_precision);
        
        Eigen::Vector3d pf_last;
        pf_last.z() = 1.0/solution.z();
        pf_last.x() = solution.x()*pf_last.z();
        pf_last.y() = solution.y()*pf_last.z();
        
        pf.setZero();
        
        if ((outer_loop_cnt >= _outer_loop_max_iter && inner_loop_cnt >= _inner_loop_max_iter) || delta_norm > _conv_precision) 
            return false;
        
        for (const auto& item : rel_sw_poses)
        {
            Eigen::Vector3d tmp = item.second->copyValueAsIso()*pf_last;
            if (tmp.z() <= _min_depth)
                return false;
        }
        
        Eigen::HouseholderQR<Eigen::MatrixXd> qr(pf_last);
        Eigen::MatrixXd Q = qr.householderQ();
        
        double base_line_max = 0.0;
        
        for (const auto& item : rel_sw_poses)
        {
            Eigen::Vector3d p_ci_A = item.second->copyValueAsIso().inverse().translation();
            
            double base_line = ((Q.block(0, 1, 3, 2)).transpose()*p_ci_A).norm();
            
            if (base_line > base_line_max)
                base_line_max = base_line;
        }
        
        if (pf_last.z() < _min_depth || pf_last.z() > _max_depth)
            return false;
        
        pf = pose_last->copyValueAsIso()*pf_last;
        
        if (pf.hasNaN())
        {
            pf.setZero();
            return false;
        }
        
        return true;
    }
    
    bool Triangulator::triangulateStereoObs(
        const std::map<double, std::shared_ptr<StereoMeas>>& stereo_obs,
        const std::map<double, std::shared_ptr<SE3>, std::less<double>>& sw_poses_raw,
        const Eigen::Isometry3d& T_cl2cr,
        Eigen::Vector3d& pf) const
    {
        std::map<double, std::shared_ptr<StereoMeas>> sobs;
        std::map<double, std::shared_ptr<SE3>, std::less<double>> sw_poses;
        
        this->filterCommonTimestamp<StereoMeas>(sw_poses_raw, stereo_obs, sw_poses, sobs);
        
        std::map<double, std::shared_ptr<MonoMeas>> mono_obs;
        std::map<double, std::shared_ptr<SE3>, std::less<double>> sw_mono_poses;
        
        double cnt = 0;
        for (const auto& item : sobs)
        {
            cnt = cnt + 1.0;
            mono_obs[cnt] = std::make_shared<MonoMeas>();
            
            mono_obs.at(cnt)->_u0 = item.second->_u0;
            mono_obs.at(cnt)->_v0 = item.second->_v0;
            
            sw_mono_poses[cnt] = sw_poses.at(item.first)->clone();
            
            cnt = cnt + 1.0;
            mono_obs[cnt] = std::make_shared<MonoMeas>();
            
            mono_obs.at(cnt)->_u0 = item.second->_u1;
            mono_obs.at(cnt)->_v0 = item.second->_v1;
            
            sw_mono_poses[cnt] = std::make_shared<SE3>();
            
            sw_mono_poses.at(cnt)->setValueByIso(sw_poses.at(item.first)->copyValueAsIso()*T_cl2cr.inverse());
        }
        
        bool flag = this->triangulateMonoObs(mono_obs, sw_mono_poses, pf);
        
        return flag;
    }
    
}
