/**
 * @file polynomial_traj.cpp
 * @brief 多项式轨迹实现
 */

#include "traj_utils/polynomial_traj.h"
#include <iostream>

namespace ego_planner {

// ========== Piece ==========

Piece::Piece(double duration, const CoefficientMat& coeffMat)
    : duration_(duration), coeffMat_(coeffMat) {
}

Eigen::Vector3d Piece::getPos(double t) const {
    Eigen::Vector3d pos(0.0, 0.0, 0.0);
    double tn = 1.0;
    
    for (int i = 0; i < coeffMat_.rows(); ++i) {
        pos += coeffMat_.row(i).transpose() * tn;
        tn *= t;
    }
    
    return pos;
}

Eigen::Vector3d Piece::getVel(double t) const {
    Eigen::Vector3d vel(0.0, 0.0, 0.0);
    double tn = 1.0;
    
    for (int i = 1; i < coeffMat_.rows(); ++i) {
        vel += i * coeffMat_.row(i).transpose() * tn;
        tn *= t;
    }
    
    return vel;
}

Eigen::Vector3d Piece::getAcc(double t) const {
    Eigen::Vector3d acc(0.0, 0.0, 0.0);
    double tn = 1.0;
    
    for (int i = 2; i < coeffMat_.rows(); ++i) {
        acc += i * (i - 1) * coeffMat_.row(i).transpose() * tn;
        tn *= t;
    }
    
    return acc;
}

// ========== Trajectory ==========

Trajectory::Trajectory() : total_duration_(0.0) {
}

Trajectory::Trajectory(const std::vector<double>& durs, const std::vector<CoefficientMat>& coeffs) {
    pieces_.clear();
    pieces_.reserve(durs.size());
    
    for (size_t i = 0; i < durs.size(); ++i) {
        pieces_.emplace_back(durs[i], coeffs[i]);
    }
    
    total_duration_ = 0.0;
    for (const auto& dur : durs) {
        total_duration_ += dur;
    }
}

int Trajectory::locatePieceIdx(double t) const {
    if (t < 0.0) return 0;
    if (t >= total_duration_) return pieces_.size() - 1;
    
    double acc_time = 0.0;
    for (size_t i = 0; i < pieces_.size(); ++i) {
        acc_time += pieces_[i].getDuration();
        if (t < acc_time) {
            return i;
        }
    }
    
    return pieces_.size() - 1;
}

Eigen::Vector3d Trajectory::getPos(double t) const {
    if (pieces_.empty()) return Eigen::Vector3d::Zero();
    
    int idx = locatePieceIdx(t);
    
    double local_t = t;
    for (int i = 0; i < idx; ++i) {
        local_t -= pieces_[i].getDuration();
    }
    
    return pieces_[idx].getPos(local_t);
}

Eigen::Vector3d Trajectory::getVel(double t) const {
    if (pieces_.empty()) return Eigen::Vector3d::Zero();
    
    int idx = locatePieceIdx(t);
    
    double local_t = t;
    for (int i = 0; i < idx; ++i) {
        local_t -= pieces_[i].getDuration();
    }
    
    return pieces_[idx].getVel(local_t);
}

Eigen::Vector3d Trajectory::getAcc(double t) const {
    if (pieces_.empty()) return Eigen::Vector3d::Zero();
    
    int idx = locatePieceIdx(t);
    
    double local_t = t;
    for (int i = 0; i < idx; ++i) {
        local_t -= pieces_[i].getDuration();
    }
    
    return pieces_[idx].getAcc(local_t);
}

double Trajectory::getTotalDuration() const {
    return total_duration_;
}

int Trajectory::getPieceNum() const {
    return pieces_.size();
}

// ========== PolynomialTraj ==========

PolynomialTraj PolynomialTraj::minSnapTraj(const Eigen::MatrixXd& Pos, const Eigen::Vector3d& start_vel,
                                             const Eigen::Vector3d& end_vel, const Eigen::Vector3d& start_acc,
                                             const Eigen::Vector3d& end_acc, const Eigen::VectorXd& Time) {
    // 简化实现：使用5次多项式拟合
    int piece_num = Time.size();
    
    std::vector<double> durations;
    std::vector<CoefficientMat> coeffMats;
    
    for (int i = 0; i < piece_num; ++i) {
        durations.push_back(Time(i));
        
        Eigen::Vector3d p0 = Pos.col(i);
        Eigen::Vector3d p1 = Pos.col(i + 1);
        double T = Time(i);
        
        // 边界条件
        Eigen::Vector3d v0 = (i == 0) ? start_vel : Eigen::Vector3d::Zero();
        Eigen::Vector3d v1 = (i == piece_num - 1) ? end_vel : Eigen::Vector3d::Zero();
        Eigen::Vector3d a0 = (i == 0) ? start_acc : Eigen::Vector3d::Zero();
        Eigen::Vector3d a1 = (i == piece_num - 1) ? end_acc : Eigen::Vector3d::Zero();
        
        // 5次多项式: p(t) = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5
        CoefficientMat coeffMat(6, 3);
        
        for (int dim = 0; dim < 3; ++dim) {
            // 构建线性方程组 A*c = b
            Eigen::Matrix<double, 6, 6> A;
            Eigen::Matrix<double, 6, 1> b;
            
            double T2 = T * T;
            double T3 = T2 * T;
            double T4 = T3 * T;
            double T5 = T4 * T;
            
            // p(0) = p0
            A.row(0) << 1, 0, 0, 0, 0, 0;
            b(0) = p0(dim);
            
            // v(0) = v0
            A.row(1) << 0, 1, 0, 0, 0, 0;
            b(1) = v0(dim);
            
            // a(0) = a0
            A.row(2) << 0, 0, 2, 0, 0, 0;
            b(2) = a0(dim);
            
            // p(T) = p1
            A.row(3) << 1, T, T2, T3, T4, T5;
            b(3) = p1(dim);
            
            // v(T) = v1
            A.row(4) << 0, 1, 2*T, 3*T2, 4*T3, 5*T4;
            b(4) = v1(dim);
            
            // a(T) = a1
            A.row(5) << 0, 0, 2, 6*T, 12*T2, 20*T3;
            b(5) = a1(dim);
            
            // 求解系数
            Eigen::Matrix<double, 6, 1> c = A.colPivHouseholderQr().solve(b);
            coeffMat.col(dim) = c;
        }
        
        coeffMats.push_back(coeffMat);
    }
    
    Trajectory traj(durations, coeffMats);
    
    PolynomialTraj poly_traj;
    poly_traj.traj_ = traj;
    
    return poly_traj;
}

PolynomialTraj PolynomialTraj::one_segment_traj_gen(const Eigen::Vector3d& start_pt, 
                                                      const Eigen::Vector3d& start_vel,
                                                      const Eigen::Vector3d& start_acc,
                                                      const Eigen::Vector3d& end_pt, 
                                                      const Eigen::Vector3d& end_vel,
                                                      const Eigen::Vector3d& end_acc, 
                                                      double duration) {
    // 构建约束矩阵（5次多项式，6个系数）
    Eigen::Matrix<double, 6, 6> C;
    Eigen::VectorXd Bx(6), By(6), Bz(6);
    
    double T = duration;
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;
    
    // p(0) = p0: [1, 0, 0, 0, 0, 0]
    C.row(0) << 1, 0, 0, 0, 0, 0;
    
    // v(0) = v0: [0, 1, 0, 0, 0, 0]
    C.row(1) << 0, 1, 0, 0, 0, 0;
    
    // a(0) = a0: [0, 0, 2, 0, 0, 0]
    C.row(2) << 0, 0, 2, 0, 0, 0;
    
    // p(T) = p1: [1, T, T^2, T^3, T^4, T^5]
    C.row(3) << 1, T, T2, T3, T4, T5;
    
    // v(T) = v1: [0, 1, 2*T, 3*T^2, 4*T^3, 5*T^4]
    C.row(4) << 0, 1, 2*T, 3*T2, 4*T3, 5*T4;
    
    // a(T) = a1: [0, 0, 2, 6*T, 12*T^2, 20*T^3]
    C.row(5) << 0, 0, 2, 6*T, 12*T2, 20*T3;
    
    // 构建右侧向量
    Bx << start_pt(0), start_vel(0), start_acc(0), end_pt(0), end_vel(0), end_acc(0);
    By << start_pt(1), start_vel(1), start_acc(1), end_pt(1), end_vel(1), end_acc(1);
    Bz << start_pt(2), start_vel(2), start_acc(2), end_pt(2), end_vel(2), end_acc(2);
    
    // 求解系数
    Eigen::VectorXd Cofx = C.colPivHouseholderQr().solve(Bx);
    Eigen::VectorXd Cofy = C.colPivHouseholderQr().solve(By);
    Eigen::VectorXd Cofz = C.colPivHouseholderQr().solve(Bz);
    
    // 构建系数矩阵（每行一个系数，每列一个维度）
    CoefficientMat coeffMat(6, 3);
    coeffMat.col(0) = Cofx;
    coeffMat.col(1) = Cofy;
    coeffMat.col(2) = Cofz;
    
    // 创建单段轨迹
    std::vector<double> durations = {duration};
    std::vector<CoefficientMat> coeffMats = {coeffMat};
    
    Trajectory traj(durations, coeffMats);
    
    PolynomialTraj poly_traj;
    poly_traj.traj_ = traj;
    
    return poly_traj;
}

Eigen::Vector3d PolynomialTraj::getPos(double t) const {
    return traj_.getPos(t);
}

Eigen::Vector3d PolynomialTraj::getVel(double t) const {
    return traj_.getVel(t);
}

Eigen::Vector3d PolynomialTraj::getAcc(double t) const {
    return traj_.getAcc(t);
}

double PolynomialTraj::getTotalDuration() const {
    return traj_.getTotalDuration();
}

} // namespace ego_planner
