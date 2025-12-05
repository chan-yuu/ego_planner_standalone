/**
 * @file polynomial_traj.h
 * @brief 多项式轨迹类
 */

#ifndef POLYNOMIAL_TRAJ_H
#define POLYNOMIAL_TRAJ_H

#include <Eigen/Eigen>
#include <vector>

namespace ego_planner {

// 系数矩阵类型：每行对应一个系数，每列对应x/y/z
typedef Eigen::MatrixXd CoefficientMat;

// 轨迹段
class Piece {
public:
    Piece() = default;
    Piece(double duration, const CoefficientMat& coeffMat);
    
    Eigen::Vector3d getPos(double t) const;
    Eigen::Vector3d getVel(double t) const;
    Eigen::Vector3d getAcc(double t) const;
    
    double getDuration() const { return duration_; }
    
private:
    double duration_;
    CoefficientMat coeffMat_;
};

// 完整轨迹
class Trajectory {
public:
    Trajectory();
    Trajectory(const std::vector<double>& durs, const std::vector<CoefficientMat>& coeffs);
    
    int locatePieceIdx(double t) const;
    
    Eigen::Vector3d getPos(double t) const;
    Eigen::Vector3d getVel(double t) const;
    Eigen::Vector3d getAcc(double t) const;
    
    double getTotalDuration() const;
    int getPieceNum() const;
    
private:
    std::vector<Piece> pieces_;
    double total_duration_;
};

// 多项式轨迹封装
class PolynomialTraj {
public:
    PolynomialTraj() = default;
    
    // 最小化snap生成轨迹
    static PolynomialTraj minSnapTraj(const Eigen::MatrixXd& Pos, const Eigen::Vector3d& start_vel,
                                       const Eigen::Vector3d& end_vel, const Eigen::Vector3d& start_acc,
                                       const Eigen::Vector3d& end_acc, const Eigen::VectorXd& Time);
    
    // 生成单段轨迹 (从起点到终点的五次多项式)
    static PolynomialTraj one_segment_traj_gen(const Eigen::Vector3d& start_pt, 
                                                const Eigen::Vector3d& start_vel,
                                                const Eigen::Vector3d& start_acc,
                                                const Eigen::Vector3d& end_pt, 
                                                const Eigen::Vector3d& end_vel,
                                                const Eigen::Vector3d& end_acc, 
                                                double duration);
    
    // 主要接口
    Eigen::Vector3d getPos(double t) const;
    Eigen::Vector3d getVel(double t) const;
    Eigen::Vector3d getAcc(double t) const;
    double getTotalDuration() const;
    
    // 兼容旧接口的别名方法
    void init() {} // 空实现，已在构造时初始化
    double getTimeSum() const { return getTotalDuration(); }
    Eigen::Vector3d evaluate(double t) const { return getPos(t); }
    Eigen::Vector3d evaluateVel(double t) const { return getVel(t); }
    Eigen::Vector3d evaluateAcc(double t) const { return getAcc(t); }
    
private:
    Trajectory traj_;
};

} // namespace ego_planner

#endif // POLYNOMIAL_TRAJ_H
