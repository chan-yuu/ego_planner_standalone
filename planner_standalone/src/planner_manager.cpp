/**
 * @file planner_manager.cpp
 * @brief 规划器管理器实现（简化版）
 * @author cyun
 */

#include "plan_manage/planner_manager.h"
#include <iostream>

namespace ego_planner {

EGOPlannerManager::EGOPlannerManager() {
}

EGOPlannerManager::~EGOPlannerManager() {
}

void EGOPlannerManager::initPlanModules(const PlannerManagerParams& params) {
    // 保存参数到成员变量 pp_
    pp_.max_vel_ = params.max_vel;
    pp_.max_acc_ = params.max_acc;
    pp_.ctrl_pt_dist = params.ctrl_pt_dist;
    pp_.planning_horizen_ = params.planning_horizon;
    pp_.feasibility_tolerance_ = params.feasibility_tolerance;
    
    // 初始化栅格地图
    grid_map_.reset(new GridMap());
    grid_map_->initMap(params.map_params.map_origin_,
                       params.map_params.map_size_,
                       params.map_params.resolution_);
    grid_map_->setOccupancyInflation(params.map_params.obstacles_inflation_);
    
    // 初始化B样条优化器
    bspline_optimizer_rebound_.reset(new BsplineOptimizer());
    bspline_optimizer_rebound_->setParam(params.opt_params);
    bspline_optimizer_rebound_->setEnvironment(grid_map_);
    
    // 初始化优化器内部的A*搜索器（用于障碍物段的路径规划）
    // 使用更大的节点池以支持长距离搜索
    bspline_optimizer_rebound_->a_star_.reset(new AStar());
    bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(200, 200, 100));
    
    std::cout << "[PlannerManager] 规划模块初始化完成" << std::endl;
    std::cout << "  最大速度: " << pp_.max_vel_ << " m/s" << std::endl;
    std::cout << "  最大加速度: " << pp_.max_acc_ << " m/s^2" << std::endl;
    std::cout << "  控制点距离: " << pp_.ctrl_pt_dist << " m" << std::endl;
}

bool EGOPlannerManager::planGlobalTraj(const Eigen::Vector3d& start_pos,
                                        const Eigen::Vector3d& start_vel,
                                        const Eigen::Vector3d& start_acc,
                                        const Eigen::Vector3d& end_pos,
                                        const Eigen::Vector3d& end_vel,
                                        const Eigen::Vector3d& end_acc) {
    std::cout << "[PlannerManager] 规划全局轨迹:" << std::endl;
    std::cout << "  起点: [" << start_pos.transpose() << "]" << std::endl;
    std::cout << "  终点: [" << end_pos.transpose() << "]" << std::endl;
    
    // 创建临时路径搜索器
    AStar path_finder;
    Eigen::Vector3i pool_size(200, 200, 100);  // 使用更大的搜索空间
    path_finder.initGridMap(grid_map_, pool_size);
    
    // 使用A*搜索路径
    bool success = path_finder.AstarSearch(grid_map_->getResolution(), start_pos, end_pos);
    
    if (!success) {
        std::cerr << "[PlannerManager] A*路径搜索失败" << std::endl;
        return false;
    }
    
    // 获取路径点
    std::vector<Eigen::Vector3d> path = path_finder.getPath();
    if (path.size() < 2) {
        std::cerr << "[PlannerManager] 路径点太少: " << path.size() << std::endl;
        return false;
    }
    
    std::cout << "[PlannerManager] A*搜索成功，路径点数: " << path.size() << std::endl;
    
    // 简化：使用路径点生成多项式轨迹
    int piece_num = path.size() - 1;
    Eigen::MatrixXd waypoints(3, path.size());
    for (size_t i = 0; i < path.size(); ++i) {
        waypoints.col(i) = path[i];
    }
    
    // 分配时间
    Eigen::VectorXd time_allocations(piece_num);
    for (int i = 0; i < piece_num; ++i) {
        double dist = (path[i+1] - path[i]).norm();
        time_allocations(i) = std::max(0.5, dist / pp_.max_vel_);
    }
    
    // 生成多项式轨迹
    PolynomialTraj poly_traj = PolynomialTraj::minSnapTraj(
        waypoints, start_vel, end_vel, start_acc, end_acc, time_allocations
    );
    
    // 保存到全局轨迹
    global_data_.setGlobalTraj(poly_traj, TimePoint::now());
    
    std::cout << "[PlannerManager] 全局轨迹生成成功，时长: " 
              << poly_traj.getTotalDuration() << "秒" << std::endl;
    
    return true;
}

bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                       Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                       Eigen::Vector3d local_target_vel, bool flag_polyInit,
                                       bool flag_randomPolyTraj) {
    // 常量定义：配合0.099膨胀参数
    constexpr double MIN_SAFE_HEIGHT = 0.2;  // 起点/终点最小高度
    constexpr double MIN_Z_HEIGHT = 0.15;     // 采样点/控制点最小高度
    
    // 修正起点和终点高度，确保在安全高度以上（避免地面碰撞检测）
    if (start_pt(2) < MIN_SAFE_HEIGHT) {
        std::cout << "[PlannerManager] 起点高度 " << start_pt(2) << "m 过低，提升至 " << MIN_SAFE_HEIGHT << "m" << std::endl;
        start_pt(2) = MIN_SAFE_HEIGHT;
    }
    if (local_target_pt(2) < MIN_SAFE_HEIGHT) {
        std::cout << "[PlannerManager] 终点高度 " << local_target_pt(2) << "m 过低，提升至 " << MIN_SAFE_HEIGHT << "m" << std::endl;
        local_target_pt(2) = std::max(local_target_pt(2), MIN_SAFE_HEIGHT);
    }
    
    std::cout << "[PlannerManager] reboundReplan: 从当前位置规划局部轨迹" << std::endl;
    std::cout << "  起点: [" << start_pt.transpose() << "], 速度: [" << start_vel.transpose() << "]" << std::endl;
    std::cout << "  终点: [" << local_target_pt.transpose() << "]" << std::endl;
    
    // 检查起点和终点距离
    double dist = (local_target_pt - start_pt).norm();
    if (dist < 0.2) {
        std::cout << "[PlannerManager] 距离目标太近 (" << dist << "m)，不需要规划" << std::endl;
        return false;
    }
    
    // Step 1: 使用A*搜索无碰撞路径
    std::cout << "[PlannerManager] Step 1: 使用A*搜索无碰撞路径..." << std::endl;
    if (!bspline_optimizer_rebound_->a_star_) {
        std::cerr << "[PlannerManager] A*未初始化！" << std::endl;
        return false;
    }
    
    bool astar_success = bspline_optimizer_rebound_->a_star_->AstarSearch(
        grid_map_->getResolution(), start_pt, local_target_pt
    );
    
    if (!astar_success) {
        std::cerr << "[PlannerManager] A*路径搜索失败，无法规划" << std::endl;
        return false;
    }
    
    std::vector<Eigen::Vector3d> a_star_path = bspline_optimizer_rebound_->a_star_->getPath();
    std::cout << "[PlannerManager] A*找到路径，路径点数: " << a_star_path.size() << std::endl;
    
    // Step 2: 基于A*路径采样控制点（确保无碰撞）
    double ts = pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2;
    std::vector<Eigen::Vector3d> point_set;
    double path_length = 0.0;
    
    // 计算路径总长度
    for (size_t i = 1; i < a_star_path.size(); ++i) {
        path_length += (a_star_path[i] - a_star_path[i-1]).norm();
    }
    
    // 沿A*路径均匀采样
    double sample_dist = pp_.ctrl_pt_dist * 1.5;  // 采样间隔
    point_set.push_back(a_star_path[0]);  // 起点
    
    double accumulated_dist = 0.0;
    double next_sample_dist = sample_dist;
    
    for (size_t i = 1; i < a_star_path.size(); ++i) {
        double seg_len = (a_star_path[i] - a_star_path[i-1]).norm();
        accumulated_dist += seg_len;
        
        // 在这段上采样
        while (next_sample_dist <= accumulated_dist && next_sample_dist < path_length) {
            double t = (next_sample_dist - (accumulated_dist - seg_len)) / seg_len;
            Eigen::Vector3d sample_pt = a_star_path[i-1] + t * (a_star_path[i] - a_star_path[i-1]);
            point_set.push_back(sample_pt);
            next_sample_dist += sample_dist;
        }
    }
    
    point_set.push_back(a_star_path.back());  // 终点
    
    // 自适应调整采样密度
    while (point_set.size() < 7) {
        sample_dist /= 1.5;
        point_set.clear();
        point_set.push_back(a_star_path[0]);
        
        accumulated_dist = 0.0;
        next_sample_dist = sample_dist;
        
        for (size_t i = 1; i < a_star_path.size(); ++i) {
            double seg_len = (a_star_path[i] - a_star_path[i-1]).norm();
            accumulated_dist += seg_len;
            
            while (next_sample_dist <= accumulated_dist && next_sample_dist < path_length) {
                double t = (next_sample_dist - (accumulated_dist - seg_len)) / seg_len;
                Eigen::Vector3d sample_pt = a_star_path[i-1] + t * (a_star_path[i] - a_star_path[i-1]);
                point_set.push_back(sample_pt);
                next_sample_dist += sample_dist;
            }
        }
        
        point_set.push_back(a_star_path.back());
    }
    
    std::cout << "[PlannerManager] 从A*路径采样 " << point_set.size() << " 个点，采样间隔: " << sample_dist << "m" << std::endl;
    
    // 修正所有采样点的Z坐标，确保在最小高度以上（避免地面碰撞）
    for (auto& pt : point_set) {
        if (pt(2) < MIN_Z_HEIGHT) {
            pt(2) = MIN_Z_HEIGHT;
        }
    }
    
    // 调试：输出采样点
    std::cout << "[PlannerManager] 采样点坐标:" << std::endl;
    for (size_t i = 0; i < std::min(point_set.size(), size_t(5)); ++i) {
        std::cout << "  点" << i << ": " << point_set[i].transpose() << std::endl;
    }
    
    // Step 3: 计算时间间隔
    // 基于路径长度和最大速度
    double time_to_goal = path_length / pp_.max_vel_ * 1.5;  // 1.5安全系数
    ts = time_to_goal / (point_set.size() - 1);
    ts = std::max(ts, pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2);  // 确保ts不会太小
    
    std::cout << "[PlannerManager] 路径长度: " << path_length << "m, 时间间隔: " << ts << "秒" << std::endl;
    
    // Step 4: 参数化为B样条控制点
    // 计算起点和终点的速度/加速度
    Eigen::Vector3d init_vel = start_vel;
    if (start_vel.norm() < 0.1) {
        // 使用A*路径的前进方向
        if (a_star_path.size() >= 2) {
            Eigen::Vector3d direction = (a_star_path[1] - a_star_path[0]).normalized();
            // 确保Z方向速度不会导致向下飞
            if (direction(2) < -0.1) {
                direction(2) = 0.0;  // 修正Z方向
                direction.normalize();
            }
            init_vel = direction * 0.5;
        } else {
            init_vel = (local_target_pt - start_pt).normalized() * 0.5;
        }
        std::cout << "[PlannerManager] 起点速度过小，设置初始速度: " << init_vel.transpose() << std::endl;
    }
    
    std::cout << "[PlannerManager] 使用速度: " << init_vel.transpose() << std::endl;
    std::vector<Eigen::Vector3d> start_end_derivatives;
    start_end_derivatives.push_back(init_vel);  // 使用init_vel
    start_end_derivatives.push_back(local_target_vel);
    start_end_derivatives.push_back(start_acc);
    start_end_derivatives.push_back(Eigen::Vector3d::Zero());
    
    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
    
    std::cout << "[PlannerManager] B样条控制点: " << ctrl_pts.cols() << " 个" << std::endl;
    std::cout << "[PlannerManager] 参数化后的前4个控制点:" << std::endl;
    for (int i = 0; i < std::min(4, (int)ctrl_pts.cols()); ++i) {
        std::cout << "  CP" << i << ": " << ctrl_pts.col(i).transpose() << std::endl;
    }
    
    // Step 5: 初始化控制点（检测障碍物并用A*规划绕障路径）
    // 注意：initControlPoints会修改ctrl_pts，所以传入副本
    std::cout << "[PlannerManager] 初始化控制点，检测障碍物..." << std::endl;
    std::vector<std::vector<Eigen::Vector3d>> a_star_pathes;
    a_star_pathes = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);
    std::cout << "[PlannerManager] 找到 " << a_star_pathes.size() << " 个障碍物段" << std::endl;
    
    // Step 6: B样条优化（避障）
    std::cout << "[PlannerManager] 开始B样条优化（避障）..." << std::endl;
    bool optimize_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
    
    if (!optimize_success) {
        std::cout << "[PlannerManager] 错误: B样条优化失败（避障失败）" << std::endl;
        return false;
    }
    
    // 修正所有控制点的Z坐标，确保在最小高度以上（避免地面碰撞）
    for (int i = 0; i < ctrl_pts.cols(); ++i) {
        if (ctrl_pts(2, i) < MIN_Z_HEIGHT) {
            ctrl_pts(2, i) = MIN_Z_HEIGHT;
        }
    }
    
    std::cout << "[PlannerManager] B样条优化成功（已避障）" << std::endl;
    
    // Step 7: 检查可行性并重新分配时间
    UniformBspline pos_traj(ctrl_pts, 3, ts);
    pos_traj.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);
    
    double ratio;
    if (!pos_traj.checkFeasibility(ratio, false)) {
        std::cout << "[PlannerManager] 轨迹不满足动力学约束，重新分配时间（ratio=" << ratio << "）" << std::endl;
        
        // 简化处理：直接缩放时间间隔
        ts = ts * ratio;
        pos_traj = UniformBspline(ctrl_pts, 3, ts);
        std::cout << "[PlannerManager] 时间重分配成功，新时间间隔: " << ts << "秒" << std::endl;
    }
    
    // Step 8: 保存结果
    local_data_.start_time_ = TimePoint::now();
    local_data_.start_pos_ = start_pt;
    local_data_.duration_ = (ctrl_pts.cols() - 3) * ts;  // B样条持续时间
    local_data_.traj_id_ += 1;
    local_data_.position_traj_ = pos_traj;
    local_data_.velocity_traj_ = pos_traj.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    
    std::cout << "[PlannerManager] 局部轨迹生成成功!" << std::endl;
    std::cout << "  轨迹时长: " << local_data_.duration_ << "秒" << std::endl;
    std::cout << "  第一个控制点: [" << ctrl_pts.col(0).transpose() << "]" << std::endl;
    std::cout << "  最后控制点: [" << ctrl_pts.col(ctrl_pts.cols()-1).transpose() << "]" << std::endl;
    
    return true;
}

} // namespace ego_planner

