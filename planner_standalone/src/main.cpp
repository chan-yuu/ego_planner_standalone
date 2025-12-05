/**
 * @file main.cpp
 * @brief EGO Planner独立主程序
 * @author cyun
 * 
 * 核心规划逻辑，通过抽象I/O接口与外部通信
 * 可以方便地切换不同的通信方式（共享内存、Socket、ROS等）
 */

#include <iostream>
#include <thread>
#include <chrono>
#include <signal.h>
#include <Eigen/Eigen>

#include "io_interface/io_interface.h"
#include "plan_env/grid_map.h"
#include "plan_manage/planner_manager.h"
#include "plan_manage/plan_container.hpp"

using namespace ego_planner;

// 全局变量
bool g_running = true;

// 信号处理
void signalHandler(int signum) {
    std::cout << "\n收到中断信号 (" << signum << "), 正在退出..." << std::endl;
    g_running = false;
}

// FSM状态定义
enum FSM_EXEC_STATE {
    INIT,
    WAIT_TARGET,
    GEN_NEW_TRAJ,
    REPLAN_TRAJ,
    EXEC_TRAJ,
    EMERGENCY_STOP
};

const char* state_names[] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

// 加载默认参数
PlannerManagerParams loadDefaultParams() {
    PlannerManagerParams params;
    
    // manager参数
    params.max_vel = 3.0;
    params.max_acc = 3.0;
    params.max_jerk = 4.0;
    params.feasibility_tolerance = 0.05;
    params.ctrl_pt_dist = 0.4;
    params.planning_horizon = 7.5;
    
    // 地图参数
    params.map_params.resolution_ = 0.1;      // 原版使用0.1分辨率
    params.map_params.map_origin_ = Eigen::Vector3d(-20.0, -20.0, 0.0);
    params.map_params.map_size_ = Eigen::Vector3d(40.0, 40.0, 3.0);
    params.map_params.obstacles_inflation_ = 0.099;  // 原版: 0.099 (非常小的膨胀)
    
    // 优化器参数 (使用原版ROS配置)
    params.opt_params.order = 3;
    params.opt_params.lambda1 = 1.0;     // 平滑性权重 (原版: 1.0)
    params.opt_params.lambda2 = 0.5;     // 碰撞权重 (原版: 0.5)
    params.opt_params.lambda3 = 0.1;     // 可行性权重 (原版: 0.1)
    params.opt_params.lambda4 = 1.0;     // 拟合权重 (原版: 1.0)
    params.opt_params.dist0 = 0.5;       // 安全距离 (原版: 0.5)
    params.opt_params.max_vel = params.max_vel;
    params.opt_params.max_acc = params.max_acc;
    params.opt_params.max_iteration_num = 100;
    params.opt_params.max_iteration_time = 0.02;
    
    return params;
}

// 将UniformBspline转换为BsplineOutput
BsplineOutput convertBsplineOutput(const UniformBspline& bspline, int traj_id) {
    BsplineOutput output;
    output.control_points = bspline.getControlPoint();
    output.knots = bspline.getKnot();
    output.order = 3;
    output.traj_id = traj_id;
    output.start_time = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    return output;
}

// 将多项式轨迹转换为GlobalPathOutput
GlobalPathOutput convertGlobalPathOutput(const PolynomialTraj& global_traj, double duration) {
    GlobalPathOutput output;
    
    // 自适应采样
    double sample_dt;
    if (duration < 5.0) {
        sample_dt = 0.05;
    } else if (duration < 10.0) {
        sample_dt = 0.1;
    } else {
        sample_dt = 0.2;
    }
    
    int num_samples = static_cast<int>(duration / sample_dt) + 1;
    output.path_points.reserve(num_samples);
    
    for (int i = 0; i < num_samples; ++i) {
        double t = std::min(i * sample_dt, duration);
        output.path_points.push_back(global_traj.evaluate(t));
    }
    
    output.path_length = duration * 2.0;  // 粗略估计
    output.planning_time = 0.0;
    output.timestamp = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    return output;
}

int main(int argc, char** argv) {
    std::cout << "========================================" << std::endl;
    std::cout << "  EGO Planner Standalone " << std::endl;
    std::cout << "========================================" << std::endl;
    
    // 注册信号处理
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    // ========== 初始化I/O接口 ==========
    // 通信方式可以在这里轻松切换: "shm", "socket", "ros"等
    std::string io_type = "shm";  // 默认使用共享内存
    
    // 从命令行参数读取I/O类型（可选）
    if (argc > 1) {
        io_type = argv[1];
        std::cout << "使用命令行指定的I/O类型: " << io_type << std::endl;
    }
    
    auto input_interface = createInputInterface(io_type);
    auto output_interface = createOutputInterface(io_type);
    
    if (!input_interface || !output_interface) {
        std::cerr << "错误: 无法创建I/O接口" << std::endl;
        return -1;
    }
    
    if (!input_interface->init() || !output_interface->init()) {
        std::cerr << "错误: I/O接口初始化失败" << std::endl;
        return -1;
    }
    
    std::cout << "I/O接口初始化成功 (类型: " << io_type << ")" << std::endl;
    
    // ========== 初始化规划器 ==========
    PlannerManagerParams params = loadDefaultParams();
    
    std::cout << "正在创建规划器..." << std::endl;
    auto planner_manager = std::make_unique<EGOPlannerManager>();
    planner_manager->initPlanModules(params);
    std::cout << "规划器创建成功" << std::endl;
    
    // ========== FSM状态变量 ==========
    FSM_EXEC_STATE exec_state = INIT;
    FSM_EXEC_STATE last_state = INIT;
    
    // 规划数据
    Eigen::Vector3d odom_pos(0, 0, 0), odom_vel(0, 0, 0), odom_acc(0, 0, 0);
    Eigen::Vector3d start_pt, start_vel, start_acc;
    Eigen::Vector3d end_pt, end_vel;
    bool has_odom = false;
    bool has_target = false;
    
    int traj_id = 0;
    
    // FSM参数
    const double replan_thresh = 1.5;      // 重规划距离阈值
    const double no_replan_thresh = 1.0;   // 接近目标点不重规划阈值
    
    // 时间记录
    auto start_time = std::chrono::steady_clock::now();
    auto last_replan_time = start_time;
    
    std::cout << "开始主循环..." << std::endl;
    std::cout << "等待里程计和航点数据..." << std::endl;
    
    // ========== 主循环 ==========
    auto loop_rate = std::chrono::milliseconds(50);  // 20Hz
    
    while (g_running) {
        auto current_time = std::chrono::steady_clock::now();
        
        // ========== 读取输入数据 ==========
        
        // 1. 读取里程计
        OdomInput odom_input;
        if (input_interface->readOdom(odom_input)) {
            odom_pos = odom_input.position;
            odom_vel = odom_input.velocity;
            odom_acc = odom_input.acceleration;
            has_odom = true;
            
            // 限流打印
            static auto last_print = std::chrono::steady_clock::now();
            if (std::chrono::duration<double>(current_time - last_print).count() > 1.0) {
                std::cout << "位置: [" << odom_pos.transpose() << "], "
                          << "速度: [" << odom_vel.transpose() << "]" << std::endl;
                last_print = current_time;
            }
        }
        
        // 2. 读取点云并更新地图
        PointCloudInput cloud_input;
        if (input_interface->readPointCloud(cloud_input) && has_odom) {
            // 限流打印
            static auto last_print = std::chrono::steady_clock::now();
            if (std::chrono::duration<double>(current_time - last_print).count() > 2.0) {
                std::cout << "收到点云: " << cloud_input.points.size() << " 个点" << std::endl;
                last_print = current_time;
            }
            
            planner_manager->grid_map_->updateOccupancyFromPointCloud(cloud_input.points);
        }
        
        // 3. 读取航点
        WaypointInput waypoint_input;
        if (input_interface->readWaypoint(waypoint_input) && waypoint_input.waypoints.size() > 0) {
            end_pt = waypoint_input.waypoints[0];
            
            // 检查终点是否在障碍物上
            if (planner_manager->grid_map_->getOccupancy(end_pt) == 1) {
                std::cout << "警告: 目标点在障碍物上，自动搜索可行高度..." << std::endl;
                
                bool found_free = false;
                double search_resolution = 0.2;
                for (double dz = search_resolution; dz < 3.0; dz += search_resolution) {
                    Eigen::Vector3d test_pt = end_pt;
                    test_pt(2) += dz;
                    
                    if (planner_manager->grid_map_->getOccupancy(test_pt) == 0) {
                        end_pt = test_pt;
                        found_free = true;
                        std::cout << "找到可行高度: [" << end_pt.transpose() << "] (+上" << dz << "m)" << std::endl;
                        break;
                    }
                }
                
                if (!found_free) {
                    std::cout << "错误: 无法找到可行高度，使用原始高度 +1.5m" << std::endl;
                    end_pt(2) += 1.5;
                }
            }
            
            // 高度检查
            if (end_pt(2) < 0.1) {
                end_pt(2) = 1.0;
                std::cout << "目标点高度过低，设置为默认高度 1.0m" << std::endl;
            }
            
            end_vel.setZero();
            has_target = true;
            
            std::cout << "收到新目标点: [" << end_pt.transpose() << "]" << std::endl;
            
            // 如果在执行状态，触发重规划
            if (exec_state == EXEC_TRAJ) {
                exec_state = REPLAN_TRAJ;
                std::cout << "状态切换: EXEC_TRAJ -> REPLAN_TRAJ" << std::endl;
            }
        }
        
        // ========== FSM状态机 ==========
        
        // 状态切换时发布状态
        if (last_state != exec_state) {
            std::cout << "FSM状态: " << state_names[exec_state] << std::endl;
            
            PlannerStateOutput state_output;
            state_output.state = static_cast<int>(exec_state);
            state_output.timestamp = std::chrono::duration<double>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            output_interface->publishState(state_output);
            
            last_state = exec_state;
        }
        
        switch (exec_state) {
            case INIT: {
                if (!has_odom) break;
                exec_state = WAIT_TARGET;
                std::cout << "状态切换: INIT -> WAIT_TARGET" << std::endl;
                break;
            }
            
            case WAIT_TARGET: {
                if (!has_target) break;
                exec_state = GEN_NEW_TRAJ;
                std::cout << "状态切换: WAIT_TARGET -> GEN_NEW_TRAJ" << std::endl;
                break;
            }
            
            case GEN_NEW_TRAJ: {
                if (!has_odom || !has_target) break;
                
                start_pt = odom_pos;
                start_vel = odom_vel;
                start_acc.setZero();
                
                std::cout << "生成新轨迹: 从 [" << start_pt.transpose() 
                          << "] 到 [" << end_pt.transpose() << "]" << std::endl;
                
                // 全局规划
                bool success = planner_manager->planGlobalTraj(
                    start_pt, start_vel, start_acc,
                    end_pt, end_vel, Eigen::Vector3d::Zero());
                
                if (success) {
                    std::cout << "全局规划成功，开始局部优化..." << std::endl;
                    
                    // 发布全局路径
                    GlobalPathOutput path_output = convertGlobalPathOutput(
                        planner_manager->global_data_.global_traj_,
                        planner_manager->global_data_.global_duration_);
                    output_interface->publishGlobalPath(path_output);
                    
                    // 局部优化
                    success = planner_manager->reboundReplan(
                        start_pt, start_vel, start_acc,
                        end_pt, end_vel, true, false);
                    
                    if (success) {
                        std::cout << "局部优化成功！" << std::endl;
                        
                        // 发布B样条轨迹
                        traj_id++;
                        BsplineOutput bspline_output = convertBsplineOutput(
                            planner_manager->local_data_.position_traj_, traj_id);
                        output_interface->publishBspline(bspline_output);
                        
                        exec_state = EXEC_TRAJ;
                        last_replan_time = current_time;
                        std::cout << "状态切换: GEN_NEW_TRAJ -> EXEC_TRAJ" << std::endl;
                    } else {
                        std::cout << "局部优化失败，重试..." << std::endl;
                    }
                } else {
                    std::cout << "全局规划失败，重试..." << std::endl;
                }
                
                break;
            }
            
            case REPLAN_TRAJ: {
                if (!has_odom || !has_target) {
                    exec_state = WAIT_TARGET;
                    std::cout << "状态切换: REPLAN_TRAJ -> WAIT_TARGET" << std::endl;
                    break;
                }
                
                // 从当前轨迹重规划
                LocalTrajData* info = &planner_manager->local_data_;
                auto now = std::chrono::steady_clock::now();
                double t_cur = std::chrono::duration<double>(now - last_replan_time).count();
                t_cur = std::min(info->duration_, t_cur);
                
                Eigen::Vector3d replan_pos = info->position_traj_.evaluateDeBoorT(t_cur);
                Eigen::Vector3d replan_vel = info->velocity_traj_.evaluateDeBoorT(t_cur);
                Eigen::Vector3d replan_acc = info->acceleration_traj_.evaluateDeBoorT(t_cur);
                
                std::cout << "重规划: 从 [" << replan_pos.transpose() 
                          << "] 到 [" << end_pt.transpose() << "]" << std::endl;
                
                bool success = planner_manager->reboundReplan(
                    replan_pos, replan_vel, replan_acc,
                    end_pt, end_vel, false, false);
                
                if (success) {
                    std::cout << "重规划成功！" << std::endl;
                    
                    traj_id++;
                    BsplineOutput bspline_output = convertBsplineOutput(
                        planner_manager->local_data_.position_traj_, traj_id);
                    output_interface->publishBspline(bspline_output);
                    
                    exec_state = EXEC_TRAJ;
                    last_replan_time = current_time;
                    std::cout << "状态切换: REPLAN_TRAJ -> EXEC_TRAJ" << std::endl;
                } else {
                    std::cout << "重规划失败，重试..." << std::endl;
                }
                
                break;
            }
            
            case EXEC_TRAJ: {
                LocalTrajData* info = &planner_manager->local_data_;
                auto now = std::chrono::steady_clock::now();
                double t_cur = std::chrono::duration<double>(now - last_replan_time).count();
                
                bool traj_time_finished = (t_cur > info->duration_ - 0.01);
                t_cur = std::min(info->duration_, t_cur);
                
                Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);
                double dist_to_goal = (end_pt - pos).norm();
                
                // 到达目标点
                if (traj_time_finished && dist_to_goal < 0.3) {
                    std::cout << "轨迹执行完成，到达目标！距离: " << dist_to_goal << "m" << std::endl;
                    has_target = false;
                    exec_state = WAIT_TARGET;
                    std::cout << "状态切换: EXEC_TRAJ -> WAIT_TARGET" << std::endl;
                    break;
                }
                
                // 时间到了但未到达
                if (traj_time_finished && dist_to_goal >= 0.3) {
                    std::cout << "轨迹时间结束但未到达目标（距离: " 
                              << dist_to_goal << "m），触发重规划" << std::endl;
                    exec_state = REPLAN_TRAJ;
                    std::cout << "状态切换: EXEC_TRAJ -> REPLAN_TRAJ" << std::endl;
                    break;
                }
                
                // 接近目标，不重规划
                if (dist_to_goal < no_replan_thresh) {
                    break;
                }
                
                // 距离起点过远，触发重规划
                if ((info->start_pos_ - pos).norm() > replan_thresh) {
                    exec_state = REPLAN_TRAJ;
                    std::cout << "距离起点超过阈值，触发重规划" << std::endl;
                }
                
                break;
            }
            
            case EMERGENCY_STOP: {
                std::cout << "紧急停止" << std::endl;
                break;
            }
        }
        
        // 控制循环频率
        auto elapsed = std::chrono::steady_clock::now() - current_time;
        if (elapsed < loop_rate) {
            std::this_thread::sleep_for(loop_rate - elapsed);
        }
    }
    
    // ========== 清理 ==========
    std::cout << "正在清理资源..." << std::endl;
    input_interface->cleanup();
    output_interface->cleanup();
    std::cout << "程序退出" << std::endl;

    
    return 0;
}
