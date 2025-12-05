/**
 * @file shm_io_interface.cpp
 * @brief 共享内存输入输出接口实现
 */

#include "io_interface/shm_io_interface.h"
#include <iostream>
#include <chrono>

namespace ego_planner {

// ==================== 共享内存输入接口 ====================

ShmInputInterface::ShmInputInterface()
    : last_odom_seq_(0)
    , last_waypoint_seq_(0)
    , last_cloud_seq_(0) {
}

ShmInputInterface::~ShmInputInterface() {
    cleanup();
}

bool ShmInputInterface::init() {
    std::cout << "[ShmInput] 正在初始化共享内存输入接口..." << std::endl;
    bool success = shm_.initAsProducer();  // 作为生产者初始化
    if (success) {
        std::cout << "[ShmInput] 共享内存输入接口初始化成功" << std::endl;
    } else {
        std::cerr << "[ShmInput] 共享内存输入接口初始化失败" << std::endl;
    }
    return success;
}

bool ShmInputInterface::readOdom(OdomInput& odom) {
    shm::OdomData* odom_data = shm_.getOdom();
    if (!odom_data || !odom_data->valid.load()) {
        return false;
    }
    
    uint64_t seq = odom_data->seq.load();
    if (seq == last_odom_seq_) {
        return false;  // 没有新数据
    }
    
    // 读取数据
    odom.position << odom_data->position.x, 
                     odom_data->position.y, 
                     odom_data->position.z;
    odom.velocity << odom_data->linear_velocity.x,
                     odom_data->linear_velocity.y,
                     odom_data->linear_velocity.z;
    odom.acceleration.setZero();  // 共享内存中没有加速度
    odom.timestamp = static_cast<double>(odom_data->stamp.sec) + 
                     static_cast<double>(odom_data->stamp.nsec) * 1e-9;
    odom.valid = true;
    
    last_odom_seq_ = seq;
    return true;
}

bool ShmInputInterface::readWaypoint(WaypointInput& waypoint) {
    shm::WaypointData* waypoint_data = shm_.getWaypoint();
    if (!waypoint_data || !waypoint_data->valid.load()) {
        return false;
    }
    
    // 检查是否有新航点
    if (!waypoint_data->new_waypoint.load()) {
        return false;
    }
    
    uint64_t seq = waypoint_data->seq.load();
    if (seq == last_waypoint_seq_) {
        return false;  // 没有新数据
    }
    
    // 读取航点数据
    waypoint.waypoints.clear();
    waypoint.velocities.clear();
    
    int num_waypoints = waypoint_data->num_waypoints;
    for (int i = 0; i < num_waypoints; ++i) {
        Eigen::Vector3d pt(waypoint_data->waypoints[i].x,
                          waypoint_data->waypoints[i].y,
                          waypoint_data->waypoints[i].z);
        waypoint.waypoints.push_back(pt);
        
        // 默认速度为零
        waypoint.velocities.push_back(Eigen::Vector3d::Zero());
    }
    
    waypoint.timestamp = static_cast<double>(waypoint_data->stamp.sec) + 
                        static_cast<double>(waypoint_data->stamp.nsec) * 1e-9;
    waypoint.is_new = true;
    waypoint.valid = true;
    
    last_waypoint_seq_ = seq;
    waypoint_data->new_waypoint.store(false);  // 标记已读取
    
    return true;
}

bool ShmInputInterface::readPointCloud(PointCloudInput& cloud) {
    shm::PointCloudData* cloud_data = shm_.getPointCloud();
    if (!cloud_data || !cloud_data->valid.load()) {
        return false;
    }
    
    uint64_t seq = cloud_data->seq.load();
    if (seq == last_cloud_seq_) {
        return false;  // 没有新数据
    }
    
    // 读取点云数据
    cloud.points.clear();
    int num_points = cloud_data->num_points;
    cloud.points.reserve(num_points);
    
    for (int i = 0; i < num_points; ++i) {
        Eigen::Vector3d pt(cloud_data->points[i*3 + 0],
                          cloud_data->points[i*3 + 1],
                          cloud_data->points[i*3 + 2]);
        cloud.points.push_back(pt);
    }
    
    cloud.timestamp = static_cast<double>(cloud_data->stamp.sec) + 
                     static_cast<double>(cloud_data->stamp.nsec) * 1e-9;
    cloud.valid = true;
    
    last_cloud_seq_ = seq;
    return true;
}

void ShmInputInterface::cleanup() {
    // 共享内存会自动清理
    std::cout << "[ShmInput] 清理共享内存输入接口" << std::endl;
}

// ==================== 共享内存输出接口 ====================

ShmOutputInterface::ShmOutputInterface() {
}

ShmOutputInterface::~ShmOutputInterface() {
    cleanup();
}

bool ShmOutputInterface::init() {
    std::cout << "[ShmOutput] 正在初始化共享内存输出接口..." << std::endl;
    bool success = shm_.initAsProducer();
    if (success) {
        std::cout << "[ShmOutput] 共享内存输出接口初始化成功" << std::endl;
    } else {
        std::cerr << "[ShmOutput] 共享内存输出接口初始化失败" << std::endl;
    }
    return success;
}

bool ShmOutputInterface::publishBspline(const BsplineOutput& bspline) {
    shm::BsplineData* bspline_data = shm_.getBspline();
    if (!bspline_data) {
        return false;
    }
    
    // 获取控制点数量（格式: 3 x N）
    int num_pts = bspline.control_points.cols();
    
    if (num_pts > shm::MAX_BSPLINE_CTRL_PTS) {
        std::cerr << "[ShmOutput] 警告: 控制点数量超过最大值 " 
                  << num_pts << " > " << shm::MAX_BSPLINE_CTRL_PTS << std::endl;
        num_pts = shm::MAX_BSPLINE_CTRL_PTS;
    }
    
    bspline_data->num_ctrl_pts = num_pts;
    bspline_data->order = bspline.order;
    bspline_data->traj_id = bspline.traj_id;
    
    // 设置时间戳
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);
    
    bspline_data->start_time.sec = seconds.count();
    bspline_data->start_time.nsec = nanoseconds.count();
    
    // 复制控制点
    for (int i = 0; i < num_pts; ++i) {
        bspline_data->ctrl_pts[i*3 + 0] = bspline.control_points(0, i);
        bspline_data->ctrl_pts[i*3 + 1] = bspline.control_points(1, i);
        bspline_data->ctrl_pts[i*3 + 2] = bspline.control_points(2, i);
    }
    
    // 复制knots
    int num_knots = bspline.knots.size();
    if (num_knots > shm::MAX_BSPLINE_KNOTS) {
        std::cerr << "[ShmOutput] 警告: 节点数量超过最大值 " 
                  << num_knots << " > " << shm::MAX_BSPLINE_KNOTS << std::endl;
        num_knots = shm::MAX_BSPLINE_KNOTS;
    }
    
    bspline_data->num_knots = num_knots;
    for (int i = 0; i < num_knots; ++i) {
        bspline_data->knots[i] = bspline.knots(i);
    }
    
    bspline_data->valid.store(true);
    bspline_data->seq.fetch_add(1);
    
    std::cout << "[ShmOutput] 发布Bspline轨迹: " << num_pts << " 控制点, " 
              << num_knots << " knots, traj_id=" << bspline.traj_id << std::endl;
    
    return true;
}

bool ShmOutputInterface::publishGlobalPath(const GlobalPathOutput& path) {
    shm::GlobalPathData* path_data = shm_.getGlobalPath();
    if (!path_data) {
        return false;
    }
    
    int num_points = path.path_points.size();
    if (num_points > shm::MAX_GLOBAL_PATH_PTS) {
        std::cerr << "[ShmOutput] 警告: 路径点数量超过最大值 " 
                  << num_points << " > " << shm::MAX_GLOBAL_PATH_PTS << std::endl;
        num_points = shm::MAX_GLOBAL_PATH_PTS;
    }
    
    path_data->num_points = num_points;
    
    // 复制路径点
    for (int i = 0; i < num_points; ++i) {
        path_data->points[i*3 + 0] = path.path_points[i](0);
        path_data->points[i*3 + 1] = path.path_points[i](1);
        path_data->points[i*3 + 2] = path.path_points[i](2);
    }
    
    // 设置时间戳
    auto now = std::chrono::system_clock::now();
    auto d = now.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(d);
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(d - seconds);
    
    path_data->stamp.sec = seconds.count();
    path_data->stamp.nsec = nanoseconds.count();
    path_data->path_length = path.path_length;
    path_data->planning_time = path.planning_time;
    
    path_data->valid.store(true);
    path_data->seq.fetch_add(1);
    
    std::cout << "[ShmOutput] 发布全局路径: " << num_points << " 个路径点" << std::endl;
    
    return true;
}

bool ShmOutputInterface::publishState(const PlannerStateOutput& state) {
    shm::PlannerStateData* state_data = shm_.getState();
    if (!state_data) {
        return false;
    }
    
    state_data->state = state.state;
    
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);
    
    state_data->stamp.sec = seconds.count();
    state_data->stamp.nsec = nanoseconds.count();
    
    state_data->valid.store(true);
    state_data->seq.fetch_add(1);
    
    return true;
}

void ShmOutputInterface::cleanup() {
    // 共享内存会自动清理
    std::cout << "[ShmOutput] 清理共享内存输出接口" << std::endl;
}

} // namespace ego_planner
