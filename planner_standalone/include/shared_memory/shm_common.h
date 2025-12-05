/**
 * @file shm_common.h
 * @brief 共享内存通用定义和数据结构
 * 
 * 定义了规划器和ROS Bridge之间通信的共享内存数据结构
 */

#ifndef SHM_COMMON_H
#define SHM_COMMON_H

#include <cstdint>
#include <cstring>
#include <atomic>
#include <chrono>

namespace ego_planner {
namespace shm {

// 共享内存名称定义
constexpr const char* SHM_ODOM_NAME = "/ego_planner_odom";
constexpr const char* SHM_POINTCLOUD_NAME = "/ego_planner_pointcloud";
constexpr const char* SHM_DEPTH_NAME = "/ego_planner_depth";
constexpr const char* SHM_WAYPOINT_NAME = "/ego_planner_waypoint";
constexpr const char* SHM_BSPLINE_NAME = "/ego_planner_bspline";
constexpr const char* SHM_CMD_NAME = "/ego_planner_cmd";
constexpr const char* SHM_GLOBAL_PATH_NAME = "/ego_planner_global_path";
constexpr const char* SHM_STATE_NAME = "/ego_planner_state";

// 最大值定义
constexpr int MAX_POINTCLOUD_SIZE = 640 * 480;  // 最大点云点数
constexpr int MAX_BSPLINE_CTRL_PTS = 200;       // 最大B样条控制点数
constexpr int MAX_BSPLINE_KNOTS = 250;          // 最大节点数
constexpr int MAX_WAYPOINTS = 50;               // 最大航点数
constexpr int MAX_GLOBAL_PATH_PTS = 500;        // 最大全局路径点数

// 图像尺寸
constexpr int DEPTH_WIDTH = 640;
constexpr int DEPTH_HEIGHT = 480;

/**
 * @brief 时间戳结构体
 */
struct Timestamp {
    int64_t sec;
    int64_t nsec;
    
    Timestamp() : sec(0), nsec(0) {}
    
    void setNow() {
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
        auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);
        sec = seconds.count();
        nsec = nanoseconds.count();
    }
    
    double toSec() const {
        return static_cast<double>(sec) + static_cast<double>(nsec) * 1e-9;
    }
};

/**
 * @brief 3D向量
 */
struct Vector3d {
    double x, y, z;
    
    Vector3d() : x(0), y(0), z(0) {}
    Vector3d(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
};

/**
 * @brief 四元数
 */
struct Quaterniond {
    double w, x, y, z;
    
    Quaterniond() : w(1), x(0), y(0), z(0) {}
    Quaterniond(double _w, double _x, double _y, double _z) 
        : w(_w), x(_x), y(_y), z(_z) {}
};

/**
 * @brief 里程计数据结构
 */
struct OdomData {
    std::atomic<uint64_t> seq;          // 序列号，用于检测数据更新
    std::atomic<bool> valid;            // 数据是否有效
    
    Timestamp stamp;
    Vector3d position;
    Quaterniond orientation;
    Vector3d linear_velocity;
    Vector3d angular_velocity;
    
    OdomData() : seq(0), valid(false) {}
};

/**
 * @brief 点云数据结构
 */
struct PointCloudData {
    std::atomic<uint64_t> seq;
    std::atomic<bool> valid;
    
    Timestamp stamp;
    int num_points;
    float points[MAX_POINTCLOUD_SIZE * 3];  // x, y, z interleaved
    
    PointCloudData() : seq(0), valid(false), num_points(0) {}
};

/**
 * @brief 深度图像数据结构
 */
struct DepthImageData {
    std::atomic<uint64_t> seq;
    std::atomic<bool> valid;
    
    Timestamp stamp;
    int width;
    int height;
    float data[DEPTH_WIDTH * DEPTH_HEIGHT];  // 深度值
    
    // 相机位姿（可选，用于深度图像同步）
    Vector3d camera_position;
    Quaterniond camera_orientation;
    
    DepthImageData() : seq(0), valid(false), width(DEPTH_WIDTH), height(DEPTH_HEIGHT) {}
};

/**
 * @brief 航点数据结构
 */
struct WaypointData {
    std::atomic<uint64_t> seq;
    std::atomic<bool> valid;
    std::atomic<bool> new_waypoint;     // 新航点标志
    
    Timestamp stamp;
    int num_waypoints;
    Vector3d waypoints[MAX_WAYPOINTS];
    
    WaypointData() : seq(0), valid(false), new_waypoint(false), num_waypoints(0) {}
};

/**
 * @brief B样条轨迹数据结构
 */
struct BsplineData {
    std::atomic<uint64_t> seq;
    std::atomic<bool> valid;
    
    Timestamp start_time;
    int traj_id;
    int order;
    
    int num_ctrl_pts;
    double ctrl_pts[MAX_BSPLINE_CTRL_PTS * 3];  // x, y, z interleaved
    
    int num_knots;
    double knots[MAX_BSPLINE_KNOTS];
    
    BsplineData() : seq(0), valid(false), traj_id(0), order(3), 
                    num_ctrl_pts(0), num_knots(0) {}
};

/**
 * @brief 全局路径数据结构（A*规划结果）
 */
struct GlobalPathData {
    std::atomic<uint64_t> seq;
    std::atomic<bool> valid;
    
    Timestamp stamp;
    int num_points;
    double points[MAX_GLOBAL_PATH_PTS * 3];  // x, y, z interleaved
    
    // 路径统计信息
    double path_length;
    double planning_time;
    
    GlobalPathData() : seq(0), valid(false), num_points(0), 
                       path_length(0.0), planning_time(0.0) {}
};

/**
 * @brief 控制命令结构
 */
struct CommandData {
    std::atomic<uint64_t> seq;
    std::atomic<bool> valid;
    
    enum CmdType {
        CMD_NONE = 0,
        CMD_START = 1,
        CMD_STOP = 2,
        CMD_EMERGENCY_STOP = 3,
        CMD_RESET = 4
    };
    
    std::atomic<int> command;
    Timestamp stamp;
    
    CommandData() : seq(0), valid(false), command(CMD_NONE) {}
};

/**
 * @brief 规划器状态数据
 */
struct PlannerStateData {
    std::atomic<uint64_t> seq;
    std::atomic<bool> valid;
    
    enum State {
        STATE_INIT = 0,
        STATE_WAIT_TARGET = 1,
        STATE_GEN_NEW_TRAJ = 2,
        STATE_REPLAN_TRAJ = 3,
        STATE_EXEC_TRAJ = 4,
        STATE_EMERGENCY_STOP = 5
    };
    
    std::atomic<int> state;
    Timestamp stamp;
    
    // 当前位置和速度
    Vector3d current_position;
    Vector3d current_velocity;
    
    // 目标位置
    Vector3d target_position;
    bool has_target;
    
    PlannerStateData() : seq(0), valid(false), state(STATE_INIT), has_target(false) {}
};

}  // namespace shm
}  // namespace ego_planner

#endif  // SHM_COMMON_H
