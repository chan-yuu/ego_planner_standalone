/**
 * @file io_interface.h
 * @brief 输入输出接口抽象层
 * 
 * 将通信方式（共享内存、Socket、ROS等）与规划器核心逻辑解耦
 * 方便后续切换不同的通信方式
 */

#ifndef IO_INTERFACE_H
#define IO_INTERFACE_H

#include <Eigen/Eigen>
#include <vector>
#include <memory>

namespace ego_planner {

// ==================== 输入数据结构 ====================

/**
 * @brief 里程计数据
 */
struct OdomInput {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
    double timestamp;
    bool valid;
};

/**
 * @brief 航点数据
 */
struct WaypointInput {
    std::vector<Eigen::Vector3d> waypoints;
    std::vector<Eigen::Vector3d> velocities;
    double timestamp;
    bool is_new;
    bool valid;
};

/**
 * @brief 点云数据
 */
struct PointCloudInput {
    std::vector<Eigen::Vector3d> points;
    double timestamp;
    bool valid;
};

// ==================== 输出数据结构 ====================

/**
 * @brief B样条轨迹数据
 */
struct BsplineOutput {
    Eigen::MatrixXd control_points;  // 3 x N, 每列是一个控制点
    Eigen::VectorXd knots;           // 节点向量
    int order;                       // B样条阶数
    int traj_id;                     // 轨迹ID
    double start_time;               // 开始时间
};

/**
 * @brief 全局路径数据（多项式轨迹采样）
 */
struct GlobalPathOutput {
    std::vector<Eigen::Vector3d> path_points;
    double path_length;
    double planning_time;
    double timestamp;
};

/**
 * @brief 规划器状态
 */
struct PlannerStateOutput {
    int state;          // 状态编号
    double timestamp;   // 时间戳
};

// ==================== 输入接口 ====================

/**
 * @brief 输入接口抽象类
 * 
 * 所有输入接口（共享内存、Socket、ROS等）都需要继承此类
 */
class InputInterface {
public:
    virtual ~InputInterface() = default;
    
    /**
     * @brief 初始化输入接口
     * @return 是否成功
     */
    virtual bool init() = 0;
    
    /**
     * @brief 读取里程计数据
     * @param odom 输出：里程计数据
     * @return 是否读取到新数据
     */
    virtual bool readOdom(OdomInput& odom) = 0;
    
    /**
     * @brief 读取航点数据
     * @param waypoint 输出：航点数据
     * @return 是否读取到新数据
     */
    virtual bool readWaypoint(WaypointInput& waypoint) = 0;
    
    /**
     * @brief 读取点云数据
     * @param cloud 输出：点云数据
     * @return 是否读取到新数据
     */
    virtual bool readPointCloud(PointCloudInput& cloud) = 0;
    
    /**
     * @brief 清理资源
     */
    virtual void cleanup() = 0;
};

// ==================== 输出接口 ====================

/**
 * @brief 输出接口抽象类
 * 
 * 所有输出接口（共享内存、Socket、ROS等）都需要继承此类
 */
class OutputInterface {
public:
    virtual ~OutputInterface() = default;
    
    /**
     * @brief 初始化输出接口
     * @return 是否成功
     */
    virtual bool init() = 0;
    
    /**
     * @brief 发布B样条轨迹
     * @param bspline B样条轨迹数据
     * @return 是否发布成功
     */
    virtual bool publishBspline(const BsplineOutput& bspline) = 0;
    
    /**
     * @brief 发布全局路径
     * @param path 全局路径数据
     * @return 是否发布成功
     */
    virtual bool publishGlobalPath(const GlobalPathOutput& path) = 0;
    
    /**
     * @brief 发布规划器状态
     * @param state 规划器状态
     * @return 是否发布成功
     */
    virtual bool publishState(const PlannerStateOutput& state) = 0;
    
    /**
     * @brief 清理资源
     */
    virtual void cleanup() = 0;
};

// ==================== 工厂函数 ====================

/**
 * @brief 创建输入接口
 * @param type 接口类型 ("shm", "socket", "ros"等)
 * @return 输入接口智能指针
 */
std::unique_ptr<InputInterface> createInputInterface(const std::string& type);

/**
 * @brief 创建输出接口
 * @param type 接口类型 ("shm", "socket", "ros"等)
 * @return 输出接口智能指针
 */
std::unique_ptr<OutputInterface> createOutputInterface(const std::string& type);

} // namespace ego_planner

#endif // IO_INTERFACE_H
