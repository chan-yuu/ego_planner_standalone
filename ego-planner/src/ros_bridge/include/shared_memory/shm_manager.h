/**
 * @file shm_manager.h
 * @brief 共享内存管理器
 * 
 * 提供共享内存的创建、打开、读写等操作
 */

#ifndef SHM_MANAGER_H
#define SHM_MANAGER_H

#include "shm_common.h"
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <stdexcept>
#include <cstring>

namespace ego_planner {
namespace shm {

/**
 * @brief 共享内存管理类
 * @tparam T 数据类型
 */
template<typename T>
class SharedMemory {
public:
    SharedMemory() : data_(nullptr), fd_(-1), is_owner_(false) {}
    
    ~SharedMemory() {
        close();
    }
    
    /**
     * @brief 创建共享内存（作为生产者）
     * @param name 共享内存名称
     * @return 是否成功
     */
    bool create(const std::string& name) {
        name_ = name;
        is_owner_ = true;
        
        // 先尝试删除已存在的共享内存
        shm_unlink(name.c_str());
        
        // 创建共享内存
        fd_ = shm_open(name.c_str(), O_CREAT | O_RDWR, 0666);
        if (fd_ == -1) {
            return false;
        }
        
        // 设置大小
        if (ftruncate(fd_, sizeof(T)) == -1) {
            ::close(fd_);
            shm_unlink(name.c_str());
            return false;
        }
        
        // 映射内存
        void* ptr = mmap(nullptr, sizeof(T), PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
        if (ptr == MAP_FAILED) {
            ::close(fd_);
            shm_unlink(name.c_str());
            return false;
        }
        
        data_ = new(ptr) T();  // placement new初始化
        return true;
    }
    
    /**
     * @brief 打开已存在的共享内存（作为消费者）
     * @param name 共享内存名称
     * @param timeout_ms 超时时间（毫秒），-1表示不超时
     * @return 是否成功
     */
    bool open(const std::string& name, int timeout_ms = -1) {
        name_ = name;
        is_owner_ = false;
        
        int elapsed = 0;
        const int interval = 100;  // 100ms检查间隔
        
        while (true) {
            fd_ = shm_open(name.c_str(), O_RDWR, 0666);
            if (fd_ != -1) {
                break;
            }
            
            if (timeout_ms >= 0 && elapsed >= timeout_ms) {
                return false;
            }
            
            usleep(interval * 1000);
            elapsed += interval;
        }
        
        // 映射内存
        void* ptr = mmap(nullptr, sizeof(T), PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
        if (ptr == MAP_FAILED) {
            ::close(fd_);
            return false;
        }
        
        data_ = static_cast<T*>(ptr);
        return true;
    }
    
    /**
     * @brief 关闭共享内存
     */
    void close() {
        if (data_ != nullptr) {
            munmap(data_, sizeof(T));
            data_ = nullptr;
        }
        
        if (fd_ != -1) {
            ::close(fd_);
            fd_ = -1;
        }
        
        if (is_owner_ && !name_.empty()) {
            shm_unlink(name_.c_str());
        }
    }
    
    /**
     * @brief 获取数据指针
     */
    T* get() { return data_; }
    const T* get() const { return data_; }
    
    T* operator->() { return data_; }
    const T* operator->() const { return data_; }
    
    T& operator*() { return *data_; }
    const T& operator*() const { return *data_; }
    
    bool isValid() const { return data_ != nullptr; }
    
private:
    T* data_;
    int fd_;
    std::string name_;
    bool is_owner_;
};

/**
 * @brief 共享内存数据接口类
 * 
 * 为规划器提供统一的数据读写接口
 */
class ShmInterface {
public:
    ShmInterface() = default;
    ~ShmInterface() = default;
    
    /**
     * @brief 初始化为生产者（规划器端）
     */
    bool initAsProducer() {
        // 规划器创建输出共享内存
        if (!bspline_shm_.create(SHM_BSPLINE_NAME)) {
            return false;
        }
        if (!global_path_shm_.create(SHM_GLOBAL_PATH_NAME)) {
            return false;
        }
        if (!state_shm_.create(SHM_STATE_NAME)) {
            return false;
        }
        
        // 清空输出数据（防止旧数据残留）
        BsplineData* bspline_data = bspline_shm_.get();
        if (bspline_data) {
            bspline_data->valid.store(false);
            bspline_data->num_ctrl_pts = 0;
            bspline_data->num_knots = 0;
            bspline_data->seq.store(0);
        }
        
        GlobalPathData* global_path_data = global_path_shm_.get();
        if (global_path_data) {
            global_path_data->valid.store(false);
            global_path_data->num_points = 0;
            global_path_data->seq.store(0);
        }
        
        PlannerStateData* state_data = state_shm_.get();
        if (state_data) {
            state_data->valid.store(false);
            state_data->state = 0;  // INIT
            state_data->seq.store(0);
        }
        
        // 打开输入共享内存（由ros_bridge创建）
        if (!odom_shm_.open(SHM_ODOM_NAME, 5000)) {
            return false;
        }
        if (!pointcloud_shm_.open(SHM_POINTCLOUD_NAME, 5000)) {
            return false;
        }
        if (!waypoint_shm_.open(SHM_WAYPOINT_NAME, 5000)) {
            return false;
        }
        if (!cmd_shm_.open(SHM_CMD_NAME, 5000)) {
            return false;
        }
        
        return true;
    }
    
    /**
     * @brief 初始化为消费者（ros_bridge端）
     */
    bool initAsConsumer() {
        // ros_bridge创建输入共享内存
        if (!odom_shm_.create(SHM_ODOM_NAME)) {
            return false;
        }
        if (!pointcloud_shm_.create(SHM_POINTCLOUD_NAME)) {
            return false;
        }
        if (!waypoint_shm_.create(SHM_WAYPOINT_NAME)) {
            return false;
        }
        if (!cmd_shm_.create(SHM_CMD_NAME)) {
            return false;
        }
        
        // 打开输出共享内存（由规划器创建）
        if (!bspline_shm_.open(SHM_BSPLINE_NAME, 5000)) {
            return false;
        }
        if (!global_path_shm_.open(SHM_GLOBAL_PATH_NAME, 5000)) {
            return false;
        }
        if (!state_shm_.open(SHM_STATE_NAME, 5000)) {
            return false;
        }
        
        return true;
    }
    
    // 获取共享内存指针
    OdomData* getOdom() { return odom_shm_.get(); }
    PointCloudData* getPointCloud() { return pointcloud_shm_.get(); }
    WaypointData* getWaypoint() { return waypoint_shm_.get(); }
    BsplineData* getBspline() { return bspline_shm_.get(); }
    GlobalPathData* getGlobalPath() { return global_path_shm_.get(); }
    CommandData* getCommand() { return cmd_shm_.get(); }
    PlannerStateData* getState() { return state_shm_.get(); }
    
private:
    SharedMemory<OdomData> odom_shm_;
    SharedMemory<PointCloudData> pointcloud_shm_;
    SharedMemory<WaypointData> waypoint_shm_;
    SharedMemory<BsplineData> bspline_shm_;
    SharedMemory<GlobalPathData> global_path_shm_;
    SharedMemory<CommandData> cmd_shm_;
    SharedMemory<PlannerStateData> state_shm_;
};

}  // namespace shm
}  // namespace ego_planner

#endif  // SHM_MANAGER_H
