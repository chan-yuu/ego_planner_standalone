/**
 * @file shm_io_interface.h
 * @brief 共享内存输入输出接口实现
 * 
 * 实现基于共享内存的输入输出接口
 */

#ifndef SHM_IO_INTERFACE_H
#define SHM_IO_INTERFACE_H

#include "io_interface/io_interface.h"
#include "shared_memory/shm_manager.h"

namespace ego_planner {

/**
 * @brief 共享内存输入接口
 */
class ShmInputInterface : public InputInterface {
public:
    ShmInputInterface();
    virtual ~ShmInputInterface();
    
    bool init() override;
    bool readOdom(OdomInput& odom) override;
    bool readWaypoint(WaypointInput& waypoint) override;
    bool readPointCloud(PointCloudInput& cloud) override;
    void cleanup() override;
    
private:
    shm::ShmInterface shm_;
    uint64_t last_odom_seq_;
    uint64_t last_waypoint_seq_;
    uint64_t last_cloud_seq_;
};

/**
 * @brief 共享内存输出接口
 */
class ShmOutputInterface : public OutputInterface {
public:
    ShmOutputInterface();
    virtual ~ShmOutputInterface();
    
    bool init() override;
    bool publishBspline(const BsplineOutput& bspline) override;
    bool publishGlobalPath(const GlobalPathOutput& path) override;
    bool publishState(const PlannerStateOutput& state) override;
    void cleanup() override;
    
private:
    shm::ShmInterface shm_;
};

} // namespace ego_planner

#endif // SHM_IO_INTERFACE_H
