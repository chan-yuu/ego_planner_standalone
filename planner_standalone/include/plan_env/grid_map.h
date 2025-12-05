/**
 * @file grid_map.h
 * @brief 栅格地图类
 */

#ifndef GRID_MAP_H
#define GRID_MAP_H

#include <Eigen/Eigen>
#include <vector>
#include <memory>

namespace ego_planner {

// 简化的地图参数
struct MappingParameters {
    double resolution_;
    double resolution_inv_;
    Eigen::Vector3d map_origin_;
    Eigen::Vector3d map_size_;
    Eigen::Vector3i map_voxel_num_;
    Eigen::Vector3d map_min_boundary_;
    Eigen::Vector3d map_max_boundary_;
    
    double obstacles_inflation_;
};

// 简化的地图数据
struct MappingData {
    std::vector<double> occupancy_buffer_;
    std::vector<char> occupancy_buffer_inflate_;
};

class GridMap {
public:
    typedef std::shared_ptr<GridMap> Ptr;
    
    GridMap();
    ~GridMap();
    
    // 初始化地图
    void initMap(const Eigen::Vector3d& map_origin,
                 const Eigen::Vector3d& map_size,
                 double resolution);
    
    // 设置障碍物膨胀半径
    void setOccupancyInflation(double inflation_radius);
    
    // 从点云更新占据信息
    void updateOccupancyFromPointCloud(const std::vector<Eigen::Vector3d>& points);
    
    // 膨胀障碍物
    void inflateOccupancy();
    
    // 查询占据状态
    int getOccupancy(const Eigen::Vector3d& pos) const;
    int getOccupancy(const Eigen::Vector3i& id) const;
    
    // 查询距离
    double getDistance(const Eigen::Vector3d& pos) const;
    
    // 获取地图区域
    void getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size) const;
    
    // 检查是否在地图内
    bool isInMap(const Eigen::Vector3d& pos) const;
    bool isInMap(const Eigen::Vector3i& idx) const;
    
    // 坐标转换
    Eigen::Vector3i posToIndex(const Eigen::Vector3d& pos) const;
    Eigen::Vector3d indexToPos(const Eigen::Vector3i& id) const;
    
    // 地址计算
    int toAddress(const Eigen::Vector3i& id) const;
    int toAddress(int x, int y, int z) const;
    
    // 获取参数
    double getResolution() const;
    Eigen::Vector3d getOrigin() const;
    Eigen::Vector3d getMapSize() const;
    
    MappingParameters mp_;
    MappingData md_;
};

} // namespace ego_planner

#endif // GRID_MAP_H
