/**
 * @file grid_map.cpp
 * @brief 栅格地图实现
 */

#include "plan_env/grid_map.h"
#include <iostream>

namespace ego_planner {

GridMap::GridMap() {
}

GridMap::~GridMap() {
}

void GridMap::initMap(const Eigen::Vector3d& map_origin,
                      const Eigen::Vector3d& map_size,
                      double resolution) {
    mp_.map_origin_ = map_origin;
    mp_.map_size_ = map_size;
    mp_.resolution_ = resolution;
    mp_.resolution_inv_ = 1.0 / resolution;
    
    // 计算体素数量
    for (int i = 0; i < 3; ++i) {
        mp_.map_voxel_num_(i) = std::ceil(mp_.map_size_(i) / mp_.resolution_);
    }
    
    mp_.map_min_boundary_ = mp_.map_origin_;
    mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;
    
    // 初始化数据缓冲区
    int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);
    
    md_.occupancy_buffer_ = std::vector<double>(buffer_size, 0.0);
    md_.occupancy_buffer_inflate_ = std::vector<char>(buffer_size, 0);
    
    std::cout << "[GridMap] 地图初始化完成:" << std::endl;
    std::cout << "  - 原点: [" << map_origin.transpose() << "]" << std::endl;
    std::cout << "  - 尺寸: [" << map_size.transpose() << "]" << std::endl;
    std::cout << "  - 分辨率: " << resolution << std::endl;
    std::cout << "  - 体素数量: [" << mp_.map_voxel_num_.transpose() << "]" << std::endl;
}

void GridMap::setOccupancyInflation(double inflation_radius) {
    mp_.obstacles_inflation_ = inflation_radius;
}

void GridMap::updateOccupancyFromPointCloud(const std::vector<Eigen::Vector3d>& points) {
    if (points.empty()) return;
    
    // 清空之前的占据信息
    std::fill(md_.occupancy_buffer_.begin(), md_.occupancy_buffer_.end(), 0.0);
    
    // 将点云中的点标记为占据
    for (const auto& pt : points) {
        if (!isInMap(pt)) continue;
        
        Eigen::Vector3i idx = posToIndex(pt);
        int addr = toAddress(idx);
        if (addr >= 0 && addr < (int)md_.occupancy_buffer_.size()) {
            md_.occupancy_buffer_[addr] = 1.0;
        }
    }
    
    // 膨胀障碍物
    inflateOccupancy();
}

void GridMap::inflateOccupancy() {
    // 简化的障碍物膨胀实现
    std::fill(md_.occupancy_buffer_inflate_.begin(), md_.occupancy_buffer_inflate_.end(), 0);
    
    int inflate_voxels = std::ceil(mp_.obstacles_inflation_ * mp_.resolution_inv_);
    
    for (int x = 0; x < mp_.map_voxel_num_(0); ++x) {
        for (int y = 0; y < mp_.map_voxel_num_(1); ++y) {
            for (int z = 0; z < mp_.map_voxel_num_(2); ++z) {
                Eigen::Vector3i idx(x, y, z);
                int addr = toAddress(idx);
                
                if (md_.occupancy_buffer_[addr] > 0.5) {
                    // 膨胀周围体素
                    for (int dx = -inflate_voxels; dx <= inflate_voxels; ++dx) {
                        for (int dy = -inflate_voxels; dy <= inflate_voxels; ++dy) {
                            for (int dz = -inflate_voxels; dz <= inflate_voxels; ++dz) {
                                Eigen::Vector3i neighbor(x + dx, y + dy, z + dz);
                                if (isInMap(neighbor)) {
                                    int neighbor_addr = toAddress(neighbor);
                                    md_.occupancy_buffer_inflate_[neighbor_addr] = 1;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

int GridMap::getOccupancy(const Eigen::Vector3d& pos) const {
    if (!isInMap(pos)) return -1;
    
    Eigen::Vector3i idx = posToIndex(pos);
    int addr = toAddress(idx);
    
    return md_.occupancy_buffer_inflate_[addr] > 0 ? 1 : 0;
}

int GridMap::getOccupancy(const Eigen::Vector3i& id) const {
    if (!isInMap(id)) return -1;
    
    int addr = toAddress(id);
    return md_.occupancy_buffer_inflate_[addr] > 0 ? 1 : 0;
}

double GridMap::getDistance(const Eigen::Vector3d& pos) const {
    if (!isInMap(pos)) return 0.0;
    
    Eigen::Vector3i idx = posToIndex(pos);
    int addr = toAddress(idx);
    
    // 简化：如果占据返回0，否则返回一个估计距离
    if (md_.occupancy_buffer_inflate_[addr] > 0) {
        return 0.0;
    }
    
    // 搜索最近的障碍物（简化实现）
    double min_dist = 100.0;
    int search_range = 5;
    
    for (int dx = -search_range; dx <= search_range; ++dx) {
        for (int dy = -search_range; dy <= search_range; ++dy) {
            for (int dz = -search_range; dz <= search_range; ++dz) {
                Eigen::Vector3i neighbor = idx + Eigen::Vector3i(dx, dy, dz);
                if (isInMap(neighbor)) {
                    int neighbor_addr = toAddress(neighbor);
                    if (md_.occupancy_buffer_inflate_[neighbor_addr] > 0) {
                        Eigen::Vector3d neighbor_pos = indexToPos(neighbor);
                        double dist = (pos - neighbor_pos).norm();
                        min_dist = std::min(min_dist, dist);
                    }
                }
            }
        }
    }
    
    return min_dist;
}

void GridMap::getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size) const {
    ori = mp_.map_origin_;
    size = mp_.map_size_;
}

bool GridMap::isInMap(const Eigen::Vector3d& pos) const {
    return pos(0) >= mp_.map_min_boundary_(0) && pos(0) <= mp_.map_max_boundary_(0) &&
           pos(1) >= mp_.map_min_boundary_(1) && pos(1) <= mp_.map_max_boundary_(1) &&
           pos(2) >= mp_.map_min_boundary_(2) && pos(2) <= mp_.map_max_boundary_(2);
}

bool GridMap::isInMap(const Eigen::Vector3i& idx) const {
    return idx(0) >= 0 && idx(0) < mp_.map_voxel_num_(0) &&
           idx(1) >= 0 && idx(1) < mp_.map_voxel_num_(1) &&
           idx(2) >= 0 && idx(2) < mp_.map_voxel_num_(2);
}

Eigen::Vector3i GridMap::posToIndex(const Eigen::Vector3d& pos) const {
    Eigen::Vector3i idx;
    for (int i = 0; i < 3; ++i) {
        idx(i) = std::floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
    }
    return idx;
}

Eigen::Vector3d GridMap::indexToPos(const Eigen::Vector3i& id) const {
    Eigen::Vector3d pos;
    for (int i = 0; i < 3; ++i) {
        pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
    }
    return pos;
}

int GridMap::toAddress(const Eigen::Vector3i& id) const {
    return id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
           id(1) * mp_.map_voxel_num_(2) + id(2);
}

int GridMap::toAddress(int x, int y, int z) const {
    return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
           y * mp_.map_voxel_num_(2) + z;
}

double GridMap::getResolution() const {
    return mp_.resolution_;
}

Eigen::Vector3d GridMap::getOrigin() const {
    return mp_.map_origin_;
}

Eigen::Vector3d GridMap::getMapSize() const {
    return mp_.map_size_;
}

} // namespace ego_planner
