/**
 * @file dyn_a_star.h
 * @brief 动态A*路径搜索
 */

#ifndef DYN_A_STAR_H
#define DYN_A_STAR_H

#include "plan_env/grid_map.h"
#include <Eigen/Eigen>
#include <vector>
#include <queue>
#include <memory>

namespace ego_planner {

// 网格节点
struct GridNode {
    enum NodeState { OPENSET = 1, CLOSEDSET = 2, UNDEFINED = 3 };
    
    int rounds{0};          // 搜索轮数
    NodeState state{UNDEFINED};
    Eigen::Vector3i index;
    
    double gScore{0.0}, fScore{0.0};
    GridNode* cameFrom{nullptr};
    
    GridNode() = default;
};

typedef GridNode* GridNodePtr;

// 节点比较器（用于优先队列）
class NodeComparator {
public:
    bool operator()(GridNodePtr node1, GridNodePtr node2) {
        return node1->fScore > node2->fScore;
    }
};

class AStar {
public:
    AStar() = default;
    ~AStar();
    
    // 初始化
    void initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size);
    
    // A*搜索
    bool AstarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
    
    // 获取路径
    std::vector<Eigen::Vector3d> getPath();
    
    typedef std::shared_ptr<AStar> Ptr;

private:
    // 启发式函数
    double getDiagHeu(GridNodePtr node1, GridNodePtr node2);
    double getManhHeu(GridNodePtr node1, GridNodePtr node2);
    double getEuclHeu(GridNodePtr node1, GridNodePtr node2);
    inline double getHeu(GridNodePtr node1, GridNodePtr node2) {
        return getDiagHeu(node1, node2);
    }
    
    // 路径回溯
    std::vector<GridNodePtr> retrievePath(GridNodePtr current);
    
    // 坐标转换和检查
    bool ConvertToIndexAndAdjustStartEndPoints(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt,
                                                Eigen::Vector3i& start_idx, Eigen::Vector3i& end_idx);
    
    inline bool Coord2Index(const Eigen::Vector3d& pt, Eigen::Vector3i& idx) {
        Eigen::Vector3d pt_scaled = (pt - center_) * inv_step_size_;
        idx = Eigen::Vector3i(
            std::floor(pt_scaled(0)),
            std::floor(pt_scaled(1)),
            std::floor(pt_scaled(2))
        ) + CENTER_IDX_;
        
        if (idx(0) < 0 || idx(0) >= POOL_SIZE_(0) ||
            idx(1) < 0 || idx(1) >= POOL_SIZE_(1) ||
            idx(2) < 0 || idx(2) >= POOL_SIZE_(2)) {
            return false;
        }
        return true;
    }
    
    inline Eigen::Vector3d Index2Coord(const Eigen::Vector3i& idx) {
        return ((idx - CENTER_IDX_).cast<double>() * step_size_) + center_;
    }
    
    inline bool checkOccupancy(const Eigen::Vector3d& pos) {
        if (!grid_map_) return true;
        return grid_map_->getOccupancy(pos) == 1;
    }
    
    GridMap::Ptr grid_map_;
    GridNodePtr*** GridNodeMap_;
    
    Eigen::Vector3i POOL_SIZE_;
    Eigen::Vector3i CENTER_IDX_;
    
    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> openSet_;
    std::vector<GridNodePtr> gridPath_;
    
    double step_size_;
    double inv_step_size_;
    Eigen::Vector3d center_;
    
    int rounds_{0};
};

} // namespace ego_planner

#endif // DYN_A_STAR_H
