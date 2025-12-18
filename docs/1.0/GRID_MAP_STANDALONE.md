# Grid Map Standalone 包说明

## 概述

`grid_map_standalone` 是从 EGO-Planner 中提取出来的**完整**栅格地图实现，作为独立节点运行。

### 为什么需要这个包？

当我们将 EGO-Planner 算法提取到 `planner_standalone` 后，原来集成在 `ego_planner_node` 中的 grid_map 功能不再可用。`planner_standalone` 需要订阅 `/grid_map/occupancy_inflate` 话题来获取障碍物信息，因此需要一个独立的 grid_map 节点。

## 功能特性

### ✅ 完整实现（非简化版）

- **深度图像处理**: 使用相机内参投影深度图到 3D 点云
- **Raycast 算法**: 对自由空间进行光线投射
- **概率占据更新**: 使用 logit 空间进行概率融合 (p_hit, p_miss)
- **障碍物膨胀**: 对占据体素进行膨胀处理
- **两种输入模式**:
  - 深度图 + 相机位姿 (`/grid_map/depth` + `/grid_map/pose`)
  - 点云 + 里程计 (`/grid_map/cloud` + `/grid_map/odom`)

## 架构

```
┌─────────────────────────────────────────────────────────────────┐
│                    仿真环境 / 真实机器人                           │
│  输出: /odom_world, /pcl_render_node/cloud, /depth, /pose        │
└────────────────────────┬────────────────────────────────────────┘
                         │
                         ▼
         ┌───────────────────────────────┐
         │    grid_map_standalone        │
         │  (grid_map_node)              │
         │                               │
         │  输入:                         │
         │   - /grid_map/odom            │
         │   - /grid_map/cloud  或       │
         │   - /grid_map/depth+pose      │
         │                               │
         │  处理:                         │
         │   - 深度投影 / 点云处理        │
         │   - Raycast                  │
         │   - 概率占据更新               │
         │   - 障碍物膨胀                 │
         │                               │
         │  输出:                         │
         │   - /grid_map/occupancy       │
         │   - /grid_map/occupancy_inflate│
         └────────────┬──────────────────┘
                      │
                      ▼
         ┌────────────────────────────┐
         │  planner_standalone        │
         │  (通过 ros_bridge 接收)     │
         │                            │
         │  订阅:                      │
         │   - /grid_map/occupancy_inflate (通过 ros_bridge 转换到共享内存) │
         └────────────────────────────┘
```

## 核心实现

### 文件结构

```
grid_map_standalone/
├── CMakeLists.txt          # 编译配置
├── package.xml             # ROS 包定义
├── include/
│   ├── grid_map.h          # GridMap 类定义（原始完整版）
│   └── raycast.h           # Raycast 算法
├── src/
│   ├── grid_map_node.cpp   # ROS 节点 wrapper（极简）
│   ├── grid_map.cpp        # GridMap 类实现（原始完整版）
│   └── raycast.cpp         # Raycast 算法实现
└── launch/
    ├── grid_map_params.xml # 完整参数配置
    └── test_grid_map.launch # 测试启动文件
```

### grid_map_node.cpp（30 行代码）

```cpp
int main(int argc, char** argv) {
    ros::init(argc, argv, "grid_map_standalone");
    ros::NodeHandle nh("~");
    
    GridMap grid_map;
    grid_map.initMap(nh);  // 所有功能都在 GridMap 类中
    
    ros::spin();
    return 0;
}
```

**设计理念**: 极简 wrapper，让原始 `GridMap` 类处理所有复杂逻辑。

## 使用方法

### 1. 编译

```bash
cd /path/to/ego-planner
catkin_make --pkg grid_map_standalone
source devel/setup.bash
```

### 2. 启动测试

```bash
# 终端 1: 启动仿真环境（提供点云和里程计话题）
roslaunch ...your_simulator.launch

# 终端 2: 启动 grid_map
roslaunch grid_map_standalone test_grid_map.launch
```

### 3. 检查话题

```bash
# 查看发布的话题
rostopic list | grep grid_map

# 应该看到:
#   /grid_map/occupancy           # 原始占据栅格
#   /grid_map/occupancy_inflate   # 膨胀后的占据栅格

# 检查频率
rostopic hz /grid_map/occupancy_inflate
```

### 4. 与 planner_standalone 集成

`ros_bridge_node` 会自动订阅 `/grid_map/occupancy_inflate`，并通过共享内存传递给 `planner_standalone`。

## 参数说明

### 地图参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `grid_map/resolution` | 0.1 | 体素分辨率 (米) |
| `grid_map/map_size_x/y/z` | 40/40/3 | 地图尺寸 (米) |
| `grid_map/local_update_range_x/y/z` | 5.5/5.5/4.5 | 局部更新范围 (米) |
| `grid_map/obstacles_inflation` | 0.099 | 障碍物膨胀半径 (米) |

### 深度处理参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `grid_map/cx, cy, fx, fy` | 321/243/387/387 | 相机内参 |
| `grid_map/depth_filter_maxdist` | 5.0 | 深度图最大距离 (米) |
| `grid_map/depth_filter_mindist` | 0.2 | 深度图最小距离 (米) |
| `grid_map/skip_pixel` | 2 | 跳过像素数（提高效率） |

### Raycast 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `grid_map/p_hit` | 0.65 | 击中概率 |
| `grid_map/p_miss` | 0.35 | 未击中概率 |
| `grid_map/min_ray_length` | 0.1 | 最小光线长度 (米) |
| `grid_map/max_ray_length` | 4.5 | 最大光线长度 (米) |

### 输入模式

| 参数 | 值 | 说明 |
|------|-----|------|
| `grid_map/pose_type` | 1 | POSE_STAMPED (深度+位姿) |
|  | 2 | ODOMETRY (点云+里程计) |

## 调试技巧

### 1. 检查是否收到输入数据

```bash
rostopic echo /grid_map/odom -n 1      # 检查里程计
rostopic echo /grid_map/cloud -n 1     # 检查点云
```

### 2. 可视化占据栅格

在 rviz 中添加 `PointCloud2` 显示：
- Topic: `/grid_map/occupancy_inflate`
- Fixed Frame: `world`
- Color: 使用高度着色或固定颜色

### 3. 查看节点日志

```bash
rosnode info /grid_map_node
```

应该显示:
- Subscriptions: `/grid_map/odom`, `/grid_map/cloud`
- Publications: `/grid_map/occupancy`, `/grid_map/occupancy_inflate`

## 与原始版本的区别

| 项目 | 原始 ego_planner | grid_map_standalone |
|------|------------------|---------------------|
| 代码来源 | 完全相同的 grid_map.cpp/h | 完全相同的 grid_map.cpp/h |
| 功能 | ✅ 深度投影、Raycast、概率更新 | ✅ 深度投影、Raycast、概率更新 |
| 运行方式 | 集成在 ego_planner_node 中 | 独立节点 |
| 包含路径 | `#include <plan_env/grid_map.h>` | `#include "grid_map.h"` |
| 依赖 | 需要 plan_env 包 | 无需其他包，自包含 |

## 常见问题

### Q: 为什么不用简化版本？

**A**: 用户明确要求 "请实现完整的功能而不是简化版本"。原始 grid_map 的功能包括:
- 深度图像投影（需要相机内参）
- Raycast 光线投射
- 概率占据更新（logit 空间）
- 深度过滤（去除噪声）
- 障碍物膨胀

简化版本会丢失这些重要功能，影响障碍物检测精度。

### Q: planner_standalone 如何获取障碍物信息？

**A**: 通过 `ros_bridge_node`:

```
grid_map_node                 ros_bridge_node              planner_standalone
     ↓                              ↓                             ↓
/grid_map/occupancy_inflate  →  订阅并转换  →  写入共享内存  →  读取共享内存
```

### Q: 可以只用点云输入吗（不用深度图）？

**A**: 可以！设置 `pose_type=2`，grid_map 会使用 `cloudCallback` 和 `odomCallback`，不需要深度图。

### Q: 膨胀半径可以调大吗？

**A**: 可以！修改 `grid_map/obstacles_inflation` 参数（单位：米）。但注意：
- 太大会让狭窄通道无法通过
- 太小可能与障碍物碰撞
- 原始默认值 0.099m 是经过测试的安全值

## 测试结果

编译输出：
```
[ 25%] Building CXX object grid_map_standalone/CMakeFiles/grid_map_node.dir/src/grid_map_node.cpp.o
[ 50%] Building CXX object grid_map_standalone/CMakeFiles/grid_map_node.dir/src/grid_map.cpp.o
[ 75%] Building CXX object grid_map_standalone/CMakeFiles/grid_map_node.dir/src/raycast.cpp.o
[100%] Linking CXX executable .../grid_map_node
[100%] Built target grid_map_node
```

✅ 编译成功

## 下一步

1. **测试 grid_map_standalone**:
   ```bash
   roslaunch grid_map_standalone test_grid_map.launch
   ```

2. **集成到 planner_standalone**:
   确保 `ros_bridge_node` 订阅 `/grid_map/occupancy_inflate` 并写入共享内存

3. **端到端测试**:
   ```bash
   # 终端 1: 仿真环境
   roslaunch ...
   
   # 终端 2: grid_map
   roslaunch grid_map_standalone test_grid_map.launch
   
   # 终端 3: planner (包含 ros_bridge 和 planner_standalone)
   ./scripts/start.sh
   ```

## 参考

- 原始代码: `ego-planner/src/planner/plan_env/`
- 相关文档: `docs/GRID_MAP_STANDALONE.md`
- 共享内存接口: `planner_standalone/include/shared_memory/shm_common.h`
