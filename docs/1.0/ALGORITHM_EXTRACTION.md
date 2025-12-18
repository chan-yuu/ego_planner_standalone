# EGO-Planner 算法提取与架构整理

## 项目概述

本项目将原始的ROS EGO-Planner分离为两部分：
1. **ROS Bridge** (`ego-planner/src/ros_bridge`): 处理ROS消息和共享内存通信
2. **Planner Standalone** (`planner_standalone`): 无ROS依赖的纯C++规划算法

## 当前架构

### 数据流

```
仿真环境 (simulator)
    ↓
ROS话题订阅
    ↓
ROS Bridge Node
    ↓
共享内存 (Shared Memory)
    ↓
Planner Standalone (FSM + 算法)
    ↓
共享内存 (Shared Memory)
    ↓
ROS Bridge Node
    ↓
ROS话题发布 → Traj Server → 控制器
```

### 输入数据（从仿真环境）

1. **定位数据** (`/visual_slam/odom`)
   - 位置、速度、姿态
   - 频率: ~100Hz

2. **点云数据** (`/grid_map/occupancy_inflate`)
   - 已膨胀的障碍物点云
   - 频率: ~10Hz
   - **注意**: 仿真已完成膨胀，算法端不需要再处理

3. **目标航点** (`/waypoint_generator/waypoints`)
   - 目标位置
   - 事件驱动（用户在RViz中点击）

### 输出数据（给控制器）

1. **B样条轨迹** (`/planning/bspline`)
   - 控制点
   - 节点向量 (knots)
   - 轨迹ID
   - 发布给 `traj_server`

2. **轨迹可视化** (`/planning/bspline_path`)
   - Path消息格式
   - 用于RViz显示

3. **规划器状态** (`/planning/planner_state`)
   - FSM状态 (INIT, WAIT_TARGET, GEN_NEW_TRAJ, etc.)

## 核心算法实现

### 1. FSM 状态机 (已完成)

**位置**: `planner_standalone/src/main.cpp`

```cpp
enum FSM_EXEC_STATE {
    INIT,           // 初始化
    WAIT_TARGET,    // 等待目标点
    GEN_NEW_TRAJ,   // 生成新轨迹
    REPLAN_TRAJ,    // 重规划
    EXEC_TRAJ,      // 执行轨迹
    EMERGENCY_STOP  // 紧急停止
};
```

**状态转换逻辑**:
- INIT → WAIT_TARGET: 收到里程计数据
- WAIT_TARGET → GEN_NEW_TRAJ: 收到目标点
- GEN_NEW_TRAJ → EXEC_TRAJ: 规划成功
- EXEC_TRAJ → REPLAN_TRAJ: 距离起点超过阈值
- EXEC_TRAJ → WAIT_TARGET: 轨迹执行完成
- REPLAN_TRAJ → EXEC_TRAJ: 重规划成功

### 2. 全局规划 (需要完善)

**位置**: `planner_standalone/src/planner_manager.cpp::planGlobalTraj()`

**当前实现**:
- ✅ A*路径搜索
- ✅ 路径点插值
- ✅ 多项式轨迹生成 (minimum snap)
- ⚠️ 需要验证和调试

**原始算法** (`ego-planner/src/planner/plan_manage/src/planner_manager.cpp:367-430`):
```cpp
// 1. 插入中间点（如果距离太远）
// 2. 分配时间
// 3. 生成多项式轨迹（minSnapTraj或one_segment_traj_gen）
```

### 3. 局部优化 (需要完善)

**位置**: `planner_standalone/src/planner_manager.cpp::reboundReplan()`

**核心步骤** (原始算法 `ego-planner/src/planner/plan_manage/src/planner_manager.cpp:42-274`):

#### Step 1: 初始化路径
```cpp
// 两种初始化方式:
// a) 从多项式轨迹初始化 (flag_polyInit=true)
//    - 使用一段式或随机多段式多项式轨迹
//    - 采样得到初始控制点
// b) 从之前的轨迹初始化 (flag_polyInit=false)
//    - 使用当前执行中的轨迹
//    - 在末端拼接新的多项式轨迹到目标点
```

#### Step 2: B样条优化
```cpp
// BsplineOptimizer::BsplineOptimizeTrajRebound()
// 优化目标:
// - 平滑性 (加加速度最小化)
// - 障碍物避让
// - 动力学可行性 (速度/加速度限制)
// - 距离global轨迹不要太远
```

#### Step 3: 时间重分配
```cpp
// 如果优化后的轨迹违反动力学约束:
// - 调整时间分配参数 ts
// - 重新优化 (refineTrajAlgo)
```

**当前实现状态**:
- ⚠️ 只有基本框架
- ❌ 缺少详细的B样条优化逻辑
- ❌ 缺少时间重分配

### 4. B样条优化器 (缺失核心实现)

**位置**: `planner_standalone/include/bspline_opt/bspline_optimizer.h`

**需要实现的功能**:

#### a) 初始化控制点 (`initControlPoints`)
- 从初始路径点生成B样条控制点
- 使用A*搜索每对控制点间的局部路径（用于避障）

#### b) Rebound优化 (`BsplineOptimizeTrajRebound`)
**优化公式**:
```
min: λ₁·J_smoothness + λ₂·J_distance + λ₃·J_feasibility + λ₄·J_global
```

- `J_smoothness`: 平滑性代价（三阶导数的积分）
- `J_distance`: 障碍物距离代价
- `J_feasibility`: 动力学可行性代价
- `J_global`: 与全局参考轨迹的偏差

**原始实现位置**:
- `ego-planner/src/planner/bspline_opt/src/bspline_optimizer.cpp`

#### c) Refine优化 (`BsplineOptimizeTrajRefine`)
- 在时间重分配后进行精细化优化
- 使用参考点约束

### 5. 动力学检查和时间重分配 (需要实现)

**位置**: `planner_standalone/src/planner_manager.cpp::refineTrajAlgo()`

**功能**:
- 检查B样条轨迹的速度和加速度是否超限
- 如果超限，重新分配时间参数
- 调用 `reparamBspline` 进行时间重参数化

## 共享内存接口

**位置**: `ego-planner/src/ros_bridge/include/shared_memory/shm_manager.h`

### 数据结构

```cpp
// 输入 (ROS → Standalone)
struct OdomData {
    TimeStamp stamp;
    Vector3 position;
    Quaternion orientation;
    Vector3 linear_velocity;
    Vector3 angular_velocity;
    atomic<bool> valid;
    atomic<uint64_t> seq;
};

struct PointCloudData {
    TimeStamp stamp;
    int num_points;
    float points[MAX_POINTCLOUD_SIZE * 3];
    atomic<bool> valid;
    atomic<uint64_t> seq;
};

struct WaypointData {
    TimeStamp stamp;
    int num_waypoints;
    Vector3 waypoints[MAX_WAYPOINTS];
    atomic<bool> new_waypoint;
    atomic<bool> valid;
    atomic<uint64_t> seq;
};

// 输出 (Standalone → ROS)
struct BsplineData {
    TimeStamp stamp;
    TimeStamp start_time;  // 轨迹开始时刻
    int traj_id;
    int order;
    int num_ctrl_pts;
    int num_knots;
    double ctrl_pts[MAX_BSPLINE_CTRL_PTS * 3];
    double knots[MAX_BSPLINE_KNOTS];
    atomic<bool> valid;
    atomic<uint64_t> seq;
};

struct PlannerStateData {
    TimeStamp stamp;
    int state;  // FSM_EXEC_STATE
    atomic<bool> valid;
    atomic<uint64_t> seq;
};
```

## 下一步工作

### 高优先级

1. **完善B样条优化器** ⭐⭐⭐
   - [ ] 复制原始的 `BsplineOptimizer` 实现
   - [ ] 实现 `BsplineOptimizeTrajRebound()`
   - [ ] 实现 `BsplineOptimizeTrajRefine()`
   - [ ] 实现 `initControlPoints()`
   - **文件**: 
     - 源代码: `ego-planner/src/planner/bspline_opt/src/bspline_optimizer.cpp`
     - 目标: `planner_standalone/src/bspline_optimizer.cpp`

2. **完善轨迹优化流程** ⭐⭐⭐
   - [ ] 完成 `reboundReplan()` 的三个步骤
   - [ ] 实现 `refineTrajAlgo()`
   - [ ] 实现 `reparamBspline()`
   - [ ] 实现 `updateTrajInfo()`
   - **文件**: `planner_standalone/src/planner_manager.cpp`

3. **GridMap完善** ⭐⭐
   - [ ] 实现 `updateOccupancyFromPointCloud()` 
   - [ ] 实现碰撞检测接口
   - [ ] 实现距离场查询
   - **文件**: `planner_standalone/src/grid_map.cpp`

### 中优先级

4. **A*路径搜索完善** ⭐
   - [ ] 验证A*实现的正确性
   - [ ] 添加跳点搜索优化
   - **文件**: `planner_standalone/src/dyn_a_star.cpp`

5. **多项式轨迹生成** ⭐
   - [ ] 验证 `minSnapTraj` 实现
   - [ ] 验证 `one_segment_traj_gen` 实现
   - **文件**: `planner_standalone/src/polynomial_traj.cpp`

### 低优先级

6. **紧急停止** 
   - [ ] 实现 `EmergencyStop()`
   - **文件**: `planner_standalone/src/planner_manager.cpp`

7. **参数调优**
   - [ ] 创建参数配置文件
   - [ ] 实现参数加载功能

8. **日志和调试**
   - [ ] 添加详细的日志输出
   - [ ] 添加性能统计

## 测试计划

### 单元测试
- [ ] GridMap点云更新测试
- [ ] A*路径搜索测试
- [ ] B样条生成和评估测试
- [ ] 共享内存读写测试

### 集成测试
1. **测试1: 静态环境规划**
   - 固定障碍物
   - 单个目标点
   - 验证生成的轨迹

2. **测试2: 动态环境规划**
   - 移动障碍物
   - 验证重规划功能

3. **测试3: 长距离规划**
   - 多个航点
   - 验证持续执行

### 系统测试
- [ ] 在仿真环境中完整运行
- [ ] 与原始ROS版本对比结果
- [ ] 性能测试（规划时间、内存占用）

## 关键文件对照表

| 功能模块 | 原始位置 | 新位置 |
|---------|---------|--------|
| FSM状态机 | `plan_manage/src/ego_replan_fsm.cpp` | `planner_standalone/src/main.cpp` |
| 规划管理器 | `plan_manage/src/planner_manager.cpp` | `planner_standalone/src/planner_manager.cpp` |
| B样条优化 | `bspline_opt/src/bspline_optimizer.cpp` | `planner_standalone/src/bspline_optimizer.cpp` (待完善) |
| A*搜索 | `path_searching/src/dyn_a_star.cpp` | `planner_standalone/src/dyn_a_star.cpp` |
| 栅格地图 | `plan_env/src/grid_map.cpp` | `planner_standalone/src/grid_map.cpp` (待完善) |
| B样条类 | `traj_utils/src/uniform_bspline.cpp` | `planner_standalone/src/uniform_bspline.cpp` |
| ROS接口 | `plan_manage/src/ego_replan_fsm.cpp` | `ego-planner/src/ros_bridge/src/ros_bridge_node.cpp` |

## 编译和运行

### 编译 Planner Standalone
```bash
cd planner_standalone/build
cmake ..
make
```

### 编译 ROS Bridge
```bash
cd ego-planner
catkin_make
source devel/setup.bash
```

### 运行顺序

1. **启动仿真环境**
```bash
roslaunch ego_planner simulator.xml
```

2. **启动Planner Standalone**
```bash
cd planner_standalone/build
./ego_planner_standalone
```

3. **启动ROS Bridge**
```bash
roslaunch ego_planner_bridge sim_only.launch
```

4. **在RViz中点击目标点触发规划**

## 注意事项

1. **启动顺序很重要**
   - 必须先启动 `planner_standalone`，它会创建共享内存
   - 然后启动 `ros_bridge`，它会连接到已存在的共享内存

2. **点云数据已膨胀**
   - 仿真环境发布的 `/grid_map/occupancy_inflate` 已经完成膨胀
   - Planner中不需要再进行膨胀处理

3. **时间同步**
   - B样条的 `start_time` 应该使用轨迹生成时的时间戳
   - 不要每次发布都更新为 `now()`

4. **轨迹持续发布**
   - ROS Bridge应持续发布最新的轨迹（用于控制）
   - 使用 `seq` 来判断是否有新轨迹生成

## 参考资料

- 原始论文: "EGO-Planner: An ESDF-free Gradient-based Local Planner for Quadrotors"
- 原始代码: https://github.com/ZJU-FAST-Lab/ego-planner
- 当前项目文档:
  - `docs/PROJECT_STATUS.md`: 项目整体状态
  - `docs/QUICK_START.md`: 快速开始指南
  - `docs/README.md`: 详细文档
