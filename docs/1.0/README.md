# EGO Planner Standalone 使用说明

本项目将EGO Planner规划算法从ROS环境中独立出来，通过共享内存与ROS仿真环境通信。

## 项目结构

```
.
├── planner_standalone/     # 独立规划器（无ROS依赖）
│   ├── include/            # 头文件
│   │   ├── shared_memory/  # 共享内存接口
│   │   ├── bspline_opt/    # B样条优化
│   │   ├── path_searching/ # 路径搜索（A*）
│   │   ├── plan_env/       # 环境表示（栅格地图）
│   │   ├── plan_manage/    # 规划管理
│   │   └── traj_utils/     # 轨迹工具
│   ├── src/                # 源文件
│   └── CMakeLists.txt      # CMake构建配置
│
├── ego-planner/            # EGO Planner工作空间
│   └── src/
│       ├── ros_bridge/     # ROS桥接功能包（已集成）
│       │   ├── src/        # ROS节点源码
│       │   ├── include/    # 包含共享内存头文件
│       │   ├── launch/     # 启动文件
│       │   ├── package.xml # ROS包描述
│       │   └── CMakeLists.txt # Catkin构建配置
│       ├── grid_map_standalone/  # **NEW** 独立栅格地图节点
│       │   ├── src/        # 完整 grid_map 实现（深度处理、Raycast）
│       │   ├── include/    # grid_map.h, raycast.h
│       │   ├── launch/     # 参数配置和测试启动文件
│       │   └── CMakeLists.txt
│       ├── planner/        # 原始规划器ROS包
│       └── uav_simulator/  # 仿真环境
```

## 编译步骤

### 1. 编译独立规划器

```bash
cd planner_standalone
mkdir build && cd build
cmake ..
make
```

编译成功后会生成可执行文件 `ego_planner_standalone`。

### 2. 编译EGO Planner工作空间（包含ROS桥接）

ROS桥接已集成到ego-planner工作空间中:

```bash
cd ego-planner
catkin_make
```

编译成功后会生成可执行文件 `ego_planner_bridge_node`，位于:
```
ego-planner/devel/lib/ego_planner_bridge/ego_planner_bridge_node
```

## 运行

### ⚠️ 重要提示

**必须按以下顺序启动**，否则ros_bridge会初始化失败：

1. **先**启动 `planner_standalone` (创建输出共享内存)
2. **再**启动 `ros_bridge` (等待planner的共享内存，创建输入共享内存)
3. **最后**启动 `sim_only.launch` (仿真环境)

### 启动顺序

**重要**: 必须按以下顺序启动各组件:

#### 终端1: 启动独立规划器 (必须最先启动)

```bash
cd planner_standalone/build
./ego_planner_standalone
```

**说明**: 规划器会创建输出共享内存段 (`/ego_planner_bspline`, `/ego_planner_state`)

#### 终端2: 启动ROS仿真环境 (仅仿真，不含原始规划器)

```bash
cd ego-planner
source devel/setup.bash
roslaunch ego_planner_bridge sim_only.launch
```

**说明**: 这个launch文件只启动仿真环境、waypoint_generator和traj_server，**不包含**原始的ego_planner_node

#### 终端3: 启动ROS桥接节点

```bash
cd ego-planner
source devel/setup.bash
roslaunch ego_planner_bridge run_bridge.launch

# 或者直接运行节点
rosrun ego_planner_bridge ego_planner_bridge_node
```

**说明**: 桥接节点负责ROS话题与共享内存之间的双向转换

#### 终端4: 发送目标点 (测试)

```bash
# 使用RViz中的"2D Nav Goal"工具发送目标点
# 或者使用命令行
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \
"header:
  frame_id: 'world'
pose:
  position: {x: 10.0, y: 10.0, z: 1.0}
  orientation: {w: 1.0}"
```

## 通信机制

### 共享内存数据流

```
┌─────────────────────────────────────────────┐
│         ROS仿真环境 (sim_only.launch)        │
│  - Simulator (障碍物、点云)                  │
│  - Waypoint Generator (接收目标点)           │
│  - Traj Server (轨迹跟踪)                   │
└──────────┬─────────────────────┬────────────┘
           │                     │
   ROS话题 │                     │ ROS话题
           │                     │
   ┌───────▼──────────┐  ┌──────▼───────────┐
   │ /odom_world      │  │ /planning/       │
   │ /grid_map/cloud  │  │  bspline_path    │
   │ /waypoints       │  │ /planning/state  │
   └───────┬──────────┘  └──────▲───────────┘
           │                     │
           │                     │
           ▼                     │
┌──────────────────────────────────────────────┐
│     ego_planner_bridge_node (ROS桥接)        │
│  - 订阅ROS话题 → 写入共享内存                │
│  - 读取共享内存 → 发布ROS话题                │
└──────────┬─────────────────────┬─────────────┘
           │                     │
  共享内存 │                     │ 共享内存
   写入    │                     │ 读取
           │                     │
   ┌───────▼──────────┐  ┌──────▼───────────┐
   │ /ego_planner_    │  │ /ego_planner_    │
   │   odom           │  │   bspline        │
   │ /ego_planner_    │  │ /ego_planner_    │
   │   pointcloud     │  │   state          │
   │ /ego_planner_    │  │                  │
   │   waypoint       │  │                  │
   └───────┬──────────┘  └──────▲───────────┘
           │                     │
  共享内存 │                     │ 共享内存
   读取    │                     │ 写入
           │                     │
           ▼                     │
┌──────────────────────────────────────────────┐
│   planner_standalone (独立规划算法)           │
│  - 读取: odom, pointcloud, waypoint          │
│  - 规划: A* + B样条优化                       │
│  - 写入: bspline, state                      │
└──────────────────────────────────────────────┘
```

### 6个共享内存段

| 共享内存名称 | 大小 | 创建者 | 写入者 | 读取者 | 说明 |
|-------------|------|--------|--------|--------|------|
| `/ego_planner_odom` | 112B | ros_bridge | ros_bridge | planner | 位置、速度 |
| `/ego_planner_pointcloud` | ~4MB | ros_bridge | ros_bridge | planner | 障碍物点云 |
| `/ego_planner_waypoint` | 64B | ros_bridge | ros_bridge | planner | 目标航点 |
| `/ego_planner_bspline` | ~16KB | planner | planner | ros_bridge | B样条轨迹 |
| `/ego_planner_command` | 64B | ros_bridge | (预留) | planner | 控制命令 |
| `/ego_planner_state` | 32B | planner | planner | ros_bridge | 规划状态 |

**同步机制**: 每个共享内存段使用`std::atomic<uint64_t> seq`作为序列号，通过序列号变化检测数据更新，无需互斥锁。

### ROS话题映射

**输入（ROS → 共享内存）:**
- `/odom_world` (nav_msgs::Odometry) → `/ego_planner_odom`
- `/grid_map/cloud` (sensor_msgs::PointCloud2) → `/ego_planner_pointcloud`
- `/waypoint_generator/waypoints` (nav_msgs::Path) → `/ego_planner_waypoint`

**输出（共享内存 → ROS）:**
- `/ego_planner_bspline` → `/planning/bspline_path` (nav_msgs::Path)
- `/ego_planner_state` → `/planning/planner_state` (std_msgs::Int32)

## 验证运行

### 检查共享内存段

```bash
# 启动planner_standalone后
ls -lh /dev/shm/ego_planner_*

# 应该看到:
# /dev/shm/ego_planner_bspline
# /dev/shm/ego_planner_state

# 启动ros_bridge后，应该看到全部6个:
# /dev/shm/ego_planner_odom
# /dev/shm/ego_planner_pointcloud
# /dev/shm/ego_planner_waypoint
# /dev/shm/ego_planner_bspline
# /dev/shm/ego_planner_command
# /dev/shm/ego_planner_state
```

### 监控数据流

```bash
# 终端1: 监控ROS输入话题
rostopic hz /odom_world
rostopic hz /grid_map/cloud

# 终端2: 监控ROS输出话题
rostopic hz /planning/bspline_path
rostopic hz /planning/planner_state

# 终端3: 查看规划器日志
# planner_standalone会打印:
# - "收到里程计数据"
# - "收到点云数据"
# - "收到新的航点"
# - "规划成功，发布轨迹"
```

### 测试完整流程

1. 启动所有组件后，在RViz中点击"2D Nav Goal"
2. 观察`planner_standalone`输出是否显示"收到新的航点"
3. 观察`/planning/bspline_path`是否发布轨迹
4. 观察RViz中是否显示规划的路径

## 配置参数

独立规划器的参数在 `planner_standalone/src/main.cpp` 中硬编码配置：

```cpp
// 地图参数
Eigen::Vector3d map_origin(-20.0, -20.0, 0.0);
Eigen::Vector3d map_size(40.0, 40.0, 3.0);
double resolution = 0.1;

// 规划参数  
double max_vel = 2.0;
double max_acc = 3.0;
```

可以根据实际应用修改这些参数后重新编译。

## 调试与故障排除

### 查看共享内存

```bash
# 列出所有ego_planner共享内存段
ls -lh /dev/shm/ego_planner_*

# 查看共享内存详细信息
ipcs -m

# 删除共享内存（需要先停止所有进程）
rm /dev/shm/ego_planner_*
```

### 常见问题

**问题1**: "无法创建共享内存"
- **原因**: 之前的共享内存段未清理
- **解决**: `rm /dev/shm/ego_planner_*`

**问题2**: ros_bridge启动时"无法打开共享内存"
- **原因**: planner_standalone未启动或启动失败
- **解决**: 确保先启动planner_standalone

**问题3**: 没有轨迹输出
- **原因**: 未收到航点或规划失败
- **解决**: 
  - 检查`/waypoint_generator/waypoints`是否发布
  - 查看planner_standalone日志是否有错误
  - 确认点云数据是否正常

### 日志输出

**planner_standalone日志**:
```
共享内存初始化成功
等待数据...
收到里程计数据: position=[x, y, z]
收到点云数据: 1000 points
收到新的航点: [x, y, z]
开始规划...
规划成功，发布轨迹
```

**ros_bridge日志**:
```
共享内存初始化成功
收到航点数据: 1个航点
发布B样条轨迹: 10个控制点
规划器状态: PLANNING
```

## 项目特点

- ✅ **完全解耦**: 规划算法与ROS完全独立，使用纯C++和CMake
- ✅ **高效通信**: POSIX共享内存实现低延迟进程间通信
- ✅ **无锁设计**: 使用`std::atomic`实现无锁同步机制
- ✅ **完整数据流**: 输入(odom/pointcloud/waypoint) + 输出(bspline/state)
- ✅ **易于集成**: ros_bridge已集成到原始ego-planner工作空间
- ✅ **独立运行**: planner_standalone可独立编译运行，便于调试

## 文件清单

**重要文档**:
- `README.md` - 本文档，完整使用说明
- `PROJECT_STATUS.md` - 项目状态总结
- `COMMUNICATION_CHECK.md` - 共享内存通信完整性检查

**核心代码**:
- `planner_standalone/` - 独立规划器 (236KB可执行文件)
- `ego-planner/src/ros_bridge/` - ROS桥接 (783KB可执行文件)

## 参考资料

- [EGO-Planner原始项目](https://github.com/ZJU-FAST-Lab/ego-planner)
- [POSIX共享内存文档](https://man7.org/linux/man-pages/man7/shm_overview.7.html)
- [B样条轨迹规划](https://ieeexplore.ieee.org/document/9196817)

## 后续开发建议

## 后续开发建议

- [ ] **配置文件**: 添加YAML配置文件，替代硬编码参数
- [ ] **日志系统**: 集成spdlog等结构化日志库
- [ ] **性能监控**: 添加规划时间统计和性能分析
- [ ] **多线程优化**: 分离地图更新和规划线程
- [ ] **完善优化器**: 完整实现B样条优化和梯度下降
- [ ] **动态障碍物**: 支持动态障碍物预测和避障
- [ ] **错误恢复**: 增强错误处理和自动恢复机制

## 许可证

本项目基于[EGO-Planner](https://github.com/ZJU-FAST-Lab/ego-planner)，遵循其原始许可证（GPLv3）。
