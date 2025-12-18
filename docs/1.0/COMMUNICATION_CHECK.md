# 共享内存通信完整性检查

## 数据流向图

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           ROS仿真环境                                    │
│  (simulator.xml + waypoint_generator + traj_server)                    │
└────────────┬──────────────────────────────────────┬─────────────────────┘
             │                                      │
             │ ROS话题                               │ ROS话题
             ▼                                      ▼
    ┌────────────────┐                    ┌────────────────┐
    │ /odom_world    │                    │ /planning/     │
    │ /grid_map/cloud│                    │  bspline_path  │
    │ /waypoints     │                    │ /planning/     │
    └────────┬───────┘                    │  planner_state │
             │                            └────────▲───────┘
             │                                     │
             ▼                                     │
┌────────────────────────────────────────────────┴─────────────────────────┐
│                         ego_planner_bridge_node                          │
│                          (ROS ↔ 共享内存转换)                             │
└────────────┬──────────────────────────────────────┬─────────────────────┘
             │                                      │
             │ 共享内存写                             │ 共享内存读
             ▼                                      ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                            POSIX共享内存                                 │
│  /ego_planner_odom                                                      │
│  /ego_planner_pointcloud                                                │
│  /ego_planner_waypoint                                                  │
│  /ego_planner_bspline         ← 由planner_standalone写入                │
│  /ego_planner_command                                                   │
│  /ego_planner_state           ← 由planner_standalone写入                │
└────────────┬──────────────────────────────────────┬─────────────────────┘
             │                                      │
             │ 共享内存读                             │ 共享内存写
             ▼                                      ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                         planner_standalone                              │
│                      (独立规划算法，无ROS依赖)                            │
│  - 读取: odom, pointcloud, waypoint                                     │
│  - 写入: bspline, state                                                 │
└─────────────────────────────────────────────────────────────────────────┘
```

## 共享内存段详细说明

### 1. /ego_planner_odom (112 bytes)
- **创建者**: ros_bridge (initAsConsumer)
- **写入者**: ros_bridge (odomCallback)
- **读取者**: planner_standalone (initAsProducer)
- **数据结构**: OdomData
  - stamp (8 bytes)
  - position (12 bytes)
  - orientation (16 bytes)
  - linear_velocity (12 bytes)
  - angular_velocity (12 bytes)
  - seq, valid
- **ROS话题**: `/odom_world` (nav_msgs::Odometry)
- **更新频率**: ~100Hz (仿真频率)

### 2. /ego_planner_pointcloud (~4MB)
- **创建者**: ros_bridge (initAsConsumer)
- **写入者**: ros_bridge (cloudCallback)
- **读取者**: planner_standalone (initAsProducer)
- **数据结构**: PointCloudData
  - stamp (8 bytes)
  - num_points (4 bytes)
  - points[MAX_POINTCLOUD_SIZE=100000] (1.2MB)
  - seq, valid
- **ROS话题**: `/grid_map/cloud` (sensor_msgs::PointCloud2)
- **更新频率**: ~10Hz

### 3. /ego_planner_waypoint (64 bytes)
- **创建者**: ros_bridge (initAsConsumer)
- **写入者**: ros_bridge (waypointCallback)
- **读取者**: planner_standalone (initAsProducer)
- **数据结构**: WaypointData
  - stamp (8 bytes)
  - num_waypoints (4 bytes)
  - waypoints[MAX_WAYPOINTS=10] (120 bytes)
  - new_waypoint (原子标志)
  - seq, valid
- **ROS话题**: `/waypoint_generator/waypoints` (nav_msgs::Path)
- **更新频率**: 事件驱动 (用户发送目标点)

### 4. /ego_planner_bspline (~16KB) ✅
- **创建者**: planner_standalone (initAsProducer)
- **写入者**: planner_standalone (main loop)
- **读取者**: ros_bridge (initAsConsumer)
- **数据结构**: BsplineData
  - stamp (8 bytes)
  - num_ctrl_pts (4 bytes)
  - ctrl_pts[MAX_BSPLINE_CTRL_PTS=100] (1200 bytes)
  - knot_span (8 bytes)
  - order (4 bytes)
  - seq, valid
- **ROS话题**: `/planning/bspline_path` (nav_msgs::Path)
- **更新频率**: ~20Hz (规划循环)

### 5. /ego_planner_command (64 bytes)
- **创建者**: ros_bridge (initAsConsumer)
- **写入者**: ros_bridge (未实现，预留)
- **读取者**: planner_standalone (initAsProducer)
- **数据结构**: CommandData
  - command (4 bytes): 0=NONE, 1=REPLAN, 2=STOP
  - seq, valid
- **用途**: 外部控制命令 (预留接口)
- **状态**: ⚠️ 未使用

### 6. /ego_planner_state (32 bytes) ✅
- **创建者**: planner_standalone (initAsProducer)
- **写入者**: planner_standalone (main loop)
- **读取者**: ros_bridge (initAsConsumer)
- **数据结构**: PlannerStateData
  - state (4 bytes): 0=IDLE, 1=PLANNING, 2=EXECUTING, 3=ERROR
  - seq, valid
- **ROS话题**: `/planning/planner_state` (std_msgs::Int32)
- **更新频率**: ~20Hz

## 通信完整性验证

### ✅ ROS → 共享内存 → planner_standalone

| 数据 | ROS话题 | 共享内存 | 写入者 | 读取者 | 状态 |
|------|---------|----------|--------|--------|------|
| 里程计 | `/odom_world` | `/ego_planner_odom` | ros_bridge | planner_standalone | ✅ 已实现 |
| 点云 | `/grid_map/cloud` | `/ego_planner_pointcloud` | ros_bridge | planner_standalone | ✅ 已实现 |
| 航点 | `/waypoint_generator/waypoints` | `/ego_planner_waypoint` | ros_bridge | planner_standalone | ✅ 已实现 |

### ✅ planner_standalone → 共享内存 → ROS

| 数据 | 共享内存 | ROS话题 | 写入者 | 读取者 | 状态 |
|------|----------|---------|--------|--------|------|
| B样条轨迹 | `/ego_planner_bspline` | `/planning/bspline_path` | planner_standalone | ros_bridge | ✅ 已实现 |
| 规划器状态 | `/ego_planner_state` | `/planning/planner_state` | planner_standalone | ros_bridge | ✅ 已实现 |

### ⚠️ 预留接口

| 数据 | 共享内存 | 用途 | 状态 |
|------|----------|------|------|
| 命令 | `/ego_planner_command` | 外部控制 | ⚠️ 未使用 |

## 同步机制

### 原子操作
- 每个共享内存段包含 `std::atomic<uint64_t> seq` 序列号
- 每个共享内存段包含 `std::atomic<bool> valid` 有效标志
- 无需互斥锁，通过序列号检测数据更新

### 数据更新检测
```cpp
// ros_bridge端
static uint64_t last_seq = 0;
uint64_t current_seq = data->seq.load();
if (current_seq != last_seq) {
    last_seq = current_seq;
    // 处理新数据
}
```

### 写入模式
```cpp
// 写入共享内存
data->field1 = value1;
data->field2 = value2;
data->valid.store(true);
data->seq.fetch_add(1);  // 最后更新序列号
```

## 验证测试步骤

### 1. 检查共享内存创建
```bash
# 启动planner_standalone
cd planner_standalone/build
./ego_planner_standalone &

# 检查共享内存段
ls -lh /dev/shm/ego_planner_*

# 应该看到:
# /dev/shm/ego_planner_bspline
# /dev/shm/ego_planner_state
```

### 2. 启动ROS桥接
```bash
# 启动ros_bridge
cd ego-planner
source devel/setup.bash
roslaunch ego_planner_bridge run_bridge.launch &

# 检查共享内存段（应该增加）
ls -lh /dev/shm/ego_planner_*

# 应该看到全部6个:
# /dev/shm/ego_planner_odom
# /dev/shm/ego_planner_pointcloud
# /dev/shm/ego_planner_waypoint
# /dev/shm/ego_planner_bspline
# /dev/shm/ego_planner_command
# /dev/shm/ego_planner_state
```

### 3. 启动仿真环境
```bash
roslaunch ego_planner_bridge sim_only.launch
```

### 4. 验证数据流

#### 验证输入 (ROS → planner_standalone)
```bash
# 终端1: 监控odom话题
rostopic hz /odom_world

# 终端2: 监控点云话题
rostopic hz /grid_map/cloud

# 终端3: 查看planner_standalone日志
# 应该看到 "收到里程计数据", "收到点云数据"
```

#### 验证输出 (planner_standalone → ROS)
```bash
# 终端1: 监控轨迹话题
rostopic echo /planning/bspline_path

# 终端2: 监控状态话题
rostopic echo /planning/planner_state

# 终端3: 发送航点
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'world'
pose:
  position:
    x: 10.0
    y: 10.0
    z: 1.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0"
```

### 5. 性能测试
```bash
# 检查共享内存数据更新频率
watch -n 0.1 'cat /dev/shm/ego_planner_odom | od -An -t u8 | head -1'

# 监控ROS话题频率
rostopic hz /odom_world
rostopic hz /grid_map/cloud
rostopic hz /planning/bspline_path
rostopic hz /planning/planner_state
```

## 总结

### ✅ 已完成的通信链路
1. **仿真环境 → ros_bridge**:
   - `/odom_world` → `OdomData`
   - `/grid_map/cloud` → `PointCloudData`
   - `/waypoint_generator/waypoints` → `WaypointData`

2. **ros_bridge → planner_standalone**:
   - `OdomData` → 规划器读取
   - `PointCloudData` → 地图更新
   - `WaypointData` → 目标点设置

3. **planner_standalone → ros_bridge**:
   - 规划结果 → `BsplineData`
   - 状态信息 → `PlannerStateData`

4. **ros_bridge → ROS可视化**:
   - `BsplineData` → `/planning/bspline_path`
   - `PlannerStateData` → `/planning/planner_state`

### 通信状态: 🎉 完整实现

所有关键数据流都已实现:
- ✅ 输入: odom, pointcloud, waypoint
- ✅ 输出: bspline, state
- ✅ 同步机制: atomic序列号
- ✅ 双向通信: ROS ↔ 共享内存 ↔ 独立算法

### CommandData说明
`CommandData`是预留接口,用于未来扩展外部控制命令(如强制重规划、紧急停止等)。当前系统通过航点驱动规划,不需要额外命令接口。
