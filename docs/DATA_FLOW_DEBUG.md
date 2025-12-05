# 数据流问题诊断和修复

## 问题描述

**症状**：
- `/grid_map/occupancy_inflate` 话题有发布，但是 `data: []` 是空的
- `planner_standalone` 没有打印 "收到点云数据" 的信息
- 数据流没有正确连接

## 问题原因

话题订阅不匹配：

```
grid_map_standalone              ros_bridge              planner_standalone
     发布                          订阅                        接收
/grid_map/occupancy_inflate  ✗  /grid_map/cloud  →  共享内存 (PointCloudData)
                              (不匹配！)
```

**错误**：ros_bridge 订阅的是 `/grid_map/cloud`，但 grid_map 实际发布的是 `/grid_map/occupancy_inflate`。

## 修复方案

### 1. 修改 ros_bridge 订阅话题

**文件**: `ego-planner/src/ros_bridge/src/ros_bridge_node.cpp`

```cpp
// 修改前:
cloud_sub_ = nh_.subscribe("/grid_map/cloud", 10, &RosBridgeNode::cloudCallback, this);

// 修改后:
cloud_sub_ = nh_.subscribe("/grid_map/occupancy_inflate", 10, &RosBridgeNode::cloudCallback, this);
```

### 2. 重新编译

```bash
cd /path/to/ego-planner
catkin_make --pkg ego_planner_bridge
source devel/setup.bash
```

### 3. 验证数据流

正确的数据流应该是：

```
仿真环境
  ↓ /pcl_render_node/cloud (原始点云)
  ↓ /odom_world (机器人位姿)
grid_map_standalone
  ↓ 深度处理 + Raycast + 概率更新 + 膨胀
  ↓ /grid_map/occupancy_inflate (膨胀后的障碍物点云)
ros_bridge
  ↓ 订阅 /grid_map/occupancy_inflate
  ↓ 写入共享内存 (PointCloudData)
planner_standalone
  ↓ 从共享内存读取
  ↓ 更新栅格地图 (grid_map_->updateOccupancyFromPointCloud)
  ↓ A* 规划 + B样条优化
```

## 测试步骤

### 1. 启动仿真环境

```bash
# 终端 1: 启动仿真器 (提供点云和里程计)
roslaunch uav_simulator xxx.launch
```

### 2. 启动 grid_map_standalone

```bash
# 终端 2: 启动 grid_map
roslaunch grid_map_standalone test_grid_map.launch
```

**预期输出**:
```
[GridMapStandalone] GridMap initialized successfully
[INFO] Subscribed to: /pcl_render_node/cloud
[INFO] Subscribed to: /odom_world
```

**检查话题**:
```bash
rostopic echo /grid_map/occupancy_inflate -n 1
```

应该看到 `data: [...]` **不为空**（如果有障碍物）。

### 3. 启动 ros_bridge

```bash
# 终端 3: 启动 ROS bridge
cd ego-planner
source devel/setup.bash
rosrun ego_planner_bridge ego_planner_bridge_node
```

**预期输出**:
```
[RosBridge] 初始化成功
  订阅话题: /odom_world
  订阅话题: /grid_map/occupancy_inflate (膨胀后的障碍物点云)
  订阅话题: /waypoint_generator/waypoints
```

**检查点云转发**（如果有障碍物）:
```
收到点云数据: XXX 个点
  示例点: [x, y, z]
点云数据已写入共享内存
```

### 4. 启动 planner_standalone

```bash
# 终端 4: 启动规划器
cd planner_standalone/build
./ego_planner_standalone
```

**预期输出**:
```
位置: [x, y, z], 速度: [vx, vy, vz]
收到点云数据: XXX 个点    # ← 这个应该出现了！
  示例点: [x, y, z]
```

### 5. 发送目标点

```bash
# 终端 5: 在 rviz 中点击 2D Nav Goal，或者发布话题:
rostopic pub /waypoint_generator/waypoints nav_msgs/Path ...
```

**预期输出**:
```
收到新目标点: [x, y, z]
生成新轨迹: 从 [...] 到 [...]
全局规划成功，开始局部优化...
发布Bspline轨迹: XX 控制点
```

## 常见问题排查

### Q1: grid_map 发布的点云是空的

**诊断**:
```bash
rostopic echo /grid_map/occupancy_inflate -n 1
# 如果 data: [] 为空
```

**原因**:
- 没有收到输入点云：检查 `/pcl_render_node/cloud` 是否发布
- 没有检测到障碍物：场景中没有障碍物，或者都在地图范围外
- 参数问题：膨胀半径、深度范围等参数不合理

**解决**:
```bash
# 检查输入
rostopic hz /pcl_render_node/cloud
rostopic hz /odom_world

# 检查参数
rosparam get /grid_map_node/grid_map/obstacles_inflation
rosparam get /grid_map_node/grid_map/max_ray_length
```

### Q2: ros_bridge 没有收到点云

**诊断**:
```bash
# 检查 ros_bridge 日志，是否有 "收到点云数据" 输出
```

**原因**:
- 订阅的话题错误（已修复）
- grid_map 未启动或崩溃

**解决**:
```bash
# 确认话题匹配
rostopic info /grid_map/occupancy_inflate

# 应该显示:
# Publishers:
#  * /grid_map_node (http://...)
# Subscribers:
#  * /ego_planner_bridge_node (http://...)
```

### Q3: planner_standalone 仍然没有收到点云

**诊断**:
```bash
# 检查共享内存
ls -lh /dev/shm/ego_planner_*
```

**原因**:
- 共享内存未初始化
- ros_bridge 没有成功写入
- 序列号（seq）未更新

**解决**:
```bash
# 重启所有节点，清理共享内存
killall ego_planner_standalone ego_planner_bridge_node grid_map_node
rm /dev/shm/ego_planner_*

# 按顺序重启
# 1. grid_map
# 2. ros_bridge
# 3. planner_standalone
```

## 调试技巧

### 1. 可视化点云

在 rviz 中添加：
- **PointCloud2** 显示：
  - Topic: `/pcl_render_node/cloud` (原始点云，红色)
  - Topic: `/grid_map/occupancy_inflate` (膨胀后，蓝色)
- **Path** 显示：
  - Topic: `/planning/global_path` (全局路径)
  - Topic: `/planning/bspline_path` (B样条轨迹)

### 2. 监控话题频率

```bash
# 监控所有关键话题
rostopic hz /odom_world &
rostopic hz /pcl_render_node/cloud &
rostopic hz /grid_map/occupancy_inflate &
rostopic hz /planning/bspline_path &
```

### 3. 录制数据包

```bash
# 录制所有数据用于回放调试
rosbag record -a -O ego_planner_test.bag

# 回放
rosbag play ego_planner_test.bag
```

## 性能优化建议

### 1. grid_map 参数调优

如果点云过于密集，可以调整：

```xml
<!-- grid_map_params.xml -->
<param name="grid_map/skip_pixel" value="4"/>  <!-- 跳过更多像素 -->
<param name="grid_map/depth_filter_maxdist" value="3.0"/>  <!-- 减少处理范围 -->
```

### 2. ros_bridge 降采样

如果点云太大，可以在 ros_bridge 中添加降采样：

```cpp
// cloudCallback 中:
if (cloud_in.points.size() > 5000) {
    // 每隔 N 个点采样一个
    int skip = cloud_in.points.size() / 5000;
    ...
}
```

### 3. planner_standalone 更新频率

如果不需要实时更新地图，可以降低点云处理频率：

```cpp
// main.cpp 中:
static int cloud_update_counter = 0;
if (++cloud_update_counter % 5 == 0) {  // 每5帧更新一次
    planner_manager->grid_map_->updateOccupancyFromPointCloud(points);
}
```

## 总结

**核心修复**：修改 `ros_bridge` 订阅话题从 `/grid_map/cloud` 改为 `/grid_map/occupancy_inflate`。

**数据流验证**：
1. ✅ 仿真环境 → grid_map_standalone (输入点云)
2. ✅ grid_map_standalone → ros_bridge (膨胀点云)
3. ✅ ros_bridge → planner_standalone (共享内存)
4. ✅ planner_standalone → ros_bridge (B样条轨迹)

修复后，应该能看到 `planner_standalone` 输出 "收到点云数据" 的日志。
