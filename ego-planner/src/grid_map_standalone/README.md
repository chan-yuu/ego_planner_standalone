# Grid Map Standalone

独立的栅格地图功能包，用于生成膨胀后的占据栅格地图。

## 功能

1. **点云处理**：订阅深度相机点云 `/camera/depth/points`
2. **占据栅格**：在机器人局部范围内更新占据栅格
3. **障碍物膨胀**：对障碍物进行膨胀处理（可配置膨胀半径）
4. **发布地图**：发布膨胀后的点云到 `/grid_map/occupancy_inflate` 和 `/grid_map/cloud`

## 为什么需要这个功能包？

原始的 EGO-Planner 将地图处理集成在 `ego_planner_node` 中。当我们把算法独立出去（planner_standalone）后，需要一个独立的节点来生成 `/grid_map/cloud` 话题，供规划器使用。

## 参数说明

### 地图尺寸
- `grid_map/resolution`: 栅格分辨率 (m)，默认 0.1
- `grid_map/map_size_x/y/z`: 地图尺寸 (m)
- `grid_map/local_update_range_x/y/z`: 局部更新范围 (m)

### 障碍物膨胀
- `grid_map/obstacles_inflation`: **膨胀半径 (m)**，默认 0.25
  - 这个参数非常重要！
  - 值越大，机器人离障碍物越远
  - 建议值：机器人半径 + 安全余量（如 0.25m）

### 点云过滤
- `grid_map/min_ray_length`: 最小射线长度 (m)，默认 0.1
- `grid_map/max_ray_length`: 最大射线长度 (m)，默认 4.5

## 使用方法

### 方式1：单独启动
```bash
roslaunch grid_map_standalone grid_map.launch
```

### 方式2：完整系统启动
```bash
# Terminal 1: 启动 planner_standalone
cd /path/to/planner_standalone/build
./ego_planner_standalone

# Terminal 2: 启动 ROS 系统（包含 grid_map）
source devel/setup.bash
roslaunch grid_map_standalone full_system.launch
```

## 话题说明

### 订阅
- `/odom_world` (nav_msgs/Odometry): 机器人位姿
- `/camera/depth/points` (sensor_msgs/PointCloud2): 深度相机点云

### 发布
- `/grid_map/occupancy_inflate` (sensor_msgs/PointCloud2): 膨胀后的障碍物点云（用于可视化）
- `/grid_map/cloud` (sensor_msgs/PointCloud2): 膨胀后的点云（供 planner_standalone 使用）

## 在 RViz 中查看

添加以下可视化：
1. **PointCloud2** → Topic: `/grid_map/occupancy_inflate` (红色，膨胀后的障碍物)
2. **PointCloud2** → Topic: `/camera/depth/points` (白色，原始点云)
3. **Path** → Topic: `/planning/global_path` (黄色，A*全局路径)
4. **Path** → Topic: `/planning/bspline_path` (绿色，B样条局部轨迹)

## 调试

### 检查点云是否接收
```bash
rostopic echo /grid_map/cloud --noarr
```
应该看到持续更新的点云数据。

### 检查节点输出
```bash
# 应该看到类似输出：
[GridMapStandalone] Updating map with 307200 points
[GridMapStandalone] Occupied: 1523, Inflated: 8764
```

### 调整膨胀半径
如果避障效果不好：
- **膨胀半径太小**：机器人会撞到障碍物
  - 解决：增加 `obstacles_inflation` 参数（如 0.3）
- **膨胀半径太大**：机器人过于保守，无法通过狭窄通道
  - 解决：减少 `obstacles_inflation` 参数（如 0.15）

## 与原始系统的区别

| 项目 | 原始 EGO-Planner | 新系统 |
|------|-----------------|--------|
| 地图处理 | 集成在 ego_planner_node | 独立的 grid_map_standalone |
| 规划器 | ROS 节点 | 无 ROS 依赖的 standalone |
| 通信方式 | ROS 话题 | 共享内存 + ROS Bridge |
| 可维护性 | 耦合 | 解耦，模块化 |

## 常见问题

### Q: 为什么没有避障？
A: 检查以下几点：
1. `/grid_map/cloud` 话题是否有数据？
2. planner_standalone 是否接收到点云？（查看终端输出）
3. 膨胀半径是否足够大？（建议 >= 0.2m）

### Q: 地图更新很慢？
A: 尝试：
1. 减小地图尺寸
2. 增加分辨率（如 0.15m）
3. 减小局部更新范围

### Q: 内存占用太高？
A: 减小地图尺寸或增加分辨率。
例如：40x40x3 @ 0.1m = 200x200x30 = 120万个体素
