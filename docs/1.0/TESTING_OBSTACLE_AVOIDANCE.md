# 测试避障功能

## 编译状态

✅ **planner_standalone 已成功编译**

包含完整的B样条优化器实现，支持：
- 碰撞避障
- 轨迹平滑性优化
- 动力学约束（速度/加速度限制）
- 终点高度自适应

## 测试步骤

### 1. 准备环境

启动所有必需的组件：

```bash
# 终端 1: 启动grid_map_standalone
cd /path/to/grid_map_standalone
./grid_map_standalone

# 终端 2: 启动ros_bridge
cd /path/to/ego-planner/devel
source setup.bash
rosrun ros_bridge ros_bridge_node

# 终端 3: 启动planner_standalone
cd /path/to/planner_standalone/build
./ego_planner_standalone
```

### 2. 发送测试目标

通过ROS发布目标点：

```bash
# 发送一个目标点（x=5.0, y=3.0, z=1.0）
rostopic pub /goal geometry_msgs/PoseStamped "{header: {frame_id: 'world'}, pose: {position: {x: 5.0, y: 3.0, z: 1.0}, orientation: {w: 1.0}}}"
```

### 3. 观察输出

**正常的避障输出应包含**:

```
[PlannerManager] 收到新目标: [5.0, 3.0, 1.0]
[PlannerManager] 开始Rebound规划
[PlannerManager] 生成初始多项式轨迹
[PlannerManager] 采样生成B样条控制点
[PlannerManager] 开始B样条优化（避障）...
[PlannerManager] B样条优化成功
[PlannerManager] 轨迹可行性检查通过
[PlannerManager] 轨迹规划成功，总时间: 15.3 ms
```

**如果目标点在障碍物上**:

```
[Main] 目标点在障碍物中，向上搜索可行高度...
[Main] 找到可行高度: [5.0, 3.0, 1.6]（提升0.6m）
```

### 4. 检查关键指标

#### 4.1 避障效果

- 轨迹应该**绕过**障碍物，而不是直接穿过
- 在rviz中可视化轨迹，应看到平滑的曲线

#### 4.2 规划时间

- 正常应在 **10-50ms** 内完成
- 如果超过 100ms，可能需要调整优化参数

#### 4.3 轨迹平滑性

- 控制点之间应该平滑过渡
- 没有突变或锯齿状轨迹

#### 4.4 终点高度自适应

- 如果设置的终点在障碍物中，应自动提升高度
- 提升幅度应在 0.2-3.0m 范围内

## 调试输出说明

### 优化器输出

```
[BsplineOptimizer] 开始优化，控制点数: 12
[BsplineOptimizer] 迭代 1/200, cost: 245.6
[BsplineOptimizer] 迭代 15/200, cost: 12.3
[BsplineOptimizer] 收敛，最终cost: 8.7
```

### 如果优化失败

```
[WARN] Solver error. Return = -1, LBFGSERR_UNKNOWNERROR
```

**可能原因**:
1. 起点在障碍物中
2. 终点不可达
3. 优化参数设置不当

## 参数调整

如果避障效果不佳，可以在 `main.cpp` 中调整以下参数：

```cpp
// 在 initPlanModules() 中
params.opt_params.lambda1 = 10.0;   // 平滑性权重（越大越平滑）
params.opt_params.lambda2 = 5.0;    // 碰撞权重（越大越避障）
params.opt_params.dist0 = 0.4;      // 安全距离（米）
params.opt_params.max_vel = 3.0;    // 最大速度
params.opt_params.max_acc = 3.0;    // 最大加速度
```

**建议调整策略**:

- **避障不够**: 增加 `lambda2` (如 10.0)
- **轨迹抖动**: 增加 `lambda1` (如 20.0)
- **距离障碍物太近**: 增加 `dist0` (如 0.6)
- **规划失败**: 降低 `max_vel` 和 `max_acc`

## 常见问题

### Q1: 轨迹仍然是直线，没有避障

**检查**:
- 确认 `grid_map_->getOccupancy()` 返回了正确的障碍物信息
- 查看 `lambda2` 是否足够大（建议 > 5.0）
- 确认优化器成功返回（不是报错后使用的初始轨迹）

### Q2: 优化时间过长 (> 100ms)

**优化**:
- 减少控制点数量（调整采样间隔）
- 设置更严格的收敛条件
- 减少最大迭代次数（在 bspline_optimizer.cpp 中）

### Q3: 规划经常失败

**检查**:
- 起点和终点是否都在自由空间
- 环境是否过于密集（无可行路径）
- 时间分配是否合理（速度/加速度限制）

### Q4: 终点高度自适应不工作

**检查**:
- `grid_map_->getOccupancy(end_pt)` 是否正确返回 1（障碍物）
- 栅格地图分辨率是否合适
- 搜索高度上限（当前 3.0m）是否足够

## 性能基准

**正常性能指标**:
- 规划成功率: > 95%
- 平均规划时间: 20-30ms
- 轨迹平滑度: 加加速度 < 10 m/s³
- 安全距离: 始终保持 > dist0

## 下一步

如果测试通过：
1. 集成到更复杂的仿真环境
2. 增加动态障碍物支持
3. 优化实时性能
4. 添加轨迹可视化

如果测试失败：
1. 检查各组件的数据流
2. 验证栅格地图质量
3. 调整优化器参数
4. 查看详细日志定位问题
