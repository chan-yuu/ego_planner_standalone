# 段错误修复总结

## 问题描述

运行 `ego_planner_standalone` 时发生段错误（Segmentation Fault）：

```
[PlannerManager] B样条控制点: 48 个
段错误 (核心已转储)
```

## 根本原因

在调用 `BsplineOptimizeTrajRebound()` 之前，有**两个关键初始化步骤被遗漏**：

1. ❌ **没有调用 `initControlPoints()` 初始化优化器的内部状态**
2. ❌ **没有初始化 `a_star_` 指针**（导致 `initControlPoints()` 中访问空指针）

### 技术细节 - 控制点结构

`BsplineOptimizer` 内部有一个 `cps_` 结构（control points structure），包含：

```cpp
struct ControlPoints {
    Eigen::MatrixXd points;           // 控制点坐标
    std::vector<std::vector<Eigen::Vector3d>> base_point;  // 障碍物基准点
    std::vector<std::vector<Eigen::Vector3d>> direction;   // 避障方向
    std::vector<bool> flag_temp;      // 临时标志
    int size;                         // 控制点数量
    double clearance;                 // 安全距离
};
```

这些成员变量必须通过 `initControlPoints()` 初始化：
1. 设置 `cps_.size` 和 `cps_.clearance`
2. 调整 `cps_.points` 的大小
3. 初始化 `base_point`、`direction`、`flag_temp` 向量
4. 检测初始轨迹中的障碍物段
5. 对每个障碍物段运行 A* 搜索生成绕障路径

**如果跳过这一步直接调用 `BsplineOptimizeTrajRebound()`**：
- `cps_.size` 未初始化（随机值）
- `base_point` 和 `direction` 向量为空
- 访问这些未初始化的成员 → **段错误**

## 修复方案

### 步骤 1: 初始化 A* 搜索器

在 `planner_manager.cpp` 的 `initPlanModules()` 中：

```cpp
// 初始化B样条优化器
bspline_optimizer_rebound_.reset(new BsplineOptimizer());
bspline_optimizer_rebound_->setParam(params.opt_params);
bspline_optimizer_rebound_->setEnvironment(grid_map_);

// ✅ 添加这两行！初始化优化器内部的A*搜索器
bspline_optimizer_rebound_->a_star_.reset(new AStar());
bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 50));
```

**为什么需要这一步？**
- `initControlPoints()` 函数内部会调用 `a_star_->AstarSearch()` 来规划绕障路径
- 如果 `a_star_` 指针为空 → **段错误**

### 步骤 2: 调用 initControlPoints

在 `planner_manager.cpp` 的 `reboundReplan()` 中，在调用优化之前添加初始化步骤：

```cpp
// 旧代码（错误）：
Eigen::MatrixXd ctrl_pts;
UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);
bool optimize_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);

// 新代码（正确）：
Eigen::MatrixXd ctrl_pts;
UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

// ✅ 添加这一步！
std::vector<std::vector<Eigen::Vector3d>> a_star_pathes;
a_star_pathes = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);

bool optimize_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
```

## 修改位置

**文件**: `planner_standalone/src/planner_manager.cpp`

**行号**: ~第 163 行（Step 3.5）

**添加的代码**:
```cpp
// Step 3.5: 初始化控制点（检测障碍物并用A*规划绕障路径）
std::cout << "[PlannerManager] 初始化控制点，检测障碍物..." << std::endl;
std::vector<std::vector<Eigen::Vector3d>> a_star_pathes;
a_star_pathes = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);
std::cout << "[PlannerManager] 找到 " << a_star_pathes.size() << " 个障碍物段" << std::endl;
```

## 验证方法

重新运行程序，应该看到：

```
[PlannerManager] B样条控制点: 48 个
[PlannerManager] 初始化控制点，检测障碍物...
[PlannerManager] 找到 0 个障碍物段        <-- 如果初始轨迹无障碍
[PlannerManager] 开始B样条优化（避障）...
iter(+1)=XX,time(ms)=XX.XXX,total_t(ms)=XX.XXX,cost=XX.XXX  <-- 优化成功
[PlannerManager] B样条优化成功（已避障）
```

**不应再出现段错误**。

## 教训

在集成复杂算法时，必须：
1. **完整理解原始代码的初始化流程**
2. **不能跳过任何必要的初始化步骤**
3. **参考原始实现的调用顺序**

原始 EGO-Planner 的调用顺序：
```cpp
// ego-planner/src/planner/plan_manage/src/planner_manager.cpp
UniformBspline::parameterizeToBspline(..., ctrl_pts);
a_star_pathes = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);  // ← 必须有
bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
```

我们之前的错误就是漏掉了中间的 `initControlPoints()` 调用。

## 相关文件

- `planner_standalone/src/planner_manager.cpp` - 修复位置
- `planner_standalone/src/bspline_optimizer.cpp` - 优化器实现
- `planner_standalone/include/bspline_opt/bspline_optimizer.h` - 接口定义

## 编译状态

✅ **已修复并重新编译成功**

```bash
cd planner_standalone/build
make
```

输出：
```
[ 81%] Built target ego_planner_standalone_lib
[100%] Built target ego_planner_standalone
```

## 下一步

1. 运行程序测试避障功能
2. 观察优化器输出
3. 验证轨迹是否能绕过障碍物

如果仍有问题，检查：
- A* 搜索器是否正确初始化（`a_star_` 指针）
- 栅格地图数据是否有效
- 控制点数量是否合理（> order = 3）
