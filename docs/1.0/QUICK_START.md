# 快速测试指南

## 启动顺序（重要！）

### 步骤1: 启动独立规划器（必须最先启动）

```bash
# 终端1
cd /home/cyun/12.4/planner_standalone/build
./ego_planner_standalone
```

**预期输出**:
```
共享内存初始化成功
等待数据...
```

**说明**: 规划器会创建以下共享内存段:
- `/dev/shm/ego_planner_bspline`
- `/dev/shm/ego_planner_state`

### 步骤2: 启动ROS桥接节点

```bash
# 终端2
cd /home/cyun/12.4/ego-planner
source devel/setup.bash
roslaunch ego_planner_bridge run_bridge.launch
```

**预期输出**:
```
========================================
  EGO Planner ROS Bridge Node
========================================
Initializing shared memory...
Shared memory initialized successfully
ROS Bridge node started successfully
```

**如果失败**: 会看到警告信息:
```
Waiting for planner_standalone to create shared memory...
Please make sure planner_standalone is running first!
```

**说明**: 桥接节点会:
1. 等待planner_standalone创建的共享内存（最多等待5秒）
2. 创建输入共享内存段:
   - `/dev/shm/ego_planner_odom`
   - `/dev/shm/ego_planner_pointcloud`
   - `/dev/shm/ego_planner_waypoint`
   - `/dev/shm/ego_planner_command`

### 步骤3: 启动ROS仿真环境

```bash
# 终端3
cd /home/cyun/12.4/ego-planner
source devel/setup.bash
roslaunch ego_planner_bridge sim_only.launch
```

**预期输出**: 启动simulator、waypoint_generator、traj_server和RViz

### 步骤4: 发送目标点测试

**方法1**: 使用RViz
- 在RViz界面中点击工具栏的"2D Nav Goal"按钮
- 在地图上点击设置目标点

**方法2**: 使用命令行
```bash
# 终端4
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped \
"header:
  frame_id: 'world'
pose:
  position: {x: 10.0, y: 10.0, z: 1.0}
  orientation: {w: 1.0}"
```

## 验证数据流

### 检查共享内存段

```bash
ls -lh /dev/shm/ego_planner_*
```

**应该看到6个文件**:
```
-rw------- 1 cyun cyun  112 Dec  4 19:00 /dev/shm/ego_planner_odom
-rw------- 1 cyun cyun 4.0M Dec  4 19:00 /dev/shm/ego_planner_pointcloud
-rw------- 1 cyun cyun   64 Dec  4 19:00 /dev/shm/ego_planner_waypoint
-rw------- 1 cyun cyun  16K Dec  4 19:00 /dev/shm/ego_planner_bspline
-rw------- 1 cyun cyun   64 Dec  4 19:00 /dev/shm/ego_planner_command
-rw------- 1 cyun cyun   32 Dec  4 19:00 /dev/shm/ego_planner_state
```

### 监控ROS话题

```bash
# 检查输入话题频率
rostopic hz /odom_world
rostopic hz /grid_map/cloud

# 检查输出话题频率
rostopic hz /planning/bspline_path
rostopic hz /planning/planner_state

# 查看具体内容
rostopic echo /planning/bspline_path
rostopic echo /planning/planner_state
```

### 查看日志

**planner_standalone** (终端1):
```
收到里程计数据: position=[x, y, z]
收到点云数据: N points
收到新的航点: [x, y, z]
开始规划...
规划成功，发布轨迹
```

**ros_bridge** (终端2):
```
Received waypoint data: 1 waypoints
Published bspline trajectory: 10 control points
Planner state: PLANNING
```

## 常见问题

### 问题1: ros_bridge启动失败
**错误**: `Failed to initialize shared memory after 5 seconds!`

**原因**: planner_standalone未启动

**解决**: 先启动planner_standalone (步骤1)

### 问题2: 中文乱码
**现象**: 日志显示"??????"

**原因**: 终端不支持UTF-8编码

**解决**: 已修复，所有消息改为英文

### 问题3: 没有轨迹输出
**检查清单**:
1. ✓ planner_standalone是否正常运行？
2. ✓ ros_bridge是否成功初始化？
3. ✓ 是否发送了目标点？
4. ✓ `/odom_world`和`/grid_map/cloud`是否有数据？

**调试**:
```bash
# 检查所有ego_planner相关进程
ps aux | grep ego_planner

# 检查共享内存
ls -lh /dev/shm/ego_planner_*

# 监控话题
rostopic list | grep planning
```

### 问题4: 清理共享内存
如果需要重启系统，先清理共享内存：

```bash
# 停止所有进程 (Ctrl+C)

# 删除共享内存
rm /dev/shm/ego_planner_*

# 重新按顺序启动
```

## 使用快速启动脚本

```bash
# 终端1: 启动planner_standalone (自动清理共享内存)
./start.sh

# 终端2-3: 手动启动ros_bridge和sim_only
# (参考上面的步骤2-3)
```

## 完整测试流程

1. **启动** planner_standalone
2. **等待** 2秒确保共享内存创建
3. **启动** ros_bridge
4. **等待** 看到"Shared memory initialized successfully"
5. **启动** sim_only.launch
6. **等待** RViz打开
7. **发送** 目标点 (2D Nav Goal)
8. **观察** 
   - planner_standalone日志: "收到新的航点"
   - ros_bridge日志: "Published bspline trajectory"
   - RViz: 显示规划路径

## 性能指标

- **共享内存更新频率**: ~20Hz (planner主循环)
- **ROS话题发布频率**: ~20Hz (ros_bridge发布定时器)
- **规划延迟**: <50ms (从收到航点到发布轨迹)
- **内存占用**: ~6MB (主要是点云数据)
