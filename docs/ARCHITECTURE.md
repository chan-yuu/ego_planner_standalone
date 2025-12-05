# EGO-Planner 独立版本 - I/O接口架构

## 文件结构

```
planner_standalone/
│
├── include/
│   ├── io_interface/              ← 【新增】I/O接口抽象层
│   │   ├── io_interface.h         # 抽象接口定义
│   │   └── shm_io_interface.h     # 共享内存实现
│   │
│   ├── shared_memory/             # 共享内存定义(原有)
│   │   ├── shm_common.h
│   │   └── shm_manager.h
│   │
│   ├── plan_env/                  # 规划环境(原有)
│   ├── plan_manage/               # 规划管理(原有)
│   └── ...
│
├── src/
│   ├── io_interface.cpp           ← 【新增】工厂函数
│   ├── shm_io_interface.cpp       ← 【新增】共享内存I/O实现
│   ├── main_new.cpp               ← 【新增】使用I/O接口的主程序
│   ├── main.cpp                   # 原主程序(保留)
│   └── ...
│
├── IO_INTERFACE_README.md         ← 【新增】详细使用文档
├── REFACTOR_NOTES.md              ← 【新增】重构说明
└── CMakeLists.txt                 # 已更新
```

## 数据流图

### 输入数据流

```
ROS Topic                共享内存              I/O接口              规划器
  ↓                        ↓                    ↓                   ↓
/odom      ─→  ros_bridge ─→ [SHM]  ─→  ShmInputInterface  ─→  main_new.cpp
/waypoint  ─→  ros_bridge ─→ [SHM]  ─→  ShmInputInterface  ─→  main_new.cpp
/cloud     ─→  ros_bridge ─→ [SHM]  ─→  ShmInputInterface  ─→  main_new.cpp
```

### 输出数据流

```
规划器                 I/O接口                共享内存             ROS Topic
  ↓                     ↓                       ↓                   ↓
main_new.cpp ─→ ShmOutputInterface ─→  [SHM]  ─→  ros_bridge  ─→  /bspline
main_new.cpp ─→ ShmOutputInterface ─→  [SHM]  ─→  ros_bridge  ─→  /global_path
main_new.cpp ─→ ShmOutputInterface ─→  [SHM]  ─→  ros_bridge  ─→  /planner_state
```

## 接口层次

```
┌─────────────────────────────────────────────┐
│           main_new.cpp                      │
│         (规划核心逻辑)                       │
│  - FSM状态机                                │
│  - 轨迹生成                                 │
│  - 重规划逻辑                               │
└──────────────┬──────────────────────────────┘
               │
   ┌───────────┴────────────┐
   ↓                        ↓
┌────────────────┐    ┌────────────────┐
│InputInterface  │    │OutputInterface │
│  (抽象类)      │    │  (抽象类)      │
└────────┬───────┘    └────────┬───────┘
         │                     │
    ┌────┴─────┬───────┬───────┴────┬─────────┐
    ↓          ↓       ↓            ↓         ↓
┌───────┐  ┌──────┐ ┌────┐    ┌───────┐  ┌──────┐
│  SHM  │  │Socket│ │ROS │    │  SHM  │  │Socket│
│       │  │      │ │    │    │       │  │      │
└───────┘  └──────┘ └────┘    └───────┘  └──────┘
  (已实现)   (待实现) (待实现)   (已实现)   (待实现)
```

## 主要类和方法

### 输入接口

```
InputInterface (抽象类)
├── init()                 # 初始化
├── readOdom()             # 读取里程计
├── readWaypoint()         # 读取航点
├── readPointCloud()       # 读取点云
└── cleanup()              # 清理资源

ShmInputInterface (共享内存实现)
└── 实现所有抽象方法
```

### 输出接口

```
OutputInterface (抽象类)
├── init()                 # 初始化
├── publishBspline()       # 发布B样条轨迹
├── publishGlobalPath()    # 发布全局路径
├── publishState()         # 发布状态
└── cleanup()              # 清理资源

ShmOutputInterface (共享内存实现)
└── 实现所有抽象方法
```

## 使用示例

### 基本使用

```cpp
// 创建I/O接口
auto input = createInputInterface("shm");
auto output = createOutputInterface("shm");

// 初始化
input->init();
output->init();

// 读取数据
OdomInput odom;
if (input->readOdom(odom)) {
    // 使用里程计数据
}

// 发布结果
BsplineOutput bspline;
// ... 填充数据
output->publishBspline(bspline);
```

### 切换通信方式

```cpp
// 只需改一行!
auto input = createInputInterface("socket");  // 或 "ros", "file" 等
```

## 扩展新接口

### 1. 创建头文件

```cpp
// socket_io_interface.h
class SocketInputInterface : public InputInterface {
    // 实现接口方法
};
```

### 2. 实现功能

```cpp
// socket_io_interface.cpp
bool SocketInputInterface::readOdom(OdomInput& odom) {
    // 从Socket读取数据
    return true;
}
```

### 3. 注册到工厂

```cpp
// io_interface.cpp
if (type == "socket") {
    return std::make_unique<SocketInputInterface>();
}
```

### 4. 使用新接口

```bash
./ego_planner_standalone socket
```

## 编译输出

```bash
$ make -j4
...
[100%] Built target ego_planner_standalone      # 新版本(使用I/O接口)
[100%] Built target ego_planner_standalone_old  # 旧版本(保留备份)
```

## 快速开始

```bash
# 1. 编译
cd planner_standalone/build
cmake ..
make -j4

# 2. 运行(使用共享内存,与原来行为一致)
./ego_planner_standalone

# 3. 或指定通信方式
./ego_planner_standalone shm
```

## 注意事项

✅ **优点**
- 代码解耦,易于维护
- 灵活切换通信方式
- 方便添加新接口
- 便于单元测试

⚠️ **注意**
- 所有I/O操作都通过接口
- 不要直接操作共享内存
- 新旧版本功能完全一致
- 可以随时回退到旧版本

## 未来计划

- [ ] Socket通信接口
- [ ] ROS2接口
- [ ] 文件记录/回放接口
- [ ] WebSocket接口(用于Web可视化)
- [ ] Mock接口(用于自动化测试)
