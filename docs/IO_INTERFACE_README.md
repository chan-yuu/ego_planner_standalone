# I/O 接口说明文档

## 概述

为了让 EGO-Planner 的输入输出更加灵活,我们将通信方式抽象为独立的 I/O 接口层。这样可以方便地切换不同的通信方式,而不需要修改核心规划逻辑。

## 架构设计

```
┌─────────────────────────────────────────────┐
│           main.cpp (规划核心逻辑)            │
└─────────────────┬───────────────────────────┘
                  │
        ┌─────────┴─────────┐
        │                   │
   ┌────▼────┐        ┌────▼────┐
   │ Input   │        │ Output  │
   │Interface│        │Interface│
   └────┬────┘        └────┬────┘
        │                  │
   ┌────┴─────────┬────────┴──────┬──────────┐
   │              │               │          │
┌──▼──┐       ┌──▼──┐        ┌──▼──┐    ┌──▼──┐
│ SHM │       │Socket│        │ ROS │    │ ... │
└─────┘       └─────┘        └─────┘    └─────┘
```

## 文件结构

```
planner_standalone/
├── include/
│   └── io_interface/
│       ├── io_interface.h          # 抽象接口定义
│       └── shm_io_interface.h      # 共享内存实现
└── src/
    ├── io_interface.cpp            # 工厂函数
    ├── shm_io_interface.cpp        # 共享内存实现
    ├── main_new.cpp                # 使用I/O接口的新主程序
    └── main.cpp                    # 原始主程序（保留用于对比）
```

## 数据结构

### 输入数据

1. **OdomInput** - 里程计数据
   - position: 位置 (x, y, z)
   - velocity: 速度 (vx, vy, vz)
   - acceleration: 加速度 (ax, ay, az)
   - timestamp: 时间戳
   - valid: 数据是否有效

2. **WaypointInput** - 航点数据
   - waypoints: 航点列表 [Vector3d, ...]
   - velocities: 速度列表 [Vector3d, ...]
   - timestamp: 时间戳
   - is_new: 是否为新航点
   - valid: 数据是否有效

3. **PointCloudInput** - 点云数据
   - points: 点云列表 [Vector3d, ...]
   - timestamp: 时间戳
   - valid: 数据是否有效

### 输出数据

1. **BsplineOutput** - B样条轨迹
   - control_points: 控制点矩阵 (3 x N)
   - knots: 节点向量
   - order: B样条阶数
   - traj_id: 轨迹ID
   - start_time: 开始时间

2. **GlobalPathOutput** - 全局路径
   - path_points: 路径点列表 [Vector3d, ...]
   - path_length: 路径长度
   - planning_time: 规划时间
   - timestamp: 时间戳

3. **PlannerStateOutput** - 规划器状态
   - state: 状态编号
   - timestamp: 时间戳

## 使用方法

### 1. 使用现有接口（共享内存）

```bash
cd planner_standalone/build
./ego_planner_standalone          # 默认使用共享内存
./ego_planner_standalone shm      # 显式指定共享内存
```

### 2. 切换通信方式

在 `main_new.cpp` 中修改:

```cpp
// 方式1: 硬编码
std::string io_type = "socket";  // 改为 "socket", "ros" 等

// 方式2: 命令行参数
./ego_planner_standalone socket
```

### 3. 实现新的通信方式

以 Socket 通信为例:

#### 步骤1: 创建头文件 `socket_io_interface.h`

```cpp
#ifndef SOCKET_IO_INTERFACE_H
#define SOCKET_IO_INTERFACE_H

#include "io_interface/io_interface.h"
#include <string>

namespace ego_planner {

class SocketInputInterface : public InputInterface {
public:
    SocketInputInterface(const std::string& host, int port);
    virtual ~SocketInputInterface();
    
    bool init() override;
    bool readOdom(OdomInput& odom) override;
    bool readWaypoint(WaypointInput& waypoint) override;
    bool readPointCloud(PointCloudInput& cloud) override;
    void cleanup() override;
    
private:
    std::string host_;
    int port_;
    int socket_fd_;
    // ... 其他私有成员
};

class SocketOutputInterface : public OutputInterface {
public:
    SocketOutputInterface(const std::string& host, int port);
    virtual ~SocketOutputInterface();
    
    bool init() override;
    bool publishBspline(const BsplineOutput& bspline) override;
    bool publishGlobalPath(const GlobalPathOutput& path) override;
    bool publishState(const PlannerStateOutput& state) override;
    void cleanup() override;
    
private:
    std::string host_;
    int port_;
    int socket_fd_;
    // ... 其他私有成员
};

} // namespace ego_planner

#endif
```

#### 步骤2: 实现 `socket_io_interface.cpp`

```cpp
#include "io_interface/socket_io_interface.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <iostream>

namespace ego_planner {

SocketInputInterface::SocketInputInterface(const std::string& host, int port)
    : host_(host), port_(port), socket_fd_(-1) {
}

bool SocketInputInterface::init() {
    std::cout << "[SocketInput] 连接到 " << host_ << ":" << port_ << std::endl;
    
    socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd_ < 0) {
        std::cerr << "[SocketInput] 创建socket失败" << std::endl;
        return false;
    }
    
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port_);
    inet_pton(AF_INET, host_.c_str(), &server_addr.sin_addr);
    
    if (connect(socket_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "[SocketInput] 连接失败" << std::endl;
        return false;
    }
    
    std::cout << "[SocketInput] 连接成功" << std::endl;
    return true;
}

bool SocketInputInterface::readOdom(OdomInput& odom) {
    // 从socket读取里程计数据
    // TODO: 实现具体的读取逻辑
    return false;
}

// ... 实现其他方法

} // namespace ego_planner
```

#### 步骤3: 注册到工厂函数

在 `io_interface.cpp` 中添加:

```cpp
#include "io_interface/socket_io_interface.h"

std::unique_ptr<InputInterface> createInputInterface(const std::string& type) {
    if (type == "shm" || type == "shared_memory") {
        return std::make_unique<ShmInputInterface>();
    }
    else if (type == "socket") {
        return std::make_unique<SocketInputInterface>("localhost", 8888);
    }
    // ... 其他类型
    
    return nullptr;
}
```

#### 步骤4: 更新 CMakeLists.txt

```cmake
set(IO_INTERFACE_SOURCES
    src/io_interface.cpp
    src/shm_io_interface.cpp
    src/socket_io_interface.cpp  # 添加新文件
)
```

## 现有实现：共享内存

### 输入接口 (ShmInputInterface)

- **readOdom()**: 从共享内存读取里程计数据
  - 检查序列号,避免重复读取
  - 返回位置、速度、时间戳
  
- **readWaypoint()**: 从共享内存读取航点
  - 检查 `new_waypoint` 标志
  - 读取后清除标志,避免重复触发
  
- **readPointCloud()**: 从共享内存读取点云
  - 批量读取所有点
  - 返回 Vector3d 列表

### 输出接口 (ShmOutputInterface)

- **publishBspline()**: 发布B样条轨迹
  - 复制控制点矩阵 (3 x N)
  - 复制节点向量
  - 设置时间戳和序列号
  
- **publishGlobalPath()**: 发布全局路径
  - 复制采样路径点
  - 设置路径长度和规划时间
  
- **publishState()**: 发布状态
  - 发送FSM状态编号
  - 设置时间戳

## 优势

1. **解耦**: 规划逻辑与通信方式分离
2. **灵活**: 可轻松切换不同通信方式
3. **扩展**: 添加新通信方式无需修改核心代码
4. **测试**: 可以用模拟接口测试规划器
5. **维护**: 通信代码集中管理,易于维护

## 注意事项

1. **线程安全**: 如果多线程访问,需要添加互斥锁
2. **数据验证**: 检查输入数据的有效性和合理性
3. **错误处理**: 通信失败时的重连和恢复机制
4. **性能优化**: 避免频繁的内存分配和复制
5. **序列化**: Socket/网络通信需要定义数据序列化格式

## 扩展示例

### 添加 ROS 接口

```cpp
class RosInputInterface : public InputInterface {
    ros::Subscriber odom_sub_;
    ros::Subscriber waypoint_sub_;
    ros::Subscriber cloud_sub_;
    // ... 实现接口方法
};
```

### 添加文件接口（用于测试）

```cpp
class FileInputInterface : public InputInterface {
    std::ifstream odom_file_;
    std::ifstream waypoint_file_;
    // 从文件读取预先录制的数据
};
```

### 添加模拟接口（用于单元测试）

```cpp
class MockInputInterface : public InputInterface {
    // 返回预设的测试数据
    // 方便自动化测试
};
```

## 编译说明

```bash
cd planner_standalone/build
rm -rf *
cmake ..
make -j4

# 编译后生成两个可执行文件:
# - ego_planner_standalone      # 新版本（使用I/O接口）
# - ego_planner_standalone_old  # 旧版本（保留用于对比）
```

## 未来规划

- [ ] 添加 TCP/UDP Socket 接口
- [ ] 添加 WebSocket 接口（用于Web可视化）
- [ ] 添加 ZeroMQ 接口（高性能消息队列）
- [ ] 添加 gRPC 接口（跨语言通信）
- [ ] 添加文件记录接口（数据录制和回放）
- [ ] 添加性能监控和统计
