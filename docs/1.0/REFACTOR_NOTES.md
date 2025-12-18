# I/O 接口重构说明

## 概述

已将 EGO-Planner 的输入输出逻辑从主程序中分离,创建了独立的 I/O 接口层。

## 主要改动

### 1. 新增文件

```
planner_standalone/
├── include/io_interface/
│   ├── io_interface.h           # 抽象I/O接口定义
│   └── shm_io_interface.h       # 共享内存实现
├── src/
│   ├── io_interface.cpp         # 工厂函数
│   ├── shm_io_interface.cpp     # 共享内存实现
│   └── main_new.cpp             # 使用I/O接口的新主程序
└── IO_INTERFACE_README.md       # 详细文档
```

### 2. 架构对比

**旧架构:**
```
main.cpp
├── 直接调用共享内存函数
├── 硬编码的数据格式转换
└── 通信逻辑与规划逻辑耦合
```

**新架构:**
```
main_new.cpp (规划核心)
    ↓
InputInterface / OutputInterface (抽象层)
    ↓
ShmInputInterface / ShmOutputInterface (具体实现)
    ↓
共享内存
```

### 3. 核心接口

#### 输入接口 (InputInterface)
```cpp
virtual bool readOdom(OdomInput& odom) = 0;
virtual bool readWaypoint(WaypointInput& waypoint) = 0;
virtual bool readPointCloud(PointCloudInput& cloud) = 0;
```

#### 输出接口 (OutputInterface)
```cpp
virtual bool publishBspline(const BsplineOutput& bspline) = 0;
virtual bool publishGlobalPath(const GlobalPathOutput& path) = 0;
virtual bool publishState(const PlannerStateOutput& state) = 0;
```

## 使用方法

### 当前默认方式（共享内存）

```bash
cd planner_standalone/build
./ego_planner_standalone          # 使用新的I/O接口
./ego_planner_standalone_old      # 使用旧版本(保留用于对比)
```

### 指定通信方式

```bash
./ego_planner_standalone shm      # 共享内存(默认)
# 未来可支持:
# ./ego_planner_standalone socket
# ./ego_planner_standalone ros
```

## 优势

1. **解耦清晰**: 规划逻辑与通信方式完全分离
2. **易于切换**: 只需修改一行代码即可切换通信方式
3. **方便扩展**: 添加新通信方式无需修改主程序
4. **便于测试**: 可用Mock接口进行单元测试
5. **代码整洁**: I/O相关代码集中管理

## 后续扩展

当您需要使用其他通信方式时,只需:

1. 创建新的接口实现类(继承 InputInterface/OutputInterface)
2. 在工厂函数中注册
3. 在 main.cpp 中切换类型字符串

**无需修改任何规划逻辑代码!**

详细扩展指南请参考: `IO_INTERFACE_README.md`

## 代码对比

### 旧版本 (main.cpp)

```cpp
// 直接操作共享内存
BsplineData* bspline_data = g_shm.getBspline();
bspline_data->num_ctrl_pts = num_pts;
// ... 大量的数据复制代码
```

### 新版本 (main_new.cpp)

```cpp
// 通过接口发布
BsplineOutput output = convertBsplineOutput(bspline, traj_id);
output_interface->publishBspline(output);
```

## 编译说明

编译后生成两个可执行文件:

- **ego_planner_standalone**: 新版本(推荐)
- **ego_planner_standalone_old**: 旧版本(用于对比和备份)

## 注意事项

1. 新旧版本功能完全一致,只是架构不同
2. 默认使用共享内存,与原来的行为一致
3. 旧版本会保留,可随时回退
4. 所有I/O相关代码都在 `io_interface/` 目录下

## 测试建议

1. 先使用旧版本运行,验证系统正常
2. 再使用新版本运行,对比行为是否一致
3. 确认无误后,可删除旧版本相关代码
