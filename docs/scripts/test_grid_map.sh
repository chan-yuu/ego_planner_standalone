#!/bin/bash

# Grid Map Standalone 测试脚本
# 测试 grid_map_standalone 是否正常工作

echo "=========================================="
echo "  Grid Map Standalone 测试"
echo "=========================================="

# 检查编译
if [ ! -f "../ego-planner/devel/lib/grid_map_standalone/grid_map_node" ]; then
    echo "❌ grid_map_node 未编译"
    echo "   请先运行: cd ../ego-planner && catkin_make --pkg grid_map_standalone"
    exit 1
fi

echo "✅ grid_map_node 已编译"

# 检查是否有 ROS master
if ! rostopic list &> /dev/null; then
    echo "❌ ROS master 未运行"
    echo "   请先启动 roscore 或仿真环境"
    exit 1
fi

echo "✅ ROS master 运行中"

# 检查必要的话题
echo ""
echo "检查输入话题..."

if rostopic list | grep -q "/odom_world"; then
    echo "✅ /odom_world 存在"
else
    echo "⚠️  /odom_world 不存在 - grid_map 需要里程计信息"
fi

if rostopic list | grep -q "/pcl_render_node/cloud"; then
    echo "✅ /pcl_render_node/cloud 存在"
elif rostopic list | grep -q "/camera/depth/points"; then
    echo "✅ /camera/depth/points 存在"
else
    echo "⚠️  点云话题不存在 - grid_map 需要点云或深度图"
fi

echo ""
echo "=========================================="
echo "  准备启动 grid_map_standalone..."
echo "=========================================="
echo ""
echo "启动命令:"
echo "  roslaunch grid_map_standalone test_grid_map.launch"
echo ""
echo "检查方法:"
echo "  1. 查看话题: rostopic list | grep grid_map"
echo "  2. 检查频率: rostopic hz /grid_map/occupancy_inflate"
echo "  3. 查看数据: rostopic echo /grid_map/occupancy_inflate -n 1"
echo ""
echo "预期输出话题:"
echo "  - /grid_map/occupancy"
echo "  - /grid_map/occupancy_inflate"
echo ""
echo "=========================================="
echo "按 Ctrl+C 停止测试"
echo "=========================================="

# 启动 grid_map
cd ../ego-planner
source devel/setup.bash
roslaunch grid_map_standalone test_grid_map.launch
