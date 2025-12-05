/**
 * @file grid_map_node.cpp
 * @brief 独立的栅格地图节点 - 使用完整的 grid_map 实现
 * 
 * 这个节点直接使用原始 EGO-Planner 的 GridMap 类，
 * 只是将其作为独立节点运行，不依赖 ego_planner_node
 */

#include <ros/ros.h>
#include "grid_map.h"  // 直接包含本地的 grid_map.h

int main(int argc, char** argv) {
    ros::init(argc, argv, "grid_map_standalone");
    ros::NodeHandle nh("~");
    
    ROS_INFO("========================================");
    ROS_INFO("  Grid Map Standalone Node");
    ROS_INFO("  Using full GridMap implementation");
    ROS_INFO("========================================");
    
    // 创建 GridMap 对象
    GridMap grid_map;
    grid_map.initMap(nh);
    
    ROS_INFO("[GridMapStandalone] GridMap initialized successfully");
    ROS_INFO("  Check topics:");
    ROS_INFO("    rostopic list | grep grid_map");
    ROS_INFO("    rostopic hz /grid_map/occupancy_inflate");
    
    ros::spin();
    
    return 0;
}
