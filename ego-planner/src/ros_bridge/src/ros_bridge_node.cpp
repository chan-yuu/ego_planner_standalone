/**
 * @file ros_bridge_node.cpp
 * @brief ROS桥接节点，负责ROS话题与共享内存之间的数据转换
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <ego_planner/Bspline.h>

#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Eigen>

#include "shared_memory/shm_manager.h"

using namespace ego_planner::shm;

class RosBridgeNode {
public:
    RosBridgeNode(ros::NodeHandle& nh) : nh_(nh) {
        // Initialize shared memory (as consumer)
        ROS_INFO("Initializing shared memory...");
        
        // Retry mechanism: wait for planner_standalone to create output shm
        bool init_success = false;
        for (int i = 0; i < 50; ++i) {
            if (shm_.initAsConsumer()) {
                init_success = true;
                break;
            }
            if (i == 0) {
                ROS_WARN("Waiting for planner_standalone to create shared memory...");
                ROS_WARN("Please make sure planner_standalone is running first!");
            }
            ros::Duration(0.1).sleep();
        }
        
        if (!init_success) {
            ROS_ERROR("Failed to initialize shared memory after 5 seconds!");
            ROS_ERROR("Please start planner_standalone first: cd planner_standalone/build && ./ego_planner_standalone");
            ros::shutdown();
            return;
        }
        ROS_INFO("Shared memory initialized successfully");
        
        // Subscribe to ROS topics
        odom_sub_ = nh_.subscribe("/odom_world", 10, &RosBridgeNode::odomCallback, this);
        cloud_sub_ = nh_.subscribe("/grid_map/occupancy_inflate", 10, &RosBridgeNode::cloudCallback, this);
        waypoint_sub_ = nh_.subscribe("/waypoint_generator/waypoints", 1, &RosBridgeNode::waypointCallback, this);
        
        ROS_INFO("  Subscribed: /odom_world");
        ROS_INFO("  Subscribed: /grid_map/occupancy_inflate (inflated obstacle cloud)");
        ROS_INFO("  Subscribed: /waypoint_generator/waypoints");
        
        // Publish ROS topics
        bspline_pub_ = nh_.advertise<ego_planner::Bspline>("/planning/bspline", 10);
        bspline_vis_pub_ = nh_.advertise<nav_msgs::Path>("/planning/bspline_path", 10);
        global_path_pub_ = nh_.advertise<nav_msgs::Path>("/planning/global_path", 10);
        state_pub_ = nh_.advertise<std_msgs::Int32>("/planning/planner_state", 10);
        
        // Timer: read from shared memory and publish
        publish_timer_ = nh_.createTimer(ros::Duration(0.05), &RosBridgeNode::publishTimerCallback, this);
        
        ROS_INFO("ROS Bridge node started successfully");
    }
    
private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        OdomData* odom_data = shm_.getOdom();
        if (!odom_data) return;
        
        // 转换ROS消息到共享内存
        odom_data->stamp.sec = msg->header.stamp.sec;
        odom_data->stamp.nsec = msg->header.stamp.nsec;
        
        odom_data->position.x = msg->pose.pose.position.x;
        odom_data->position.y = msg->pose.pose.position.y;
        odom_data->position.z = msg->pose.pose.position.z;
        
        odom_data->orientation.w = msg->pose.pose.orientation.w;
        odom_data->orientation.x = msg->pose.pose.orientation.x;
        odom_data->orientation.y = msg->pose.pose.orientation.y;
        odom_data->orientation.z = msg->pose.pose.orientation.z;
        
        odom_data->linear_velocity.x = msg->twist.twist.linear.x;
        odom_data->linear_velocity.y = msg->twist.twist.linear.y;
        odom_data->linear_velocity.z = msg->twist.twist.linear.z;
        
        odom_data->angular_velocity.x = msg->twist.twist.angular.x;
        odom_data->angular_velocity.y = msg->twist.twist.angular.y;
        odom_data->angular_velocity.z = msg->twist.twist.angular.z;
        
        odom_data->valid.store(true);
        odom_data->seq.fetch_add(1);
    }
    
    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        PointCloudData* cloud_data = shm_.getPointCloud();
        if (!cloud_data) return;
        
        // 输出点云统计信息（限流）
        static auto last_print_time = ros::Time::now();
        if ((msg->header.stamp - last_print_time).toSec() > 2.0) {
            ROS_INFO("Received point cloud: %d points", msg->width * msg->height);
            last_print_time = msg->header.stamp;
        }
        
        // 简化：假设点云是xyz格式
        int num_points = msg->width * msg->height;
        if (num_points > MAX_POINTCLOUD_SIZE) {
            num_points = MAX_POINTCLOUD_SIZE;
        }
        
        cloud_data->stamp.sec = msg->header.stamp.sec;
        cloud_data->stamp.nsec = msg->header.stamp.nsec;
        cloud_data->num_points = num_points;
        
        // 解析点云数据
        const uint8_t* data_ptr = msg->data.data();
        int point_step = msg->point_step;
        
        for (int i = 0; i < num_points; ++i) {
            const float* xyz = reinterpret_cast<const float*>(data_ptr + i * point_step);
            cloud_data->points[i*3 + 0] = xyz[0];
            cloud_data->points[i*3 + 1] = xyz[1];
            cloud_data->points[i*3 + 2] = xyz[2];
        }
        
        cloud_data->valid.store(true);
        cloud_data->seq.fetch_add(1);
    }
    
    void waypointCallback(const nav_msgs::Path::ConstPtr& msg) {
        if (msg->poses.empty()) return;
        
        WaypointData* waypoint_data = shm_.getWaypoint();
        if (!waypoint_data) return;
        
        waypoint_data->stamp.sec = msg->header.stamp.sec;
        waypoint_data->stamp.nsec = msg->header.stamp.nsec;
        
        int num_waypoints = std::min((int)msg->poses.size(), MAX_WAYPOINTS);
        waypoint_data->num_waypoints = num_waypoints;
        
        for (int i = 0; i < num_waypoints; ++i) {
            waypoint_data->waypoints[i].x = msg->poses[i].pose.position.x;
            waypoint_data->waypoints[i].y = msg->poses[i].pose.position.y;
            waypoint_data->waypoints[i].z = msg->poses[i].pose.position.z;
        }
        
        waypoint_data->new_waypoint.store(true);
        waypoint_data->valid.store(true);
        waypoint_data->seq.fetch_add(1);
        
        ROS_INFO("Received waypoint data: %d waypoints", num_waypoints);
    }
    
    void publishTimerCallback(const ros::TimerEvent&) {
        // 从共享内存读取B样条轨迹并发布
        BsplineData* bspline_data = shm_.getBspline();
        if (!bspline_data || !bspline_data->valid.load()) {
            return;
        }
        
        static uint64_t last_bspline_seq = 0;
        uint64_t current_bspline_seq = bspline_data->seq.load();
        
        // 当有新轨迹时记录
        if (current_bspline_seq != last_bspline_seq) {
            last_bspline_seq = current_bspline_seq;
            ROS_INFO("New bspline trajectory received: %d control points", bspline_data->num_ctrl_pts);
        }
        
        // 持续发布最新的轨迹（用于控制和可视化）
        int num_pts = bspline_data->num_ctrl_pts;
        if (num_pts > 0) {
            // 发布Bspline消息（给traj_server使用）
            ego_planner::Bspline bspline_msg;
            bspline_msg.order = bspline_data->order;
            bspline_msg.traj_id = bspline_data->traj_id;
            // 使用共享内存中的start_time（轨迹开始时刻），不要每次都用now()
            bspline_msg.start_time.sec = bspline_data->start_time.sec;
            bspline_msg.start_time.nsec = bspline_data->start_time.nsec;
            
            // 设置knots (B样条节点向量)
            // 从knots数组复制，或者如果为空则计算dt
            double dt = 0.2; // 默认值
            if (bspline_data->num_knots > 1) {
                // 使用实际的knots
                for (int i = 0; i < bspline_data->num_knots; ++i) {
                    bspline_msg.knots.push_back(bspline_data->knots[i]);
                }
                // 计算平均dt用于yaw_dt
                dt = (bspline_data->knots[bspline_data->num_knots-1] - 
                      bspline_data->knots[0]) / (bspline_data->num_knots - 1);
            } else {
                // 如果knots为空，使用均匀节点向量
                for (int i = 0; i < num_pts + bspline_data->order + 1; ++i) {
                    bspline_msg.knots.push_back(i * dt);
                }
            }
            
            // 设置控制点
            for (int i = 0; i < num_pts; ++i) {
                geometry_msgs::Point pt;
                pt.x = bspline_data->ctrl_pts[i*3 + 0];
                pt.y = bspline_data->ctrl_pts[i*3 + 1];
                pt.z = bspline_data->ctrl_pts[i*3 + 2];
                bspline_msg.pos_pts.push_back(pt);
            }
            
            // yaw设置为0（简化）
            bspline_msg.yaw_dt = dt;
            for (int i = 0; i < num_pts; ++i) {
                bspline_msg.yaw_pts.push_back(0.0);
            }
            
            bspline_pub_.publish(bspline_msg);
            
            // 同时发布Path消息用于可视化
            nav_msgs::Path path_msg;
            path_msg.header.stamp = ros::Time::now();
            path_msg.header.frame_id = "world";
            
            for (int i = 0; i < num_pts; ++i) {
                geometry_msgs::PoseStamped pose;
                pose.header = path_msg.header;
                pose.pose.position.x = bspline_data->ctrl_pts[i*3 + 0];
                pose.pose.position.y = bspline_data->ctrl_pts[i*3 + 1];
                pose.pose.position.z = bspline_data->ctrl_pts[i*3 + 2];
                pose.pose.orientation.w = 1.0;
                path_msg.poses.push_back(pose);
            }
            
            bspline_vis_pub_.publish(path_msg);
        }
        
        // 从共享内存读取全局路径并发布
        GlobalPathData* global_path_data = shm_.getGlobalPath();
        if (global_path_data && global_path_data->valid.load()) {
            static uint64_t last_global_path_seq = 0;
            uint64_t current_global_path_seq = global_path_data->seq.load();
            
            if (current_global_path_seq != last_global_path_seq) {
                last_global_path_seq = current_global_path_seq;
                
                int num_pts = global_path_data->num_points;
                ROS_INFO("New global path received: %d points, length: %.2fm", 
                         num_pts, global_path_data->path_length);
                
                if (num_pts > 0) {
                    nav_msgs::Path global_path_msg;
                    global_path_msg.header.stamp = ros::Time::now();
                    global_path_msg.header.frame_id = "world";
                    
                    for (int i = 0; i < num_pts; ++i) {
                        geometry_msgs::PoseStamped pose;
                        pose.header = global_path_msg.header;
                        pose.pose.position.x = global_path_data->points[i*3 + 0];
                        pose.pose.position.y = global_path_data->points[i*3 + 1];
                        pose.pose.position.z = global_path_data->points[i*3 + 2];
                        pose.pose.orientation.w = 1.0;
                        global_path_msg.poses.push_back(pose);
                    }
                    
                    global_path_pub_.publish(global_path_msg);
                }
            }
        }
        
        // 发布规划器状态
        PlannerStateData* state_data = shm_.getState();
        if (state_data && state_data->valid.load()) {
            static uint64_t last_state_seq = 0;
            uint64_t current_state_seq = state_data->seq.load();
            
            if (current_state_seq != last_state_seq) {
                last_state_seq = current_state_seq;
                
                std_msgs::Int32 state_msg;
                state_msg.data = state_data->state;
                state_pub_.publish(state_msg);
                
                const char* state_names[] = {"IDLE", "PLANNING", "EXECUTING", "ERROR"};
                if (state_data->state >= 0 && state_data->state < 4) {
                    ROS_INFO_THROTTLE(2.0, "Planner state: %s", state_names[state_data->state]);
                }
            }
        }
    }
    
    ros::NodeHandle& nh_;
    ShmInterface shm_;
    
    // 订阅者
    ros::Subscriber odom_sub_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber waypoint_sub_;
    
    // 发布者
    ros::Publisher bspline_pub_;       // 发布给traj_server
    ros::Publisher bspline_vis_pub_;   // 发布给RViz可视化
    ros::Publisher global_path_pub_;   // 发布A*全局路径
    ros::Publisher state_pub_;
    
    // 定时器
    ros::Timer publish_timer_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ego_planner_bridge");
    ros::NodeHandle nh("~");
    
    ROS_INFO("========================================");
    ROS_INFO("  EGO Planner ROS Bridge Node");
    ROS_INFO("========================================");
    
    RosBridgeNode bridge(nh);
    
    ros::spin();
    
    return 0;
}
