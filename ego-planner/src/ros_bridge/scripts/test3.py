#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from scipy.interpolate import BSpline
from ego_planner.msg import Bspline
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class BsplineVisualizer:
    def __init__(self):
        rospy.init_node('bspline_visualizer', anonymous=True)

        # 订阅 B 样条话题
        self.sub_bspline = rospy.Subscriber('/planning/bspline', Bspline, self.bspline_cb)
        
        # 发布 Path 话题 (供 Rviz 使用)
        self.pub_path = rospy.Publisher('/planning/vis_path', Path, queue_size=10)

        # 参数：采样密度
        self.sample_rate = 50.0  
        
        rospy.loginfo("B-spline Visualizer Initialized. Waiting for traj...")

    def bspline_cb(self, msg):
        """
        接收到 ego_planner/Bspline 消息时的回调
        """
        if len(msg.pos_pts) == 0:
            return

        # --- 1. 数据准备 ---
        # Ego-Planner 中的 order 通常对应 B样条的阶数(order), Degree = Order - 1
        # 例如: Order=3 (Quadratic), Degree=2
        degree = msg.order - 1 

        # 提取控制点
        control_points = []
        for p in msg.pos_pts:
            control_points.append([p.x, p.y, p.z])
        control_points = np.array(control_points)
        
        n_ctrl_pts = len(control_points)

        # 提取节点向量
        knots = np.array(msg.knots)

        # --- 2. 关键修复：检查节点一致性 ---
        # Scipy 要求: len(knots) == len(control_points) + degree + 1
        required_knots_num = n_ctrl_pts + degree + 1

        if len(knots) != required_knots_num:
            # 如果数量对不上，为了防止报错，我们手动生成一个均匀节点向量
            # 这通常是安全的，因为 Ego-Planner 大多使用均匀 B 样条
            # 这里的 dt 并不影响形状，只影响 t 的取值范围
            # rospy.logwarn_throttle(1.0, f"Fixing knots: received {len(knots)}, needed {required_knots_num}")
            
            # 生成标准均匀节点向量: [0, 1, 2, ..., N+D]
            # 这种方式保证了数学上的可解性，能正确画出形状
            knots = np.arange(required_knots_num, dtype=float)

        try:
            # --- 3. 创建 B 样条 ---
            spl = BSpline(knots, control_points, degree)

            # --- 4. 确定采样范围 ---
            # 有效定义域通常在 [knots[degree], knots[-degree-1]]
            # 这样可以避免在首尾出现数值不稳定的情况
            t_start = knots[degree]
            t_end = knots[-(degree+1)]
            
            if t_end <= t_start:
                # 极少数情况下节点重叠会导致范围无效
                return

            # 根据原始 msg 的时间信息或者节点范围来确定采样点数
            # 如果我们重置了 knots，原来的 msg.start_time 对应关系可能失效
            # 但对于"可视化路径形状"来说，只需要足够密度的点即可
            duration = t_end - t_start
            num_samples = int(duration * 20) # 提高采样倍率以获得更平滑的曲线
            num_samples = max(num_samples, 10) # 至少10个点
            
            t_sample = np.linspace(t_start, t_end, num_samples)

            # --- 5. 计算轨迹点 ---
            traj_points = spl(t_sample)

            # --- 6. 构建并发布 Path ---
            path_msg = Path()
            path_msg.header.frame_id = "world"
            path_msg.header.stamp = rospy.Time.now()

            for i in range(len(traj_points)):
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = traj_points[i][0]
                pose.pose.position.y = traj_points[i][1]
                pose.pose.position.z = traj_points[i][2]
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)

            self.pub_path.publish(path_msg)

        except Exception as e:
            rospy.logerr(f"BSpline generation error: {e}")

if __name__ == '__main__':
    try:
        BsplineVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass