#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from ego_planner.msg import Bspline
from geometry_msgs.msg import Point

def generate_knots(n_ctrl_pts, order, dt):
    """
    生成均匀 B 样条的节点向量 (Knots)
    n_ctrl_pts: 控制点数量
    order: 阶数 (Degree + 1)
    dt: 节点间的时间间隔
    
    对于均匀 B 样条，节点数 = 控制点数 + 阶数
    有效时间范围通常是从第 (order-1) 个节点开始
    """
    n_knots = n_ctrl_pts + order
    # 为了让轨迹从 t=0 开始有效，我们需要将节点向量向前平移
    # 比如 order=3, 有效范围是从 knots[2] 开始
    # 构造 [-2, -1, 0, 1, 2, 3 ...] 这样的序列 * dt
    knots = np.arange(n_knots) - (order - 1)
    knots = knots * dt
    return knots.tolist()

def talker():
    # 1. 初始化节点
    rospy.init_node('test_bspline_sender', anonymous=True)
    
    # 2. 创建发布者
    pub = rospy.Publisher('/planning/bspline', Bspline, queue_size=10)
    
    # 等待连接建立，防止消息丢失
    rospy.sleep(1.0)
    
    msg = Bspline()
    
    # --- 3. 配置 B 样条参数 ---
    
    # 阶数 (Order): 3 (即 Degree=2，二次曲线) 或 4 (三次曲线)
    # Ego-Planner 通常使用 3
    msg.order = 3
    
    # 轨迹 ID (每次发送新轨迹最好增加这个ID)
    msg.traj_id = 1
    
    # 开始时间 (立即开始)
    msg.start_time = rospy.Time.now()

    # --- 4. 定义控制点 (Control Points) ---
    # 这里定义一条向 X 轴正方向飞行的直线
    # 假设当前无人机在 (0,0,1)，我们要它飞到 (5,0,1)
    # 我们设置 6 个控制点，每隔 1 米一个
    control_points_list = [
        [-18.0, 0.0, 0.0],  # 起点 (应该与当前无人机位置一致)
        [-17.0, 0.0, 1.0],
        [-16.0, 0.0, 1.0],
        [-15.0, 0.0, 1.0],
        [-14.0, 0.0, 1.0],
        [-13.0, 0.0, 1.0]   # 终点
    ]
    
    for pt in control_points_list:
        p = Point()
        p.x = pt[0]
        p.y = pt[1]
        p.z = pt[2]
        msg.pos_pts.append(p)

    # --- 5. 生成节点向量 (Knots) ---
    # 时间间隔 dt 决定了飞行的速度。
    # 控制点间距 1米，如果 dt=1.0秒，速度约为 1m/s
    dt = 1.0 
    msg.knots = generate_knots(len(msg.pos_pts), msg.order, dt)

    # --- 6. 偏航角 (Yaw) ---
    # traj_server 代码中似乎注释掉了 yaw_pts 的解析，
    # 但为了兼容性，我们还是给一些默认值
    msg.yaw_dt = dt
    # 给定全 0 的 yaw (朝向正前方)
    msg.yaw_pts = [0.0] * len(msg.pos_pts)

    # --- 7. 发送 ---
    rospy.loginfo("Sending B-spline Trajectory...")
    rospy.loginfo(f"Order: {msg.order}, Points: {len(msg.pos_pts)}")
    rospy.loginfo(f"Knots: {msg.knots}")
    
    pub.publish(msg)
    
    # 保持一小段时间确保发送成功
    rospy.sleep(0.5)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass