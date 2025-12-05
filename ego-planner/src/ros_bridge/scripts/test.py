#!/usr/bin/env python
'''
测试无人机的控制话题
'''
import rospy
from quadrotor_msgs.msg import PositionCommand
from geometry_msgs.msg import Point, Vector3

def talker():
    # 1. 初始化节点
    rospy.init_node('test_ctrl_publisher', anonymous=True)
    
    # 注意话题名称要与 rostopic info 里的 subscriber 对应
    pub = rospy.Publisher('/planning/pos_cmd', PositionCommand, queue_size=10)
    
    rate = rospy.Rate(50) # 50Hz，控制指令通常需要较高频率

    # 2. 设置目标高度
    target_z = 1.0 
    
    print("Start publishing Hover command at 1.0m...")

    trajectory_id = 0

    while not rospy.is_shutdown():
        cmd = PositionCommand()
        
        # --- 核心字段 ---
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "world"  # 确保这里和你的里程计frame一致
        cmd.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        cmd.trajectory_id = trajectory_id
        trajectory_id += 1

        # 位置: 悬停在 (0, 0, target_z)
        cmd.position.x = -18.0
        cmd.position.y = 0.0
        cmd.position.z = target_z

        # 偏航: 朝向 0 度
        cmd.yaw = 0.0

        # --- 前馈项 (悬停时都为0) ---
        cmd.velocity.x = 0.0
        cmd.velocity.y = 0.0
        cmd.velocity.z = 0.0
        
        cmd.acceleration.x = 0.0
        cmd.acceleration.y = 0.0
        cmd.acceleration.z = 0.0
        
        cmd.yaw_dot = 0.0

        # --- 增益 (通常设为0以使用控制器默认参数) ---
        # 如果控制器不响应，尝试解开下面的注释
        # cmd.kx = [4.0, 4.0, 4.0]
        # cmd.kv = [2.5, 2.5, 2.5]

        pub.publish(cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
