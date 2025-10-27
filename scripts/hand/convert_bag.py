#!/usr/bin/env python3
import rospy
import rosbag
from geometry_msgs.msg import PoseStamped
import numpy as np

# 输入 TUM 文件
input_file = "pose_zeroed.tum"
# 输出 bag 文件
output_bag = "pose_zeroed.bag"
topic_name = "/vrpn_client_node/uav1/pose"

# 读取 TUM
data = []
with open(input_file, "r") as f:
    for line in f:
        if line.startswith("#") or line.strip() == "":
            continue
        parts = line.strip().split()
        data.append([float(p) for p in parts])
data = np.array(data)
timestamps = data[:,0]
positions = data[:,1:4]
quaternions = data[:,4:8]  # qx, qy, qz, qw

# 创建 ROS bag
bag = rosbag.Bag(output_bag, 'w')
try:
    for i in range(len(data)):
        pose_msg = PoseStamped()
        # ROS 时间
        sec = int(timestamps[i])
        nsec = int((timestamps[i] - sec) * 1e9)
        pose_msg.header.stamp = rospy.Time(sec, nsec)
        pose_msg.header.frame_id = "world"  # 可根据需要修改
        
        # 位置
        pose_msg.pose.position.x = positions[i,0]
        pose_msg.pose.position.y = positions[i,1]
        pose_msg.pose.position.z = positions[i,2]
        # 四元数
        pose_msg.pose.orientation.x = quaternions[i,0]
        pose_msg.pose.orientation.y = quaternions[i,1]
        pose_msg.pose.orientation.z = quaternions[i,2]
        pose_msg.pose.orientation.w = quaternions[i,3]

        bag.write(topic_name, pose_msg, t=pose_msg.header.stamp)
finally:
    bag.close()

print(f"ROS bag 已生成: {output_bag}")

