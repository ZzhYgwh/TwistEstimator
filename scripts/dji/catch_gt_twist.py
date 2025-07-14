#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistWithCovarianceStamped, Vector3, Quaternion, QuaternionStamped, PointStamped, Vector3Stamped
from sensor_msgs.msg import Imu
from collections import deque
import threading
import time

# 全局变量存储数据队列
pose_buffer = deque()  # 存储pose数据
twist_buffer = deque()  # 存储twist数据
attitude_queue = deque(maxlen=10)  # 存储四元数数据队列，最多存储10个数据
position_queue = deque(maxlen=10)  # 存储位置数据队列，最多存储10个数据
velocity_queue = deque(maxlen=10)  # 存储速度数据队列，最多存储10个数据
imu_queue = deque(maxlen=10)  # 存储IMU数据队列，最多存储10个数据

# 发布器
pose_pub = rospy.Publisher('/dji/pose_stamped', PoseStamped, queue_size=1000)
twist_pub = rospy.Publisher('/dji/twist_with_covariance', TwistWithCovarianceStamped, queue_size=1000)

# 回调函数：接收数据并存储到队列
def attitude_callback(msg):
    attitude_queue.append(msg)  # 将四元数数据存入队列

def position_callback(msg):
    position_queue.append(msg)  # 将位置数据存入队列

def velocity_callback(msg):
    velocity_queue.append(msg)  # 将速度数据存入队列

def imu_callback(msg):
    imu_queue.append(msg)  # 将IMU数据存入队列

# 查找时间戳最接近的消息
def find_closest_data(timestamp, data_queue):
    closest_data = None
    # min_diff = float('inf')
    min_diff = 3.0
    for data in data_queue:
        time_diff = abs(data.header.stamp.to_sec() - timestamp)
        if time_diff < min_diff:
            min_diff = time_diff
            closest_data = data
    return closest_data

# 删除时间戳小于 (current_time - 1.0) 的数据
def remove_old_data(current_time, data_queue):
    while len(data_queue) > 0 and data_queue[0].header.stamp.to_sec() < current_time - 1.0:
        data_queue.popleft()  # 删除最早的数据

# 同步线程：查询并组合数据
def sync_thread():
    while not rospy.is_shutdown():
        if len(attitude_queue) > 0 and len(position_queue) > 0 and len(velocity_queue) > 0 and len(imu_queue) > 0:
            position_msg = position_queue[0]
            current_time = position_msg.header.stamp.to_sec()

            # 在队列中寻找距离当前时间戳最近的数据
            attitude_msg = find_closest_data(current_time, attitude_queue)
            velocity_msg = find_closest_data(current_time, velocity_queue)
            imu_msg = find_closest_data(current_time, imu_queue)

            # 删除早于 current_time - 1.0 秒的数据
            remove_old_data(current_time, attitude_queue)
            remove_old_data(current_time, position_queue)
            if len(position_queue) > 1:
                position_queue.popleft()
            remove_old_data(current_time, velocity_queue)
            remove_old_data(current_time, imu_queue)

            if not all([attitude_msg, position_msg, velocity_msg, imu_msg]):
                position_queue.popleft()
                continue  # 如果没有找到对应的数据，则跳过

            quaternion = attitude_msg.quaternion
            position = position_msg.point
            velocity = velocity_msg.vector
            angular_velocity = imu_msg.angular_velocity

            # 创建PoseStamped消息并发布
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.stamp = rospy.Time.from_sec(current_time)
            pose_stamped_msg.header.frame_id = "enu"  # 根据需要修改坐标系
            pose_stamped_msg.pose.position.x = position.x
            pose_stamped_msg.pose.position.y = position.y
            pose_stamped_msg.pose.position.z = position.z
            pose_stamped_msg.pose.orientation = quaternion  # 设置四元数方向

            pose_pub.publish(pose_stamped_msg)  # 发布PoseStamped消息

            # 创建TwistWithCovarianceStamped消息并发布
            twist_with_covariance_msg = TwistWithCovarianceStamped()
            twist_with_covariance_msg.header.stamp = rospy.Time.from_sec(current_time)
            twist_with_covariance_msg.header.frame_id = "enu"  # 根据需要修改坐标系
            twist_with_covariance_msg.twist.twist.linear.x = velocity.x
            twist_with_covariance_msg.twist.twist.linear.y = velocity.y
            twist_with_covariance_msg.twist.twist.linear.z = velocity.z
            twist_with_covariance_msg.twist.twist.angular.x = angular_velocity.x
            twist_with_covariance_msg.twist.twist.angular.y = angular_velocity.y
            twist_with_covariance_msg.twist.twist.angular.z = angular_velocity.z

            # 协方差矩阵可以先填充零矩阵
            twist_with_covariance_msg.twist.covariance = [0.0] * 36  # 6x6 协方差矩阵填充零

            twist_pub.publish(twist_with_covariance_msg)  # 发布TwistWithCovarianceStamped消息

            # 将结果保存为 TUM 格式
            with open('/home/hao/Desktop/twist_ws/src/TwistEstimator/output/twist.tum', 'a') as f:
                f.write(f"{current_time} {twist_with_covariance_msg.twist.twist.linear.x} {twist_with_covariance_msg.twist.twist.linear.y} {twist_with_covariance_msg.twist.twist.linear.z} "
                        f"{twist_with_covariance_msg.twist.twist.angular.x} {twist_with_covariance_msg.twist.twist.angular.y} {twist_with_covariance_msg.twist.twist.angular.z}\n")

            # 同步处理并移除已同步数据
            if len(pose_buffer) > 1:
                pose_buffer.popleft()
            if len(twist_buffer) > 1:
                twist_buffer.popleft()

        time.sleep(0.1)  # 等待一段时间再进行下一次同步检查

# 启动 ROS 节点并订阅数据
def start_ros_node():
    rospy.init_node('sensor_data_sync')

    # 订阅话题
    rospy.Subscriber('/dji_osdk_ros/attitude', QuaternionStamped, attitude_callback)
    rospy.Subscriber('/dji_osdk_ros/local_position', PointStamped, position_callback)
    rospy.Subscriber('/dji_osdk_ros/rtk_velocity', Vector3Stamped, velocity_callback)
    rospy.Subscriber('/dji_osdk_ros/imu', Imu, imu_callback)

    # 启动 ROS 循环
    rospy.spin()

if __name__ == "__main__":
    # 启动同步线程
    sync_thread = threading.Thread(target=sync_thread)
    sync_thread.daemon = True  # 设置为守护线程，程序退出时自动退出
    sync_thread.start()

    # 启动 ROS 节点
    start_ros_node()

