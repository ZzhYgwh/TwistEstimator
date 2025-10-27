#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped, QuaternionStamped, Vector3
import threading
import numpy as np
import tf.transformations
from collections import deque

def quaternion_to_rotation_matrix(quaternion):
    """将四元数转换为旋转矩阵。"""
    return tf.transformations.quaternion_matrix([quaternion.x, quaternion.y, quaternion.z, quaternion.w])[:3, :3]

def rotate_vector_by_quaternion(vector, quaternion):
    """使用四元数旋转向量。"""
    rotation_matrix = quaternion_to_rotation_matrix(quaternion)
    vector_np = np.array([vector.x, vector.y, vector.z])
    rotated_vector_np = np.dot(rotation_matrix, vector_np)
    return Vector3(*rotated_vector_np)

class PoseDataStore:
    def __init__(self, max_size=1000):
        self.pose_queue = deque()  # 存储位姿数据
        self.twist_queue = deque()  # 存储雷达数据
        self.max_size = max_size
        self.lock = threading.Lock()  # Lock for thread-safe access

    def add_pose(self, timestamp, quaternion):
        """将位姿数据加入队列，队列大小超过最大值时删除最旧的数据。"""
        with self.lock:
            if len(self.pose_queue) >= self.max_size:
                self.pose_queue.popleft()  # 删除最旧的数据
            self.pose_queue.append((timestamp, quaternion))

    def add_twist(self, timestamp, twist_msg):
        """将雷达数据加入队列。"""
        with self.lock:
            if len(self.twist_queue) >= self.max_size:
                self.twist_queue.popleft()  # 删除最旧的数据
            self.twist_queue.append((timestamp, twist_msg))

    def get_closest_pose(self, timestamp):
        """返回与指定时间戳最接近的位姿数据。"""
        with self.lock:
            closest_pose = None
            min_time_diff = float('inf')
            # 查找与指定时间戳最接近的位姿数据
            for pose_timestamp, quaternion in self.pose_queue:
                time_diff = abs(pose_timestamp - timestamp)
                if time_diff < min_time_diff:
                    min_time_diff = time_diff
                    closest_pose = quaternion
            return closest_pose  # 返回最接近的位姿数据

    def get_closest_twist(self, timestamp):
        """返回与指定时间戳最接近的雷达数据。"""
        with self.lock:
            closest_twist = None
            min_time_diff = float('inf')
            # 查找与指定时间戳最接近的雷达数据
            for twist_timestamp, twist_msg in self.twist_queue:
                time_diff = abs(twist_timestamp - timestamp)
                if time_diff < min_time_diff:
                    min_time_diff = time_diff
                    closest_twist = twist_msg
            return closest_twist  # 返回最接近的雷达数据
            
# 创建发布器
pub = rospy.Publisher('/radar/twist_enu', TwistWithCovarianceStamped, queue_size=1000)
def publish_twist_data(timestamp, linear_velocity, angular_velocity):
    """发布 TwistWithCovarianceStamped 消息。"""
    global pub

    # 创建消息
    twist_msg = TwistWithCovarianceStamped()

    # 填充消息头
    twist_msg.header.stamp = rospy.Time.from_sec(timestamp)
    twist_msg.header.frame_id = "enu"  # 根据需要修改坐标系
    
    # 填充线性速度
    twist_msg.twist.twist.linear.x = linear_velocity.x
    twist_msg.twist.twist.linear.y = linear_velocity.y
    twist_msg.twist.twist.linear.z = linear_velocity.z

    # 填充角速度
    twist_msg.twist.twist.angular.x = angular_velocity.x
    twist_msg.twist.twist.angular.y = angular_velocity.y
    twist_msg.twist.twist.angular.z = angular_velocity.z

    # 协方差矩阵（此处填充零矩阵，如果你有实际的协方差可以修改）
    twist_msg.twist.covariance = [0.0] * 36  # 6x6 协方差矩阵填充零

    # 发布消息
    pub.publish(twist_msg)
    rospy.loginfo(f"Published Twist message at timestamp: {timestamp}")

# 定义旋转矩阵 R（pitch -45°）
R = np.array([
    [1.0, 0.0,  -0.0],
    [0.0,    0.7071,  -0.7071],
    [0.0, 0.7071, 0.7071]
])

R_ch = np.array([
    [1.0, 0.0,  -0.0],
    [0.0,    1.0,  0.0],
    [0.0, 0.0, 1.0]
])


def radar_twist_callback(pose_data_store):
    global R
    """处理雷达数据和位姿数据的同步回调函数。"""
    while not rospy.is_shutdown():
        # 获取最新的雷达数据和位姿数据
        if pose_data_store.twist_queue and pose_data_store.pose_queue:
            # 获取雷达数据
            twist_timestamp, twist_msg = pose_data_store.twist_queue[0]

            # 获取与雷达数据时间戳最接近的位姿数据
            attitude_msg = pose_data_store.get_closest_pose(twist_timestamp)

            if attitude_msg:
                # 获取旋转后的速度
                # linear_velocity = twist_msg.twist.twist.linear
                # angular_velocity = twist_msg.twist.twist.angular
                linear_velocity = Vector3()
                angular_velocity = Vector3()

                # 将雷达斜下视转为 大疆机体
                # 从消息中取线速度、角速度
                lin = twist_msg.twist.twist.linear
                ang = twist_msg.twist.twist.angular
                # 转换为 numpy 向量
                v = np.array([lin.x, lin.y, lin.z])
                w = np.array([ang.x, ang.y, ang.z])
                v_rot = R @ R_ch @ v
                w_rot = R @ R_ch @ w

                linear_velocity.x = v_rot[0]
                linear_velocity.y = v_rot[1]
                linear_velocity.z = v_rot[2]

                angular_velocity.x = w_rot[0]
                angular_velocity.y = w_rot[1]
                angular_velocity.z = w_rot[2]


                rotated_linear_velocity = rotate_vector_by_quaternion(linear_velocity, attitude_msg)
                rotated_linear_velocity.z = -1.0 * rotated_linear_velocity.z
                rotated_angular_velocity = angular_velocity

                # 将结果保存为 TUM 格式
                with open('/home/hao/Desktop/twist_ws/src/TwistEstimator/output/estimate.tum', 'a') as f:
                    f.write(f"{twist_timestamp} {rotated_linear_velocity.x} {rotated_linear_velocity.y} {rotated_linear_velocity.z} "
                           f"{rotated_angular_velocity.x} {rotated_angular_velocity.y} {rotated_angular_velocity.z}\n")
                            
                publish_twist_data(twist_timestamp, rotated_linear_velocity, rotated_angular_velocity)
		
                # 移除已经处理过的数据
                with pose_data_store.lock:
                    pose_data_store.twist_queue.popleft()  # 删除已经处理过的雷达数据
            else:
                rospy.logwarn("没有找到与雷达数据时间戳匹配的位姿数据！")

        rospy.sleep(0.1)  # 等待 100 毫秒

def attitude_callback(attitude_msg, pose_data_store):
    """处理 /dji_osdk_ros/attitude 消息的回调函数。"""
    timestamp = attitude_msg.header.stamp.to_sec()
    quaternion = attitude_msg.quaternion
    pose_data_store.add_pose(timestamp, quaternion)  # 将新收到的位姿数据加入存储

def twist_callback(twist_msg, pose_data_store):
    """处理 /radar/twist 消息的回调函数。"""
    timestamp = twist_msg.header.stamp.to_sec()
    pose_data_store.add_twist(timestamp, twist_msg)

def main():
    rospy.init_node('catch_radar_estimator', anonymous=True)

    # 创建 PoseDataStore 实例，用于存储和管理位姿数据
    pose_data_store = PoseDataStore(max_size=100000000)

    # 创建订阅者
    rospy.Subscriber('/radar/twist_esti', TwistWithCovarianceStamped, twist_callback, pose_data_store)
    rospy.Subscriber('/dji_osdk_ros/attitude', QuaternionStamped, attitude_callback, pose_data_store)
    
    

    # 启动一个单独的线程来处理数据同步
    processing_thread = threading.Thread(target=radar_twist_callback, args=(pose_data_store,))
    processing_thread.daemon = True  # 设置为守护线程，程序退出时自动退出
    processing_thread.start()

    rospy.spin()

if __name__ == '__main__':
    main()

