#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistWithCovarianceStamped, Vector3, Quaternion, QuaternionStamped, PointStamped, Vector3Stamped
from sensor_msgs.msg import Imu
from collections import deque
import threading
import time
import tf
import tf.transformations as tft
import numpy as np

# 全局变量存储数据队列
pose_buffer = deque()  # 存储pose数据
twist_buffer = deque()  # 存储twist数据
attitude_queue = deque(maxlen=10)  # 存储四元数数据队列，最多存储10个数据
position_queue = deque(maxlen=10)  # 存储位置数据队列，最多存储10个数据
velocity_queue = deque(maxlen=10)  # 存储速度数据队列，最多存储10个数据
imu_queue = deque(maxlen=10)  # 存储IMU数据队列，最多存储10个数据

# 发布器
pose_pub = rospy.Publisher('/dji/pose_stamped', PoseStamped, queue_size=10)
twist_pub = rospy.Publisher('/dji/twist_with_covariance', TwistWithCovarianceStamped, queue_size=10)

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
    min_diff = float('inf')
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

def quaternion_to_numpy(q: Quaternion):
    return [q.x, q.y, q.z, q.w]

def vector3_to_numpy(v):
    return np.array([v.x, v.y, v.z])

def transform_velocity_to_body_frame(q_enu_to_body, velocity_msg):
    # 将 velocity 从 geometry_msgs 转成 numpy 向量
    v_enu = vector3_to_numpy(velocity_msg.vector)

    # 构造成四元数形式 (向量 + 0 标志位)
    v_enu_quat = [v_enu[0], v_enu[1], v_enu[2], 0.0]

    # 旋转到 body frame: v_body = q * v * q⁻¹
    v_body_quat = tf.transformations.quaternion_multiply(
        tf.transformations.quaternion_multiply(q_enu_to_body, v_enu_quat),
        tf.transformations.quaternion_conjugate(q_enu_to_body)
    )

    # 提取前三项作为向量部分
    v_body = np.array(v_body_quat[:3])
    return v_body

def transform_velocity_to_body_frame2(q_enu_to_body, v_enu):
    # 构造成四元数形式 (向量 + 0 标志位)
    v_enu_quat = [v_enu[0], v_enu[1], v_enu[2], 0.0]

    # 旋转到 body frame: v_body = q * v * q⁻¹
    v_body_quat = tf.transformations.quaternion_multiply(
        tf.transformations.quaternion_multiply(q_enu_to_body, v_enu_quat),
        tf.transformations.quaternion_conjugate(q_enu_to_body)
    )

    # 提取前三项作为向量部分
    v_body = np.array(v_body_quat[:3])
    return v_body


def transform_velocity_to_relative_body_frame(quaternion1, quaternion2, velocity_flu):
    """
    将 ENU 系下的速度向量投影到 body2 系下，并相对于 body1 做变换。

    参数：
        quaternion1: t1 时刻的四元数（geometry_msgs/Quaternion） — body1 -> ENU
        quaternion2: t2 时刻的四元数（geometry_msgs/Quaternion） — body2 -> ENU
        velocity_enu: np.array([vx, vy, vz]) — ENU 坐标系下的速度

    返回：
        velocity_body_rel: np.array([vx, vy, vz]) — body2 坐标系下表示的速度（相对 body1）
    """
    import numpy as np
    import tf.transformations as tf_trans

    # 将 geometry_msgs/Quaternion 转换为 numpy 格式
    q1 = quaternion_to_numpy(quaternion1)  # body1 -> ENU
    q2 = quaternion_to_numpy(quaternion2)  # body2 -> ENU

    # 计算 body1 -> body2 的旋转
    q_rel = tf_trans.quaternion_multiply(
        tf_trans.quaternion_inverse(q2),  # ENU -> body2
        q1                                # body1 -> ENU
    )

    # print("q_rel = ", q_rel)

    # 将 ENU 中的速度向量构造成四维纯向量四元数
    v_quat = np.array([velocity_flu[0], velocity_flu[1], velocity_flu[2], 0.0])

    # 使用四元数旋转：v_body = q * v * q^-1
    v_body_quat = tf_trans.quaternion_multiply(
        tf_trans.quaternion_multiply(q_rel, v_quat),
        tf_trans.quaternion_conjugate(q_rel)
    )

    return np.array(v_body_quat[:3])


# 同步线程：查询并组合数据
base0_quaternion = None
def sync_thread():
    global base0_quaternion
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

            # # quaternion = attitude_msg.quaternion
            # quaternion_ros = attitude_msg.quaternion
            # # quaternion = tft.quaternion_inverse(quaternion)  # 返回的仍是 [x, y, z, w]

            # q_np = np.array([quaternion_ros.x, quaternion_ros.y, quaternion_ros.z, quaternion_ros.w])  # 转为 numpy array

            # quaternion = tft.quaternion_inverse(q_np)  # 现在就不会报错了


            quaternion_ros_stamped = attitude_msg  # 这是 QuaternionStamped 类型
            q_raw = quaternion_ros_stamped.quaternion  # 提取出其中的 geometry_msgs/Quaternion  flu2enu

            q_np = np.array([q_raw.x, q_raw.y, q_raw.z, q_raw.w])  # flu2enu
            q_inv = tf.transformations.quaternion_inverse(q_np)  # enu2flu

            print("q_np = ", q_np)
            print("q_inv = ", q_inv)

            if base0_quaternion is None:        #  初始姿态
                base0_quaternion = q_inv
                print("base0_quaternion = ", base0_quaternion)

            # q_d_0 = q_raw * base0_quaternion.inv() #  flu2enu * enu2flu 去掉初始姿态的旋转
            # base0_quat_inv = tf.transformations.quaternion_inverse(base0_quaternion)
            q_d_0 = tf.transformations.quaternion_multiply(q_np, base0_quaternion)  # flu2enu_ti * enu2flu0 = flu_ti 2 flu0 

            print("q_d_0 = ", q_d_0)

            position = position_msg.point
            # velocity = velocity_msg.vector
            # velocity = np.array([velocity_msg.vector.x, velocity_msg.vector.y, velocity_msg.vector.z])
            velocity = transform_velocity_to_body_frame(q_inv, velocity_msg)  # flu velocity  flu_ti
            velocity = transform_velocity_to_body_frame2(q_d_0, velocity)       # 去掉初始姿态的旋转 

            # print("velocity = [",velocity[0], ", ",velocity[1], ", ",velocity[2], "]")
            # if base0_quaternion is not None:
            #     velocity = transform_velocity_to_relative_body_frame(base0_quaternion, quaternion, velocity)
            # print("velocity2 = [",velocity[0], ", ",velocity[1], ", ",velocity[2], "]")
            angular_velocity = imu_msg.angular_velocity

            base0_quaternion = q_inv

            # 创建PoseStamped消息并发布
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header.stamp = rospy.Time.from_sec(current_time)
            pose_stamped_msg.header.frame_id = "enu"  # 根据需要修改坐标系
            pose_stamped_msg.pose.position.x = position.x
            pose_stamped_msg.pose.position.y = position.y
            pose_stamped_msg.pose.position.z = position.z
            pose_stamped_msg.pose.orientation = q_raw  # 设置四元数方向

            pose_pub.publish(pose_stamped_msg)  # 发布PoseStamped消息

            # 创建TwistWithCovarianceStamped消息并发布
            twist_with_covariance_msg = TwistWithCovarianceStamped()
            twist_with_covariance_msg.header.stamp = rospy.Time.from_sec(current_time)
            # twist_with_covariance_msg.header.frame_id = "enu"  # 根据需要修改坐标系
            # twist_with_covariance_msg.twist.twist.linear.x = velocity.x
            # twist_with_covariance_msg.twist.twist.linear.y = velocity.y
            # twist_with_covariance_msg.twist.twist.linear.z = velocity.z
            twist_with_covariance_msg.header.frame_id = "radar"  # 根据需要修改坐标系
            twist_with_covariance_msg.twist.twist.linear.x = velocity[0]
            twist_with_covariance_msg.twist.twist.linear.y = velocity[1]
            twist_with_covariance_msg.twist.twist.linear.z = velocity[2]
            twist_with_covariance_msg.twist.twist.angular.x = angular_velocity.x
            twist_with_covariance_msg.twist.twist.angular.y = angular_velocity.y
            twist_with_covariance_msg.twist.twist.angular.z = angular_velocity.z

            # 协方差矩阵可以先填充零矩阵
            twist_with_covariance_msg.twist.covariance = [0.0] * 36  # 6x6 协方差矩阵填充零

            twist_pub.publish(twist_with_covariance_msg)  # 发布TwistWithCovarianceStamped消息

            # 将结果保存为 TUM 格式
            with open('/home/hao/Desktop/radar-event-new/src/TwistEstimator/output/twist.tum', 'a') as f:
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

