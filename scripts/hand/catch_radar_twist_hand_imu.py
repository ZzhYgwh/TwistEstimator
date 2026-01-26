#!/usr/bin/env python3
"""
imu_to_twist.py
订阅 /imu/data (sensor_msgs/Imu)，将线加速度积分为线速度，并把角速度 + 线速度写入 TUM 风格的 twist 文件：
time vx vy vz wx wy wz
"""
import rospy
import numpy as np
from sensor_msgs.msg import Imu
import tf.transformations as tft  # ROS 自带的 quaternion -> rotation matrix

import os

class ImuToTwistLogger:
    def __init__(self):
        # 参数
        topic = rospy.get_param("~topic", "/imu/data")
        # out_file = rospy.get_param("~outfile", "/home/hao/Desktop/twist_ws/src/TwistEstimator/output/imu_twist.tum")
        self.output_path = os.path.join(
            os.path.dirname(__file__),
            "../../output/estimate.twist"
        )
        self.output_path = os.path.abspath(self.output_path)

        self.max_dt = rospy.get_param("~max_dt", 0.1)      # 跳变保护
        self.clip_vel = rospy.get_param("~clip_vel", 50.0) # 速度限幅 (m/s)
        self.gravity = rospy.get_param("~gravity", 9.80665)

        # 状态
        self.prev_time = None
        self.prev_acc_w = np.zeros(3)  # 上一帧的加速度（世界系）
        self.vel = np.zeros(3)         # 当前估计线速度（世界系）

        # 文件
        self.f = open(self.output_path, "w")
        rospy.on_shutdown(self._on_shutdown)

        # 订阅
        rospy.Subscriber(topic, Imu, self.imu_cb, queue_size=200)
        rospy.loginfo("✅ imu_to_twist logger started. topic=%s outfile=%s", topic, self.output_path)

    def imu_cb(self, msg: Imu):
        t = msg.header.stamp.to_sec()
        # 角速度（机体坐标系）直接使用
        omega = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ], dtype=float)

        # 线性加速度（机体坐标系）
        acc_b = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ], dtype=float)

        # 将加速度转换到世界坐标系： acc_w = R(q) * acc_b
        q = msg.orientation
        q_xyzw = [q.x, q.y, q.z, q.w]
        # tf.transformations uses (x,y,z,w)
        R_mat = tft.quaternion_matrix(q_xyzw)[:3, :3]  # 3x3
        acc_w = R_mat.dot(acc_b)

        # 减去重力（世界坐标系假设 z 朝上）
        acc_w[2] -= self.gravity

        # 时间差
        if self.prev_time is None:
            # 初始化
            self.prev_time = t
            self.prev_acc_w = acc_w
            # 初始速度按0写出一条（也可选择不写）
            self._write_line(t, self.vel, omega)
            return

        dt = t - self.prev_time
        if not (0 < dt <= self.max_dt):
            # 非法 dt（太大或负值），跳过积分但更新状态
            rospy.logwarn_once("imu_to_twist: abnormal dt=%.6f (skipped integration)", dt)
            self.prev_time = t
            self.prev_acc_w = acc_w
            # 仍写当前（不更新速度）
            self._write_line(t, self.vel, omega)
            return

        # 梯形规则积分： v += 0.5 * (a_prev + a_curr) * dt
        self.vel += 0.5 * (self.prev_acc_w + acc_w) * dt

        # 限幅防爆
        self.vel = np.clip(self.vel, -self.clip_vel, self.clip_vel)

        # 写入文件（time vx vy vz wx wy wz）
        self._write_line(t, self.vel, omega)

        # 更新缓存
        self.prev_time = t
        self.prev_acc_w = acc_w

    def _write_line(self, t, v, omega):
        line = f"{t:.6f} {v[0]:.6f} {v[1]:.6f} {v[2]:.6f} {omega[0]:.6f} {omega[1]:.6f} {omega[2]:.6f}\n"
        try:
            self.f.write(line)
            # 可按需立即 flush
            # self.f.flush()
        except Exception as e:
            rospy.logerr("Failed to write to imu_twist file: %s", str(e))

    def _on_shutdown(self):
        try:
            self.f.close()
            rospy.loginfo("✅ imu_twist.tum saved and closed.")
        except Exception:
            pass


if __name__ == "__main__":
    rospy.init_node("imu_to_twist_logger", anonymous=True)
    logger = ImuToTwistLogger()
    rospy.spin()

