#!/usr/bin/env python3
import rospy
import math
import numpy as np
from sensor_msgs.msg import Imu, NavSatFix
from tf_conversions import transformations as tf_trans

# 坐标转换工具：WGS84->ENU近似
def latlon_to_enu(lat, lon, alt, ref_lat, ref_lon, ref_alt):
    R = 6378137.0  # 地球半径
    dlat = math.radians(lat - ref_lat)
    dlon = math.radians(lon - ref_lon)
    dx = R * dlon * math.cos(math.radians(ref_lat))
    dy = R * dlat
    dz = alt - ref_alt
    return np.array([dx, dy, dz])

class GPSVelocityRecorder:
    def __init__(self, output_file="twist.tum"):
        rospy.init_node("gps_velocity_recorder", anonymous=True)
        self.output_file = output_file
        self.file = open(self.output_file, "w")

        self.prev_pos = None
        self.prev_time = None
        self.ref_gps = None

        self.latest_q = None
        self.latest_imu_time = None

        rospy.Subscriber("/rtk/gps", NavSatFix, self.gps_callback)
        rospy.Subscriber("/rtk/imu", Imu, self.imu_callback)
        rospy.spin()

    def imu_callback(self, msg: Imu):
        self.latest_q = [msg.orientation.x,
                         msg.orientation.y,
                         msg.orientation.z,
                         msg.orientation.w]
        self.latest_imu_time = msg.header.stamp.to_sec()
        self.latest_omega = np.array([msg.angular_velocity.x,
                                      msg.angular_velocity.y,
                                      msg.angular_velocity.z])

    def gps_callback(self, msg: NavSatFix):
        t = msg.header.stamp.to_sec()
        lat = msg.latitude
        lon = msg.longitude
        alt = msg.altitude

        if self.ref_gps is None:
            self.ref_gps = (lat, lon, alt)
            self.prev_pos = np.array([0.0, 0.0, 0.0])
            self.prev_time = t
            return

        # 转 ENU
        curr_pos = latlon_to_enu(lat, lon, alt, *self.ref_gps)

        dt = t - self.prev_time
        if dt <= 0.0:
            return

        # GPS差分速度（世界坐标系ENU）
        vel_enu = (curr_pos - self.prev_pos) / dt

        # 使用最近IMU姿态投影到机体坐标系
        if self.latest_q is not None:
            R_w2b = tf_trans.quaternion_matrix(self.latest_q)[:3, :3].T
            vel_body = R_w2b @ vel_enu
        else:
            vel_body = vel_enu

        # 使用IMU时间戳写入文件
        if hasattr(self, "latest_imu_time") and hasattr(self, "latest_omega"):
            timestamp = self.latest_imu_time
            vx, vy, vz = vel_body
            wx, wy, wz = self.latest_omega
            self.file.write(f"{timestamp:.6f} {vx:.6f} {vy:.6f} {vz:.6f} {wx:.6f} {wy:.6f} {wz:.6f}\n")

        # 更新上一帧
        self.prev_pos = curr_pos
        self.prev_time = t

    def __del__(self):
        if hasattr(self, "file") and self.file:
            self.file.close()
            rospy.loginfo(f"✅ twist.tum saved to {self.output_file}")

if __name__ == "__main__":
    GPSVelocityRecorder("twist.tum")
