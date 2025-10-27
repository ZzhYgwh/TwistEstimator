#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3
import numpy as np

class RadarTwistLogger:
    def __init__(self):
        # 输出文件路径
        self.output_path = "/home/hao/Desktop/twist_ws/src/TwistEstimator/output/detector2.tum"
        self.file = open(self.output_path, "w")
        # self.file.write("# timestamp vx vy vz wx wy wz\n")

        # 订阅 radar twist
        rospy.Subscriber("/radar/twist", TwistWithCovarianceStamped, self.twist_callback, queue_size=100)
        rospy.loginfo(f"✅ Radar Twist Logger started, writing to {self.output_path}")

        self.R = np.array([
            [1.0, 0.0,  -0.0],
            [0.0,    0.7071,  -0.7071],
            [0.0, 0.7071, 0.7071]
        ])

        self.R_ch = np.array([
            [-1.0, 0.0,  -0.0],
            [0.0,    -1.0,  0.0],
            [0.0, 0.0, -1.0]
        ])

        self.linear_velocity = Vector3()
        self.angular_velocity = Vector3()

    def twist_callback(self, msg):
        t = msg.header.stamp.to_sec()
        lin = msg.twist.twist.linear
        ang = msg.twist.twist.angular


        # 将雷达斜下视转为 大疆机体
        # 转换为 numpy 向量
        v = np.array([lin.x, lin.y, lin.z])
        w = np.array([ang.x, ang.y, ang.z])
        v_rot = self.R @ self.R_ch @ v
        w_rot = self.R @ self.R_ch @ w

        self.linear_velocity.x = v_rot[0]
        self.linear_velocity.y = v_rot[1]
        self.linear_velocity.z = v_rot[2]

        self.angular_velocity.x = w_rot[0]
        self.angular_velocity.y = w_rot[1]
        self.angular_velocity.z = w_rot[2]

        # 写入一行：时间 线速度 角速度
        self.file.write(f"{t:.6f} {self.linear_velocity.x:.6f} {self.linear_velocity.y:.6f} {self.linear_velocity.z:.6f} {self.angular_velocity.x:.6f} {self.angular_velocity.y:.6f} {self.angular_velocity.z:.6f}\n")

    def __del__(self):
        try:
            self.file.close()
            rospy.loginfo("✅ Radar twist log saved and closed.")
        except Exception:
            pass

if __name__ == "__main__":
    rospy.init_node("radar_twist_logger", anonymous=True)
    RadarTwistLogger()
    rospy.spin()

