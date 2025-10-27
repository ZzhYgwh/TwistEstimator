#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TwistStamped

class RadarTwistLogger:
    def __init__(self):
        # 输出文件路径 
        self.output_path = "twist.tum" # /home/hao/Desktop/twist_ws/src/TwistEstimator/output/twist.tum
        self.file = open(self.output_path, "w")
        # self.file.write("# timestamp vx vy vz wx wy wz\n")

        # 订阅 radar twist
        rospy.Subscriber("/vrpn_client_node/uav1/pose/velEKF", TwistStamped, self.twist_callback, queue_size=100)
        rospy.loginfo(f"✅ Radar Twist Logger started, writing to {self.output_path}")

    def twist_callback(self, msg):
        t = msg.header.stamp.to_sec()
        lin = msg.twist.linear
        ang = msg.twist.angular

        # 写入一行：时间 线速度 角速度
        self.file.write(f"{t:.6f} {lin.x:.6f} {lin.y:.6f} {lin.z:.6f} {ang.x:.6f} {ang.y:.6f} {ang.z:.6f}\n")

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

