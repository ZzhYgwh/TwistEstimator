#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

class VRPNTumSaver:
    def __init__(self):
        self.file = open("pose.tum", "w")
        # self.file.write("# timestamp tx ty tz qx qy qz qw\n")
        rospy.Subscriber("/vrpn_client_node/uav1/pose", PoseStamped, self.pose_callback)
        rospy.loginfo("✅ VRPN Pose -> TUM saver started, writing to /tmp/vrpn_pose_tum.txt")

    def pose_callback(self, msg):
        t = msg.header.stamp.to_sec()
        p = msg.pose.position
        q = msg.pose.orientation
        self.file.write(f"{t:.6f} {p.x:.6f} {p.y:.6f} {p.z:.6f} {q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n")

    def __del__(self):
        if hasattr(self, "file"):
            self.file.close()
            rospy.loginfo("✅ TUM pose file saved and closed.")

if __name__ == "__main__":
    rospy.init_node("vrpn_pose_tum_saver", anonymous=True)
    VRPNTumSaver()
    rospy.spin()

