#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TwistStamped
import os


class RTKVelSaver:
    def __init__(self):
        # ===== resolve relative path: ../../output/ =====
        # script_dir = os.path.dirname(os.path.abspath(__file__))
        # default_output_dir = os.path.abspath(
        #     os.path.join(script_dir, "../../output")
        # )
        # default_output_path = os.path.join(default_output_dir, "gt.twist")

        # self.output_path = rospy.get_param("~output", default_output_path)

        self.output_path = os.path.join(
            os.path.dirname(__file__),
            "../../output/gt.twist"
        )
        self.output_path = os.path.abspath(self.output_path)

        self.use_header_stamp = rospy.get_param("~use_header_stamp", True)

        # create directory if not exists
        os.makedirs(os.path.dirname(self.output_path), exist_ok=True)

        self.file = open(self.output_path, "w")
        rospy.loginfo(f"[RTKVelSaver] Saving /vrpn_client_node/uav1/vel to: {self.output_path}")

        # subscriber
        self.sub = rospy.Subscriber(
            "/vrpn_client_node/uav1/vel",
            TwistStamped,
            self.callback,
            queue_size=1000
        )

    def callback(self, msg: TwistStamped):
        # timestamp
        if self.use_header_stamp:
            t = msg.header.stamp.to_sec()
        else:
            t = rospy.Time.now().to_sec()

        v = msg.twist.linear
        w = msg.twist.angular

        # write line
        line = (
            f"{t:.9f} "
            f"{v.x:.6f} {v.y:.6f} {v.z:.6f} "
            f"{w.x:.6f} {w.y:.6f} {w.z:.6f}\n"
        )
        self.file.write(line)

    def shutdown(self):
        rospy.loginfo("[RTKVelSaver] Closing file.")
        self.file.close()


if __name__ == "__main__":
    rospy.init_node("rtk_vel_saver", anonymous=False)
    saver = RTKVelSaver()
    rospy.on_shutdown(saver.shutdown)
    rospy.spin()
