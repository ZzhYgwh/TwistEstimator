#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped
import os


OUTPUT_PATH = "/home/hao/Desktop/twist_ws/src/TwistEstimator/output/estimate.twist"


class RadarTwistSaver:
    def __init__(self):
        # ensure output directory exists
        os.makedirs(os.path.dirname(OUTPUT_PATH), exist_ok=True)

        self.file = open(OUTPUT_PATH, "a")
        rospy.loginfo(f"[RadarTwistSaver] Writing to {OUTPUT_PATH}")

        self.sub = rospy.Subscriber(
            "/radar/twist_esti",
            TwistWithCovarianceStamped,
            self.callback,
            queue_size=1000
        )

    def callback(self, msg: TwistWithCovarianceStamped):
        t = msg.header.stamp.to_sec()

        lin = msg.twist.twist.linear
        ang = msg.twist.twist.angular

        line = (
            f"{t:.9f} "
            f"{lin.x:.6f} {lin.y:.6f} {lin.z:.6f} "
            f"{ang.x:.6f} {ang.y:.6f} {ang.z:.6f}\n"
        )

        self.file.write(line)

    def shutdown(self):
        rospy.loginfo("[RadarTwistSaver] Closing file.")
        self.file.close()


def main():
    rospy.init_node("radar_twist_to_tum", anonymous=False)
    saver = RadarTwistSaver()
    rospy.on_shutdown(saver.shutdown)
    rospy.spin()


if __name__ == "__main__":
    main()
