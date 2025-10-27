#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import math

def cloud_callback(msg):
    # 提取 (x, y, z)
    points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    distances = []

    for p in points:
        x, y, z = p[0], p[1], p[2]
        r = math.sqrt(x*x + y*y + z*z)
        distances.append(r)

    if not distances:
        rospy.logwarn("Empty point cloud!")
        return

    distances = np.array(distances)
    rospy.loginfo(f"Frame: {msg.header.frame_id} | Points: {len(distances)}")
    rospy.loginfo(f"Distance range: [{distances.min():.3f}, {distances.max():.3f}] (m)")

def main():
    rospy.init_node("cloud_stats_py", anonymous=True)
    topic = "/pcl2_visualize_2"
    rospy.Subscriber(topic, PointCloud2, cloud_callback)
    rospy.loginfo(f"Listening to {topic} ...")
    rospy.spin()

if __name__ == "__main__":
    main()
