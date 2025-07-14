#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import queue
import threading

class PointCloudSync:
    def __init__(self):
        self.pc_queue = queue.Queue()
        self.trigger_queue = queue.Queue()

        self.pc_sub = rospy.Subscriber('/pcl2_visualize_2', PointCloud2, self.pc_callback)
        self.trigger_sub = rospy.Subscriber('/radar/trigger', Header, self.trigger_callback)
        
        self.pc_pub = rospy.Publisher('/radar/data', PointCloud2, queue_size=10)

        self.sync_thread = threading.Thread(target=self.sync_data)
        self.sync_thread.start()

    def pc_callback(self, msg):
        msg.fields[4].name = 'doppler'
        self.pc_queue.put(msg)

    def trigger_callback(self, msg):
        self.trigger_queue.put(msg)

    def sync_data(self):
        while not rospy.is_shutdown():
            if not self.pc_queue.empty() and not self.trigger_queue.empty():
                pc_msg = self.pc_queue.get()
                trigger_msg = self.trigger_queue.get()

                new_pc_msg = pc_msg
                new_pc_msg.header = trigger_msg
                self.pc_pub.publish(new_pc_msg)
            rospy.sleep(0.01)

if __name__ == '__main__':
    rospy.init_node('combi_radar', anonymous=True)
    sync = PointCloudSync()
    rospy.spin()

