#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped
import os

# 设置输出文件路径
output_file_path = "/home/hao/Desktop/twist_ws/src/TwistEstimator/output/detector2.tum"

# 打开文件（追加模式）
f = open(output_file_path, 'a')

def twist_callback(msg):
    # 获取时间戳（单位：秒）
    timestamp = msg.header.stamp.to_sec()

    # 提取线速度和角速度
    lin = msg.twist.twist.linear
    ang = msg.twist.twist.angular

    # 构造 TUM 格式字符串
    line = "{:.9f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(
        timestamp, lin.x, lin.y, lin.z, ang.x, ang.y, ang.z
    )

    # 写入文件
    f.write(line)
    f.flush()  # 立即写入磁盘，避免缓存延迟

def main():
    rospy.init_node('twist_logger_node')
    rospy.Subscriber('radar/twist', TwistWithCovarianceStamped, twist_callback)
    rospy.loginfo("Twist logger started. Writing to: %s", output_file_path)
    rospy.spin()
    f.close()  # 程序退出时关闭文件

if __name__ == '__main__':
    main()

