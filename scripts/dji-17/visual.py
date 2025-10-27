#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

def integrate_motion(v, w, dt, steps):
    """
    简单积分模型：从当前速度预测未来轨迹
    v, w: 当前线速度与角速度 (3D)
    dt: 每步时间间隔
    steps: 步数
    """
    poses = []
    pos = np.zeros(3)
    R = np.eye(3)
    for i in range(steps):
        # 角速度反对称矩阵
        wx, wy, wz = w
        w_hat = np.array([[0, -wz, wy],
                          [wz, 0, -wx],
                          [-wy, wx, 0]])
        R = R @ (np.eye(3) + w_hat * dt)  # 一阶积分
        pos += R @ (v * dt)
        poses.append((pos.copy(), R.copy()))
    return poses

def rotation_to_quaternion(R):
    """旋转矩阵转四元数"""
    qw = np.sqrt(1 + np.trace(R)) / 2
    qx = (R[2,1] - R[1,2]) / (4*qw)
    qy = (R[0,2] - R[2,0]) / (4*qw)
    qz = (R[1,0] - R[0,1]) / (4*qw)
    return qx, qy, qz, qw

class TwistPredictor:
    def __init__(self):
        rospy.init_node("predict_traj_from_radar_twist", anonymous=True)
        rospy.Subscriber("/radar/twist", TwistWithCovarianceStamped, self.twist_callback)

        self.path_pub = rospy.Publisher("/predicted_path", Path, queue_size=1)
        self.marker_pub = rospy.Publisher("/predicted_path_color", Marker, queue_size=1)

        self.dt = 0.02  # 50Hz
        self.pred_time = 1.0  # 预测1秒
        rospy.loginfo("✅ Ready to predict trajectory from /radar/twist ...")

    def twist_callback(self, msg):
        # 从 Twist 消息提取速度
        v = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])
        w = np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z
        ])

        # 生成轨迹点
        steps = int(self.pred_time / self.dt)
        poses = integrate_motion(v, w, self.dt, steps)

        # 构建 Path 消息
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        speed = np.linalg.norm(v)
        colors = []

        for pos, R in poses:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = pos[0]
            pose.pose.position.y = pos[1]
            pose.pose.position.z = pos[2]
            qx, qy, qz, qw = rotation_to_quaternion(R)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path_msg.poses.append(pose)

            # 颜色映射（速度强度）
            intensity = min(speed / 3.0, 1.0)
            color = ColorRGBA(intensity, 0.2, 1 - intensity, 1.0)
            colors.append(color)

        # 发布 Path
        self.path_pub.publish(path_msg)

        # 发布彩色 Marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "predicted_traj"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # 线宽
        marker.pose.orientation.w = 1.0
        marker.points = [p.pose.position for p in path_msg.poses]
        marker.colors = colors
        marker.lifetime = rospy.Duration(0.1)  # 每次新Twist覆盖旧轨迹
        self.marker_pub.publish(marker)

if __name__ == "__main__":
    predictor = TwistPredictor()
    rospy.spin()

