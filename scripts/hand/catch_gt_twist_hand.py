#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation as R
from collections import deque

# ================================================================
# 稳健角速度估计器（符号修正 + 中心差分 + 中值滤波 + 限幅）
# ================================================================
class RobustAngularVelEstimator:
    def __init__(self, max_angle=np.pi * 0.9, med_window=5):
        self.buf = deque(maxlen=3)  # 存 (t, quat)
        self.max_angle = max_angle
        self.omega_hist = deque(maxlen=med_window)
        self.last_output_time = None
        self.output_period = 0.02  # 50Hz 输出

    def push(self, t, q):
        q = np.array(q, dtype=float)
        q = q / np.linalg.norm(q)
        if len(self.buf) > 0:
            _, q_last = self.buf[-1]
            if np.dot(q_last, q) < 0:
                q = -q  # 保证符号连续
        self.buf.append((t, q))

    def compute(self):
        if len(self.buf) < 3:
            return None, None
        (t0, q0), (t1, q1), (t2, q2) = self.buf[0], self.buf[1], self.buf[2]
        dt = t2 - t0
        if dt <= 0:
            return None, t1

        # 控制输出频率
        if self.last_output_time and (t1 - self.last_output_time) < self.output_period:
            return None, t1

        R0 = R.from_quat(q0)
        R2 = R.from_quat(q2)
        R_rel = R0.inv() * R2
        rotvec = R_rel.as_rotvec()

        angle = np.linalg.norm(rotvec)
        if angle > self.max_angle:
            return None, t1

        omega_center = rotvec / dt
        if np.linalg.norm(omega_center) > 10:  # 限幅
            return None, t1

        self.omega_hist.append(omega_center)
        if len(self.omega_hist) >= 3:
            med = np.median(np.vstack(self.omega_hist), axis=0)
            self.last_output_time = t1
            return med, t1
        else:
            self.last_output_time = t1
            return omega_center, t1


# ================================================================
# 主节点：计算线速度 + 角速度，并输出 pose.tum / twist.tum
# ================================================================
class VRPNVelocityLogger:
    def __init__(self):
        # 文件输出路径
        self.pose_file = open("/home/hao/Desktop/twist_ws/src/TwistEstimator/output/pose.tum", "w")
        self.twist_file = open("/home/hao/Desktop/twist_ws/src/TwistEstimator/output/twist.tum", "w")

        # 缓冲
        self.pos_buf = deque(maxlen=5)  # 平滑线速度
        self.prev_pose = None
        self.last_v_output_time = None
        self.output_period = 0.02  # 降采样频率 50Hz

        # 滤波
        self.v_filt = np.zeros(3)
        self.ang_est = RobustAngularVelEstimator()

        rospy.Subscriber("/vrpn_client_node/uav1/pose", PoseStamped, self.pose_callback, queue_size=100)
        rospy.loginfo("✅ VRPN velocity logger started (with smoothing & rate limit).")

    # 平滑线速度计算
    def compute_linear_velocity(self, t, p):
        self.pos_buf.append((t, p))
        if len(self.pos_buf) < 3:
            return None, None
        t0, p0 = self.pos_buf[0]
        t2, p2 = self.pos_buf[-1]
        dt = t2 - t0
        if dt <= 0:
            return None, None
        v = (p2 - p0) / dt
        return v, (t0 + t2) / 2

    def pose_callback(self, msg):
        t = msg.header.stamp.to_sec()
        p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        q = np.array([
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w
        ])

        # 写位姿
        self.pose_file.write(f"{t:.6f} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}\n")

        # 降采样频率
        if self.last_v_output_time and (t - self.last_v_output_time) < self.output_period:
            self.ang_est.push(t, q)
            return

        # --- 线速度 ---
        v, t_v = self.compute_linear_velocity(t, p)
        if v is not None:
            # 简单低通滤波
            alpha = 0.2
            self.v_filt = alpha * v + (1 - alpha) * self.v_filt
        else:
            self.v_filt = np.zeros(3)

        # --- 角速度 ---
        self.ang_est.push(t, q)
        omega, t_mid = self.ang_est.compute()

        # --- 输出 ---
        if omega is not None:
            t_out = t_mid if t_mid is not None else t
            self.twist_file.write(
                f"{t_out:.6f} {self.v_filt[0]:.6f} {self.v_filt[1]:.6f} {self.v_filt[2]:.6f} "
                f"{omega[0]:.6f} {omega[1]:.6f} {omega[2]:.6f}\n"
            )
            self.last_v_output_time = t

        self.prev_pose = (t, p)

    def __del__(self):
        try:
            self.pose_file.close()
            self.twist_file.close()
            rospy.loginfo("✅ pose.tum & twist.tum saved and closed.")
        except Exception:
            pass


# ================================================================
# main()
# ================================================================
if __name__ == "__main__":
    rospy.init_node("vrpn_velocity_logger", anonymous=True)
    logger = VRPNVelocityLogger()
    rospy.spin()
