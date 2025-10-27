#!/usr/bin/env python3
import numpy as np
from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation as R

# 输入 TUM 文件
pose_file = "pose_zeroed.tum"
twist_file = "twist.tum"

# 读取 pose
data = []
with open(pose_file,"r") as f:
    for line in f:
        if line.startswith("#") or line.strip()=="":
            continue
        data.append([float(x) for x in line.strip().split()])
data = np.array(data)
timestamps = data[:,0]
positions = data[:,1:4]
quaternions = data[:,4:8]  # qx,qy,qz,qw

# 保证四元数连续
for i in range(1,len(quaternions)):
    if np.dot(quaternions[i-1], quaternions[i]) < 0:
        quaternions[i] *= -1

# 平滑位置
window = 31 if len(positions)>31 else (len(positions)//2*2+1)
positions_smooth = savgol_filter(positions, window_length=window, polyorder=3, axis=0)

dt = np.gradient(timestamps)

# 线速度 world -> body
vel_world = np.gradient(positions_smooth, axis=0)/dt[:,None]
vel_body = np.zeros_like(vel_world)
for i in range(len(vel_world)):
    r = R.from_quat([quaternions[i,0], quaternions[i,1], quaternions[i,2], quaternions[i,3]])
    vel_body[i] = r.inv().apply(vel_world[i])

# 角速度机体系
def quat_derivative(q, dt):
    dq = np.zeros_like(q)
    dq[1:-1] = (q[2:] - q[:-2]) / (dt[1:-1,None]*2)
    dq[0] = (q[1]-q[0])/dt[0]
    dq[-1] = (q[-1]-q[-2])/dt[-1]
    return dq

dq = quat_derivative(quaternions, dt)
omega_body = np.zeros((len(quaternions),3))
for i in range(len(quaternions)):
    q = quaternions[i]
    q_conj = np.array([q[3], -q[0], -q[1], -q[2]])  # 四元数共轭
    dq_i = dq[i]
    xyz = dq_i[:3]*q_conj[3] + q_conj[:3]*dq_i[3] + np.cross(dq_i[:3], q_conj[:3])
    omega_body[i] = 2*xyz

# 保存 twist.tum
with open(twist_file,"w") as f:
    f.write("# timestamp vx vy vz wx wy wz\n")
    for t,v,w in zip(timestamps, vel_body, omega_body):
        f.write(f"{t:.6f} {v[0]:.6f} {v[1]:.6f} {v[2]:.6f} {w[0]:.6f} {w[1]:.6f} {w[2]:.6f}\n")

print(f"twist.tum 已生成: {twist_file}")

