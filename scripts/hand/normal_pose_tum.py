#!/usr/bin/env python3
import numpy as np

input_file = "pose.tum"
output_file = "pose_zeroed.tum"

# 1. 读取 TUM 文件（跳过注释）
data = []
with open(input_file, "r") as f:
    for line in f:
        if line.startswith("#") or line.strip() == "":
            continue
        parts = line.strip().split()
        data.append([float(p) for p in parts])

data = np.array(data)  # shape (N,8)
timestamps = data[:,0]
positions = data[:,1:4]  # tx, ty, tz
quaternions = data[:,4:8]  # qx, qy, qz, qw

# 2. 以第一帧位置归零
origin = positions[0].copy()
positions_zeroed = positions - origin

# 3. 写入新的 TUM 文件
with open(output_file, "w") as f:
    f.write("# timestamp tx ty tz qx qy qz qw (zeroed)\n")
    for i in range(len(data)):
        line = [timestamps[i]] + positions_zeroed[i].tolist() + quaternions[i].tolist()
        line_str = " ".join(f"{x:.6f}" for x in line)
        f.write(line_str + "\n")

print(f"归零后的轨迹已保存到 {output_file}")

