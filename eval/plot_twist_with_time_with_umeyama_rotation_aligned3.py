import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
from scipy import interpolate

# 设置全局字体
plt.rcParams.update({
    "font.family": "serif",
    "font.size": 14, 
    "axes.titlesize": 16,
    "axes.labelsize": 14,
    "legend.fontsize": 12,
    "xtick.labelsize": 12,
    "ytick.labelsize": 12,
    "text.usetex": True  # 启用 LaTeX
})

# 1. 加载数据
# camera_twist = pd.read_csv('../../../dji8-5-20/twist.tum', header=None, delim_whitespace=True)
camera_twist = pd.read_csv('./twist.tum', header=None, delim_whitespace=True) # twist
detector = pd.read_csv('../output/detector2.tum', header=None, delim_whitespace=True) # detector2 ./river.tum
estimator = pd.read_csv('../output/estimate.tum', header=None, delim_whitespace=True) # estimate.tum

# 2. 提取前7列的时间戳、线速度、角速度
camera_time = camera_twist.iloc[:, 0].values  # 第一列是时间
camera_linear_velocity = camera_twist.iloc[:, 1:4].values  # 第二到第四列是线性速度 (vx, vy, vz)
camera_angular_velocity = camera_twist.iloc[:, 4:7].values  # 第五到第七列是角速度 (wx, wy, wz)

detector_time = detector.iloc[:, 0].values  # 第一列是时间
detector_linear_velocity = detector.iloc[:, 1:4].values  # 第二到第四列是线性速度 (vx, vy, vz)
detector_angular_velocity = detector.iloc[:, 4:7].values  # 第五到第七列是角速度 (wx, wy, wz)

estimator_time = estimator.iloc[:, 0].values  # 第一列是时间
estimator_linear_velocity = estimator.iloc[:, 1:4].values  # 第二到第四列是线性速度 (vx, vy, vz)
estimator_angular_velocity = estimator.iloc[:, 4:7].values  # 第五到第七列是角速度 (wx, wy, wz)

# 3. 设置低通滤波器
def butter_lowpass(cutoff, fs, order=5):
    nyquist = 0.5 * fs  # 奈奎斯特频率
    normal_cutoff = cutoff / nyquist  # 标准化频率
    b, a = butter(order, normal_cutoff, btype='low', analog=False)  # butter滤波器系数
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order)
    return filtfilt(b, a, data)  # 使用filtfilt进行零相位滤波

def umeyama_rotation_alignment(X, Y):
    """
    Umeyama method (rotation only) to align two 3xN point sets.
    X: Source points (e.g., detector_linear_velocity) (3xN)
    Y: Target points (e.g., camera_linear_velocity)   (3xN)
    Returns: 3x3 rotation matrix R
    """
    assert X.shape[0] == 3 and Y.shape[0] == 3, "Input must be 3xN"

    # Subtract centroids
    mu_X = np.mean(X, axis=1, keepdims=True)
    mu_Y = np.mean(Y, axis=1, keepdims=True)
    X_c = X - mu_X
    Y_c = Y - mu_Y

    # Compute covariance matrix
    Sigma = Y_c @ X_c.T / X.shape[1]

    # SVD
    U, _, Vt = np.linalg.svd(Sigma)

    # Ensure proper rotation (det(R) = 1)
    D = np.eye(3)
    if np.linalg.det(U @ Vt) < 0:
        D[2, 2] = -1

    R = U @ D @ Vt
    print("R = ", R)
    return R

# 设置采样率和截止频率
fs = 20  # 采样率，单位 Hz 12
cutoff = 1  # 截止频率，单位 Hz

# 对线性速度进行低通滤波
# detector_linear_velocity = np.apply_along_axis(butter_lowpass_filter, 0, detector_linear_velocity, cutoff, fs)
# estimator_linear_velocity = np.apply_along_axis(butter_lowpass_filter, 0, estimator_linear_velocity, cutoff, fs)
# camera_linear_velocity = np.apply_along_axis(butter_lowpass_filter, 0, camera_linear_velocity, cutoff, fs)
# camera_angular_velocity = np.apply_along_axis(butter_lowpass_filter, 0, camera_angular_velocity, cutoff, fs)
# detector_angular_velocity = np.apply_along_axis(butter_lowpass_filter, 0, detector_angular_velocity, cutoff, fs)
# estimator_angular_velocity = np.apply_along_axis(butter_lowpass_filter, 0, estimator_angular_velocity, cutoff, fs)

temp = 1.0 * detector_linear_velocity[:, 0].copy()
detector_linear_velocity[:, 0] = 1.0 * detector_linear_velocity[:, 1]
detector_linear_velocity[:, 1] = temp
# camera_linear_velocity[:, 2] = -1.0 * camera_linear_velocity[:, 2]

temp = estimator_linear_velocity[:, 0].copy()
estimator_linear_velocity[:, 0] = estimator_linear_velocity[:, 1]
estimator_linear_velocity[:, 1] = temp
# estimator_linear_velocity[:, 2] = -1.0 * estimator_linear_velocity[:, 2]


temp = detector_angular_velocity[:, 0].copy()
detector_angular_velocity[:, 0] = detector_angular_velocity[:, 1]
detector_angular_velocity[:, 1] = temp

temp = estimator_angular_velocity[:, 0].copy()
estimator_angular_velocity[:, 0] = estimator_angular_velocity[:, 1]
estimator_angular_velocity[:, 1] = temp

#temp = estimator_angular_velocity[:, 0].copy()
#estimator_angular_velocity[:, 0] = estimator_angular_velocity[:, 2]
#estimator_angular_velocity[:, 2] = temp


# estimator_angular_velocity[:, 2] = -1.0 * estimator_angular_velocity[:, 2]
# 对角速度进行低通滤波
# detector_angular_velocity = np.apply_along_axis(butter_lowpass_filter, 0, detector_angular_velocity, cutoff, fs)
# detector_angular_velocity = np.clip(detector_angular_velocity, -10, 10)

# temp = -1.0 * detector_angular_velocity[:, 0].copy()
# detector_angular_velocity[:, 0] = -1.0 * detector_angular_velocity[:, 1]
# detector_angular_velocity[:, 1] = temp
# detector_angular_velocity[:, 2] = -1.0 * detector_angular_velocity[:, 2]



camera_interpolated = interpolate.interp1d(camera_time, camera_linear_velocity, kind='linear', axis=0, fill_value="extrapolate")
aligned_camera_velocities = camera_interpolated(detector_time)

# 转置为 3xN 矩阵
cam_vel = aligned_camera_velocities.T
det_vel = detector_linear_velocity.T
est_vel = estimator_linear_velocity.T

# === 旋转对齐 ===
R = umeyama_rotation_alignment(det_vel, cam_vel)
R = np.eye(3)
# === 应用旋转 ===
det_aligned = R @ det_vel  # 仍然是 3xN
est_aligned = R @ est_vel  # 仍然是 3xN

detector_linear_velocity = det_aligned.T
estimator_linear_velocity = est_aligned.T

# 4. 绘制图窗：线速度和角速度的 x, y, z 分量

# 设置相对时间
# camera_time = camera_time - camera_time[0]
# detector_time = detector_time - detector_time[0]
# 以 detector_time[0] 为初始时间
start_time = detector_time[0]

start_time = estimator_time[0]

camera_time = camera_time - start_time
detector_time = detector_time - start_time
estimator_time = estimator_time - start_time

# 使用布尔索引过滤掉负时间的元素
valid_indices = camera_time >= 0
camera_time = camera_time[valid_indices]
camera_linear_velocity = camera_linear_velocity[valid_indices]
camera_angular_velocity = camera_angular_velocity[valid_indices]
valid_indices = detector_time >= 0
detector_time = detector_time[valid_indices]
detector_linear_velocity = detector_linear_velocity[valid_indices]
detector_angular_velocity = detector_angular_velocity[valid_indices]

max_time = max(estimator_time)
valid_indices = camera_time <= max_time
camera_time = camera_time[valid_indices]
camera_linear_velocity = camera_linear_velocity[valid_indices]
camera_angular_velocity = camera_angular_velocity[valid_indices]
valid_indices = detector_time <= max_time
detector_time = detector_time[valid_indices]
detector_linear_velocity = detector_linear_velocity[valid_indices]
detector_angular_velocity = detector_angular_velocity[valid_indices]



#camera_linear_velocity = np.clip(camera_angular_velocity, -5, 5)
#detector_linear_velocity = np.clip(detector_angular_velocity, -5, 5)
#estimator_linear_velocity = np.clip(estimator_angular_velocity, -5, 5)
#camera_angular_velocity = np.clip(camera_angular_velocity, -0.8, 0.8)
#detector_angular_velocity = np.clip(detector_angular_velocity, -0.8, 0.8)
#estimator_angular_velocity = np.clip(estimator_angular_velocity, -0.8, 0.8)

camera_angular_velocity[np.abs(camera_angular_velocity) > 0.48] = 0
detector_angular_velocity[np.abs(detector_angular_velocity) > 0.48] = 0
estimator_angular_velocity[np.abs(estimator_angular_velocity) > 0.48] = 0


# 创建子图
fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
# fig.suptitle(r'Linear Velocity Comparison: Camera Twist vs Estimator', fontsize=18, fontweight='bold')  # 标题加粗

# 颜色与样式
# colors = ['#E74C3C', '#3498DB', '#2ECC71']
# colors = ['#E74C3C', '#2980B9', '#27AE60']
# colors = ['#E74C3C', '#1F77B4', '#FFB000']
# colors = ['#E74C3C', '#0072B2', '#009E73']
colors = ['#E74C3C',  '#1ABC9C', '#8E44AD']


linestyles = ['-', '-', '-']
# X 轴范围
min_time, max_time = camera_time.min(), camera_time.max()

# 画三条线速度曲线
velocity_legends = [r'$v_x$', r'$v_y$', r'$v_z$']
velocity_labels = [r'$v_x\ (m/s)$', r'$v_y\ (m/s)$', r'$v_z\ (m/s)$']  # 为单位添加空格

for i, ax in enumerate(axs):
    # 画出第一条线 (Dji)
    ax.plot(camera_time, camera_linear_velocity[:, i], label=f'Dji {velocity_legends[i]}', 
            color=colors[0], linestyle=linestyles[0], linewidth=1.5, alpha=1.0)
    # 画出第二条线 (Dector)
    ax.plot(detector_time, detector_linear_velocity[:, i], label=f'Detector {velocity_legends[i]}', 
            color=colors[1], linestyle=linestyles[1], linewidth=1.5, alpha=0.6)

    # 画出第三条线 (Estimator)
    ax.plot(estimator_time, estimator_linear_velocity[:, i], label=f'Estimator {velocity_legends[i]}', 
            color=colors[2], linestyle=linestyles[2], linewidth=1.5, alpha=0.8)


    # 计算 Y 轴范围，确保对称并确保 0 居中
    # 计算 Y 轴范围，确保对称并确保 0 居中
    # min_y = np.min([detector_linear_velocity[:, i], dji_linear_velocity[:, i]])

    min_y_dji = np.min(camera_linear_velocity[:, i])
    min_y_detector = np.min(detector_linear_velocity[:, i])
    min_y_estimator = np.min(estimator_linear_velocity[:, i])
    min_y = np.min([min_y_dji, min_y_detector, min_y_estimator])

    # max_y = np.max([detector_linear_velocity[:, i], dji_linear_velocity[:, i]])

    max_y_dji = np.max(camera_linear_velocity[:, i])
    max_y_detector = np.max(detector_linear_velocity[:, i])
    max_y_estimator = np.max(estimator_linear_velocity[:, i])
    max_y = np.max([max_y_dji, max_y_detector, max_y_estimator])

    max_abs = max(abs(min_y), abs(max_y))  # 取最大绝对值，确保对称

    ax.set_yticks([-max_abs, -max_abs / 2, 0, max_abs / 2, max_abs])  # 确保 Y 轴对称并且 0 居中
    ax.set_ylabel(velocity_labels[i], fontsize=20, fontweight='bold')  # Y 轴标签加粗 14 

    # ax.legend(loc='upper right', frameon=True, fontsize=15, fancybox=True) # 12
    # ax.legend(loc='upper right', frameon=True, fontsize=14, fancybox=True, framealpha=0.5, bbox_to_anchor=(1, 1))  # 透明度设置为0.5，图例在外面
    ax.legend(loc='upper right', frameon=True, fontsize=14, fancybox=True, framealpha=0.5)

    ax.grid(True, linestyle='--', alpha=0.6)  # 添加网格

    # 设置坐标轴刻度加粗
    ax.tick_params(axis='both', labelsize=20, width=2)  # 设置坐标轴刻度加粗 # 12

# 设置 X 轴
axs[2].set_xlabel(r'$T(s)$', fontsize=20, fontweight='bold')  # X 轴标签加粗 # 14
axs[2].set_xticks(np.arange(min_time, max_time, 50))  # 每 50 取值标注

# 调整布局
plt.tight_layout(rect=[0, 0, 1, 0.96])

# 保存高分辨率图片
# plt.savefig("linear_velocity_comparison.svg", dpi=300, bbox_inches='tight')

# plt.savefig("linear_velocity_comparison.svg", bbox_inches='tight')
plt.savefig("linear_velocity_comparison.pdf", dpi=900, bbox_inches='tight')
plt.show()


##
# 创建子图
fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
# fig.suptitle(r'Linear Velocity Comparison: Camera Twist vs Estimator', fontsize=18, fontweight='bold')  # 标题加粗

# 颜色与样式
# colors = ['#E74C3C', '#3498DB']
# colors = ['#E74C3C', '#3498DB', '#2ECC71']

linestyles = ['-', '-', '-']

# X 轴范围
min_time, max_time = camera_time.min(), camera_time.max()

# 画三条线速度曲线
velocity_legends = [r'$w_x$', r'$w_y$', r'$w_z$']
velocity_labels = [r'$w_x\ (rad/s)$', r'$w_y\ (rad/s)$', r'$w_z\ (rad/s)$']  # 为单位添加空格

for i, ax in enumerate(axs):
    # 画出第一条线 (Dji)
    ax.plot(camera_time, camera_angular_velocity[:, i], label=f'Dji {velocity_legends[i]}', 
            color=colors[0], linestyle=linestyles[0], linewidth=1.5, alpha=1.0) # 0.65
    # 画出第二条线 (Dector)
    ax.plot(detector_time, detector_angular_velocity[:, i], label=f'Detector {velocity_legends[i]}', 
            color=colors[1], linestyle=linestyles[1], linewidth=1.5, alpha=0.8)

    # 画出第三条线 (Estimator)
    ax.plot(estimator_time, estimator_angular_velocity[:, i], label=f'Estimator {velocity_legends[i]}', 
            color=colors[2], linestyle=linestyles[2], linewidth=1.5, alpha=0.6)

    # 计算 Y 轴范围，确保对称并确保 0 居中
    # min_y = np.min([detector_linear_velocity[:, i], dji_linear_velocity[:, i]])

    min_y_dji = np.min(camera_angular_velocity[:, i])
    min_y_detector = np.min(detector_angular_velocity[:, i])
    min_y_estimator = np.min(estimator_angular_velocity[:, i])
    min_y = np.min([min_y_dji, min_y_detector, min_y_estimator])

    # max_y = np.max([detector_linear_velocity[:, i], dji_linear_velocity[:, i]])

    max_y_dji = np.max(camera_angular_velocity[:, i])
    max_y_detector = np.max(detector_angular_velocity[:, i])
    max_y_estimator = np.min(estimator_angular_velocity[:, i])
    max_y = np.max([max_y_dji, max_y_detector, max_y_estimator])

    max_abs = max(abs(min_y), abs(max_y))  # 取最大绝对值，确保对称

    ax.set_yticks([-max_abs, -max_abs / 2, 0, max_abs / 2, max_abs])  # 确保 Y 轴对称并且 0 居中
    ax.set_ylabel(velocity_labels[i], fontsize=20, fontweight='bold')  # Y 轴标签加粗 14
    
    ax.legend(loc='upper right', frameon=True, fontsize=14, fancybox=True, framealpha=0.5)
    # ax.legend(loc='upper right', frameon=True, fontsize=15, fancybox=True) # 12
    ax.grid(True, linestyle='--', alpha=0.6)  # 添加网格

    # 设置坐标轴刻度加粗
    ax.tick_params(axis='both', labelsize=20, width=2)  # 设置坐标轴刻度加粗 12

# 设置 X 轴
axs[2].set_xlabel(r'$T(s)$', fontsize=20, fontweight='bold')  # X 轴标签加粗 14
axs[2].set_xticks(np.arange(min_time, max_time, 50))  # 每 50 取值标注

# 调整布局
plt.tight_layout(rect=[0, 0, 1, 0.96])

# 保存高分辨率图片
# plt.savefig("linear_velocity_comparison.svg", dpi=300, bbox_inches='tight'

# plt.savefig("angular_velocity_comparison.svg", bbox_inches='tight')
plt.savefig("angular_velocity_comparison.pdf", dpi=900, bbox_inches='tight')
plt.show()


## 线速度误差分析
import numpy as np
from scipy.interpolate import interp1d

# 假设数据为 numpy 数组
# camera_time: (N,), camera_angular_velocity: (N, 3)
# detector_time: (M,), detector_angular_velocity: (M, 3)

# 对 detector 的每个轴进行插值
interp_fx = interp1d(detector_time, detector_linear_velocity[:, 0], kind='linear', fill_value='extrapolate')
interp_fy = interp1d(detector_time, detector_linear_velocity[:, 1], kind='linear', fill_value='extrapolate')
interp_fz = interp1d(detector_time, detector_linear_velocity[:, 2], kind='linear', fill_value='extrapolate')

# 插值到 camera_time 上
detector_interp_linear_velocity = np.stack([
    interp_fx(camera_time),
    interp_fy(camera_time),
    interp_fz(camera_time)
], axis=1)

# 误差向量
linear_error = detector_interp_linear_velocity - camera_linear_velocity

# 每帧的欧几里得误差
euclidean_error = np.linalg.norm(linear_error, axis=1)

# RMSE
rmse = np.sqrt(np.mean(euclidean_error**2))

print(f"Linear velocity RMSE: {rmse:.6f} rad/s")

epsilon = 1e-6  # 避免除以0

# 欧几里得范数
true_norm = np.linalg.norm(camera_linear_velocity, axis=1)
error_norm = np.linalg.norm(detector_interp_linear_velocity - camera_linear_velocity, axis=1)

# 相对误差
relative_error = error_norm / (true_norm + epsilon)

# 计算平均相对误差或中位数等
mean_relative_error = np.mean(relative_error)
print(f"Mean Relative Linear Velocity Error: {mean_relative_error:.4f}")



## 角速度误差分析
import numpy as np
from scipy.interpolate import interp1d

# 假设数据为 numpy 数组
# camera_time: (N,), camera_angular_velocity: (N, 3)
# detector_time: (M,), detector_angular_velocity: (M, 3)

# 对 detector 的每个轴进行插值
interp_fx = interp1d(detector_time, detector_angular_velocity[:, 0], kind='linear', fill_value='extrapolate')
interp_fy = interp1d(detector_time, detector_angular_velocity[:, 1], kind='linear', fill_value='extrapolate')
interp_fz = interp1d(detector_time, detector_angular_velocity[:, 2], kind='linear', fill_value='extrapolate')

# 插值到 camera_time 上
detector_interp_angular_velocity = np.stack([
    interp_fx(camera_time),
    interp_fy(camera_time),
    interp_fz(camera_time)
], axis=1)

# 误差向量
angular_error = detector_interp_angular_velocity - camera_angular_velocity

# detector_time 和 linear_error angular_error 需要保存

# 构造 DataFrame
error_df = pd.DataFrame({
    'time': camera_time,
    'linear_x': linear_error[:, 0],
    'linear_y': linear_error[:, 1],
    'linear_z': linear_error[:, 2],
    'angular_x': angular_error[:, 0],
    'angular_y': angular_error[:, 1],
    'angular_z': angular_error[:, 2]
})

# 保存为 CSV
error_df.to_csv('twist_estimator_error.csv', index=False)
print("保存成功：twist_estimator_error.csv")


# 每帧的欧几里得误差
euclidean_error = np.linalg.norm(angular_error, axis=1)

# RMSE
rmse = np.sqrt(np.mean(euclidean_error**2))

print(f"Angular velocity RMSE: {rmse:.6f} rad/s")

epsilon = 1e-6  # 避免除以0

# 欧几里得范数
true_norm = np.linalg.norm(camera_angular_velocity, axis=1)
error_norm = np.linalg.norm(detector_interp_angular_velocity - camera_angular_velocity, axis=1)

# 相对误差
relative_error = error_norm / (true_norm + epsilon)

# 计算平均相对误差或中位数等
mean_relative_error = np.mean(relative_error)
print(f"Mean Relative Angular Velocity Error: {mean_relative_error:.4f}")



