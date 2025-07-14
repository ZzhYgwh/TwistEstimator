#!/usr/bin/env python

import rospy
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt
from geometry_msgs.msg import TwistWithCovarianceStamped
from std_msgs.msg import Header
from io import BytesIO
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import threading
import time

# 设置全局字体
plt.rcParams.update({
    "font.family": "serif",
    "font.size": 14, 
    "axes.titlesize": 16,
    "axes.labelsize": 14,
    "legend.fontsize": 12,
    "xtick.labelsize": 12,
    "ytick.labelsize": 12,
    "text.usetex": True,  
    "axes.facecolor": 'black',
    "figure.facecolor": 'black',  
    "figure.edgecolor": 'black',  
    "grid.color": 'gray',  
    "xtick.color": 'white',  
    "ytick.color": 'white',  
    "axes.labelcolor": 'white',  
    "legend.facecolor": 'black',  
    "legend.edgecolor": 'gray',  
    "legend.framealpha": 0.5  
})

# 1. 设置低通滤波器函数
def butter_lowpass(cutoff, fs, order=5):
    nyquist = 0.5 * fs  
    normal_cutoff = cutoff / nyquist  
    b, a = butter(order, normal_cutoff, btype='low', analog=False)  
    return b, a

def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order)
    return filtfilt(b, a, data)

# 2. 获取数据并处理
dji_time = []
dji_linear_velocity = []
dji_angular_velocity = []

detector_time = []
detector_linear_velocity = []
detector_angular_velocity = []

def dji_callback(msg):
    global dji_time, dji_linear_velocity, dji_angular_velocity
    dji_time.append(msg.header.stamp.to_sec())
    dji_linear_velocity.append([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
    dji_angular_velocity.append([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])

def radar_callback(msg):
    global detector_time, detector_linear_velocity, detector_angular_velocity
    detector_time.append(msg.header.stamp.to_sec())
    detector_linear_velocity.append([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])
    detector_angular_velocity.append([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])

def listener():
    rospy.init_node('velocity_comparison_node', anonymous=True)
    
    # 订阅两个话题
    rospy.Subscriber('/dji/twist_with_covariance', TwistWithCovarianceStamped, dji_callback)
    rospy.Subscriber('/radar/twist_enu', TwistWithCovarianceStamped, radar_callback)
    
    rospy.spin()

init_time = False
# 3. 数据准备与低通滤波
def process_data():
    while not rospy.is_shutdown():
        # print("A")
        fs = 20  
        cutoff = 1  
        global dji_time, dji_linear_velocity, dji_angular_velocity, detector_time, detector_linear_velocity, detector_angular_velocity,init_time

        # print("len(dji_time), len(detector_time), init_time = ", len(dji_time), len(detector_time), init_time)
        if len(dji_time) > 0 and len(detector_time) > 0 and not init_time:
            # dji_time = dji_time - dji_time[0]
            dji_time = [x - dji_time[0] for x in dji_time]

            # detector_time = detector_time - detector_time[0]
            detector_time = [x - detector_time[0] for x in detector_time]

            init_time = True
        # else:
        #     time.sleep(0.02)
        #     continue

        # print("B")

        dji_time_np = np.array(dji_time)
        dji_linear_velocity_np = np.array(dji_linear_velocity)
        dji_angular_velocity_np = np.array(dji_angular_velocity)
        
        detector_time_np = np.array(detector_time)
        detector_linear_velocity_np = np.array(detector_linear_velocity)
        detector_angular_velocity_np = np.array(detector_angular_velocity)
        if len(detector_linear_velocity_np) > 18:
            detector_linear_velocity_np = np.apply_along_axis(butter_lowpass_filter, 0, detector_linear_velocity_np, cutoff, fs)
        # temp = detector_linear_velocity[:, 0].copy()
        # detector_linear_velocity[:, 0] = detector_linear_velocity[:, 1]
        # detector_linear_velocity[:, 1] = temp

        if len(detector_angular_velocity_np) > 18:
            detector_angular_velocity_np = np.apply_along_axis(butter_lowpass_filter, 0, detector_angular_velocity_np, cutoff, fs)
        detector_angular_velocity_np = np.clip(detector_angular_velocity_np, -10, 10)

        # print("dji_linear_velocity_np.shape = ", dji_linear_velocity_np.shape)
        # print("dji_linear_velocity_np.shape[0] = ", dji_linear_velocity_np.shape[0])
        # print("len(dji_linear_velocity_np) = ", len(dji_linear_velocity_np))
        
        dji_linear_velocity_np = np.reshape(dji_linear_velocity_np, (-1, 3))  # 将其重塑为二维数组，每行 3 个元素
        dji_angular_velocity_np = np.reshape(dji_angular_velocity_np, (-1, 3))  # 将其重塑为二维数组，每行 3 个元素
        detector_linear_velocity_np = np.reshape(detector_linear_velocity_np, (-1, 3))  # 将其重塑为二维数组，每行 3 个元素
        detector_angular_velocity_np = np.reshape(detector_angular_velocity_np, (-1, 3))  # 将其重塑为二维数组，每行 3 个元素
        # if dji_linear_velocity_np.shape[0] > 0:
        #     print("dji_linear_velocity_np[0][0]", dji_linear_velocity_np[0][0])


        if dji_linear_velocity_np.shape[0] > 5:
            plot_data(dji_time_np, dji_linear_velocity_np, dji_angular_velocity_np, detector_time_np, detector_linear_velocity_np, detector_angular_velocity_np)

        # return dji_time, dji_linear_velocity, dji_angular_velocity, detector_time, detector_linear_velocity, detector_angular_velocity

linear_pub = rospy.Publisher('/velocity_anlysis/linear', Image, queue_size=10)
angluar_pub = rospy.Publisher('/velocity_anlysis/angular', Image, queue_size=10)
def plot_data(dji_time_np, dji_linear_velocity_np, dji_angular_velocity_np, detector_time_np, detector_linear_velocity_np, detector_angular_velocity_np):
    global image_pub, init_time
    # print("dji_time_np , detector_time_np size = ", dji_time_np.shape[0], detector_time_np.shape[0])
    # print("dji_linear_velocity_np , dji_angular_velocity_np = ", dji_linear_velocity_np.shape[0], dji_angular_velocity_np.shape[0])
    # print("detector_linear_velocity_np , detector_angular_velocity_np = ", detector_linear_velocity_np.shape[0], detector_angular_velocity_np.shape[0])

    fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True, facecolor='black')
    colors = ['#E74C3C', '#3498DB']
    linestyles = ['-', '-']

    for i, ax in enumerate(axs[:3]):
        # print("dji.shape = ", dji_time_np)
        ax.plot(dji_time_np, dji_linear_velocity_np[:, i], label=f'Dji $v_{{x}}$' if i == 0 else f'Dji $v_{{y}}$' if i == 1 else f'Dji $v_{{z}}$',
                color=colors[0], linestyle=linestyles[0], linewidth=1, alpha=0.65)
        ax.plot(detector_time_np, detector_linear_velocity_np[:, i], label=f'Detector $v_{{x}}$' if i == 0 else f'Detector $v_{{y}}$' if i == 1 else f'Detector $v_{{z}}$',
                color=colors[1], linestyle=linestyles[1], linewidth=1)

        min_y_dji = np.min(dji_linear_velocity_np[:, i])
        min_y_detector = np.min(detector_linear_velocity_np[:, i])
        min_y = np.min([min_y_dji, min_y_detector])

        max_y_dji = np.max(dji_linear_velocity_np[:, i])
        max_y_detector = np.max(detector_linear_velocity_np[:, i])
        max_y = np.max([max_y_dji, max_y_detector])

        max_abs = max(abs(min_y), abs(max_y))
        ax.set_yticks([-max_abs, -max_abs / 2, 0, max_abs / 2, max_abs])
        ax.set_ylabel(f'$v_{{{["x","y","z"][i]}}}$ (m/s)', fontsize=14, fontweight='bold')
        ax.legend(loc='upper right', frameon=True, fontsize=12, fancybox=True)
        ax.grid(True, linestyle='--', alpha=0.6)
        ax.tick_params(axis='both', labelsize=12, width=2, colors='white')  

    axs[2].set_xlabel(r'$T(s)$', fontsize=14, fontweight='bold')
    fig.tight_layout(pad=3.0)
    buf = BytesIO()
    plt.savefig(buf, format='png', dpi=300)
    buf.seek(0)
    bridge = CvBridge()
    linear_img_msg = bridge.cv2_to_imgmsg(cv2.imdecode(np.frombuffer(buf.read(), np.uint8), cv2.IMREAD_COLOR), encoding="bgr8")
    linear_pub.publish(linear_img_msg)
    plt.close(fig)

    fig2, axs2 = plt.subplots(3, 1, figsize=(10, 12), sharex=True, facecolor='black')
    colors = ['#E74C3C', '#3498DB']
    linestyles = ['-', '-']

    for i, ax in enumerate(axs2[:3]):
        ax.plot(dji_time_np, dji_angular_velocity_np[:, i], label=f'Dji $\\omega_{{x}}$' if i == 0 else f'Dji $\\omega_{{y}}$' if i == 1 else f'Dji $\\omega_{{z}}$',
                color=colors[0], linestyle=linestyles[0], linewidth=1, alpha=0.65)
        ax.plot(detector_time_np, detector_angular_velocity_np[:, i], label=f'Detector $\\omega_{{x}}$' if i == 0 else f'Detector $\\omega_{{y}}$' if i == 1 else f'Detector $\\omega_{{z}}$',
                color=colors[1], linestyle=linestyles[1], linewidth=1)

        min_y_dji = np.min(dji_angular_velocity_np[:, i])
        min_y_detector = np.min(detector_angular_velocity_np[:, i])
        min_y = np.min([min_y_dji, min_y_detector])

        max_y_dji = np.max(dji_angular_velocity_np[:, i])
        max_y_detector = np.max(detector_angular_velocity_np[:, i])
        max_y = np.max([max_y_dji, max_y_detector])

        max_abs = max(abs(min_y), abs(max_y))
        ax.set_yticks([-max_abs, -max_abs / 2, 0, max_abs / 2, max_abs])
        ax.set_ylabel(f'$\\omega_{{{["x","y","z"][i]}}}$ (rad/s)', fontsize=14, fontweight='bold')
        ax.legend(loc='upper right', frameon=True, fontsize=12, fancybox=True)
        ax.grid(True, linestyle='--', alpha=0.6)
        ax.tick_params(axis='both', labelsize=12, width=2, colors='white')

    axs2[2].set_xlabel(r'$T(s)$', fontsize=14, fontweight='bold')
    fig2.tight_layout(pad=3.0)
    buf2 = BytesIO()
    plt.savefig(buf2, format='png', dpi=300)
    buf2.seek(0)
    bridge = CvBridge()
    angular_img_msg = bridge.cv2_to_imgmsg(cv2.imdecode(np.frombuffer(buf2.read(), np.uint8), cv2.IMREAD_COLOR), encoding="bgr8")
    angluar_pub.publish(angular_img_msg)
    plt.close(fig2)

def main():

    # rospy.init_node("visual_twist")
    # # 启动 ROS 节点监听器线程
    # listener_thread = threading.Thread(target=listener)
    # listener_thread.daemon = True
    # listener_thread.start()

    # 启动数据处理线程
    data_processing_thread = threading.Thread(target=process_data)
    data_processing_thread.daemon = True
    data_processing_thread.start()

    listener()

    # rospy.spin()

if __name__ == '__main__':
    main()
