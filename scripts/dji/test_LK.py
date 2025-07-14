#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge
import message_filters
import os
import sensor_msgs.point_cloud2 as pc2

# 全局变量
prev_gray = None
prev_pts = None
flow_img = None
bridge = CvBridge()

last_img_time = None

# 假设你已经定义了相关参数
corner_params = {
    'maxCorners': 100,  # 最大特征点数量
    'qualityLevel': 0.3,  # 特征点质量标准
    'minDistance': 7,  # 最小特征点距离
    'blockSize': 7  # 计算特征点时使用的块大小
}

lk_params = {
    'winSize': (15, 15),  # 光流窗口大小
    'maxLevel': 2,  # 光流金字塔的最大层数
    'criteria': (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)  # 终止条件
}

# 保存路径
output_dir = '/media/hao/hao2/228/test/lab/raw_img'  # 修改为你想保存的目录
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# 保存计数器
frame_count = 0

def image_callback(image_msg):
    global prev_gray, prev_pts, flow_img, frame_count, last_img_time

    # 将 ROS 图像消息转换为 OpenCV 格式
    frame = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    cur_img_time = image_msg.header.stamp.to_sec()

    # 如果之前没有图像，跳过处理
    if prev_gray is None:
        prev_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        prev_pts = cv2.goodFeaturesToTrack(prev_gray, mask=None, **corner_params)
        flow_img = np.zeros_like(frame)  # 初始化 flow_img
        last_img_time = image_msg.header.stamp.to_sec()
        return

    # 转换为灰度图
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 检查 prev_gray 和 gray 的尺寸是否相同
    if prev_gray.shape != gray.shape:
        print(f"Warning: Image sizes do not match. prev_gray: {prev_gray.shape}, gray: {gray.shape}")
        gray = cv2.resize(gray, (prev_gray.shape[1], prev_gray.shape[0]))  # 调整大小
        print(f"Resizing gray to match prev_gray: {gray.shape}")

    # 计算光流 (使用 Lucas-Kanade 光流法)
    next_pts, status, err = cv2.calcOpticalFlowPyrLK(prev_gray, gray, prev_pts, None, **lk_params)

    # 只保留跟踪成功的点
    good_new = next_pts[status == 1]
    good_old = prev_pts[status == 1]

    # 新加入的点：根据状态标志筛选
    good_new_new = next_pts[status == 0]  # 新加入的点

    # 绘制光流（仅用于可视化）
    # flow_img = frame.copy()
    # for i, (new, old) in enumerate(zip(good_new, good_old)):
    #     a, b = new.ravel()
    #     c, d = old.ravel()
    #     flow_img = cv2.line(flow_img, (a, b), (c, d), (0, 255, 0), 1)  # 绿色光流线
    #     flow_img = cv2.circle(flow_img, (a, b), 1, (0, 0, 255), -1)  # 红色圆点标记匹配的点

    # 绘制真实时间尺度下的光流（仅用于可视化）
    flow_img = frame.copy()
    for i, (new, old) in enumerate(zip(good_new, good_old)):
        a, b = new.ravel()
        c, d = old.ravel()
        time_s = 1.0 / (cur_img_time - last_img_time)
        c = a + (c - a) * time_s
        d = b + (d - b) * time_s
        # 检查 c 和 d 在 flow_img 像素范围内
        height, width = flow_img.shape[:2]
        c = np.clip(c, 0, width - 1)
        d = np.clip(d, 0, height - 1)

        c, d = int(c), int(d)
        # a, b = int(a), int(b)

        flow_img = cv2.line(flow_img, (a, b), (c, d), (0, 255, 0), 1)  # 绿色光流线
        flow_img = cv2.circle(flow_img, (a, b), 1, (0, 0, 255), -1)  # 红色圆点标记匹配的点


    # 确保总共有100个特征点
    if len(good_new) < 100:
        # 需要添加新特征点
        remaining_points = 100 - len(good_new)

        # 提取新的特征点，确保不与已经的光流失败点重复
        # 创建掩码，用于避免重复特征点
        mask = np.ones(gray.shape, dtype=np.uint8)
        
        # 用已有的匹配失败点标记掩码中的区域
        for pt in good_new:
            x, y = pt.ravel().astype(int)
            cv2.circle(mask, (x, y), 5, 0, -1)  # 标记半径为5的区域为0，防止选择这些区域

        # 重新提取特征点
        new_pts = cv2.goodFeaturesToTrack(gray, mask=mask, **corner_params)

        # 如果有新点，添加到现有的特征点列表
        if new_pts is not None:
            new_pts = new_pts.reshape(-1, 2)  # 转换为 (x, y) 的格式

            # 去除重复点（如果新提取的点和失败点有重叠）
            new_pts_filtered = []
            for pt in new_pts:
                if not any(np.linalg.norm(pt - good_failed) < 5 for good_failed in good_new):  # 阈值为5，可以根据需要调整
                    new_pts_filtered.append(pt)
        
            # 确保 new_pts_filtered 不为空并且维度一致
            if len(new_pts_filtered) > 0:
                new_pts_filtered = np.array(new_pts_filtered)  # 转换为 numpy 数组
                if new_pts_filtered.ndim == 1:
                    new_pts_filtered = new_pts_filtered.reshape(-1, 2)  # 保证是 (N, 2) 形状

                # 合并新的特征点
                good_new = np.vstack((good_new, new_pts_filtered))  # 追加新特征点
                good_new = good_new[:100]  # 保证最多100个特征点
            else:
                print("No new points were added.")
    
            # 如果还不够 100 个特征点，继续提取直到满足
            # good_new = np.vstack((good_new, new_pts_filtered))  # 追加新特征点
            # good_new = good_new[:100]  # 保证最多100个特征点

    # 绘制新加入的特征点（以蓝色标记）
    for i, new in enumerate(good_new_new):
        if new is not None and len(new) == 2:  # 确保 new 是一个有效的二维点
            a, b = new.ravel()  # 展平为 x, y 坐标
            flow_img = cv2.circle(flow_img, (a, b), 5, (255, 0, 0), -1)  # 蓝色圆点标记新加入的点

    # 显示光流图像
    cv2.imshow("Optical Flow", flow_img)
    cv2.waitKey(1)

    # 保存光流图像（每帧保存）
    frame_count += 1
    flow_image_path = os.path.join(output_dir, f"flow_image_{frame_count:04d}.png")
    cv2.imwrite(flow_image_path, flow_img)

    # 更新上一帧数据
    prev_gray = gray.copy()
    prev_pts = good_new.reshape(-1, 1, 2)
    last_img_time = cur_img_time
    

def radar_callback(radar_msg):
    # # 处理雷达点云数据
    # # 使用 sensor_msgs.point_cloud2 的 read_points 函数来提取点云
    # pc_data = pc2.read_points(radar_msg, field_names=("x", "y", "z"), skip_nans=True)
    
    # # 打印或处理点云数据
    # # 假设你要打印第一个点的坐标
    # for point in pc_data:
    #     x, y, z = point
    #     rospy.loginfo(f"Radar Point: x={x}, y={y}, z={z}")
    #     break  # 只打印一个点，移除此行可以打印所有点
    return

def main():
    global corner_params, lk_params

    # LK光流参数
    lk_params = dict(winSize=(15, 15), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
    
    # 角点检测参数
    corner_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)

    # 初始化 ROS 节点
    rospy.init_node('optical_flow_node', anonymous=True)

    # 订阅 /dvs/image_raw 和 /radar/data (PointCloud2)
    image_sub = message_filters.Subscriber('/dvs/image_raw', Image)
    radar_sub = message_filters.Subscriber('/radar/data', PointCloud2)

    # 同步消息（图像和雷达）
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, radar_sub], 1000, 0.3, allow_headerless=True)
    ts.registerCallback(lambda image_msg, radar_msg: (image_callback(image_msg), radar_callback(radar_msg)))

    # 循环等待回调
    rospy.spin()

if __name__ == "__main__":
    main()
