

// this is Surface of Active Events class
#ifndef SAE_MANAGER_H
#define SAE_MANAGER_H

#include <opencv2/opencv.hpp>
#include <dvs_msgs/Event.h>
#include <vector>



class SAE { 
public:
    using Ptr = std::shared_ptr<SAE>;

    SAE(){};

    SAE(int width, int height)
        : width_(width), height_(height) {
        sae_[0] = cv::Mat::zeros(height_, width_, CV_64F); // OFF
        sae_[1] = cv::Mat::zeros(height_, width_, CV_64F); // ON
    }

    // 添加事件（更新时间表面）
    void addEvent(const dvs_msgs::Event& e) {
        if (isInside(e.x, e.y))
            sae_[e.polarity].at<double>(e.y, e.x) = e.ts.toSec();
    }

    // 获取某像素最近事件时间
    double getTimestamp(int x, int y, bool polarity) const {
        if (!isInside(x, y)) return 0.0;
        return sae_[polarity].at<double>(y, x);
    }

    // 获取整张 SAE（0 = OFF, 1 = ON）
    const cv::Mat& getSAE(bool polarity) const {
        return sae_[polarity];
    }

    // 清空 SAE
    void clear() {
        sae_[0] = cv::Mat::zeros(height_, width_, CV_64F);
        sae_[1] = cv::Mat::zeros(height_, width_, CV_64F);
    }

    cv::Mat GetPostiveImg() {
        
        return sae_[1];
    }

    cv::Mat decaySAE(const cv::Mat& saeTimeSurface, double maxTimeSec = 2.0) {
        cv::Mat decayed(saeTimeSurface.size(), CV_64F);

        double latestTime = 0.0;
        double minTime = 1e15;
        // 假设 saeTimeSurface 里存的是事件时间戳，找最大时间（全局最新时间）
        // 你可以根据实际情况传入或计算最新时间，这里演示找最大值
        for (int y = 0; y < saeTimeSurface.rows; ++y) {
            for (int x = 0; x < saeTimeSurface.cols; ++x) {
                double t = saeTimeSurface.at<double>(y, x);
                if (t > latestTime) latestTime = t;
                if (t < minTime && t > 0) minTime = t;
            }
        }
        // LOG(ERROR) << "latestTime = " << latestTime << ", minTime = " << minTime << std::endl;
        for (int y = 0; y < saeTimeSurface.rows; ++y) {
            for (int x = 0; x < saeTimeSurface.cols; ++x) {
                double eventTime = saeTimeSurface.at<double>(y, x);
                if(eventTime < 0.1) // 特别的 没有事件
                {
                    decayed.at<double>(y, x) = 0.0;
                    continue;
                }
                double dt = latestTime - eventTime;
                // LOG(ERROR) << "latestTime = " << latestTime << std::endl;
                // LOG(ERROR) << "eventTime = " << eventTime << std::endl;
                if (dt < 0 || dt > maxTimeSec) {
                    decayed.at<double>(y, x) = 0.0;  // 超过2秒或未来事件置0
                } else {
                    // 指数衰减，衰减速率根据 maxTimeSec 调节
                    double val = std::exp(-dt / maxTimeSec);
                    // LOG(ERROR) << "dt = " << dt << std::endl;
                    // LOG(ERROR) << "maxTimeSec = " << maxTimeSec << std::endl;
                    // LOG(ERROR) << "val = " << val << std::endl;
                    decayed.at<double>(y, x) = val;
                }
            }
        }

        // 归一化到 0-255 8位图
        cv::Mat decayedNormalized;
        decayed.convertTo(decayedNormalized, CV_8U, 255.0);

        // static long int img_index2 = 0;
        // cv::imwrite("/home/hao/Desktop/twist_ws/src/TwistEstimator/output/decay_" 
        //     + std::to_string(img_index2++) + ".png", decayedNormalized);

        // return decayedNormalized;
        return decayed;
    }

private:
    bool isInside(int x, int y) const {
        return x >= 0 && x < width_ && y >= 0 && y < height_;
    }

    int width_, height_;
    cv::Mat sae_[2]; // 0: OFF, 1: ON
};

#endif // SAE_MANAGER_H
