#include <cmath>
#include <vector>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "shooter.h"

using namespace cv;

// 【定义】全局弹速变量 (单位: m/s)
float v = 1.50f; 

// ========================================================
// calculate 函数：计算云台需要的 Yaw 和 Pitch
// 输入: 目标在相机坐标系下的坐标 (x, y, z)
//       x: 右为正, y: 下为正, z: 前为正
// 输出: [yaw, pitch, roll] (单位: 弧度)
// ========================================================
std::vector<double> calculate(float x, float y, float z, double g)
{
    std::vector<double> angle(3, 0.0);

    // 1. 计算水平距离 (xz平面)
    double dist_xz = std::sqrt(x * x + z * z);
    if (dist_xz < 1e-6) return angle; // 避免除零

    // 2. 计算 Yaw (偏航角)
    // 在相机坐标系中，Yaw 是 x 和 z 的夹角
    // atan2(x, z) 返回的是以 z 轴为 0 度，向 x 轴旋转的角度
    angle[0] = std::atan2(x, z); 

    // 3. 计算 Pitch (俯仰角)
    // 使用抛物线方程补偿重力下坠
    // 设抛射角为 theta (Pitch)
    // y_target = -y (因为相机坐标系 y 向下为正，而物理公式通常 y 向上为正)
    // 公式: y = x * tan(theta) - (g * x^2) / (2 * v^2 * cos^2(theta))
    // 令 a = g*x^2 / (2*v^2), y_real = -y
    // y_real = x * tan(theta) - a * (1 + tan^2(theta))
    // a * tan^2(theta) - x * tan(theta) + (a + y_real) = 0
    // 解一元二次方程求 tan(theta)
    
    double y_real = -y; // 转换为向上为正的高度
    double a = (g * dist_xz * dist_xz) / (2.0 * v * v);
    
    // 方程: a * t^2 - dist_xz * t + (a - y_real) = 0
    // 判别式 delta = b^2 - 4ac
    double b = -dist_xz;
    double c = a - y_real;
    double delta = b * b - 4 * a * c;

    if (delta >= 0) {
        // 有解，取较小的角度（直射，弹道低）
        double t = (-b - std::sqrt(delta)) / (2 * a);
        angle[1] = std::atan(t);
    } else {
        // 无解（目标太远或弹速太低），使用最大射程角（45度）或直接指向目标
        // 这里退化为直接指向目标
        angle[1] = std::atan2(y_real, dist_xz);
    }

    // Roll 通常为 0
    angle[2] = 0;

    return angle;
}

// ======================
// 利用PnP来转化为三维的坐标
// ======================
cv::Point3f transform_2d_3d(const std::vector<cv::Point2f>& points)
{
    // 0. 内参矩阵 (建议从配置文件读取)
    Mat cameraMatrix = (Mat_<double>(3, 3) <<
           554.383, 0.0,     320.0,
           0.0,     554.383, 320.0,
           0.0,     0.0,     1.0
       );
    Mat distCoeffs = (Mat_<double>(1, 5) << 0.0, 0.0, 0.0, 0.0, 0.0);

    // 1. 定义世界坐标系下的装甲板点 (单位：米)
    // 用户指定尺寸：0.705m x 0.230m
    float half_w = 0.705f / 2.0f;
    float half_h = 0.230f / 2.0f;
    
    std::vector<cv::Point3f> objectpoints;
    // 输入点序: 左下、右下、右上、左上
    // 对应 3D 坐标 (建立物体坐标系，中心为原点，X右Y下Z前)
    
    // 左下 (Bottom-Left) -> (-w/2, h/2)
    objectpoints.push_back(Point3f(-half_w,  half_h, 0)); 
    // 右下 (Bottom-Right) -> (w/2, h/2)
    objectpoints.push_back(Point3f( half_w,  half_h, 0)); 
    // 右上 (Top-Right) -> (w/2, -h/2)
    objectpoints.push_back(Point3f( half_w, -half_h, 0)); 
    // 左上 (Top-Left) -> (-w/2, -h/2)
    objectpoints.push_back(Point3f(-half_w, -half_h, 0)); 

    // 2. 检查输入点数
    if (points.size() != 4) return cv::Point3f(NAN, NAN, NAN);

    // 3. PnP 解算
    Mat rvec, tvec;
    bool success = solvePnP(objectpoints, points, cameraMatrix,
                           distCoeffs, rvec, tvec, false, SOLVEPNP_IPPE); // IPPE 对平面物体更稳

    if (!success) {
        return cv::Point3f(NAN, NAN, NAN);
    }

    // tvec 就是相机坐标系下的坐标 (x, y, z)
    // x: 右, y: 下, z: 前
    return cv::Point3f(
        (float)tvec.at<double>(0),
        (float)tvec.at<double>(1),
        (float)tvec.at<double>(2)
    );
}

// ==================================
// CorrectCircularMotionEKF中函数介绍
// ==================================
//1. 初始化
    void  CorrectCircularMotionEKF::initialize() {
        state_ = cv::Mat::zeros(5, 1, CV_32F);
        covariance_ = cv::Mat::eye(5, 5, CV_32F) * 1000.0f;

        // 过程噪声设置
        processNoise_ = cv::Mat::zeros(5, 5, CV_32F);
        float noise_std[] = {0.1f, 0.1f, 0.5f, 0.5f, 0.01f};
        for (int i = 0; i < 5; ++i) {
            processNoise_.at<float>(i, i) = noise_std[i] * noise_std[i];
        }

        measurementMatrix_ = (cv::Mat_<float>(2, 5) <<
            1, 0, 0, 0, 0,
            0, 1, 0, 0, 0);

        is_initialized_ = false;
    }
    //2. 重载
    void  CorrectCircularMotionEKF::initialize(const cv::Mat& first_measurement) {
        state_.at<float>(0) = first_measurement.at<float>(0);
        state_.at<float>(1) = first_measurement.at<float>(1);
        state_.at<float>(4) = 0.1f; // 初始猜测：逆时针

        is_initialized_ = true;
    }
    //3. 预测函数
    void CorrectCircularMotionEKF::predict() {
        float x = state_.at<float>(0);
        float y = state_.at<float>(1);
        float vx = state_.at<float>(2);
        float vy = state_.at<float>(3);
        float omega = state_.at<float>(4);

        float cos_wdt = std::cos(omega * dt_);
        float sin_wdt = std::sin(omega * dt_);

        // 修正的状态转移 - 使用旋转矩阵
        cv::Mat new_state = (cv::Mat_<float>(5, 1) <<
            x * cos_wdt - y * sin_wdt,     // x' = x*cos - y*sin
            x * sin_wdt + y * cos_wdt,     // y' = x*sin + y*cos
            -omega * y,                    // vx = -ωy (从运动方程推导)
            omega * x,                     // vy = ωx
            omega                          // ω保持不变
        );

        state_ = new_state;

        cv::Mat F = computeJacobian();
        covariance_ = F * covariance_ * F.t() + processNoise_;
    }
    //4. 纠正函数
    void CorrectCircularMotionEKF::correct(const cv::Mat& measurement) {
        cv::Mat y = measurement - measurementMatrix_ * state_;
        cv::Mat S = measurementMatrix_ * covariance_ * measurementMatrix_.t() +
                   cv::Mat::eye(2, 2, CV_32F) * 4.0f;

        cv::Mat K = covariance_ * measurementMatrix_.t() * S.inv();
        state_ = state_ + K * y;

        cv::Mat I = cv::Mat::eye(5, 5, CV_32F);
        covariance_ = (I - K * measurementMatrix_) * covariance_;
    }
    //5. 计算雅比达矩阵
    cv::Mat CorrectCircularMotionEKF::computeJacobian() {
        float x = state_.at<float>(0);
        float y = state_.at<float>(1);
        float vx = state_.at<float>(2);
        float vy = state_.at<float>(3);
        float omega = state_.at<float>(4);

        float cos_wdt = std::cos(omega * dt_);
        float sin_wdt = std::sin(omega * dt_);

        cv::Mat F = cv::Mat::eye(5, 5, CV_32F);

        // 位置关于状态的偏导
        F.at<float>(0, 0) = cos_wdt;
        F.at<float>(0, 1) = -sin_wdt;
        F.at<float>(0, 4) = (-x * sin_wdt - y * cos_wdt) * dt_;

        F.at<float>(1, 0) = sin_wdt;
        F.at<float>(1, 1) = cos_wdt;
        F.at<float>(1, 4) = (x * cos_wdt - y * sin_wdt) * dt_;

        // 速度关于状态的偏导（从运动方程推导）
        F.at<float>(2, 1) = -omega;  // ∂vx/∂y = -ω
        F.at<float>(2, 4) = -y;      // ∂vx/∂ω = -y

        F.at<float>(3, 0) = omega;   // ∂vy/∂x = ω
        F.at<float>(3, 4) = x;       // ∂vy/∂ω = x

        return F;
    }


