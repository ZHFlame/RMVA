//
// Created by Liuzhenqi on 2025/11/26.
//
#ifndef SHOOTER_ARMOR_H
#define SHOOTER_ARMOR_H
#include <opencv2/opencv.hpp>
#include <vector>

// 【新增】声明全局弹速变量，确保多文件共享
extern float v; 

// 【修改】calculate 接受 3D 坐标
std::vector<double> calculate(float x, float y, float z, double g);

cv::Point3f transform_2d_3d(const std::vector<cv::Point2f>& points); // 注意这里修正了参数类型
struct Subject{
    std::string name;
    std::vector<cv::Point2f> points;
    int number=0;
    int id=0;
    std::string color_type="none";
    cv::Scalar color_bgr=cv::Scalar(0,0,0);
};
//EKF类
class CorrectCircularMotionEKF
{
public:
    CorrectCircularMotionEKF(float dt = 0.1f) : dt_(dt) {
        initialize();
    }
    //更新函数
    void update(const cv::Point2f& measurement) {
        cv::Mat meas = (cv::Mat_<float>(2, 1) << measurement.x, measurement.y);

        if (is_initialized_) {
            predict();
            correct(meas);
        } else {
            initialize(meas);
        }
    }
    //预测位置
    cv::Point2f getPosition() const {
        return cv::Point2f(state_.at<float>(0), state_.at<float>(1));
    }
    //线速度估计
    cv::Point2f getVelocity() const {
        return cv::Point2f(state_.at<float>(2), state_.at<float>(3));
    }
    //角速度估计
    float getOmega() const {
        return state_.at<float>(4);
    }

    // 判断旋转方向
    std::string getRotationDirection() const {
        float omega = getOmega();
        if (omega > 0.01f) return "逆时针";
        else if (omega < -0.01f) return "顺时针";
        else return "静止或直线运动";
    }

    // 计算运动半径
    float getRadius() const {
        return std::sqrt(state_.at<float>(0)*state_.at<float>(0) +
                        state_.at<float>(1)*state_.at<float>(1));
    }

private:
    void initialize();
    void initialize(const cv::Mat& first_measurement);
    void predict();
    void correct(const cv::Mat& measurement);
    cv::Mat computeJacobian();
    float dt_;
    cv::Mat state_, covariance_, processNoise_, measurementMatrix_;
    bool is_initialized_;
};
std::vector<cv::Point2f> Armor_Detector(cv::Mat img_input);
std::vector<struct Subject> detector(cv::Mat &src);
std::vector<Subject> dynamic(cv::Mat &src);
std::vector<cv::Point2f> Sphere_Processer(const std::vector<cv::Point> &contour);
int digit_detector(const cv::Mat& src);
#endif
