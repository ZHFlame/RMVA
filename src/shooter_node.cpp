#include <rclcpp/rclcpp.hpp>
#include "referee_pkg/srv/hit_armor.hpp"
#include <opencv2/opencv.hpp>
#include "shooter/shooter.h"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point.hpp>
#include "referee_pkg/msg/object.hpp"
#include "referee_pkg/msg/multi_object.hpp"
#include "referee_pkg/msg/race_stage.hpp"
#include <functional>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <referee_pkg/srv/detail/hit_armor__struct.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include "sensor_msgs/msg/image.hpp"
#include <eigen3/Eigen/Dense>

class shooter_node : public rclcpp::Node
{
public:
    shooter_node(const std::string& name) : rclcpp::Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "Node created");
        
        // 1. 创建击打服务接口
        armor_hit_service = this->create_service<referee_pkg::srv::HitArmor>(
            "/referee/hit_arror", std::bind(&shooter_node::oula_angle, this,
            std::placeholders::_1, std::placeholders::_2));

        // 2. 订阅图像话题，供视觉测量
        Image_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&shooter_node::img_process, this, std::placeholders::_1));
            
        // 3. 创建定时器，分别用于 EKF 更新和圆心拟合
        // EKF 负责预测，频率可以高一点
        timer_1 = this->create_wall_timer(std::chrono::milliseconds(10),
            std::bind(&shooter_node::EKF_callback, this));
        
        // 拟合圆心可以慢一点，或者直接在 EKF 中处理
        timer_2 = this->create_wall_timer(std::chrono::milliseconds(50),
            std::bind(&shooter_node::center_r_callback, this));

        // 3. 订阅裁判系统
        subscription_ = this->create_subscription<referee_pkg::msg::RaceStage>(
            "/referee/race_stage", 10,
            [this](const referee_pkg::msg::RaceStage::SharedPtr msg)
            {
                RCLCPP_INFO(this->get_logger(), "Received race stage %d",msg->stage);
            });
            
        // 5. 实例化 EKF，统一在 10ms 周期内更新
        ekf_ = std::make_shared<CorrectCircularMotionEKF>(0.01f); // dt = 10ms
    } 

private:
    double x0 = 0, y0 = 0, z0 = 0;
    float g = 9.8; 
    float v = 1.5; 

    // 存储最近的历史 3D 点 (相机坐标系)
    std::vector<cv::Point3f> object_3d_history;
    
    // 存储用于圆拟合的平面点 (X-Z平面)
    std::vector<cv::Point2f> xz_points;

    cv::Point3f center = cv::Point3f(0,0,0);
    float radius = 0.0f;
    float omega = 0.0f;

    // EKF 指针
    std::shared_ptr<CorrectCircularMotionEKF> ekf_;

    void img_process(sensor_msgs::msg::Image::SharedPtr msg);
    void EKF_callback(); 
    void center_r_callback();
    void oula_angle(
        const std::shared_ptr<referee_pkg::srv::HitArmor::Request> request,
        std::shared_ptr<referee_pkg::srv::HitArmor::Response> response);

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr Image_sub;
    rclcpp::Subscription<referee_pkg::msg::RaceStage>::SharedPtr subscription_;
    rclcpp::Service<referee_pkg::srv::HitArmor>::SharedPtr armor_hit_service;
    rclcpp::TimerBase::SharedPtr timer_1;
    rclcpp::TimerBase::SharedPtr timer_2;
};

//主函数
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<shooter_node>("shooter_node");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
/*
*geometry_msgs/Point[] modelpoint   # 模型的真值点 (左下起始,逆时针方向)
std_msgs/Header header		       # 时间戳
float64 g						   # 重力加速度
---
float64 yaw						   # 欧拉角
float64 pitch
float64 roll
 */
// ===============================
// 服务端：进行接收信息并且发布欧拉角
// ===============================
void shooter_node::oula_angle(const std::shared_ptr<referee_pkg::srv::HitArmor::Request> request,
    std::shared_ptr<referee_pkg::srv::HitArmor::Response> response)
{
    //0.日志，以确保自己接收到信息
    RCLCPP_INFO(this->get_logger(), "收到g: %lf, "
                                    "Point1(%lf,%lf,%lf),"
                                    "Point2(%lf,%lf,%lf),"
                                    "Point3(%lf,%lf,%lf),"
                                    "Point4(%lf,%lf,%lf)",
                request->g,
                request->modelpoint[0].x, request->modelpoint[0].y, request->modelpoint[0].z,
                request->modelpoint[1].x, request->modelpoint[1].y, request->modelpoint[1].z,
                request->modelpoint[2].x, request->modelpoint[2].y, request->modelpoint[2].z,
                request->modelpoint[3].x, request->modelpoint[3].y, request->modelpoint[3].z);
    /*double x0 = request->modelpoint[0].x+request->modelpoint[1].x+request->modelpoint[2].x+request->modelpoint[3].x;
    double y0 = request->modelpoint[0].y+request->modelpoint[1].y+request->modelpoint[2].y+request->modelpoint[3].y;
    double z0 = request->modelpoint[0].z+request->modelpoint[1].z+request->modelpoint[2].z+request->modelpoint[3].z;*/

    //1.异常处理
    if (object_3d_history.empty()) {
        response->yaw = response->pitch = response->roll = 0;
        return;
    }
    
    // 使用预测后的坐标 (x0, y0, z0) 进行解算
    // 注意：calculate 函数需要根据你的坐标系定义进行调整
    // 这里假设 x0, y0, z0 已经是预测好的击打点
    std::vector<double> oula_angles = calculate(x0, y0, z0, request->g); // 注意这里传参是否匹配
    
    response->yaw = oula_angles[0];
    response->pitch = oula_angles[1];
    response->roll = oula_angles[2];
    // 2.日志，确保自己发送了信息
    RCLCPP_INFO(this->get_logger(), "发送欧拉角: yaw=%lf, pitch=%lf, roll=%lf",
                response->yaw, response->pitch, response->roll);
}

// ================================
// 图像处理函数
// ================================
void shooter_node::img_process(sensor_msgs::msg::Image::SharedPtr msg)
{
    // 1. ROS 图像 -> OpenCV 格式
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat src = cv_ptr->image;
    if (src.empty()) return;

    // 2. 运行通用 detector 获取目标集合
    std::vector<Subject> subjects = detector(src);
    
    // 【新增】打印检测结果
    static int img_count = 0;
    if (++img_count % 30 == 0) { // 每秒打印一次（假设 30fps）
        RCLCPP_INFO(this->get_logger(), "Detected %zu subjects in image %d", subjects.size(), img_count);
    }
    
    bool found_armor = false;
    for (const auto& sub : subjects) {
        if (sub.name.find("armor") != std::string::npos && sub.points.size() == 4) {
            cv::Point3f p3d = transform_2d_3d(sub.points);
            
            // 【新增】打印 PnP 结果
            RCLCPP_INFO(this->get_logger(), "PnP: (%.3f, %.3f, %.3f)", p3d.x, p3d.y, p3d.z);
            
            if (std::isnan(p3d.x) || std::isnan(p3d.z)) {
                RCLCPP_WARN(this->get_logger(), "Invalid PnP result (NaN)");
                continue;
            }

            object_3d_history.push_back(p3d);
            if (object_3d_history.size() > 50) object_3d_history.erase(object_3d_history.begin());

            xz_points.push_back(cv::Point2f(p3d.x, p3d.z));
            if (xz_points.size() > 50) xz_points.erase(xz_points.begin());
            
            ekf_->update(cv::Point2f(p3d.x, p3d.z));
            found_armor = true;
            break;
        }
    }
    
    if (!found_armor && img_count % 30 == 0) {
        RCLCPP_WARN(this->get_logger(), "No valid armor detected");
    }
}

// ================================
// EKF 回调：获取状态并预测
// ================================
void shooter_node::EKF_callback()
{
    // 1. 定期打印，监控 EKF 状态
    static int call_count = 0;
    call_count++;
    
    // 每 100 次打印一次，证明函数在运行
    if (call_count % 100 == 0) {
        RCLCPP_INFO(this->get_logger(), "EKF_callback called %d times, history: %zu, radius: %.3f",
                    call_count, object_3d_history.size(), radius);
    }
    
    if (object_3d_history.empty()) return;
    if (radius < 0.01) return;

    cv::Point2f pos_xz = ekf_->getPosition();
    omega = ekf_->getOmega();
    radius = ekf_->getRadius();
    
    float current_y = object_3d_history.back().y;
    float dist = std::sqrt(pos_xz.x * pos_xz.x + pos_xz.y * pos_xz.y);
    float t_fly = dist / v; 

    float current_angle = std::atan2(pos_xz.x - center.x, pos_xz.y - center.z);
    float pred_angle = current_angle + omega * t_fly;

    // 【修正】预测位置使用圆心坐标
    x0 = center.x + radius * std::sin(pred_angle);
    y0 = current_y; 
    z0 = center.z + radius * std::cos(pred_angle);
    
    // 【新增】调试输出
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Predicted hit point: (%.3f, %.3f, %.3f), omega: %.3f rad/s",
                         x0, y0, z0, omega);
}

// ===============================
// 最小二乘法拟合圆心 (X-Z 平面)
// ===============================
void shooter_node::center_r_callback()
{
    // 1. 周期打印，确认拟合线程运行
    static int call_count = 0;
    call_count++;
    
    if (call_count % 20 == 0) { // 50ms * 20 = 1秒
        RCLCPP_INFO(this->get_logger(), "center_r_callback called %d times, points: %zu",
                    call_count, xz_points.size());
    }
    
    int n = xz_points.size();
    if (n < 10) return;

    // 2. 构建最小二乘方程 A·x=b，并解出圆参数
    // x^2 + z^2 = 2*A*x + 2*B*z + C
    // 其中圆心 (xc, zc) = (A, B)，r^2 = A^2 + B^2 - C
    
    Eigen::MatrixXf A(n, 3);
    Eigen::VectorXf b(n);
    
    for (int i = 0; i < n; i++) {
        float x = xz_points[i].x;
        float z = xz_points[i].y; // 注意：存储在 Point2f 的 y 分量
        
        A(i, 0) = 2.0f * x;
        A(i, 1) = 2.0f * z;
        A(i, 2) = 1.0f;
        b(i) = x * x + z * z;
    }
    
    // 求解 [A, B, C]^T
    Eigen::Vector3f res = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    
    center.x = res[0]; // A
    center.z = res[1]; // B
    center.y = 0; 
    
    float r_squared = res[0] * res[0] + res[1] * res[1] - res[2];
    if (r_squared > 0) {
        radius = std::sqrt(r_squared);
    } else {
        radius = 0.0f;
    }
    
    // 3. 根据拟合结果更新圆心、半径并记录日志
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Fitted Circle - Center: (%.3f, %.3f), Radius: %.3f, Omega: %.3f",
                         center.x, center.z, radius, omega);
}

