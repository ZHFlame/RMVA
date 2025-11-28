#include "vision/vision.h"
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point.hpp>
#include "referee_pkg/msg/object.hpp"
#include "referee_pkg/msg/multi_object.hpp"
#include "referee_pkg/msg/race_stage.hpp"
#include <iostream>
#include <memory>
#include <functional>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include "sensor_msgs/msg/image.hpp"
#include <string>
/*
 *定义类
 *图片转换
 *定义一个subjects来接收detector
 *通过接收的信息来发布消息（可以新建一个函数）
 *启动节点
 */
//定义类
class vision_node : public rclcpp::Node {
public:
    vision_node(const std::string& name) : Node(name) {
        RCLCPP_INFO(this->get_logger(), "Initializing vision_node");

        Image_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            std::bind(&vision_node::callback, this, std::placeholders::_1));

        subscription_ = this->create_subscription<referee_pkg::msg::RaceStage>(
            "/referee/race_stage", 10,
            [this](const referee_pkg::msg::RaceStage::SharedPtr msg) {
                // RaceStage message fields may vary; avoid accessing unknown field names here
                RCLCPP_INFO(this->get_logger(), "Received race stage %d",msg->stage);
            });

        Target_pub = this->create_publisher<referee_pkg::msg::MultiObject>(
            "/vision/target", 10);

        cv::namedWindow("Detection Result", cv::WINDOW_AUTOSIZE);

        RCLCPP_INFO(this->get_logger(), "vision_node initialized successfully");
    }

    ~vision_node() { cv::destroyWindow("Detection Result"); }

private:
    void callback(sensor_msgs::msg::Image::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr Image_sub;
    rclcpp::Subscription<referee_pkg::msg::RaceStage>::SharedPtr subscription_;
    rclcpp::Publisher<referee_pkg::msg::MultiObject>::SharedPtr Target_pub;
};
//主函数
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    const auto node = std::make_shared<vision_node>("vision_node");
    RCLCPP_INFO(node->get_logger(), "Starting vision_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
//类函数callback的实现
void vision_node::callback(sensor_msgs::msg::Image::SharedPtr msg)
{try
{
    //定义消息
    referee_pkg::msg::MultiObject msg_object;
    msg_object.header = msg->header;
    //导入图片
    cv_bridge::CvImagePtr cv_ptr;

    if (msg->encoding == "rgb8" || msg->encoding == "R8G8B8") {
        cv::Mat src(msg->height, msg->width, CV_8UC3,
                      const_cast<unsigned char *>(msg->data.data()));
        cv::Mat bgr_image;
        cv::cvtColor(src, bgr_image, cv::COLOR_RGB2BGR);
        cv_ptr = std::make_shared<cv_bridge::CvImage>();
        cv_ptr->header = msg->header;
        cv_ptr->encoding = "bgr8";
        cv_ptr->image = bgr_image;
    } else {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }

    cv::Mat src = cv_ptr->image;

    if (src.empty())
    {
        RCLCPP_WARN(this->get_logger(), "Received empty image");
        return;
    }
    ///调用detector
    std::vector<Subject> subjects;
    subjects.clear();
    subjects = detector(src);
    cv::imshow("Detection Result", src);
    cv::waitKey(1);
    RCLCPP_INFO(this->get_logger(), "Detected %d subjects", static_cast<int>(subjects.size()));
    if (subjects.empty()) {
        RCLCPP_WARN(this->get_logger(), "No subjects detected");
    }
    //for循环
    for (auto &subject : subjects)
    {
        referee_pkg::msg::Object obj;
        obj.target_type = subject.name;
        std::cout<<"Subject Name: "<<subject.name<<std::endl;
        // convert cv::Point2f -> geometry_msgs::msg::Point
        obj.corners.clear();
        for (const auto &p : subject.points) {
            geometry_msgs::msg::Point gp;
            gp.x = p.x;
            gp.y = p.y;
            gp.z = 0.0;
            obj.corners.push_back(gp);
        }
        msg_object.objects.push_back(obj);
        //msg_object.num_objects = subject.id;
        for (auto point : subject.points)
        {
            RCLCPP_INFO(this->get_logger(),
                        "%s : %d, Point: (%.1f, %.1f)",
                        subject.name.c_str(),subject.id,
                        point.x, point.y);
        }
        msg_object.num_objects = static_cast<int>(msg_object.objects.size());
    }
    Target_pub->publish(msg_object);
    RCLCPP_INFO(this->get_logger(), "Published %d targets",
                msg_object.num_objects);
}
    catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in callback: %s", e.what());
        return;
    }
}