//
// Created by LyChee on 2025/11/25.
//

#include "vision.h"
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <iostream>
using namespace cv;
using namespace std;
// 将模板存储和加载标志移至函数作用域的顶部，确保只初始化一次
static std::map<int, Mat> templates;
static bool templates_loaded = false;
int digit_detector(const Mat& src) {
    // 1. 加载预训练的ONNX模型
    std::string modelPath = "mnist-12.onnx"; // 替换为您的模型路径
    cv::dnn::Net net = cv::dnn::readNetFromONNX(modelPath);

    // 可选：启用CUDA加速（需OpenCV编译了CUDA支持）
    // net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    // net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

    // 2. 读取并预处理图
    Mat image;
    cvtColor(src, image, cv::COLOR_RGB2GRAY);
    if (image.empty()) {
        std::cerr << "Could not read the image." << std::endl;
        return -1;
    }

    // 调整尺寸至模型期望的输入（例如MNIST常用28x28）
    cv::Mat resizedImage;
    cv::resize(image, resizedImage, cv::Size(28, 28));

    // 将图像转换为Blob（归一化、调整通道等）
    cv::Mat blob = cv::dnn::blobFromImage(resizedImage, 1.0/255.0, cv::Size(28, 28), cv::Scalar(0,0,0), false, CV_32F);

    // 3. 执行推理
    net.setInput(blob);
    cv::Mat output = net.forward();

    // 4. 解析输出结果
    // 输出通常是一个概率向量，取最大概率的索引作为预测数字
    // 4. 解析输出结果
    cv::Point classIdPoint;
    double confidence;
    cv::minMaxLoc(output.reshape(1, 1), 0, &confidence, 0, &classIdPoint);
    int predictedDigit = classIdPoint.x;

    // 设置一个置信度阈值，例如0.8（这个值需要根据你的模型和需求调整）
    double confidence_threshold = 0.8;

    // 先判断置信度是否达标，再判断数字范围
    if (confidence > confidence_threshold) {
        std::cout << "Predicted digit: " << predictedDigit << " with confidence: " << confidence << std::endl;
        return predictedDigit;
    }else return -1;


}