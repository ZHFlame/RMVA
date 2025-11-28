#include <cstdlib>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "vision.h"
// 将模板存储和加载标志移至函数作用域的顶部，确保只初始化一次
static std::map<int, Mat> templates;
static bool templates_loaded = false;
std::vector<cv::Point2f> sortPoints(std::vector<cv::Point2f> points) {
    if (points.size() != 4) return points;
    // 1. 按 Y 坐标排序，区分上下
    std::sort(points.begin(), points.end(), [](const cv::Point& a, const cv::Point& b) {
        return a.y < b.y;
    });
    // 2. 分别按 X 坐标排序，获取“左-右”顺序
    std::sort(points.begin(), points.begin() + 2, [](const cv::Point& a, const cv::Point& b) {
        return a.x < b.x;
    });
    std::sort(points.begin() + 2, points.end(), [](const cv::Point& a, const cv::Point& b) {
        return a.x < b.x;
    });
    // 现在顺序是：左上0、右上1、左下2、右下3
    // 调整为：左下2 右下3 右上1 左上0
    return {points[2], points[3], points[1], points[0]};
}
std::vector<cv::Point2f> Ellipse(cv::Mat binary/*,cv::Mat draw_bg*/) {
    std::vector<std::vector<cv::Point>> contours;
    findContours(binary, contours,cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<cv::Point2f> Points;
    for (auto contour : contours) {
        if (contour.size() >5) {
            // 1. 拟合旋转椭圆并解析长轴
            cv::RotatedRect ellipse = fitEllipse(contour);

            // --- 获取椭圆顶点的新方法 ---
            // 1. 获取旋转矩形的四个顶点
            cv::Point2f rect_points[4];
            ellipse.points(rect_points);

            // 2. 找出代表长轴的两个点
            // fitEllipse 返回的矩形中，width 和 height 的关系不固定
            // 我们需要比较相邻边的长度来确定哪条边对应长轴

            cv::Point2f top_point, bottom_point;

            // 计算第一条边 (0-1) 和第二条边 (1-2) 的长度平方
            float d1 = pow(rect_points[0].x - rect_points[1].x, 2) + pow(rect_points[0].y - rect_points[1].y, 2);
            float d2 = pow(rect_points[1].x - rect_points[2].x, 2) + pow(rect_points[1].y - rect_points[2].y, 2);

            if (d1 > d2) {
                // d2 是长边，说明 1-2 和 3-0 是长边
                // 取 0-1 的中点和 2-3 的中点作为椭圆长轴顶点
                top_point = (rect_points[1] + rect_points[2]) * 0.5f;
                bottom_point = (rect_points[3] + rect_points[0]) * 0.5f;
            } else {
                // d1 是长边，说明 0-1 和 2-3 是长边
                // 取 1-2 的中点和 3-0 的中点
                top_point = (rect_points[0] + rect_points[1]) * 0.5f;
                bottom_point = (rect_points[2] + rect_points[3]) * 0.5f;
            }

            // 2. 取长轴中点，得到灯条上下端
            Points.push_back(top_point);
            Points.push_back(bottom_point);
        }
    }
    // 3. 统一点顺序，保障下游计算的稳定性
    Points = sortPoints(Points);
    return Points;
}

// 注意：请记得在 vision.h 中同步修改函数声明为：
// std::pair<std::vector<cv::Point2f>, int> Armor_Detector(cv::Mat img_input);

std::pair<std::vector<cv::Point2f>, int> Armor_Detector(cv::Mat img_input)
{
    // 0. 懒加载数字模板，防止重复 I/O
    if (!templates_loaded) {
        for (int idx = 1; idx <= 5; ++idx) {
            std::string template_path = "./src/challenge/src/vision/templates/" + std::to_string(idx) + ".png";
            Mat tmp = imread(template_path, IMREAD_GRAYSCALE);
            if (tmp.empty()) {
                template_path = "../templates/" + std::to_string(idx) + ".png";
                tmp = imread(template_path, IMREAD_GRAYSCALE);
            }
            if (tmp.empty()) {
                std::cerr << "无法加载模板: " << template_path << "，请检查路径或使用绝对路径\n";
                continue;
            }
            templates[idx] = tmp;
        }
        templates_loaded = true;
    }

    // 1. 转换 HSV 空间，便于颜色分割
    cv::Mat img_hsv;
    cv::cvtColor(img_input, img_hsv, cv::COLOR_BGR2HSV);

    // 2. 构建双区间红色掩码并进行形态学滤波
    cv::Scalar lower_red1(0, 160, 100);
    cv::Scalar upper_red1(10, 255, 255);
    cv::Scalar lower_red2(160, 160, 100);
    cv::Scalar upper_red2(180, 255, 255);

    // 在 HSV 图上创建两个掩码并合并
    cv::Mat mask1, mask2;
    cv::inRange(img_hsv, lower_red1, upper_red1, mask1);
    cv::inRange(img_hsv, lower_red2, upper_red2, mask2);
    cv::Mat mask = mask1 | mask2;
    dilate(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, {5,5}));
    // 形态学去噪
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_ELLIPSE, {5,5}));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, {3,3}));
    
    // 仅保留原图中的红色部分
    cv::Mat withmask;
    cv::bitwise_and(img_input, img_input, withmask, mask);
    
    // 3. 提取椭圆灯条顶点并做安全检查
    std::vector<cv::Point2f> points = Ellipse(mask/*,img_input*/);
    if(points.size()!=4){
        return {{}, -1};
    }

    // 4. 根据灯条推算四个装甲角点
    float x1,x2,y1,y2;
    x1=points[0].x - points[3].x;
    x2=points[1].x - points[2].x;
    y1=points[0].y - points[3].y;
    y2=points[1].y - points[2].y;

    std::vector<cv::Point2f> armor_points;
    armor_points.emplace_back(points[0].x + x1 * 0.5f, points[0].y + y1 * 0.5f);
    armor_points.emplace_back(points[1].x + x2 * 0.5f, points[1].y + y2 * 0.5f);
    armor_points.emplace_back(points[2].x - x2 * 0.5f, points[2].y - y2 * 0.5f);
    armor_points.emplace_back(points[3].x - x1 * 0.5f, points[3].y - y1 * 0.5f);

    // 5. 透视展开 ROI，获得统一尺寸的数字贴纸
    Mat imgCopy;
    img_input.copyTo(imgCopy);
    
    Mat matrix,imgWarp;
    matrix = getPerspectiveTransform(armor_points, std::vector<Point2f>{
        Point2f(0,193),
        Point2f(255, 193),
        Point2f(255, 0),
        Point2f(0, 0)
    });
    warpPerspective(imgCopy, imgWarp, matrix, Size(255,193));
    

    // 6. 模板匹配识别装甲数字
    Mat imgWarpGray, imgWarpBinary;
    cvtColor(imgWarp, imgWarpGray, COLOR_BGR2GRAY);
    threshold(imgWarpGray, imgWarpBinary, 128, 255, THRESH_BINARY); 
    double max_score = -1.0;
    int best_match_digit = -1;
    for (const auto &kv : templates) {
        if (kv.second.empty()) continue;//跳过未加载的模板
        Mat resized_roi;
        resize(imgWarpBinary, resized_roi, kv.second.size());
        Mat result;
        matchTemplate(resized_roi, kv.second, result, TM_CCOEFF_NORMED);
        double minVal, maxVal;
        minMaxLoc(result, &minVal, &maxVal);// 获取匹配分数
        if (maxVal > max_score) {
            max_score = maxVal;
            best_match_digit = kv.first;
        }
    }
    if(max_score < 0.5){
        best_match_digit = -1;
    }
    // 7. 返回识别顶点与数字编号
    return {points, best_match_digit};
}
