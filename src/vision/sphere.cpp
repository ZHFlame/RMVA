//
// Created by LyChee on 2025/11/13.
//
#include <vector>
#include "vision.h"
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

/*
vector<Point2f> calculateStableSpherePoints(const Point2f &center,
                                                      float radius) {
    vector<Point2f> points;

    // 简单稳定的几何计算，避免漂移
    // 左、下、右、上
    points.push_back(Point2f(center.x - radius, center.y));  // 左点 (1)
    points.push_back(Point2f(center.x, center.y + radius));  // 下点 (2)
    points.push_back(Point2f(center.x + radius, center.y));  // 右点 (3)
    points.push_back(Point2f(center.x, center.y - radius));  // 上点 (4)

    return points;
}

vector<Point2f> sphere(Point2f center,float radius,Mat &result_image) {
    vector<Point2f> sphere_points =
            calculateStableSpherePoints(center, radius);

    // 绘制检测到的球体
    cv::circle(result_image, center, static_cast<int>(radius),
               cv::Scalar(0, 255, 0), 2);  // 绿色圆圈
    cv::circle(result_image, center, 3, cv::Scalar(0, 0, 255),
               -1);  // 红色圆心

    // 显示半径信息
    string info_text = "R:" + to_string((int)radius);
    cv::putText(
        result_image, info_text, cv::Point(center.x - 15, center.y + 5),
        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 2);

    return sphere_points;
}*/

    /*vector<Point2f> Sphere_Processer(const vector<Point> &contour) {
        //float radius = (ellipse.size.width + ellipse.size.height) / 4.0;

        RotatedRect ellipse = fitEllipse(contour);
        Point2f center = ellipse.center;
        float a = ellipse.size.width / 2.0f;  // 长轴半径
        float b = ellipse.size.height / 2.0f; // 短轴半径
        float angle = ellipse.angle * CV_PI / 180.0f; // 角度转弧度

        vector<Point2f> points(4);
        // 右（长轴正方向）
        points[0] = Point2f(center.x + a * cos(angle), center.y + a * sin(angle));
        // 下（短轴正方向）
        points[1] = Point2f(center.x - b * sin(angle), center.y + b * cos(angle));
        // 左（长轴负方向）
        points[2] = Point2f(center.x - a * cos(angle), center.y - a * sin(angle));
        // 上（短轴负方向）
        points[3] = Point2f(center.x + b * sin(angle), center.y - b * cos(angle));
        return points;
}*/

// cpp
vector<Point2f> Sphere_Processer(const vector<Point> &contour) {
    Rect Rectangle_prame = boundingRect(contour);
    vector<Point2f> points;
    points.push_back(Point2f(Rectangle_prame.x, Rectangle_prame.y + Rectangle_prame.height / 2)); // 左
    points.push_back(Point2f(Rectangle_prame.x + Rectangle_prame.width / 2, Rectangle_prame.y + Rectangle_prame.height)); // 下
    points.push_back(Point2f(Rectangle_prame.x + Rectangle_prame.width, Rectangle_prame.y + Rectangle_prame.height / 2)); // 右
    points.push_back(Point2f(Rectangle_prame.x + Rectangle_prame.width / 2, Rectangle_prame.y)); //
    return points;
}
