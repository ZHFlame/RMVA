//
// Created by LyChee on 2025/11/13.
//
#include <vector>
#include "vision.h"
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

vector<Point2f> Sphere_Processer(const vector<Point> &contour) {
    // 1. 通过包围矩形估计轮廓尺寸
    Rect Rectangle_prame = boundingRect(contour);
    vector<Point2f> points;
    // 2. 按“左-下-右-上”的次序输出关键点
    points.push_back(Point2f(Rectangle_prame.x, Rectangle_prame.y + Rectangle_prame.height / 2)); // 左
    points.push_back(Point2f(Rectangle_prame.x + Rectangle_prame.width / 2, Rectangle_prame.y + Rectangle_prame.height)); // 下
    points.push_back(Point2f(Rectangle_prame.x + Rectangle_prame.width, Rectangle_prame.y + Rectangle_prame.height / 2)); // 右
    points.push_back(Point2f(Rectangle_prame.x + Rectangle_prame.width / 2, Rectangle_prame.y)); // 上
    // 3. 返回顺时针排序后的四点，供后续 PnP 使用
    return points;
}
