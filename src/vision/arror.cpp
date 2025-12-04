#include <algorithm>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <map>
#include <opencv4/opencv2/core/matx.hpp>
#include <opencv4/opencv2/core/types.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/stitching/detail/matchers.hpp>
#include <vector>
#include "vision.h"
using namespace std;
using namespace cv;

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
    return {points[1], points[3], points[2], points[0]};
}

vector<Subject> arror_detector(Mat &imgsource) {
    cout<<"arror_detector Begin"<<endl;
    // ============================================================
    // 0. 复制输入图像并准备工作容器
    // ============================================================
    Mat src = imgsource.clone();
    vector<Subject> subjects;

    // ============================================================
    // 1. 多通道边缘融合预处理
    // ============================================================
    GaussianBlur(src, src, Size(5, 5), 0);
    
    // 2. 构建双区间红色掩码并进行形态学滤波
    cv::Scalar lower_red1(0, 160, 100);
    cv::Scalar upper_red1(10, 255, 255);
    cv::Scalar lower_red2(160, 160, 100);
    cv::Scalar upper_red2(180, 255, 255);
    Mat srcBlur;
    cvtColor(src, srcBlur, COLOR_BGR2HSV);
    // 在 HSV 图上创建两个掩码并合并
    cv::Mat mask1, mask2;
    cv::inRange(srcBlur, lower_red1, upper_red1, mask1);
    cv::inRange(srcBlur, lower_red2, upper_red2, mask2);
    cv::Mat mask = mask1 | mask2;
    dilate(mask, mask, cv::getStructuringElement(cv::MORPH_ELLIPSE, {5,5}));
    // 形态学去噪
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_ELLIPSE, {5,5}));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, {3,3}));
    
    // 仅保留原图中的红色部分
    cv::Mat withmask;
    cv::bitwise_and(src, src, withmask, mask);
    Mat grey;
    cvtColor(withmask, grey, COLOR_BGR2GRAY);
    Mat imgcanny;
    Canny(grey, imgcanny, 40, 120);
    Mat kernel = getStructuringElement(MORPH_RECT,Size(3,3));
    Mat imgDil;
    dilate(imgcanny,imgDil,kernel);
    // ============================================================
    // 2. 提取轮廓作为候选目标
    // ============================================================
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(imgDil, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    cout<<contours.size()<<endl;
    vector<vector<Point>> approxPoints(contours.size());
    vector<vector<Point>> Hull(contours.size());
    vector<RotatedRect> box(contours.size());
    Point2f rectpoints[4];
    for(size_t i=0;i<contours.size();i++){
        float peri = arcLength(contours[i], true);
        approxPolyDP(contours[i],approxPoints[i],0.02*peri,true);
    }
    for(size_t i=0;i<contours.size();i++){
        convexHull(approxPoints[i], Hull[i]);
    }
    cout<<"appNum"<<approxPoints.size()<<endl;
    cout<<"hullNUm"<<Hull.size()<<endl;
    for (size_t i = 0; i < contours.size(); i++) {
    cout<<"appNumi"<<approxPoints[i].size()<<endl;
    cout<<"hullNUmi"<<Hull[i].size()<<endl;
        //if((approxPoints[i].size()-Hull[i].size())==2){
            Rect bbox = boundingRect(contours[i]);
            box[i] = minAreaRect(Hull[i]);
            rectangle(src,bbox,Scalar(0,0,255),2);
            box[i].points(rectpoints);
            vector<Point2f> arrorPoints;
            for(int j=0;j<4;j++){
                arrorPoints.push_back(rectpoints[j]);
                line(src, rectpoints[j], rectpoints[(j+1)%4], Scalar(0, 0, 255), 2, 8);
            }
            for(auto point:arrorPoints){
                cout<<point<<endl;
            }
            Subject arror;
            
            arror.name="arrow";
            vector<Point2f> resultPoints;
            resultPoints = sortPoints(arrorPoints);
            for (auto point:resultPoints) {
                cout<<point<<endl;
                arror.points.push_back(point);
            }
            subjects.push_back(arror);
        //}
    }
    imshow("src",src);
    cout<<"arror.cpp,subjectnum:"<<subjects.size()<<endl;
    return subjects;

}