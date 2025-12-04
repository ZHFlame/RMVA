#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <map>
#include <vector>
#include "vision.h"
using namespace std;
using namespace cv;
vector<Subject> ring_detector(Mat &imgsource) {


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
    /*Mat imgcanny;
    Canny(grey, imgcanny, 40, 120);
    Mat kernel = getStructuringElement(MORPH_RECT,Size(3,3));
    Mat imgDil;
    dilate(imgcanny,imgDil,kernel);*/
    // ============================================================
    // 2. 提取轮廓作为候选目标
    // ============================================================
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    findContours(grey, contours, hierarchy,RETR_TREE, CHAIN_APPROX_NONE);
    cout<<contours.size()<<endl;

    for (size_t i = 0; i < contours.size(); i++) {
        // 遍历所有contour
        double area = cv::contourArea(contours[i]);
        if (area < 100) continue;


        /*** 以下为圆形处理 ***/
        // 计算最小外接圆
        Point2f center;
        float radius = 0;
        minEnclosingCircle(contours[i], center, radius);
        //minEnclosingCircle(approx, center, radius);
        // 计算圆形度
        double perimeter = arcLength(contours[i], true);
        double circularity = 4 * CV_PI * area / (perimeter * perimeter);
        cout<<"circularity:"<<circularity<<endl;
        cout<<"radius"<<radius<<endl;
        if (circularity > 0.85 && radius > 15 /*&& radius < 200解除最大半径设置*/) {
            //如果有子轮廓
            if(hierarchy[i][2]!=-1)
            {
                Subject Ring_red;
                Ring_red.name ="Ring_red";
                Ring_red.id =subjects.size()+1;
                Ring_red.points1 = Sphere_Processer(contours[i]);
                Ring_red.points2 = Sphere_Processer(contours[hierarchy[i][2]]);
                Mat mask = Mat::zeros(src.size(), CV_8UC1);
                drawContours(mask, contours, static_cast<int>(i), Scalar(255), FILLED);
                Scalar mean_color = mean(src, mask);
                Ring_red.color_bgr = mean_color;
                subjects.push_back(Ring_red);
                    cout<<"ring"<<endl;
            }
            //如果没有子轮廓
            else if (hierarchy[i][3]==-1)
            {
                    Subject Sphere;
                    Sphere.name = "sphere";
                    Sphere.id = subjects.size() + 1;
                    //Sphere.points = sphere(center,radius,src);
                    Sphere.points1 = Sphere_Processer(contours[i]);
                    Mat mask = Mat::zeros(src.size(), CV_8UC1);
                    drawContours(mask, contours, static_cast<int>(i), Scalar(255), FILLED);
                    Scalar mean_color = mean(src, mask);
                    Sphere.color_bgr = mean_color;
                    subjects.push_back(Sphere);
                    cout<<"sphere"<<endl;
            }
    }
}
    return subjects;
}