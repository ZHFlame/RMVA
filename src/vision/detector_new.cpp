#include "opencv2/opencv.hpp"
#include <iostream>
#include "vision.h"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
using namespace std;
using namespace cv;
vector<Subject> detector(Mat &imgsource) {
    // 1. 图像的传入与预处理
    Mat src = imgsource.clone();
    // 1.1. 高斯滤波
    GaussianBlur(src, src, Size(5, 5), 0);
    
    // 1.2. 光照均衡化

    // 1.3. 通道分离
    vector<Mat> bgr_channels;
    split(src, bgr_channels);
    Mat b_channel = bgr_channels[0];
    Mat g_channel = bgr_channels[1];
    Mat r_channel = bgr_channels[2];
    imshow("R_Channel", r_channel);
    imshow("G_Channel", g_channel);
    imshow("B_Channel", b_channel);
    // 1.4. 边缘检测
    Mat edges_r, edges_g, edges_b;
    Canny(r_channel, edges_r, 50, 150);
    Canny(g_channel, edges_g, 50, 150);
    Canny(b_channel, edges_b, 50, 150);
    // 1.5. 膨胀与腐蚀
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    morphologyEx(edges_r, edges_r, MORPH_CLOSE, kernel);
    morphologyEx(edges_r, edges_r, MORPH_OPEN, kernel);
    morphologyEx(edges_g, edges_g, MORPH_CLOSE, kernel);
    morphologyEx(edges_g, edges_g, MORPH_OPEN, kernel);
    morphologyEx(edges_b, edges_b, MORPH_CLOSE, kernel);
    morphologyEx(edges_b, edges_b, MORPH_OPEN, kernel);
    // 2. 轮廓检测
    vector<vector<Point>> contours_r;
    vector<Vec4i> hierarchy_r;
    vector<vector<Point>> contours_g;
    vector<Vec4i> hierarchy_g;
    vector<vector<Point>> contours_b;
    vector<Vec4i> hierarchy_b;
    findContours(edges_r, contours_r, hierarchy_r, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    findContours(edges_g, contours_g, hierarchy_g, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    findContours(edges_b, contours_b, hierarchy_b, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    drawContours(src, contours_r, -1, Scalar(255, 0, 255), 5);
    drawContours(src, contours_g, -1, Scalar(0, 255, 0), 5);
    drawContours(src, contours_b, -1, Scalar(0, 0, 255), 5);
    imshow("src", src);
    waitKey();

}