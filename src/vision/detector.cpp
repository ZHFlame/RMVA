#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <map>
#include <vector>
#include "vision.h"
using namespace std;
using namespace cv;
vector<Subject> detector(Mat &imgsource) {


    // ============================================================
    // 0. 复制输入图像并准备工作容器
    // ============================================================
    Mat src = imgsource.clone();
    vector<Subject> subjects;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    // ============================================================
    // 1. 多通道边缘融合预处理
    // ============================================================
    // 1.1 先整体高斯滤波，压制全局噪声
    GaussianBlur(src, src, Size(5, 5), 0);
    // 1.2 拆分 B/G/R 通道，逐通道检测边缘以覆盖不同颜色对比
    vector<Mat> channels;
    split(src, channels); // B, G, R

    Mat edges_combined = Mat::zeros(src.size(), CV_8UC1);

    // 1.3 逐通道：中值滤波 + Canny，将边缘累加
    for (int i = 0; i < 3; i++) {
        Mat ch_blur, ch_edges;
        // 1.3.1 中值滤波消除椒盐噪声
        medianBlur(channels[i], ch_blur, 5);
        // 1.3.2 低阈值 Canny，保留较弱的结构
        Canny(ch_blur, ch_edges, 30, 100);
        // 1.3.3 多通道 OR 融合
        bitwise_or(edges_combined, ch_edges, edges_combined);
    }

    // 1.4 灰度边缘补充，避免纯亮度目标被遗漏
    Mat gray, gray_edges;
    cvtColor(src, gray, COLOR_BGR2GRAY);
    medianBlur(gray, gray, 5);
    Canny(gray, gray_edges, 40, 120);
    bitwise_or(edges_combined, gray_edges, edges_combined);

    // 1.5 形态学闭运算连接断裂边缘，再膨胀确保闭合
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
    Mat edges_closed;
    morphologyEx(edges_combined, edges_closed, MORPH_CLOSE, kernel, Point(-1,-1), 2);

    Mat Premask;
    dilate(edges_closed, Premask, kernel, Point(-1,-1), 1);
    
    // 1.6 基于外轮廓填充实心掩码，滤除面积过小的噪点
    vector<vector<Point>> tmpCnts;
    findContours(Premask, tmpCnts, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    
    Premask = Mat::zeros(src.size(), CV_8UC1);
    for (size_t i = 0; i < tmpCnts.size(); ++i) {
        double a = contourArea(tmpCnts[i]);
        if (a < 100) continue;
        drawContours(Premask, tmpCnts, static_cast<int>(i), Scalar(255), FILLED);
    }

    // 1.7 最后一次平滑，获得稳定的前景掩码
    morphologyEx(Premask, Premask, MORPH_CLOSE, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

    Mat proc = Premask;

    // ============================================================
    // 2. 提取轮廓作为候选目标
    // ============================================================
    findContours(proc, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);


    for (size_t i = 0; i < contours.size(); i++) {
        // 遍历所有contour
        double area = cv::contourArea(contours[i]);
        if (area < 100) continue;
        //drawContours(src,contours,i,Scalar(255,0,0),2);
        // debug：显示轮廓数
        cout<<"轮廓数："<<contours.size()<<endl;

        /*** 以下为圆形处理 ***/
        // 2.1 对高圆度轮廓执行球体分支
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

            Subject Sphere;
            Sphere.name = "sphere";
            Sphere.id = subjects.size() + 1;
            //Sphere.points = sphere(center,radius,src);
            Sphere.points = Sphere_Processer(contours[i]);
            Mat mask = Mat::zeros(src.size(), CV_8UC1);
            drawContours(mask, contours, static_cast<int>(i), Scalar(255), FILLED);
            Scalar mean_color = mean(src, mask);
            Sphere.color_bgr = mean_color;
            subjects.push_back(Sphere);

        }

        /*** 以上为圆形处理 ***/

        /*** 以下为Armor和矩形处理 ***/
        else {

            // 2.2 对其余轮廓执行装甲板/矩形判定
            // 计算最小外接矩形
            Rect Rectangle_prame;
            Rectangle_prame = boundingRect(contours[i]);
            // 生成矩阵切片
            Mat roi = src(Rectangle_prame);
            // 转为灰度图
            Mat roi_gray;
            cvtColor(roi, roi_gray, COLOR_BGR2GRAY);
            GaussianBlur(roi_gray, roi_gray, Size(9,9), 0);
            // 二值化   
            Mat roi_binary;
            threshold(roi_gray, roi_binary, 128, 255, THRESH_BINARY);
            std::pair<std::vector<Point2f>, int> armor_points;
            armor_points = Armor_Detector(roi);
            cout<<armor_points.first.size()<<endl;
            cout<<armor_points.second<<endl;
            if (armor_points.first.size() == 4 && armor_points.second != -1) {
                for (size_t j=0;j<4;j++) {
                    // 调整点坐标到原图像坐标系
                    armor_points.first[j].x += Rectangle_prame.x;
                    armor_points.first[j].y += Rectangle_prame.y;
                    
                }
                Subject Armor;
                Armor.name = "armor_red_" + to_string(armor_points.second);
                Armor.id = subjects.size() + 1;
                Armor.number = armor_points.second;
                Armor.points = armor_points.first;
                subjects.push_back(Armor);
            }else if(area/(Rectangle_prame.width*Rectangle_prame.height) >0.95){
                // 2.2.4 实心矩形 -> 视作 square 目标
                rectangle(src, Rectangle_prame.tl(), Rectangle_prame.br(), Scalar(0, 0, 255), 2); // 红色矩
                vector<Point2f> Points;
                Points.push_back(Point2f(Rectangle_prame.x,Rectangle_prame.y+Rectangle_prame.height));
                Points.push_back(Point2f(Rectangle_prame.x+Rectangle_prame.width , Rectangle_prame.y+Rectangle_prame.height));
                Points.push_back(Point2f(Rectangle_prame.x+Rectangle_prame.width,Rectangle_prame.y));
                Points.push_back(Point2f(Rectangle_prame.x,Rectangle_prame.y));
                Subject Square;
                Square.name = "square";
                Square.id = subjects.size() + 1;
                Square.points = Points;
                Mat mask = Mat::zeros(src.size(), CV_8UC1);
                drawContours(mask, contours, static_cast<int>(i), Scalar(255), FILLED);
                Scalar mean_color = mean(src, mask);
                Square.color_bgr = mean_color;
                subjects.push_back(Square);
            }else{
                continue;
            }
            
        }
    }
    // ============================================================
    // 4. 根据平均颜色推断颜色类型
    // ============================================================
for (auto &subject : subjects) {
    if (subject.name == "armor") continue;

    const auto &color = subject.color_bgr;
    double b = color[0], g = color[1], r = color[2];

    // 计算亮度值和相对强度
    double brightness = (r + g + b) / 3.0;
    double max_component = std::max({r, g, b});
    double min_component = std::min({r, g, b});

    // 判断黑白灰色系（基于亮度差异）
    if (max_component - min_component < 30) { // 颜色分量差异小
        if (brightness > 200) {
            subject.color_type = "White";
        } else if (brightness < 50) {
            subject.color_type = "Black";
        } else {
            subject.color_type = "Gray";
        }
        continue;
    }

    // 计算各颜色的相对强度比例
    double r_ratio = r / (r + g + b + 0.001); // 避免除零
    double g_ratio = g / (r + g + b + 0.001);
    double b_ratio = b / (r + g + b + 0.001);

    // 基于色相的主导颜色判断
    if (r > g && r > b) { // 红色系主导
        if (g > 150 && r > 200 && b < 100) {
            subject.color_type = "Orange";
        } else if (g > 180 && r > 200 && b < 120) {
            subject.color_type = "Yellow";
        } else if (r_ratio > 0.6 && g_ratio < 0.3) {
            subject.color_type = "Red";
        } else if (r_ratio > 0.4 && b_ratio > 0.3) {
            subject.color_type = "Purple";
        } else {
            subject.color_type = "Red"; // 默认红色系
        }
    }
    else if (g > r && g > b) { // 绿色系主导
        if (r > 150 && g > 180 && b < 100) {
            subject.color_type = "Yellow-Green";
        } else if (g_ratio > 0.5 && b_ratio < 0.3) {
            subject.color_type = "Green";
        } else {
            subject.color_type = "Green";
        }
    }
    else if (b > r && b > g) { // 蓝色系主导
        if (r > 150 && b > 150 && g < 120) {
            subject.color_type = "Purple";
        } else if (b_ratio > 0.5 && r_ratio < 0.3) {
            subject.color_type = "Blue";
        } else {
            subject.color_type = "Blue";
        }
    }
    else { // 颜色分量相近的混合色
        if (r > 150 && g > 150 && b < 100) {
            subject.color_type = "Yellow";
        } else if (r > 150 && b > 150 && g < 120) {
            subject.color_type = "Purple";
        } else if (r > 180 && g > 100 && b > 180) {
            subject.color_type = "Pink";
        } else if (r > 120 && g > 80 && b < 80) {
            subject.color_type = "Brown";
        } else {
            // 最后保障：基于最接近的基本颜色分类
            double red_dist = std::abs(r-255) + std::abs(g-0) + std::abs(b-0);
            double green_dist = std::abs(r-0) + std::abs(g-255) + std::abs(b-0);
            double blue_dist = std::abs(r-0) + std::abs(g-0) + std::abs(b-255);

            if (red_dist <= green_dist && red_dist <= blue_dist)
                subject.color_type = "Red";
            else if (green_dist <= blue_dist)
                subject.color_type = "Green";
            else
                subject.color_type = "Blue";
        }
    }

}


    //imshow("Detected Objects", src);
    //waitKey(2);
    //imshow("blur",imgBlur) ;
    //imshow("Dilate", imgDil);
    //imshow("RAW",src);
    // ============================================================
    // 5. 输出所有检测目标
    // ============================================================
    return subjects;
}