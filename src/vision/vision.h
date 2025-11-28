//
// Created by LyChee on 2025/11/12.
//
#ifndef VISION_ARMOR_H
#define VISION_ARMOR_H
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <vector>
using namespace cv;
using namespace std;

// 【修改】删除原来的
/*class kcf : public cv::TrackerKCF{
public:
 // 实现纯虚函数
    void init(cv::InputArray image, const cv::Rect& boundingBox) override {
        // 调用基类的实现或提供自定义实现
        cv::TrackerKCF::init(image, boundingBox);
    }
    bool update(cv::InputArray image, cv::Rect& boundingBox) override {
        // 调用基类的实现或提供自定义实现
        return cv::TrackerKCF::update(image, boundingBox);
    }
    void setFeatureExtractor(cv::tracking::TrackerKCF::FeatureExtractorCallbackFN callback, bool pca_func = false) override {
        // 调用基类的实现或提供自定义实现
        cv::TrackerKCF::setFeatureExtractor(callback, pca_func);
    }
    kcf(int ID,string NAME,string COLOR){
        fu_zhi(ID,NAME,COLOR);//赋值
    }
private:
    int id;
    string name;
    string color_type;
    void fu_zhi(int ID,string NAME,string COLOR);//赋值
    void hui_zhi(int ID,string NAME,string COLOR);//回值
};*/
// 改用结构体组合的方式
struct Subject{
    std::string name;
    std::vector<cv::Point2f> points;
    int number=0;
    int id=0;
    std::string color_type="none";
    cv::Scalar color_bgr=cv::Scalar(0,0,0);
};

std::pair<std::vector<cv::Point2f>, int> Armor_Detector(cv::Mat img_input);
std::vector<struct Subject> detector(cv::Mat &src);
std::vector<Subject> dynamic(Mat &src);
std::vector<cv::Point2f> Sphere_Processer(const std::vector<cv::Point> &contour);
int digit_detector(const cv::Mat& src);
#endif //VISION_ARMOR_H
