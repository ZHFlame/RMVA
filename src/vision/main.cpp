#include "opencv2/opencv.hpp"
#include "vision.h"
#include <iostream>
using namespace std;
int main(int argc, const char * argv[]) {
    string file;
    if (argc == 3 && string(argv[1]) == "img") {
        file = argv[2];

    cv::Mat img = cv::imread(file);
    std::vector<Subject> subjects = detector(img);
    for (auto subject : subjects) {
        std::cout << "ID: " << subject.id << std::endl;
        std::cout <<"Type: " << subject.name << std::endl;
        std::cout << "Points: ";
        for (const auto& point : subject.points) {
            std::cout << "(" << point.x << ", " << point.y << ") ";
        }
        std::cout << std::endl;
        std::cout << "Number: " << subject.number << std::endl;
        std::cout << "Color:" << subject.color_type << std::endl;
        std::cout << "------------------------" << std::endl;

    }
    // 绘制subject上的四个点
    vector<string> point_names = {"左", "下", "右", "上"};
    vector<cv::Scalar> point_colors = {
        cv::Scalar(255, 0, 0),    // 蓝色 - 左
        cv::Scalar(0, 255, 0),    // 绿色 - 下
        cv::Scalar(0, 255, 255),  // 黄色 - 右
        cv::Scalar(255, 0, 255)   // 紫色 - 上
    };
for (const auto& subject : subjects) {
    Rect bbox = boundingRect(subject.points);
    cv::rectangle(img, bbox, cv::Scalar(0, 0, 255), 2);
    for (int j = 0; j < 4; j++) {
        cv::circle(img, subject.points[j], 6, point_colors[j], -1);
        cv::circle(img,subject.points[j], 6, cv::Scalar(0, 0, 0), 2);
        // 标注序号
        string point_text = to_string(j + 1);
        cv::putText(
            img, point_text,
            cv::Point(subject.points[j].x + 10, subject.points[j].y - 10),
            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 3);
        cv::putText(
            img, point_text,
            cv::Point(subject.points[j].x + 10, subject.points[j].y - 10),
            cv::FONT_HERSHEY_SIMPLEX, 0.6, point_colors[j], 2);
    }
    if (subject.name.substr(0,5) == "armor") {
        cv::putText(img, "No."+to_string(subject.id)+": "+subject.name,
                cv::Point(subject.points[3].x, subject.points[3].y - 90),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        cv::putText(img, "Num: " + to_string(subject.number),
                        cv::Point(subject.points[3].x, subject.points[3].y - 60),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

    }else{
        cv::putText(img, "No. "+to_string(subject.id)+": "+subject.name,
                cv::Point(subject.points[3].x, subject.points[3].y - 60),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        cv::putText(img, "Color: "+subject.color_type,
                       cv::Point(subject.points[3].x, subject.points[3].y - 30),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

    }
}
    imshow("OpenCV", img);
    cv::waitKey(0);
    }else if (argc == 3 && string(argv[1])=="video") {
        cv::VideoCapture capture;
        capture.open(argv[2]);
        //capture.open(0);
        if (!capture.isOpened()) {
            cerr << "Error opening video file" << endl;
        }
        cv::Mat frame = cv::Mat();

        while (true) {
            // 清空img内容

            cv::Mat img = cv::Mat();
            capture >> frame;
            frame.copyTo(img);
            if (img.empty()) {
                break;
            }
            std::vector<Subject> subjects;
            subjects = detector(img);
            cout<<"检测到 "<<subjects.size()<<" 个目标"<<endl;
            /*for (auto subject : subjects) {
                std::cout << "ID: " << subject.id << std::endl;
                std::cout <<"Type: " << subject.name << std::endl;
                std::cout << "Points: ";
                for (const auto& point : subject.points) {
                    std::cout << "(" << point.x << ", " << point.y << ") ";
                }
                std::cout << std::endl;
                std::cout << "Number: " << subject.number << std::endl;
                std::cout << "Color:" << subject.color_type << std::endl;
                std::cout << "------------------------" << std::endl;

            }*/

            // 绘制subject上的四个点
            vector<string> point_names = {"左", "下", "右", "上"};
            vector<cv::Scalar> point_colors = {
                cv::Scalar(255, 0, 0),    // 蓝色 - 左
                cv::Scalar(0, 255, 0),    // 绿色 - 下
                cv::Scalar(0, 255, 255),  // 黄色 - 右
                cv::Scalar(255, 0, 255)   // 紫色 - 上
            };
            for (const auto &subject: subjects) {
                    Rect bbox = boundingRect(subject.points);
    cv::rectangle(img, bbox, cv::Scalar(0, 0, 255), 2);
                for (int j = 0; j < 4; j++) {
                    cv::circle(img, subject.points[j], 6, point_colors[j], -1);
                    cv::circle(img, subject.points[j], 6, cv::Scalar(0, 0, 0), 2);
                    // 标注序号
                    string point_text = to_string(j + 1);
                    cv::putText(
                        img, point_text,
                        cv::Point(subject.points[j].x + 10, subject.points[j].y - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 3);
                    cv::putText(
                        img, point_text,
                        cv::Point(subject.points[j].x + 10, subject.points[j].y - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, point_colors[j], 2);
                }
                if (subject.name == "armor") {
                    cv::putText(img, "No." + to_string(subject.id) + ": " + subject.name,
                                cv::Point(subject.points[3].x, subject.points[3].y - 90),
                                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                    cv::putText(img, "Num: " + to_string(subject.number),
                                cv::Point(subject.points[3].x, subject.points[3].y - 60),
                                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                } else {
                    cv::putText(img, "No. " + to_string(subject.id) + ": " + subject.name,
                                cv::Point(subject.points[3].x, subject.points[3].y - 60),
                                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                    cv::putText(img, "Color: " + subject.color_type,
                                cv::Point(subject.points[3].x, subject.points[3].y - 30),
                                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                }
            }
            imshow("OpenCV", img);
            cv::waitKey(50);

        }
    }
    else {
        cout << "argc:" << argc << endl;
        cerr << "WRONG!" << endl;
        return -1;
    };

}