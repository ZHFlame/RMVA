//
// Created by LyChee on 2025/11/18.
//
#include <iostream>
#include <opencv2/tracking.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <map>
#include <vector>
#include "vision.h"
using namespace std;
using namespace cv;

// 【修改】使用 Subject 列表来存储正在跟踪的目标
// 这个列表需要持久化（static 或全局），因为它保存了上一帧的跟踪状态
vector<Subject> active_tracks; 

vector<Subject> dynamic(Mat &src){
    // 1. 先尝试检测新目标
    vector<Subject> detected_subjects = detector(src);
    /*
    // 简单逻辑：如果当前没有正在跟踪的目标，则初始化跟踪器
    // (实际应用中可能需要更复杂的逻辑来匹配检测结果和跟踪结果)
    if (active_tracks.empty()) {
        active_tracks.clear(); // 确保清空
        
        for (auto &sub : detected_subjects) {
            if (sub.points.empty()) continue;
            Rect r = boundingRect(sub.points);
            if (r.area() < 500) continue;

            // 【修改】直接在 Subject 对象内部初始化跟踪器
            sub.tracker = TrackerKCF::create();
            sub.tracker->init(src, r);
            
            // 将初始化好跟踪器的对象加入活跃列表
            active_tracks.push_back(sub);
        }
        // 如果刚初始化，返回检测到的结果
        return active_tracks; 
    } else {
        // 2. 如果已有跟踪目标，则进行更新
        // 我们需要返回这一帧跟踪到的物体位置
        vector<Subject> current_frame_subjects;

        for (auto it = active_tracks.begin(); it != active_tracks.end(); ) {
            Rect bbox;
            // 【修改】调用 Subject 内部的 tracker
            bool ok = it->tracker->update(src, bbox);
            
            if (ok) {
                // 更新 Subject 的 points (将 bbox 转回 points)
                it->points.clear();
                it->points.push_back(Point2f(bbox.x, bbox.y)); // tl
                it->points.push_back(Point2f(bbox.x + bbox.width, bbox.y)); // tr
                it->points.push_back(Point2f(bbox.x + bbox.width, bbox.y + bbox.height)); // br
                it->points.push_back(Point2f(bbox.x, bbox.y + bbox.height)); // bl

                // 可视化
                cv::rectangle(src, bbox, cv::Scalar(0, 0, 255), 2);
                cv::putText(src, it->name, bbox.tl(), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0), 1);
                
                // 将更新后的对象加入当前帧结果
                current_frame_subjects.push_back(*it);
                ++it;
            } else {
                // 跟踪失败，从活跃列表中移除
                it = active_tracks.erase(it);
                cv::putText(src, "Lost", cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
            }
        }
        return current_frame_subjects;
    }*/
}