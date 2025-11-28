#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include<iostream>
#include<string>
#include "vision.h"
using namespace cv;
using namespace std;
void kcf::fu_zhi(int ID,string NAME,string COLOR)
{
id=ID;
name=NAME;
color_type=COLOR;
}
void kcf::hui_zhi(int ID,string NAME,string COLOR)
{
ID=id;
NAME=name;
COLOR=color_type;
}

/*
定义vector的tracker
在detector的传入参数中增加&vector<Tracker>
在detector中实现跟踪器：
1. 包括检测Tracker是否为空
 1. 若为空，则调用create，
 2. 否则，更新Tracker
2.然后每次回调结束，将Tracker的subjects某些值回给原来的subjects。//好像不是很行。


缺点：
就是可能会跟丢，然后就是结果会更不准确。
但是，我感觉，在仿真环境下的那个有规律的运动下，应该遮挡会小很多。
*/

int main()
{
    VideoCapture capture("/home/liubai/rmva/src/challenge/src/vision/testphotos/testvideo.mp4");
    if (!capture.isOpened())
    {
        cout<<"视频打开异常"<<endl;
        return -1;
    }
    int fps = capture.get(CAP_PROP_FPS);
    int width = capture.get(CAP_PROP_FRAME_WIDTH);
    int height = capture.get(CAP_PROP_FRAME_HEIGHT);
    int numberOfFrames = capture.get(CAP_PROP_FRAME_COUNT);
    cout<<"视频宽度"<<width<<"视频高度"<<height
    <<"视频帧率"<<fps<<"视频总帧数"<<numberOfFrames<<endl;
    Mat preFrame;
    if (!capture.read(preFrame)) {
        cout << "无法读取第一帧" << endl;
        return -1;
    }
    Mat postFrame;
    // 跟踪器和对应的边界框应在循环外保存，以便跨帧更新
    vector<Ptr<Tracker> > trackers;
    vector<Rect> bboxes;
    vector<Subject> a = detector(preFrame,trackers,bboxes);
    while (true)
    {
        if (!capture.read(postFrame))
        {
            // 视频结束或读取错误
            break;
        }
        vector<Subject> a= detector(postFrame,trackers,bboxes);
        imshow("input",postFrame);
        int key = waitKey(50);
        if (key == 27) { // ESC
            break;
        }
    }
    return 0;
}
