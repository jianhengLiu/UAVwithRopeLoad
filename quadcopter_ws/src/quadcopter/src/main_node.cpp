//
// Created by chrisliu on 2019/12/7.
//
#include "ros/ros.h"
#include <stdio.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>

using namespace cv;
using namespace std;

ros::Publisher target_vel;

Mat image;
Mat rectImage;
Mat imageCopy; //绘制矩形框时用来拷贝原图的图像
bool leftButtonDownFlag = false; //左键单击后视频暂停播放的标志位
Point originalPoint; //矩形框起点
Point processPoint; //矩形框终点

Mat targetImageHSV;
int histSize = 200;
float histR[] = {0, 255};
const float *histRange = histR;
int channels[] = {0, 1};
Mat dstHist;
Rect rect(100, 100, 50, 50);
vector<Point> pt; //保存目标轨迹
void onMouse(int event, int x, int y, int flags, void *ustc); //鼠标回调函数
void target_camshift(Mat input) {
    Mat rectImage = input(rect); //子图像显示
    imshow("Sub Image", rectImage);
    cvtColor(rectImage, targetImageHSV, CV_RGB2HSV);
    inRange(targetImageHSV, Scalar(100, 43, 46), Scalar(124, 255, 255), targetImageHSV);
    imshow("targetImageHSV", targetImageHSV);
    calcHist(&targetImageHSV, 2, channels, Mat(), dstHist, 1, &histSize, &histRange, true, false);
    normalize(dstHist, dstHist, 0, 255, CV_MINMAX);
    imshow("dstHist", dstHist);


    Mat imageHSV;
    Mat calcBackImage;
    cvtColor(input, imageHSV, CV_RGB2HSV);
    calcBackProject(&imageHSV, 2, channels, dstHist, calcBackImage, &histRange);  //反向投影
    TermCriteria criteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 1000, 0.001);
    CamShift(calcBackImage, rect, criteria);
    Mat imageROI = imageHSV(rect);   //更新模板
    targetImageHSV = imageHSV(rect);
    calcHist(&imageROI, 2, channels, Mat(), dstHist, 1, &histSize, &histRange);
    normalize(dstHist, dstHist, 0.0, 1.0, NORM_MINMAX);   //归一化
    rectangle(input, rect, Scalar(255, 0, 0), 3);    //目标绘制
    pt.push_back(Point(rect.x + rect.width / 2, rect.y + rect.height / 2));
//    for (int i = 0; i < pt.size() - 1; i++) {
//        line(input, pt[i], pt[i + 1], Scalar(0, 255, 0), 2.5);
//    }
    circle(input, pt.back(), 10, Scalar(0, 255, 0), -1);

//    cout<<pt.back ()<<endl;

    geometry_msgs::Twist vel;
    vel.linear.x = (float)(-input.cols/2 + pt.back().x);
    vel.linear.y = (float)(-input.rows/2 + pt.back().y);
    target_vel.publish(vel);

    imshow("track", input);


//    if(!leftButtonDownFlag) //判定鼠标左键没有按下，采取播放视频，否则暂停
//        {
//            video>>image;
//        }
//        if(!image.data||waitKey(pauseTime)==27)  //图像为空或Esc键按下退出播放
//        {
//            break;
//        }
//        if(originalPoint!=processPoint&&!leftButtonDownFlag)
//        {
//            Mat imageHSV;
//            Mat calcBackImage;
//            cvtColor(image,imageHSV,CV_RGB2HSV);
//            calcBackProject(&imageHSV,2,channels,dstHist,calcBackImage,&histRange);  //反向投影
//            TermCriteria criteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 1000, 0.001);
//            CamShift(calcBackImage, rect, criteria);
//            Mat imageROI=imageHSV(rect);   //更新模板
//            targetImageHSV=imageHSV(rect);
//            calcHist(&imageROI, 2, channels, Mat(), dstHist, 1, &histSize, &histRange);
//            normalize(dstHist, dstHist, 0.0, 1.0, NORM_MINMAX);   //归一化
//            rectangle(image, rect, Scalar(255, 0, 0),3);    //目标绘制
//            pt.push_back(Point(rect.x+rect.width/2,rect.y+rect.height/2));
//            for(int i=0;i<pt.size()-1;i++)
//            {
//                line(image,pt[i],pt[i+1],Scalar(0,255,0),2.5);
//            }
//        }
//        imshow("跟踪木头人",image);


}

//鼠标回调函数
//void onMouse(int event,int x,int y,int flags,void *ustc)
//{
//    if(event==CV_EVENT_LBUTTONDOWN)
//    {
//        leftButtonDownFlag=true; //标志位
//        originalPoint=Point(x,y);  //设置左键按下点的矩形起点
//        processPoint=originalPoint;
//    }
//    if(event==CV_EVENT_MOUSEMOVE&&leftButtonDownFlag)
//    {
//        imageCopy=image.clone();
//        processPoint=Point(x,y);
//        if(originalPoint!=processPoint)
//        {
//            //在复制的图像上绘制矩形
//            rectangle(imageCopy,originalPoint,processPoint,Scalar(255,0,0),2);
//        }
//        imshow("跟踪木头人",imageCopy);
//    }
//    if(event==CV_EVENT_LBUTTONUP)
//    {
//        leftButtonDownFlag=false;
//        rect=Rect(originalPoint,processPoint);
//        rectImage=image(rect); //子图像显示
//        imshow("Sub Image",rectImage);
//        cvtColor(rectImage,targetImageHSV,CV_RGB2HSV);
//        imshow("targetImageHSV",targetImageHSV);
//        calcHist(&targetImageHSV,2,channels,Mat(),dstHist,1,&histSize,&histRange,true,false);
//        normalize(dstHist,dstHist,0,255,CV_MINMAX);
//        imshow("dstHist",dstHist);
//    }
//}


void floorCamera_cb(const sensor_msgs::ImageConstPtr &floorImage) {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(floorImage, "bgr8");
    Mat img_floor = cv_ptr->image;
    /**
     * 0 means flipping around the x-axis
     * positive value (for example, 1) means flipping around y-axis.
     * Negative value (for example, -1) means flipping around both axes.
     */
    flip(img_floor, img_floor, 0);
    imshow("quadcopter", img_floor);
    target_camshift(img_floor);
    waitKey(1);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub_floorCamera = it.subscribe("/floorCamera", 1, floorCamera_cb);

    target_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

//    setMouseCallback("跟踪木头人",onMouse);

    while (ros::ok()) {

        ros::spin();
    }

    return 0;
}