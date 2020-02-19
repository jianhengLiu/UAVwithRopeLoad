//
// Created by chrisliu on 2019/12/7.
//
#include "ros/ros.h"
#include <stdio.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/tracking.hpp>

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
Point2d targetTargetCamshift(Mat input) {
    Mat rectImage = input(rect); //子图像显示
//    imshow("Sub Image", rectImage);
    cvtColor(rectImage, targetImageHSV, CV_RGB2HSV);
    inRange(targetImageHSV, Scalar(100, 43, 46), Scalar(124, 255, 255), targetImageHSV);
    calcHist(&targetImageHSV, 2, channels, Mat(), dstHist, 1, &histSize, &histRange, true, false);
    normalize(dstHist, dstHist, 0, 255, CV_MINMAX);

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

    circle(input, pt.back(), 10, Scalar(0, 255, 0), -1);


    geometry_msgs::Twist vel;
    vel.linear.x = (float) (-input.cols / 2 + pt.back().x);
    vel.linear.y = (float) (-input.rows / 2 + pt.back().y);
    target_vel.publish(vel);

    imshow("track", input);

    Point2d returnTarget = pt.back();
    return returnTarget;

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

Point2f refineTargetContours(Mat input, Rect inputRect) {
    if (inputRect.x > 0 && inputRect.y > 0 && inputRect.x + inputRect.width < input.cols &&
        inputRect.y + inputRect.height < input.rows) {

        Mat grayImage = input(inputRect);

        cvtColor(grayImage, grayImage, COLOR_BGR2GRAY);
        blur(grayImage, grayImage, Size(3, 3));
        Canny(grayImage, grayImage, 100, 200);
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        findContours(grayImage, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
//        vector<Moments> mu(contours.size());
//        for(int i=0;i<contours.size();i++)
//        {
//            mu[i] = moments(contours[i],false);
//        }


        Point2f returnPoint;
//    float tempRadius;

        for (int i = 0; i < contours.size(); i++) {
//            cout << contourArea(contours[i]) << endl;
//
//            if (contourArea(contours[i]) > 1000 && contourArea(contours[i]) < 3000) {
//            drawContours(grayImage,contours,i,Scalar(255));
            RotatedRect returnRect = minAreaRect(contours[i]);
            imshow("rectImage", grayImage);
            if(returnRect.size.area()>1000){
                circle(grayImage, returnRect.center, 2, Scalar(255), -1);

                return returnRect.center;
            }

        }

    }
}

Rect2d roi;
Ptr<Tracker> tracker;
bool isInitializeTracker = false;

void trackTargetOpenCVInit(Mat input) {
    TrackerKCF::Params params;
    params.detect_thresh = 0.3f;
    tracker = TrackerKCF::create(params);
//    tracker = TrackerMIL::create();
//    tracker = TrackerTLD::create();
//    tracker = TrackerMOSSE::create();


    namedWindow("output", WINDOW_AUTOSIZE);
    roi = selectROI("output", input);
    if (roi.width == 0 || roi.height == 0) {
        return;
    }
    //跟踪
    tracker->init(input, roi);
}

void trackTargetOpenCVAPI(Mat input) {
    if (isInitializeTracker == false) {
        trackTargetOpenCVInit(input);
        isInitializeTracker = true;
    }

    tracker->update(input, roi);

//    imshow("output", input);

    Point2d targetPoint = roi.tl();

//    无优化
    targetPoint.x += roi.width / 2;
    targetPoint.y += roi.height / 2;

//  有优化
//    Rect tempRect = Rect(roi.x, roi.y, roi.width, roi.height);
//    Point2f tempPoint = refineTargetContours(input, tempRect);
//    targetPoint.x += tempPoint.x;
//    targetPoint.y += tempPoint.y;

    rectangle(input, roi, Scalar(255, 0, 0), 2, 8, 0);
    circle(input, targetPoint, 10, Scalar(0, 255, 0), -1);

    geometry_msgs::Twist vel;
    vel.linear.x = (float) (-input.cols / 2 + targetPoint.x);
    vel.linear.y = (float) (-input.rows / 2 + targetPoint.y);
    target_vel.publish(vel);

    imshow("output", input);

}

void trackTargetORB(Mat input) {
    //读取图像
    Mat img = input;
    //定义特征点向量和描述子
    vector<KeyPoint> keypoints_1;
    Mat descriptor;
    //ORB特征提取器
    Ptr<ORB> orb = ORB::create();
    //检测特征点
    orb->detect(img, keypoints_1);
    //计算描述子
    orb->compute(img, keypoints_1, descriptor);
    //画出特征点
    Mat outimg;
    drawKeypoints(img, keypoints_1, outimg, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    imshow("ORB", outimg);

    cout << "描述子维度：" << descriptor.size() << endl;
    cout << "特征点数：" << keypoints_1.size() << endl;
    Mat row_data = descriptor.rowRange(1, 2).clone();
    cout << "其中一个描述子：" << row_data << endl;

    waitKey(1);
}


void floorCamera_cb(const sensor_msgs::ImageConstPtr &floorImage) {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(floorImage, "bgr8");
    Mat img_floor = cv_ptr->image;
    /**
     * 0 means flipping around the x-axis
     * positive value (for example, 1) means flipping around y-axis.
     * Negative value (for example, -1) means flipping around both axes.
     */
    flip(img_floor, img_floor, 0);
//    imshow("quadcopter", img_floor);
    targetTargetCamshift(img_floor);
//    trackTargetOpenCVAPI(img_floor);
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