#include "staple_tracker.hpp"
#include "antiSwing.h"

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <numeric>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>

#include <math.h>

float length = 2.81122;
int resolutionX = 512;
int resolutionY = 512;
float massiveQuadcopter = 1;
float massivePayload = 0.2;

using namespace std;
using namespace cv;

ros::Publisher pubTargetVector;
ros::Publisher pubTestVector;

Eigen::Vector3d cVector_body(0, 0, 0);

Eigen::Vector3d pVector_world(0, 0, 0);

Eigen::Vector3d pDesiredVector_world(0, 0, 0);
Eigen::Vector3d ppDesiredVector_world(0, 0, 0);

Eigen::Vector3d e3(0, 0, 1);

void callbackImu(const sensor_msgs::ImuConstPtr &imu)
{

    Eigen::Quaterniond quaternion(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
    Eigen::Vector3d omega(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);

    Eigen::Matrix3d rotationMatrix = quaternion.toRotationMatrix();

    Eigen::Vector3d cVector_world = toVectorRefToWorld(cVector_body, rotationMatrix);
    Eigen::Vector3d cOrientation_world = cVector_world / cVector_world.norm();

    Eigen::Vector3d cDesiredVector_world = e3 * length;
    Eigen::Vector3d cDesiredOrientation_world = e3;

    if (!cVector_body.isZero())
    {
        Eigen::Vector3d vectorD = getOrientationD(cVector_world, pVector_world);



        Eigen::Vector3d k_p(-1,-1,-1);
        Eigen::Vector3d k_d(3,3,3);
        Eigen::Vector3d output = (cVector_world - cDesiredVector_world).cwiseProduct(k_p) +  vectorD.cwiseProduct(k_d);
        geometry_msgs::Twist targetError;
        targetError.linear.x = output.x();//output.x();
        targetError.linear.y = output.y();//output.y();

        pubTestVector.publish(targetError);

    }

    pVector_world = cVector_world;
    ppDesiredVector_world = pDesiredVector_world;
    pDesiredVector_world = cDesiredVector_world;
}

cv::Point2f refineTracker(cv::Mat input, int times)
{
//    imshow("input", input);
    int cols = input.cols;
    int rows = input.rows;
    for (int i = 0; i < times; ++i)
    {
        cv::pyrUp(input, input);
    }
//    imshow("inputUp", input);

    cvtColor(input, input, cv::COLOR_BGR2GRAY);

    cv::threshold(input, input, input.at<uchar>(input.cols / 2, input.rows / 2) - 10, 255, cv::THRESH_BINARY);
//    blur(input, input, cv::Size(3, 3));
//    Canny(input, input, 100, 200);

//    imshow("rectImage", input);
    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;
    findContours(input, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//        vector<Moments> mu(contours.size());
//        for(int i=0;i<contours.size();i++)
//        {
//            mu[i] = moments(contours[i],false);
//        }

    for (int i = 0; i < contours.size(); i++)
    {
//            cout << contourArea(contours[i]) << endl;
        if (contourArea(contours[i]) > 400 * pow(4, times))
        {
            cv::RotatedRect returnRect = minAreaRect(contours[i]);
            cv::Point2f returnPoint = returnRect.center;
            cv::circle(input, returnPoint, 2, cv::Scalar(255), -1);
            returnPoint.x /= pow(2, times);
            returnPoint.y /= pow(2, times);
//            imshow("rectImage", input);
            return returnPoint;
        }
    }
    return cv::Point2f(cols / 2, cols / 2);
}

STAPLE_TRACKER staple;

void trackerStapleInit(cv::Mat input)
{
    namedWindow("STAPLE", cv::WINDOW_AUTOSIZE);
    cv::Rect roi = selectROI("STAPLE", input);
    if (roi.width == 0 || roi.height == 0)
    {
        cout << "Failed to get target ROI!" << endl;
        return;
    }

    staple.tracker_staple_initialize(input, roi);
    staple.tracker_staple_train(input, true);
}

bool isTrackerStapleInit = false;
std::vector<cv::Rect_<float>> result_rects;
bool show_visualization = true;
double calculateTime = 0;

void trackerStaple(cv::Mat input)
{
    if (!isTrackerStapleInit)
    {
        trackerStapleInit(input);
        isTrackerStapleInit = true;
    }

    int64 tic = cv::getTickCount();

    cv::Rect_<float> location = staple.tracker_staple_update(input);
    staple.tracker_staple_train(input, false);
    result_rects.push_back(location);

    cv::Point2f targetPoint = location.tl();
    if (location.x > 0 && location.y > 0 && location.x + location.width < input.cols &&
        location.y + location.height < input.rows)
    {
        cv::Point2f returnPoint = refineTracker(input(location), 4);
        targetPoint += returnPoint;
    } else
    {
        targetPoint.x += location.width / 2;
        targetPoint.y += location.height / 2;
        ROS_ERROR("Target is out of field!");
    }

    int64 toc = cv::getTickCount() - tic;
    calculateTime += toc;


    geometry_msgs::Twist targetVector;
    targetVector.linear.x = (float) (-input.cols) / 2 + targetPoint.x;
    targetVector.linear.y = (float) (-input.rows) / 2 + targetPoint.y;
    pubTargetVector.publish(targetVector);

    float errorX = (float) (-input.cols) / 2 + targetPoint.x;
    float errorY = (float) (-input.rows) / 2 + targetPoint.y;

    cVector_body = pixelerrorToVector(errorX, errorY, length, resolutionX, resolutionY);

    if (show_visualization)
    {
        double sumTime = calculateTime / double(cv::getTickFrequency());
        float fps = double(result_rects.size()) / sumTime;
        cv::putText(input, std::to_string(fps), cv::Point(20, 40), 6, 1,
                    cv::Scalar(0, 255, 255), 2);
        cv::circle(input, targetPoint, 2, cv::Scalar(255, 0, 0), -1);
        cv::rectangle(input, location, cv::Scalar(0, 128, 255), 2);
        cv::imshow("STAPLE", input);

    } else
    {
        double sumTime = calculateTime / double(cv::getTickFrequency());
        float fps = double(result_rects.size()) / sumTime;
        std::cout << "fps:" << fps << std::endl;
    }
//    cv::waitKey();

}

Mat targetImageHSV;
int histSize = 200;
float histR[] = {0, 255};
const float *histRange = histR;
int channels[] = {0, 1};
Mat dstHist;
Rect rect(200, 200, 50, 50);
vector<Point> pt; //保存目标轨迹
void onMouse(int event, int x, int y, int flags, void *ustc); //鼠标回调函数
Point2d targetTargetCamshift(Mat input)
{
//    Mat rectImage = input(rect); //子图像显示
//    imshow("Sub Image", rectImage);
    cvtColor(input, targetImageHSV, CV_RGB2HSV);
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

//    geometry_msgs::Twist targetVector;
//    targetVector.linear.x = (float) (-input.cols) / 2 + pt.back().x;
//    targetVector.linear.y = (float) (-input.rows) / 2 + pt.back().y;
//    pubTargetVector.publish(targetVector);

//    float errorX = (float) (-input.cols) / 2 + pt.back().x;
//    float errorY = (float) (-input.rows) / 2 + pt.back().y;
//
//    cOrientation_body = pixelerrorToVector(errorX, errorY, length, resolutionX, resolutionY);


    imshow("track", input);

    Point2d returnTarget = pt.back();
    return returnTarget;
}

void callbackFloorCamera(const sensor_msgs::ImageConstPtr &floorImage)
{
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(floorImage, "bgr8");
    cv::Mat img_floor = cv_ptr->image;
    /**
     * 0 means flipping around the x-axis
     * positive value (for example, 1) means flipping around y-axis.
     * Negative value (for example, -1) means flipping around both axes.
     */
    flip(img_floor, img_floor, 0);
//    targetTargetCamshift(img_floor);
//    trackerStaple(img_floor);
    cv::waitKey(1);
}

void callbackPayload(const geometry_msgs::TwistConstPtr &payload)
{
    cVector_body = Eigen::Vector3d(payload->linear.x, payload->linear.y, payload->linear.z);
//    cout<<cOrientation_body<<endl;
    pubTargetVector.publish(*payload);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracker");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subFloorCamera = it.subscribe("/floorCamera", 1, callbackFloorCamera);

    ros::Subscriber subIMU = nh.subscribe("/imu", 1, callbackImu);
    ros::Subscriber subPayloadPosition = nh.subscribe("/payload", 1, callbackPayload);
    pubTargetVector = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pubTestVector = nh.advertise<geometry_msgs::Twist>("/testVector", 1);

    while (ros::ok())
    {
        ros::spin();
    }

    return 0;
}


