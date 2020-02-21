#include "staple_tracker.hpp"

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

using namespace std;

ros::Publisher pubTargetVector;


cv::Point2f refineTracker(cv::Mat input, int times) {
//    imshow("input", input);
    int cols = input.cols;
    int rows = input.rows;
    for (int i = 0; i < times; ++i) {
        cv::pyrUp(input, input);
    }
//    imshow("inputUp", input);

//    if (inputRect.x > 0 && inputRect.y > 0 && inputRect.x + inputRect.width < input.cols &&
//        inputRect.y + inputRect.height < input.rows) {

    cvtColor(input, input, cv::COLOR_BGR2GRAY);
    cv::threshold(input, input, 200, 255, cv::THRESH_BINARY);
    blur(input, input, cv::Size(3, 3));
    Canny(input, input, 80, 160);

    vector<vector<cv::Point> > contours;
    vector<cv::Vec4i> hierarchy;
    findContours(input, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
//        vector<Moments> mu(contours.size());
//        for(int i=0;i<contours.size();i++)
//        {
//            mu[i] = moments(contours[i],false);
//        }

    for (int i = 0; i < contours.size(); i++) {
//            cout << contourArea(contours[i]) << endl;
        if (contourArea(contours[i]) > 400 * pow(4, times)) {
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

void trackerStapleInit(cv::Mat input) {
    namedWindow("STAPLE", cv::WINDOW_AUTOSIZE);
    cv::Rect roi = selectROI("STAPLE", input);
    if (roi.width == 0 || roi.height == 0) {
        cout << "Failed to get target ROI!" << endl;
        return;
    }

    staple.tracker_staple_initialize(input, roi);
    staple.tracker_staple_train(input, true);
}

bool isTrackerStapleInit = false;
std::vector<cv::Rect_<float>> result_rects;
bool show_visualization = false;
double calculateTime = 0;

void trackerStaple(cv::Mat input) {
    if (!isTrackerStapleInit) {
        trackerStapleInit(input);
        isTrackerStapleInit = true;
    }

    int64 tic = cv::getTickCount();

    cv::Rect_<float> location = staple.tracker_staple_update(input);
    staple.tracker_staple_train(input, false);
    result_rects.push_back(location);

    cv::Point2f targetPoint = location.tl();
//    cv::Point2f returnPoint = refineTracker(input(location), 6);
//    targetPoint += returnPoint;
    targetPoint.x += location.width / 2;
    targetPoint.y += location.height / 2;
//    cout << targetPoint << endl;

    int64 toc = cv::getTickCount() - tic;
    calculateTime += toc;


    geometry_msgs::Twist targetVector;
    targetVector.linear.x = (float) (-input.cols) / 2 + targetPoint.x;
    targetVector.linear.y = (float) (-input.rows) / 2 + targetPoint.y;
    pubTargetVector.publish(targetVector);

    if (show_visualization) {
        double sumTime = calculateTime / double(cv::getTickFrequency());
        float fps = double(result_rects.size()) / sumTime;
        cv::putText(input, std::to_string(fps), cv::Point(20, 40), 6, 1,
                    cv::Scalar(0, 255, 255), 2);
        cv::circle(input, targetPoint, 2, cv::Scalar(255, 0, 0), -1);
        cv::rectangle(input, location, cv::Scalar(0, 128, 255), 2);
        cv::imshow("STAPLE", input);

    } else {
        double sumTime = calculateTime / double(cv::getTickFrequency());
        float fps = double(result_rects.size()) / sumTime;
        std::cout << "fps:" << fps << std::endl;
    }
//    cv::waitKey();

}


void floorCamera_cb(const sensor_msgs::ImageConstPtr &floorImage) {
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(floorImage, "bgr8");
    cv::Mat img_floor = cv_ptr->image;
    /**
     * 0 means flipping around the x-axis
     * positive value (for example, 1) means flipping around y-axis.
     * Negative value (for example, -1) means flipping around both axes.
     */
    flip(img_floor, img_floor, 0);
    trackerStaple(img_floor);
    cv::waitKey(1);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tracker");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subFloorCamera = it.subscribe("/floorCamera", 1, floorCamera_cb);

    pubTargetVector = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    while (ros::ok()) {
        ros::spin();
    }

    return 0;
}
