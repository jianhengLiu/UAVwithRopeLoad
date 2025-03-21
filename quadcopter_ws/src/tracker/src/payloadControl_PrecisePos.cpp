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
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>

#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using namespace std;

ros::Publisher pubTargetVector;
ros::Publisher pubTestVector;

PayloadController controller;

float length = 2.845;
int resolutionX = 512;
int resolutionY = 512;

void callbackPixelError(const std_msgs::Float64MultiArrayConstPtr &pixelerror)
{

    Eigen::Vector3d cVector_body = controller.pixelerrorToVector(pixelerror->data[0], pixelerror->data[1], length,
                                                                 resolutionX, resolutionY);
//    controller.updatePayloadStates(cVector_body);
//    cout << cVector_body << endl;

//    geometry_msgs::Twist targetVector;
//    targetVector.linear.x = cVector_body.x();
//    targetVector.linear.y = cVector_body.y();
//    targetVector.linear.z = cVector_body.z();
//    pubTargetVector.publish(targetVector);
}

void callbackOdometry(const nav_msgs::OdometryConstPtr &odom)
{
    Eigen::Vector3d positionBody(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
    Eigen::Vector3d linearVelocity(odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z);

    controller.positionBody = positionBody;
    controller.velocityBody = linearVelocity;
}

void callbackImu(const sensor_msgs::ImuConstPtr &imu)
{

    Eigen::Quaterniond quaternion(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
    Eigen::Vector3d linearAcceleration(imu->linear_acceleration.x, imu->linear_acceleration.y,
                                       imu->linear_acceleration.z);
    Eigen::Vector3d omega(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);

    controller.acceleration = linearAcceleration;
    controller.cAngularVelocity_body = omega;
    controller.rotationMatrix_BuW = quaternion.toRotationMatrix();

}

void callbackTarget(const geometry_msgs::PoseConstPtr &pose)
{
    Eigen::Vector3d positionTarget(pose->position.x, pose->position.y, pose->position.z);
    Eigen::Quaterniond quaternionTarget(pose->orientation.w, pose->orientation.x, pose->orientation.y,
                                        pose->orientation.z);

    controller.updateTargetStates(positionTarget, quaternionTarget);
}

void callbackTag(const apriltag_ros::AprilTagDetectionArrayConstPtr &tag_msg)
{
//    apriltag_ros::AprilTagDetectionArray：
//    std_msgs/Header header
//    apriltags2_ros/AprilTagDetection[] detections

//    apriltag_ros::AprilTagDetection：
//    int32[] id
//    float64[] size
//    geometry_msgs/PoseWithCovarianceStamped pose

    if (!tag_msg->detections.empty())
    {
        Eigen::Vector3d positionTarget(tag_msg->detections[0].pose.pose.pose.position.x,
                                       tag_msg->detections[0].pose.pose.pose.position.y,
                                       tag_msg->detections[0].pose.pose.pose.position.z);

        controller.updatePayloadStates(positionTarget);

        geometry_msgs::Twist targetVector;
        targetVector.linear.x = positionTarget.x();
        targetVector.linear.y = positionTarget.y();
        targetVector.linear.z = positionTarget.z();
        pubTargetVector.publish(targetVector);
    } else
    {
        Eigen::Vector3d positionTarget(0, 0, length);

        controller.updatePayloadStates(positionTarget);

        geometry_msgs::Twist targetVector;
        targetVector.linear.x = positionTarget.x();
        targetVector.linear.y = positionTarget.y();
        targetVector.linear.z = positionTarget.z();
        pubTargetVector.publish(targetVector);

        cout << "No tag detected!" << endl;
    }

}

void callbackPayload(const geometry_msgs::TwistConstPtr &payload)
{
    Eigen::Vector3d cVector_body = Eigen::Vector3d(payload->linear.x, payload->linear.y, payload->linear.z);
    controller.updatePayloadStates(cVector_body);
//    cout<<cVector_body<<endl;

    geometry_msgs::Twist targetVector;
    targetVector.linear.x = cVector_body.x();
    targetVector.linear.y = cVector_body.y();
    targetVector.linear.z = cVector_body.z();
    pubTargetVector.publish(targetVector);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracker");
    ros::NodeHandle nh;
    ros::Rate loop_rate(100);

    //下面三个缺一不可
    ros::Subscriber subIMU = nh.subscribe("/imu", 1, callbackImu);
    ros::Subscriber subOdometry = nh.subscribe("/odom", 1, callbackOdometry);
    ros::Subscriber subTarget = nh.subscribe("/target", 1, callbackTarget);


    ros::Subscriber subTag = nh.subscribe("/tag_detections", 1, callbackTag);

//    使用视觉估计的位置
//    ros::Subscriber subPixelError = nh.subscribe("/pixelerror", 1, callbackPixelError);
//    使用仿真返回的负载位置
//    ros::Subscriber subPayloadPosition = nh.subscribe("/payload", 1, callbackPayload);

    //多线程订阅
    ros::AsyncSpinner spinner(4); // Use 4 threads


    pubTargetVector = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    ros::Publisher pubRotorRevs = nh.advertise<std_msgs::Float64MultiArray>("/rotorRevs", 1);


    double massQuadcopter = 1;
    double massPayload = 0.2;
    controller.initializeParameter(massQuadcopter, massPayload, length);

    while (ros::ok())
    {
//        geometry_msgs::Twist targetError;
//        targetError.linear.x = controller.cVector_world.x();
//        targetError.linear.y = controller.cVector_world.y();
//        targetError.linear.z = controller.cVector_world.z();
//        pubTestVector.publish(targetError);

        {
            Eigen::Vector3d e3(0, 0, -1);
            Eigen::Vector3d inputDesiredVector_world = e3 * length;
            controller.updateDesiredPayloadStates(inputDesiredVector_world);

            Eigen::Vector4d revs = controller.getRevs();

            std_msgs::Float64MultiArray revsArray;
            revsArray.data = {revs.x(), revs.y(), revs.z(), revs.w()};

            pubRotorRevs.publish(revsArray);
        }

        controller.updatePastStates();

        spinner.start();

//        ros::spinOnce();
    }

    ros::waitForShutdown();
    return 0;
}


