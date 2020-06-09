#include "flightController.h"
#include "trajectory_generator_waypoint_f.h"

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;
using namespace Eigen;

PayloadController controller;
TrajectoryGeneratorWaypoint trajectoryGeneratorWaypoint;
bool isNewTraj = false;

ros::Subscriber _way_pts_sub;
ros::Publisher _wp_traj_vis_pub, _wp_path_vis_pub;
//Get the path points
void rcvWaypointsCallBack(const nav_msgs::Path &wp)
{
    vector<Vector3d> wp_list;
    wp_list.clear();

    for (int k = 0; k < (int) wp.poses.size(); k++)
    {
        Vector3d pt(wp.poses[k].pose.position.x, wp.poses[k].pose.position.y, wp.poses[k].pose.position.z);
        wp_list.push_back(pt);

        if (wp.poses[k].pose.position.z < 0.0)
            break;
    }

    MatrixXd waypoints(wp_list.size() + 1, 3);
    //导入初始位置
    Vector3d _startPos = Vector3d::Zero();
    waypoints.row(0) = _startPos;

    for (int k = 0; k < (int) wp_list.size(); k++)
        waypoints.row(k + 1) = wp_list[k];

    //Trajectory generation: use minimum snap trajectory generation method
    //waypoints is the result of path planning (Manual in this homework)
    //启动轨迹生成
    bool isGenerated = trajectoryGeneratorWaypoint.trajGeneration(waypoints);
    if(isGenerated)
    {
        isNewTraj = true;
        //获取可视化数据
        _wp_path_vis_pub.publish(trajectoryGeneratorWaypoint.visWayPointPath());
        _wp_traj_vis_pub.publish(trajectoryGeneratorWaypoint.visWayPointTraj());
    }
}

void callbackOdometry(const nav_msgs::OdometryConstPtr &odom)
{
    Eigen::Vector3d positionBody(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
    Eigen::Vector3d linearVelocity(odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z);

    controller.positionBody = positionBody;
    controller.cVelocityBody = linearVelocity;
}

void callbackImu(const sensor_msgs::ImuConstPtr &imu)
{

    Eigen::Quaterniond quaternion(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
    Eigen::Vector3d linearAcceleration(imu->linear_acceleration.x, imu->linear_acceleration.y,
                                       imu->linear_acceleration.z);
    Eigen::Vector3d omega(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z);

    controller.accelerationBody = linearAcceleration;
    controller.cAngularVelocity_body = omega;
    controller.rotationMatrix_BuW = quaternion.toRotationMatrix();

}

void callbackTarget(const geometry_msgs::PoseConstPtr &pose)
{
    Eigen::Vector3d positionTarget(pose->position.x, pose->position.y, pose->position.z);
    Eigen::Quaterniond quaternionTarget(pose->orientation.w, pose->orientation.x, pose->orientation.y,
                                        pose->orientation.z);

//    controller.updateTargetStates(positionTarget, 0);
}

void updataTarget(int time){
    double roll = 0,pitch = 0,yaw = 0;
//    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
//    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
//    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
//    Eigen::Quaterniond quaternionTarget = yawAngle * pitchAngle * rollAngle;
    Eigen::Vector3d newPose;
//    newPose <<0,0,0;
    controller.updateTargetStates(time);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tracker");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1000);

    ros::Subscriber subIMU = nh.subscribe("/imu", 1, callbackImu);
    ros::Subscriber subOdometry = nh.subscribe("/odom", 1, callbackOdometry);

    //多线程订阅
    ros::AsyncSpinner spinner(4); // Use 4 threads

    ros::Publisher pubRotorRevs = nh.advertise<std_msgs::Float64MultiArray>("/rotorRevs", 1);

    //订阅路径节点
    _way_pts_sub = nh.subscribe("/waypoint_generator/waypoints", 1, rcvWaypointsCallBack);//("waypoints", 1,rcvWaypointsCallBack);//

    //发布可视化节点
    _wp_traj_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_trajectory", 1);
    _wp_path_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_waypoint_path", 1);


    double massQuadcopter = 1;
    controller.initializeParameter(massQuadcopter);
    trajectoryGeneratorWaypoint.init(1, 1);
    double startTime = ros::Time::now().toSec();
    while (ros::ok())
    {
        if(trajectoryGeneratorWaypoint.isTraj==true)
        {
            if(isNewTraj == true)
            {
                startTime = ros::Time::now().toSec();
                isNewTraj = false;
            }
            double t = ros::Time::now().toSec()-startTime;
            if(trajectoryGeneratorWaypoint._totalTime > t)
            {
                Eigen::Vector3d inputDesiredPos = trajectoryGeneratorWaypoint.getTrajectoryStates(t,0);
                Eigen::Vector3d inputDesiredVel = trajectoryGeneratorWaypoint.getTrajectoryStates(t,1);
                Eigen::Vector3d inputDesiredAcc = trajectoryGeneratorWaypoint.getTrajectoryStates(t,2);
                Eigen::Vector3d inputDesiredJerk = trajectoryGeneratorWaypoint.getTrajectoryStates(t,3);
                Eigen::Vector4d revs = controller.getRevs(inputDesiredPos,inputDesiredVel,inputDesiredAcc,inputDesiredJerk);
                std_msgs::Float64MultiArray revsArray;
                revsArray.data = {revs.x(), revs.y(), revs.z(), revs.w()};
                pubRotorRevs.publish(revsArray);
            } else{
                t = trajectoryGeneratorWaypoint._totalTime-0.001;
                Eigen::Vector3d inputDesiredPos = trajectoryGeneratorWaypoint.getTrajectoryStates(t,0);
                Eigen::Vector3d inputDesiredVel = trajectoryGeneratorWaypoint.getTrajectoryStates(t,1);
                Eigen::Vector3d inputDesiredAcc = trajectoryGeneratorWaypoint.getTrajectoryStates(t,2);
                Eigen::Vector3d inputDesiredJerk = trajectoryGeneratorWaypoint.getTrajectoryStates(t,3);
                Eigen::Vector4d revs = controller.getRevs(inputDesiredPos,inputDesiredVel,inputDesiredAcc,inputDesiredJerk);
                std_msgs::Float64MultiArray revsArray;
                revsArray.data = {revs.x(), revs.y(), revs.z(), revs.w()};
                pubRotorRevs.publish(revsArray);
            }

        }
//        else
//        {
//            Eigen::Vector4d revs = controller.getRevs();
//            std_msgs::Float64MultiArray revsArray;
//            revsArray.data = {revs.x(), revs.y(), revs.z(), revs.w()};
//            pubRotorRevs.publish(revsArray);
//        }

//        controller.updatePastStates();
        loop_rate.sleep();
        spinner.start();
    }

    ros::waitForShutdown();
    return 0;
}


