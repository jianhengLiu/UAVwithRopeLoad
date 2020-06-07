#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <random>
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
//#include <quadrotor_msgs/PolynomialTrajectory.h>
#include <sensor_msgs/Joy.h>
#include <algorithm>

// Useful customized headers
#include "trajectory_generator_waypoint.h"

#include "eigen_conversions/eigen_msg.h"

using namespace std;
using namespace Eigen;

// Param from launch file
double _vis_traj_width;
double _Vel=1, _Acc=1;

// ros related
ros::Subscriber _way_pts_sub;
ros::Publisher _wp_traj_vis_pub, _wp_path_vis_pub, _polyCoeff_with_time_pub;

// for planning
Vector3d _startPos = Vector3d::Zero();// 初始化初始位置
Vector3d _startVel = Vector3d::Zero();

//
TrajectoryGeneratorWaypoint trajectoryGeneratorWaypoint;

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
    waypoints.row(0) = _startPos;

    for (int k = 0; k < (int) wp_list.size(); k++)
        waypoints.row(k + 1) = wp_list[k];

    //Trajectory generation: use minimum snap trajectory generation method
    //waypoints is the result of path planning (Manual in this homework)
    //启动轨迹生成
    bool isGenerated = trajectoryGeneratorWaypoint.trajGeneration(waypoints);
    if(isGenerated)
    {
        //获取可视化数据
        _wp_path_vis_pub.publish(trajectoryGeneratorWaypoint.visWayPointPath());
        _wp_traj_vis_pub.publish(trajectoryGeneratorWaypoint.visWayPointTraj());

        //获取实时状态变量示例
        double startTime = ros::Time::now().toSec();
        double t = ros::Time::now().toSec()-startTime;
        while (trajectoryGeneratorWaypoint._totalTime > t)
        {
            MatrixXd states = trajectoryGeneratorWaypoint.getTrajectoryStates(t);
//            cout << "p=" << states.row(0) << endl;
//            cout << "v=" << states.row(1) << endl;
//            cout << "a=" << states.row(2) << endl;
            t = ros::Time::now().toSec()-startTime;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_node");
    ros::NodeHandle nh("~");

    nh.param("planning/vel", _Vel, 1.0);
    nh.param("planning/acc", _Acc, 1.0);
    nh.param("vis/vis_traj_width", _vis_traj_width, 0.15);

    trajectoryGeneratorWaypoint.init(_Vel, _Acc);


    //state of start point
    _startPos(0) = 0;
    _startPos(1) = 0;
    _startPos(2) = 0;

    _startVel(0) = 0;
    _startVel(1) = 0;
    _startVel(2) = 0;

    //订阅路径节点
    _way_pts_sub = nh.subscribe("/waypoint_generator/waypoints", 1, rcvWaypointsCallBack);//("waypoints", 1,rcvWaypointsCallBack);//

    //发布可视化节点
    _wp_traj_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_trajectory", 1);
    _wp_path_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_waypoint_path", 1);

    ros::Rate rate(100);
    bool status = ros::ok();
    while (status)
    {
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }
    return 0;
}

