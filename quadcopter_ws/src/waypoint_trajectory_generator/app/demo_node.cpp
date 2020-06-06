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
int _poly_num1D;
Vector3d _startPos = Vector3d::Zero();
Vector3d _startVel = Vector3d::Zero();

//
TrajectoryGeneratorWaypoint trajectoryGeneratorWaypoint;

// declare
void visWayPointTraj();

void visWayPointPath(MatrixXd path);

void trajGeneration(Eigen::MatrixXd path);

void rcvWaypointsCallBack(const nav_msgs::Path &wp);

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
    waypoints.row(0) = _startPos;

    for (int k = 0; k < (int) wp_list.size(); k++)
        waypoints.row(k + 1) = wp_list[k];

    //Trajectory generation: use minimum snap trajectory generation method
    //waypoints is the result of path planning (Manual in this homework)
    trajGeneration(waypoints);
}

void trajGeneration(Eigen::MatrixXd path)
{
    MatrixXd vel = MatrixXd::Zero(2, 3);
    MatrixXd acc = MatrixXd::Zero(2, 3);
    MatrixXd jerk = MatrixXd::Zero(2, 3);

    vel.row(0) = _startVel;

    // give an arbitraty time allocation, all set all durations as 1 in the commented function.
    trajectoryGeneratorWaypoint.timeAllocation(path);

    // generate a minimum-snap piecewise monomial polynomial-based trajectory
    trajectoryGeneratorWaypoint.PolyQPGeneration(path);

    visWayPointPath(path);

    visWayPointTraj();

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

    _way_pts_sub = nh.subscribe("/waypoint_generator/waypoints", 1, rcvWaypointsCallBack);//("waypoints", 1,rcvWaypointsCallBack);//


    _wp_traj_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_trajectory", 1);
    _wp_path_vis_pub = nh.advertise<visualization_msgs::Marker>("vis_waypoint_path", 1);

    ros::Rate rate(100);
    bool status = ros::ok();
    while (status)
    {
//        if (trajectoryGeneratorWaypoint.isTraj == true)
//        {
//            MatrixXd states(3, 3);
//            double t = 1;
//            states = trajectoryGeneratorWaypoint.getTrajectoryStates(t);
//            cout << "p=" << states.row(0) << endl;
//            cout << "v=" << states.row(1) << endl;
//            cout << "a=" << states.row(2) << endl;
//        }


        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }
    return 0;
}

void visWayPointPath(MatrixXd path)
{
    visualization_msgs::Marker points, line_list;
    int id = 0;
    points.header.frame_id = line_list.header.frame_id = "map";
    points.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_list.ns = "wp_path";
    points.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_list.pose.orientation.w = 1.0;
    points.pose.orientation.x = line_list.pose.orientation.x = 0.0;
    points.pose.orientation.y = line_list.pose.orientation.y = 0.0;
    points.pose.orientation.z = line_list.pose.orientation.z = 0.0;

    points.id = id;
    line_list.id = id;

    points.type = visualization_msgs::Marker::SPHERE_LIST;
    line_list.type = visualization_msgs::Marker::LINE_STRIP;

    points.scale.x = 0.3;
    points.scale.y = 0.3;
    points.scale.z = 0.3;
    points.color.a = 1.0;
    points.color.r = 0.0;
    points.color.g = 0.0;
    points.color.b = 0.0;

    line_list.scale.x = 0.15;
    line_list.scale.y = 0.15;
    line_list.scale.z = 0.15;
    line_list.color.a = 1.0;


    line_list.color.r = 0.0;
    line_list.color.g = 1.0;
    line_list.color.b = 0.0;

    line_list.points.clear();

    for (int i = 0; i < path.rows(); i++)
    {
        geometry_msgs::Point p;
        p.x = path(i, 0);
        p.y = path(i, 1);
        p.z = path(i, 2);

        points.points.push_back(p);

        if (i < (path.rows() - 1))
        {
            geometry_msgs::Point p_line;
            p_line = p;
            line_list.points.push_back(p_line);
            p_line.x = path(i + 1, 0);
            p_line.y = path(i + 1, 1);
            p_line.z = path(i + 1, 2);
            line_list.points.push_back(p_line);
        }
    }

    _wp_path_vis_pub.publish(points);
    _wp_path_vis_pub.publish(line_list);
}

void visWayPointTraj()
{
    visualization_msgs::Marker _traj_vis;

    _traj_vis.header.stamp = ros::Time::now();
    _traj_vis.header.frame_id = "map";

    _traj_vis.ns = "traj_node/trajectory_waypoints";
    _traj_vis.id = 0;
    _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _traj_vis.action = visualization_msgs::Marker::ADD;
    _traj_vis.scale.x = 0.2;
    _traj_vis.scale.y = 0.2;
    _traj_vis.scale.z = 0.2;
    _traj_vis.pose.orientation.x = 0.0;
    _traj_vis.pose.orientation.y = 0.0;
    _traj_vis.pose.orientation.z = 0.0;
    _traj_vis.pose.orientation.w = 1.0;

    _traj_vis.color.a = 1.0;
    _traj_vis.color.r = 0.0;
    _traj_vis.color.g = 0.0;
    _traj_vis.color.b = 1.0;

    double traj_len = 0.0;
    int count = 0;
    Vector3d cur, pre;
    cur.setZero();
    pre.setZero();

    _traj_vis.points.clear();
    MatrixXd states(3, 3);
    Vector3d pos;
    geometry_msgs::Point pt;

    VectorXd time = trajectoryGeneratorWaypoint._polyTime;

//    double t_temp = 0;
    for (int i = 0; i < time.size(); i++)
    {
        for (double t = 0.0; t < time(i); t += 0.01, count += 1)
        {
//            states = getTrajectoryStates(polyCoeff, time, t+t_temp);
//            cout<<"states"<<endl<<states<<endl;
//            cur(0) = pt.x = states(0,0);
//            cur(1) = pt.y = states(0,1);
//            cur(2) = pt.z = states(0,2);

            pos = trajectoryGeneratorWaypoint.getPolyStates(i, t, 0);
            cur(0) = pt.x = pos(0);
            cur(1) = pt.y = pos(1);
            cur(2) = pt.z = pos(2);
//            cout<<pos<<endl;
            _traj_vis.points.push_back(pt);

            if (count) traj_len += (pre - cur).norm();
            pre = cur;
        }
//        t_temp += time(i);
    }

    _wp_traj_vis_pub.publish(_traj_vis);
}

