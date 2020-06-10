#ifndef _TRAJECTORY_GENERATOR_WAYPOINT_H_
#define _TRAJECTORY_GENERATOR_WAYPOINT_H_

#include <Eigen/Eigen>
#include <vector>
#include <math.h>

#include <visualization_msgs/Marker.h>

class TrajectoryGeneratorWaypoint
{
private:
    const int d_order = 6;// 决定了要最小化的阶次，这里minimum x^(6)

    double _Vel, _Acc;

    Eigen::MatrixXd _waypoints;

    Eigen::VectorXd get_dF(int k, int m, const Eigen::MatrixXd &Path); // waypoints coordinates (3d));

    double timeAllocation_1D(double dis);

    double Factorial(int x); //阶乘

    void PolyQPGeneration(const Eigen::MatrixXd &Path); //轨迹系数生成函数

    void timeAllocation(Eigen::MatrixXd Path); //时间分配函数

    /**
     *
     * @param k : 第几段轨迹
     * @param t_seg 对应轨迹段里的时间点
     * @param order :p=0,v=1,a=2,...
     * @return 轨迹段里的时间点对应order的x,y,z
     */
    Eigen::Vector3d getPolyStates(int k, double t_seg, int order);

public:
    bool isTraj = false;

    double _totalTime = 0;

    Eigen::MatrixXd _polyCoeff;
    Eigen::VectorXd _polyTime;

    TrajectoryGeneratorWaypoint();

    ~TrajectoryGeneratorWaypoint();

    void init(double Vel,double Acc);//必须初始化速度和加速度

    /**
     *
     * @param waypoints 输入的期望路径（中间点）
     * @return
     */
    bool trajGeneration(Eigen::MatrixXd waypoints);//启动轨迹生成函数

    /**
     *
     * @param t 此处t是对应全局时间，即轨迹开始到t的时间
     * @param order :p=0,v=1,a=2,...
     * @return 包含对应order的x,y,z信息,组成如下
     * p:x,y,z
     * v:x,y,z
     * a:x,y,z
     * ...
     */
    Eigen::Vector3d getTrajectoryStates(double t, int order);

    visualization_msgs::Marker visWayPointPath();//返回用于可视化的路径Marker
    visualization_msgs::Marker visWayPointTraj();//返回用于可视化的轨迹Marker
};


#endif