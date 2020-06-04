#ifndef _TRAJECTORY_GENERATOR_WAYPOINT_H_
#define _TRAJECTORY_GENERATOR_WAYPOINT_H_

#include <Eigen/Eigen>
#include <vector>
#include <math.h>

#include <visualization_msgs/Marker.h>

class TrajectoryGeneratorWaypoint
{
private:
    double _qp_cost;
    Eigen::MatrixXd _Q;
    Eigen::VectorXd _Px, _Py, _Pz;
    double _Vel, _Acc;

    Eigen::VectorXd
    get_dF(int k, int m, const int d_order, const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
           const Eigen::MatrixXd &Vel,           // boundary velocity
           const Eigen::MatrixXd &Acc,           // boundary acceleration
           const Eigen::MatrixXd &Jerk);

    double timeAllocation_1D(double dis);

public:
    bool isTraj = 0;

    Eigen::MatrixXd _polyCoeff;
    Eigen::VectorXd _polyTime;

    TrajectoryGeneratorWaypoint();

    ~TrajectoryGeneratorWaypoint();

    void init(double Vel,double Acc);

    void PolyQPGeneration(
            const int order,
            const Eigen::MatrixXd &Path,
            const Eigen::MatrixXd &Vel,
            const Eigen::MatrixXd &Acc,
            const Eigen::MatrixXd &Jerk);

    int Factorial(int x);

    void timeAllocation(Eigen::MatrixXd Path);

//    order:p=0,v=1,a=2,...
//返回对应order的x,y,z
//注意：此处t_seg是对应轨迹段里的时间
    Eigen::Vector3d getPolyStates(int k, double t_seg, int order);

//    返回包含x,y,z信息的矩阵
//    p:x,y,z
//    v:x,y,z
//    a:x,y,z
//注意：此处t是对应全局时间，即轨迹开始到t的时间
    Eigen::MatrixXd getTrajectoryStates(double t);
};


#endif
