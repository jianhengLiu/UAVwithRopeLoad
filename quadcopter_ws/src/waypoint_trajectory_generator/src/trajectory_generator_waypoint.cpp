#include "../include/trajectory_generator_waypoint.h"
#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;
using namespace Eigen;

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint(double Vel,double Acc)
{
    _Vel = Vel;
    _Acc = Acc;

}

TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint()
{}

//define factorial function, input i, output i!
int TrajectoryGeneratorWaypoint::Factorial(int x)
{
    int fac = 1;
    for (int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}

/*

    STEP 2: Learn the "Closed-form solution to minimum snap" in L5, then finish this PolyQPGeneration function

    variable declaration: input       const int d_order,                    // the order of derivative
                                      const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
                                      const Eigen::MatrixXd &Vel,           // boundary velocity
                                      const Eigen::MatrixXd &Acc,           // boundary acceleration
                                      const Eigen::VectorXd &Time)          // time allocation in each segment
                          output      MatrixXd PolyCoeff(m, 3 * p_num1d);   // position(x,y,z), so we need (3 * p_num1d) coefficients

*/

VectorXd get_dF(int k, int m, const int d_order, const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
                const Eigen::MatrixXd &Vel,           // boundary velocity
                const Eigen::MatrixXd &Acc,           // boundary acceleration
                const Eigen::MatrixXd &Jerk);


void TrajectoryGeneratorWaypoint::PolyQPGeneration(
        const int d_order,                    // the order of derivative
        const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
        const Eigen::MatrixXd &Vel,           // boundary velocity
        const Eigen::MatrixXd &Acc,           // boundary acceleration
        const Eigen::MatrixXd &Jerk)            // boundary jerk
{
    // enforce initial and final velocity and accleration, for higher order derivatives, just assume them be 0;
    int p_order = 2 * d_order - 1;              // the order of polynomial
    int p_num1d = p_order + 1;                  // the number of variables in each segment

    int m = _polyTime.size();                          // the number of segments

    /*   Produce Mapping Matrix A to the entire trajectory, A is a mapping matrix that maps polynomial coefficients to derivatives.   */

//    getQ
//先只计算一个维度的Q，因为我们取三个维度的时间间隔是一样的
//即Q_x = Q_y = Q_z
    MatrixXd Q = MatrixXd::Zero(m * p_num1d, m * p_num1d);
    MatrixXd Qj;
    for (int j = 0; j <= m - 1; ++j)
    {
        Qj = MatrixXd::Zero(p_num1d, p_num1d);
        for (int i = 4; i <= p_order; ++i)
        {
            for (int l = i; l <= p_order; ++l)
            {
                Qj(i, l) =
                        Factorial(i) / Factorial(i - 4) * Factorial(l) / Factorial(l - 4) *
                        pow(_polyTime(j), i + l - p_order) / (i + l - p_order);

                Qj(l, i) = Qj(i, l);
            }
        }
        Q.block(j * p_num1d, j * p_num1d, p_num1d, p_num1d) = Qj;
    }

    /*   Produce the dereivatives in X, Y and Z axis directly.  */
    //  getM
    //先只计算一个维度的M，因为我们取三个维度的时间间隔是一样的，所以可以通过拼接减少运算量
    //即M_x = M_y = M_z
    Eigen::MatrixXd M = MatrixXd::Zero(m * p_num1d, m * p_num1d);
    for (int j = 0; j <= m - 1; ++j)
    {
        for (int k = 0; k <= d_order - 1; ++k)
        {
            M(k + j * p_num1d, k + j * p_num1d) = Factorial(k);
            for (int i = k; i <= p_order; ++i)
            {
                M(4 + k + j * p_num1d, i + j * p_num1d) =
                        Factorial(i) / Factorial(i - k) * pow(_polyTime(j), i - k);
            }
        }
    }


//    getCt
//  Ct  for start point
    Eigen::MatrixXd Ct_start = MatrixXd::Zero(d_order, d_order * (m + 1));
//block 矩阵块（起始位置x，y,大小行，列）
    Ct_start.block(0, 0, d_order, d_order) = MatrixXd::Identity(d_order, d_order);

    //  Ct  for middle point
    Eigen::MatrixXd Ct_mid = MatrixXd::Zero(2 * d_order * (m - 1), d_order * (m + 1));
    Eigen::MatrixXd Cj;
    double start_idx_2 = 0;
    double start_idx_1 = 0;
    for (int j = 0; j <= m - 2; ++j)
    {
        Cj = MatrixXd::Zero(d_order, d_order * (m + 1));
        Cj(0, d_order + j) = 1;
        start_idx_2 = 2 * d_order + m - 1 + 3 * j;
        Cj.block(1, start_idx_2, d_order - 1, d_order - 1) = MatrixXd::Identity(d_order - 1, d_order - 1);
        start_idx_1 = 2 * d_order * j;
        Eigen::MatrixXd Cj_adjacen(2 * d_order, d_order * (m + 1));
        Cj_adjacen << Cj,
                Cj;
        Ct_mid.block(start_idx_1, 0, 2 * d_order, d_order * (m + 1)) = Cj_adjacen;
    }

    //  Ct  for end point
    Eigen::MatrixXd Ct_end = MatrixXd::Zero(d_order, d_order * (m + 1));
    Ct_end.block(0, d_order + m - 1, d_order, d_order) = MatrixXd::Identity(d_order, d_order);

    Eigen::MatrixXd Ct(2 * d_order * m, d_order * (m + 1));
    Ct << Ct_start,
            Ct_mid,
            Ct_end;


    /*   Produce the Minimum Snap cost function, the Hessian Matrix   */
//    即对应的R矩阵

    Eigen::MatrixXd C = Ct.transpose();
    Eigen::MatrixXd R = C * M.inverse().transpose() * Q * M.inverse() * Ct;

//   中间点m-1个，始末点4个边界约束p,v,a,j
//   (m-1)+8 = m+7,最后一个减一是因为位置从0开始
//  待确定的量每个中间点三个,v,a,j
//  3*(m-1)
//  所以总长度
    Eigen::MatrixXd R_pp = R.block(m + 7, m + 7, 3 * (m - 1), 3 * (m - 1));
    Eigen::MatrixXd R_fp = R.block(0, m + 7, m + 7, 3 * (m - 1));

    VectorXd dF_x = get_dF(0, m, d_order, Path, Vel, Acc, Jerk);
    VectorXd dF_y = get_dF(1, m, d_order, Path, Vel, Acc, Jerk);
    VectorXd dF_z = get_dF(2, m, d_order, Path, Vel, Acc, Jerk);

    VectorXd dP_x = -R_pp.inverse() * R_fp.transpose() * dF_x;
    VectorXd dP_y = -R_pp.inverse() * R_fp.transpose() * dF_y;
    VectorXd dP_z = -R_pp.inverse() * R_fp.transpose() * dF_z;

    VectorXd Px(p_num1d * m), Py(p_num1d * m), Pz(p_num1d * m);

    VectorXd d_x(dF_x.size() + dP_x.size(), 1);
    d_x << dF_x,
            dP_x;

    VectorXd d_y(dF_y.size() + dP_y.size());
    d_y << dF_y,
            dP_y;

    VectorXd d_z(dF_z.size() + dP_z.size());
    d_z << dF_z,
            dP_z;

    VectorXd P_x = M.inverse() * Ct * d_x;
    VectorXd P_y = M.inverse() * Ct * d_y;
    VectorXd P_z = M.inverse() * Ct * d_z;

    MatrixXd PolyCoeff = MatrixXd::Zero(m, 3 * p_num1d);// position(x,y,z), so we need (3 * p_num1d) coefficients
    for (int i = 0; i < m; ++i)
    {
        PolyCoeff.block(i, 0, 1, p_num1d) = P_x.block(i * p_num1d, 0, p_num1d, 1).transpose();
    }
    for (int i = 0; i < m; ++i)
    {
        PolyCoeff.block(i, p_num1d, 1, p_num1d) = P_y.block(i * p_num1d, 0, p_num1d, 1).transpose();
    }
    for (int i = 0; i < m; ++i)
    {
        PolyCoeff.block(i, 2 * p_num1d, 1, p_num1d) = P_z.block(i * p_num1d, 0, p_num1d, 1).transpose();
    }

    _polyCoeff = PolyCoeff;
    isTraj = true;
}

VectorXd
TrajectoryGeneratorWaypoint::get_dF(int k, int m, const int d_order,
                                    const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
                                    const Eigen::MatrixXd &Vel,           // boundary velocity
                                    const Eigen::MatrixXd &Acc,           // boundary acceleration
                                    const Eigen::MatrixXd &Jerk)
{
    VectorXd dF(m + 7);


    for (int i = 0; i <= m; ++i)
    {
        if (i == 0)
        {
            dF(0) = Path(0, k);
            dF(1) = Vel(0, k);
            dF(2) = Acc(0, k);
            dF(3) = Jerk(0, k);
        } else if (i == m)
        {
            dF(m + 3) = Path(i, k);
            dF(m + 4) = Vel(1, k);
            dF(m + 5) = Acc(1, k);
            dF(m + 6) = Jerk(1, k);
        } else
        {
            dF(d_order + i - 1) = Path(i, k);
        }

    }

    return dF;
}


double TrajectoryGeneratorWaypoint::timeAllocation_1D(double dis)
{
    double T = 0;
    if (dis <= _Vel * _Vel / _Acc)
    {
        T = 2 * sqrt(dis / _Acc);
    } else
    {
        T = _Vel / _Acc + dis / _Vel;
    }
    return T;
}

void TrajectoryGeneratorWaypoint::timeAllocation(MatrixXd Path)
{
    cout<<Path<<endl;
    VectorXd time(Path.rows() - 1);

    /*

    STEP 1: Learn the "trapezoidal velocity" of "TIme Allocation" in L5, then finish this timeAllocation function

    variable declaration: _Vel, _Acc: _Vel = 1.0, _Acc = 1.0 in this homework, you can change these in the test.launch

    You need to return a variable "time" contains time allocation, which's type is VectorXd

    The time allocation is many relative timeline but not one common timeline

    */

//    trapezoidal velocity
    double delta_x = 0;
    double delta_y = 0;
    double delta_z = 0;

    double time_x = 0;
    double time_y = 0;
    double time_z = 0;

    for (int i = 0; i < time.size(); ++i)
    {
        delta_x = fabs(Path(i + 1, 0) - Path(i, 0));
        delta_y = fabs(Path(i + 1, 1) - Path(i, 1));
        delta_z = fabs(Path(i + 1, 2) - Path(i, 2));

        time_x = timeAllocation_1D(delta_x);
        time_y = timeAllocation_1D(delta_y);
        time_z = timeAllocation_1D(delta_z);

        time(i) = fmax(time_x, fmax(time_y, time_z));
//        time(i) = 2;
    }
    cout << "time:" << endl << time << endl;
    _polyTime = time;
}


Vector3d TrajectoryGeneratorWaypoint::getPolyStates(int k, double t_seg, int order)
{

    int _poly_num1D = _polyCoeff.cols() / 3;

    Vector3d ret;

    for (int dim = 0; dim < 3; dim++)
    {
        VectorXd coeff = (_polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
        VectorXd time = VectorXd::Zero(_poly_num1D);

        for (int j = order; j < _poly_num1D; j++)
            if (j == 0)
                time(j) = 1.0;
            else
                time(j) = Factorial(j) / Factorial(j - order) * pow(t_seg, j - order);

        ret(dim) = coeff.dot(time);
        //cout << "dim:" << dim << " coeff:" << coeff << endl;
    }

    return ret;
}

MatrixXd TrajectoryGeneratorWaypoint::getTrajectoryStates(double t)
{
//    每一段轨迹的时间起始于0
//    t所在的时间段的初始时间
    double t_init = 0;
//    t在其对应时间段上的时间
    double t_seg = 0;

    int seg_idx = 0;

    for (int i = 0; i < _polyTime.size(); i++)
    {
        if (t >= t_init + _polyTime(i))
        {
            t_init += _polyTime(i);
        } else
        {
            t_seg = t - t_init;
            seg_idx = i;
            break;
        }
    }


    Vector3d pos = getPolyStates(seg_idx, t_seg, 0);
    Vector3d vel = getPolyStates(seg_idx, t_seg, 1);
    Vector3d acc = getPolyStates(seg_idx, t_seg, 2);

//    p:x,y,z
//    v:x,y,z
//    a:x,y,z
    MatrixXd states(3, 3);
    states << pos.transpose(),
            vel.transpose(),
            acc.transpose();
    return states;
}