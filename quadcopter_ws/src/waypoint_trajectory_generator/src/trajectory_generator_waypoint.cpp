#include "trajectory_generator_waypoint.h"
#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;
using namespace Eigen;

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint()
{}

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


Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
        const int d_order,                    // the order of derivative
        const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
        const Eigen::MatrixXd &Vel,           // boundary velocity
        const Eigen::MatrixXd &Acc,           // boundary acceleration
        const Eigen::MatrixXd &Jerk,           // boundary jerk
        const Eigen::VectorXd &Time)          // time allocation in each segment
{
    // enforce initial and final velocity and accleration, for higher order derivatives, just assume them be 0;
    int p_order = 2 * d_order - 1;              // the order of polynomial
    int p_num1d = p_order + 1;                  // the number of variables in each segment

    int m = Time.size();                          // the number of segments

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
                        pow(Time(j), i + l - p_order) / (i + l - p_order);

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
                        Factorial(i) / Factorial(i - k) * pow(Time(j), i - k);
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

    return PolyCoeff;
}

VectorXd
get_dF(int k, int m, const int d_order, const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
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