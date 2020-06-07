# Minimum Snap浅度还行文



## 0. 轨迹多项式阶数的选择

每一段轨迹，我们有两个约束点，因为最小化snap，可将snap视为变量。因此，由连续性约束，即得保证0-3阶导数应该是连续的，我们即有2x4个约束条件，即可以最高求出8个参数，所以为了保证轨迹得平滑性，选取11阶得多项式

得每段轨迹得表达形式

![image-20200607202044943](C:\Users\94367\AppData\Roaming\Typora\typora-user-images\image-20200607202044943.png)

## 1. 代价函数

主要思想，通过优化代价函数得到对应系数使得最小化期望的状态变量（snap）

![image-20200607201259698](C:\Users\94367\AppData\Roaming\Typora\typora-user-images\image-20200607201259698.png)

对于其中一条的轨迹，通过简单的计算可以得到代价函数的矩阵形式

![image-20200607202226166](C:\Users\94367\AppData\Roaming\Typora\typora-user-images\image-20200607202226166.png)

得到的代价矩阵一般为二次正定，这样才能保证能够通过凸优化得到期望的参数，把这个矩阵记为**Q**

```cpp
//    getQ
//先只计算一个维度的Q，因为我们取三个维度的时间间隔是一样的
//即Q_x = Q_y = Q_z
    MatrixXd Q = MatrixXd::Zero(m * p_num1d, m * p_num1d);
    MatrixXd Qj;
    for (int j = 0; j <= m - 1; ++j)
    {
        Qj = MatrixXd::Zero(p_num1d, p_num1d);
        for (int i = d_order; i <= p_order; ++i)
        {
            for (int l = i; l <= p_order; ++l)
            {
                Qj(i, l) =
                        Factorial(i) / Factorial(i - d_order) * Factorial(l) / Factorial(l - d_order) *pow(_polyTime(j), i + l - p_order) / (i + l - p_order);
                Qj(l, i) = Qj(i, l);

            }
        }
        Q.block(j * p_num1d, j * p_num1d, p_num1d, p_num1d) = Qj;
    }
```





## 2.边界条件

即起点与终点的状态p,v,a,j，以及中间点的位置（对应轨迹间的相接处）

这些边界条件都是规划前就已知的

![image-20200607202829101](C:\Users\94367\AppData\Roaming\Typora\typora-user-images\image-20200607202829101.png)

## 3. 连续性约束

即保证轨迹间相接点处的v,a,j，在轨迹两端是一致的保证连续

![image-20200607201554475](C:\Users\94367\AppData\Roaming\Typora\typora-user-images\image-20200607201554475.png)

![image-20200607202843740](C:\Users\94367\AppData\Roaming\Typora\typora-user-images\image-20200607202843740.png)



## 4. 时间分配

每段轨迹的运行时间必须是要知道的

选用梯形速度规划获得一个较为保守的时间分配

![image-20200607201943482](C:\Users\94367\AppData\Roaming\Typora\typora-user-images\image-20200607201943482.png)

同时应当尽可能的使用相对时间，保证数值稳定，否则很有可

能会导致越界

![image-20200607201829675](C:\Users\94367\AppData\Roaming\Typora\typora-user-images\image-20200607201829675.png)

```cpp
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
```



## 5. 综上得到了需要求解的二次规划问题

![image-20200607202955755](C:\Users\94367\AppData\Roaming\Typora\typora-user-images\image-20200607202955755.png)

# 求解方法

## 1. 使用凸优化工具进行求解

## 2. 闭式解的系数

观察上述的约束条件，实际是系数到状态变量的映射（计映射为**M**），因此我们可以直接将约束条件代入代价函数中，从而避免了复杂的凸优化求解问题

![image-20200607203327337](C:\Users\94367\AppData\Roaming\Typora\typora-user-images\image-20200607203327337.png)

```cpp
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
                M(d_order + k + j * p_num1d, i + j * p_num1d) =
                        Factorial(i) / Factorial(i - k) * pow(_polyTime(j), i - k);
            }
        }
    }
```



对于状态变量，我们是有部分是已知的，未知的其实只有中间点除了位置的状态变量，因此希望能够重构状态变量向量，将已知的变量$d_F$和未知的变量$d_P$分离出来，这样我们就可以通过矩阵运算直接得到未知的变量进而获得期望的系数啦

![image-20200607211248038](C:\Users\94367\AppData\Roaming\Typora\typora-user-images\image-20200607211248038.png)

而对于这个映射矩阵$C$也有点讲究，我们稍微观察就可以以发现每个中间点其实都有一组重复的状态变量，所以实际我们可以在进行映射的时候也省略掉，这样维数也能够大大的减少呢

![image-20200607211620438](C:\Users\94367\AppData\Roaming\Typora\typora-user-images\image-20200607211620438.png)

```cpp
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
        start_idx_2 = 2 * d_order + m - 1 + (d_order-1) * j;
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
```



求得了所有的状态变量我们就可以很轻易的通过矩阵求逆来求得系数啦

# 代码

```cpp
void TrajectoryGeneratorWaypoint::PolyQPGeneration(
        const Eigen::MatrixXd &Path)            // boundary jerk
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
        for (int i = d_order; i <= p_order; ++i)
        {
            for (int l = i; l <= p_order; ++l)
            {
                Qj(i, l) =
                        Factorial(i) / Factorial(i - d_order) * Factorial(l) / Factorial(l - d_order) *
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
                M(d_order + k + j * p_num1d, i + j * p_num1d) =
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
        start_idx_2 = 2 * d_order + m - 1 + (d_order-1) * j;
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

//   中间点m-1个，始末点6个边界约束p0,v1,a2,j3,snap4,x5
//   (m-1)+2x6 = m+11个约束条件
//  待确定的量每个中间点三个,v1,a2,j3,snap4,x5
//  5*(m-1)
//  所以总长度
    int n_boundary_constraint = (m-1)+2*d_order;
    int n_continuity_constraint = (d_order-1)*(m-1);

    Eigen::MatrixXd R_pp = R.block(n_boundary_constraint, n_boundary_constraint, n_continuity_constraint, n_continuity_constraint);
    Eigen::MatrixXd R_fp = R.block(0, n_boundary_constraint, n_boundary_constraint, n_continuity_constraint);

    VectorXd dF_x = get_dF(0, m, Path);
    VectorXd dF_y = get_dF(1, m, Path);
    VectorXd dF_z = get_dF(2, m, Path);

    VectorXd dP_x = -R_pp.inverse() * R_fp.transpose() * dF_x;
    VectorXd dP_y = -R_pp.inverse() * R_fp.transpose() * dF_y;
    VectorXd dP_z = -R_pp.inverse() * R_fp.transpose() * dF_z;

    VectorXd d_x(dF_x.size() + dP_x.size(), 1);
    d_x << dF_x,
            dP_x;

    VectorXd d_y(dF_y.size() + dP_y.size());
    d_y << dF_y,
            dP_y;

    VectorXd d_z(dF_z.size() + dP_z.size());
    d_z << dF_z,
            dP_z;

    VectorXd Px(p_num1d * m), Py(p_num1d * m), Pz(p_num1d * m);

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

/*
 * 把所有除了位置的边界条件都设为0
 */
VectorXd
TrajectoryGeneratorWaypoint::get_dF(int k, int m,const Eigen::MatrixXd &Path)        // waypoints coordinates (3d))
{
    int n_boundary_constraint = (m-1)+2*d_order;

    VectorXd dF = VectorXd::Zero(n_boundary_constraint);

    for (int i = 0; i <= m; ++i)
    {
        if (i == 0)
        {
            dF(0) = Path(0, k);
        } else if (i == m)
        {
            dF(n_boundary_constraint-d_order) = Path(i, k);
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
```

