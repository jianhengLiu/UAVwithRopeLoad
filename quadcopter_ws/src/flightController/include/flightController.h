//
// Created by chrisliu on 2020/2/26.
//

#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H

#include <Eigen/Core>
#include <Eigen/Geometry>

#define PI 3.1415926
/*
 * 控制类
 */
class PayloadController
{
public:
    // 时间
    int time;
    //姿态环计数器
    int cnt;

    // 无人机质量
    double massQuadcopter;
    // 差分中的时间变量
    double dt;
     bool first_time = true;
    /**
  * @brief 反馈参数
  */

    // 当前机身角速度
    Eigen::Vector3d cAngularVelocity_body;
    // 当前机身坐标系旋转矩阵
    Eigen::Matrix3d rotationMatrix_BuW;
    // 当前机身坐标系旋转矩阵
    Eigen::Vector3d errorRotationMatrix_inte;
    // 机身位置
    Eigen::Vector3d positionBody;

    // 机身速度
    Eigen::Vector3d cVelocityBody;

    // 机身加速度
    Eigen::Vector3d accelerationBody;

    /**
    * @brief 期望参数
    */
    //机身期望yaw角
    double cDesiredPsi;
    // 期望机身yaw叫速度
    double cDesiredDotPsi;
    // 期望推力
    double u1;
    // 当前目标位置
    Eigen::Vector3d cDesiredPositionBody;
    // 上次目标位置
    Eigen::Vector3d pDesiredPositionBody;
    // 上次机期望身上次速度
    Eigen::Vector3d pDesiredVelocityBody;
    // 期望机身速度
    Eigen::Vector3d cDesiredVelocityBody;
    // 期望机身加速度
    Eigen::Vector3d dAccelerationBody;
    // 期望机身加加速度
    Eigen::Vector3d cDesiredJerk;
    // 期望机角速度
    Eigen::Vector3d cDesiredWd;
    // 上次期望机角速度
    Eigen::Vector3d pDesiredWd;
    // 期望机角加速度
    Eigen::Vector3d cDesiredDotWd;
    // 期望机身四元数
    Eigen::Quaterniond desiredQuaterniondBody;
    // 期望机身旋转矩阵
    Eigen::Matrix3d desiredRotationMatrix;




    /**
    * @brief 初始化参数
    * @param None
    * @retval None
    */
    void initializeParameter(double inputMassQuadcopter);
    /**
    * @brief 更新无人机状态
    * @param None
    * @retval None
    */
    void updatePastStates();
    /**
    * @brief 更新目标状态
    * @param None
    * @retval None
    */
    void updateTargetStates(int cnt);
    /**
    * @brief 计算螺旋桨拉力
    * @param None
    * @retval None
    */
    Eigen::Vector4d getRevs();


private:
    /**
    * @brief 求差分
    * @param None
    * @retval None
    */
    Eigen::Vector3d getVectorD(Eigen::Vector3d cVector, Eigen::Vector3d pVector);
    /**
    * @brief 反对称阵到向量
    * @param None
    * @retval None
    */
    Eigen::Vector3d antisymmetricMatrixToVector(Eigen::Matrix3d antisymmetricMatrix);
    /**
    * @brief 分配拉力
    * @param None
    * @retval None
    */

    Eigen::Vector4d getAllocatedRevs(double Force, Eigen::Vector3d Moment);
};


#endif //FLIGHT_CONTROLLER_H
