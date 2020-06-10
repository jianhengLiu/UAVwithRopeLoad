/
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
    // 无人机质量
    double massPayload;
    // 绳子长度
    double length;
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
//    欧拉角(Z-Y-X，即RPY)
    Eigen::Vector3d eulerAngle;

    Eigen::Vector3d cPositionPayload;
    Eigen::Vector3d pPositionPayload;
    Eigen::Vector3d ppPositionPayload;

    Eigen::Vector3d cVelocityPayload;
    Eigen::Vector3d cAccelerationPayload;

//    cVector = p = 机体到负载的向量
    Eigen::Vector3d cVector;
    Eigen::Vector3d pVector;
//    机体到负载的向量差分
    Eigen::Vector3d cVectorD;

    Eigen::Vector3d cCalculateVector;
    Eigen::Vector3d pCalculateVector;
//    机体到负载的向量差分
    Eigen::Vector3d cCalculateVectorD;

    Eigen::Matrix3d cCalculateR;
    Eigen::Matrix3d pCalculateR;


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
    void initializeParameter(double inputMassQuadcopter,double inputMassPayload,double inputLength);
    /**
     * @brief 更新负载状态
     * @param inputPositionPayload
     */
    void updatePayloadStates(Eigen::Vector3d inputPositionPayload);
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

    Eigen::Vector4d getRevs(Eigen::Vector3d inputDesiredPos,
                            Eigen::Vector3d inputDesiredVel,
                            Eigen::Vector3d inputDesiredAcc,
                            Eigen::Vector3d inputDesiredJerk);

    Eigen::Vector4d getRevs_Payload(Eigen::Vector3d inputDesiredPos,
                            Eigen::Vector3d inputDesiredVel,
                            Eigen::Vector3d inputDesiredAcc,
                            Eigen::Vector3d inputDesiredJerk);


private:
    /**
    * @brief 求差分
    * @param None
    * @retval None
    */
    Eigen::Vector3d getVectorD(Eigen::Vector3d cVector, Eigen::Vector3d pVector);
    /**
     * 求差差分
     * @param cVector
     * @param pVector
     * @param ppVector
     * @return
     */
    Eigen::Vector3d getVectorDD(Eigen::Vector3d cVector, Eigen::Vector3d pVector, Eigen::Vector3d ppVector);



    /**
    * @brief 反对称阵到向量
    * @param None
    * @retval None
    */
    Eigen::Vector3d antisymmetricMatrixToVector(Eigen::Matrix3d antisymmetricMatrix);
    /**
     * @brief 向量到反对称阵
     * @param vector
     * @return
     */
    Eigen::Matrix3d vectorToAntisymmetricMatrix(Eigen::Vector3d vector);
    /**
    * @brief 分配拉力
    * @param None
    * @retval None
    */
    Eigen::Vector4d getAllocatedRevs(double Force, Eigen::Vector3d Moment);

    void payloadPosController(Eigen::Vector3d inputDesiredPos,
                                         Eigen::Vector3d inputDesiredVel);
    Eigen::Vector3d e_ixL = Eigen::Vector3d(0,0,0);

    void payloadAttitudeController();

    void QuadcopterAttitudeController();
};


#endif //FLIGHT_CONTROLLER_H
