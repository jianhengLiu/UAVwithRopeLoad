//
// Created by chrisliu on 2020/2/26.
//

#ifndef TRACKER_ANTISWING_H
#define TRACKER_ANTISWING_H

#include <Eigen/Core>
#include <Eigen/Geometry>

class PayloadController
{
public:

    double massQuadcopter;
    double massPayload;
    double length;



    Eigen::Vector3d cAngularVelocity_body;
    Eigen::Matrix3d rotationMatrix_BuW;

    Eigen::Vector3d positionBody;
    Eigen::Vector3d cDesiredPositionBody;
    Eigen::Vector3d pDesiredPositionBody;


    Eigen::Vector3d velocityBody;
    Eigen::Vector3d desiredVelocityBody;

    Eigen::Quaterniond desiredQuaterniondBody;

    Eigen::Vector3d cVector_world;
    Eigen::Vector3d pVector_world;
    Eigen::Vector3d ppVector_world;

    Eigen::Vector3d cVectorD_world;


    Eigen::Vector3d cDesiredVector_world;
    Eigen::Vector3d pDesiredVector_world;
    Eigen::Vector3d ppDesiredVector_world;

    Eigen::Vector3d cDesiredVectorD_world;
    Eigen::Vector3d cDesiredVectorDD_world;

    Eigen::Vector3d acceleration;
    Eigen::Vector3d desiredAcceleration;

    Eigen::Vector3d cOrientation_world;
    Eigen::Vector3d cOrientationD_world;



    Eigen::Vector3d cDesiredOrientation_world;
    Eigen::Vector3d cDesiredOrientationD_world;
    Eigen::Vector3d cDesiredOrientationDD_world;

    Eigen::Vector3d pDesiredOrientation_world;
    Eigen::Vector3d ppDesiredOrientation_world;

    void initializeParameter(double inputMassQuadcopter, double inputMassPayload,double inputLength);

    //输入XY方向在图片上的误差
    //返回机体坐标系下的单位向量
    Eigen::Vector3d pixelerrorToVector(double errorX, double errorY, double length, int resolutionX, int resolutionY);

    //输入机体坐标系下的向量
    //输出世界坐标系下的向量
    void updatePayloadStates(Eigen::Vector3d vector_body);

    void updateDesiredPayloadStates(Eigen::Vector3d inputDesiredVector_world);

    void updatePastStates();

    void updateTargetStates(Eigen::Vector3d positionTarget,Eigen::Quaterniond quaternionTarget);


    Eigen::Vector4d getRevs();


private:

    Eigen::Vector3d getVectorD(Eigen::Vector3d cVector, Eigen::Vector3d pVector);

    Eigen::Vector3d getVectorDD(Eigen::Vector3d cVector, Eigen::Vector3d pVector, Eigen::Vector3d ppVector);


    //输入向量
    //输出相应的反对称矩阵
    Eigen::Matrix3d vectorToAntisymmetricMatrix(Eigen::Vector3d vector);

    Eigen::Vector3d antisymmetricMatrixToVector(Eigen::Matrix3d antisymmetricMatrix);


    //输入世界坐标系下的物体向量及期望向量
    //返回定义的方向的相对误差
    Eigen::Vector3d getOrientationError();

    //输入世界坐标系下的当前与上一刻的物体向量及期望向量
    //返回定义的一阶方向相对误差
    Eigen::Vector3d getOrientationErrorD();

    Eigen::Vector3d
    getForceFF(double massiveBody, double length, Eigen::Vector3d orientation, Eigen::Vector3d orientationD,
               Eigen::Vector3d desiredOrientation,
               Eigen::Vector3d desiredOrientationD, Eigen::Vector3d desiredOrientationDD);

    Eigen::Vector4d getAllocatedRevs(double Force, Eigen::Vector3d Moment);

    Eigen::Vector3d getMoment(Eigen::Vector3d force, Eigen::Matrix3d rotationMatrix, Eigen::Vector3d omega);
};


#endif //TRACKER_ANTISWING_H
