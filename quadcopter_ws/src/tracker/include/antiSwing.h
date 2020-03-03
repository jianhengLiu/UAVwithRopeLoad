//
// Created by chrisliu on 2020/2/26.
//

#ifndef TRACKER_ANTISWING_H
#define TRACKER_ANTISWING_H

#include <Eigen/Core>
#include <Eigen/Geometry>

//TODO: union by class
//class

Eigen::Vector3d getOrientationD(Eigen::Vector3d cVector, Eigen::Vector3d pVector);

Eigen::Vector3d getOrientationDD(Eigen::Vector3d cVector, Eigen::Vector3d pVector,Eigen::Vector3d ppVector);

//输入向量
//输出相应的反对称矩阵
Eigen::Matrix3d vectorToAntisymmetricMatrix(Eigen::Vector3d vector);

Eigen::Vector3d antisymmetricMatrixToVector(Eigen::Matrix3d antisymmetricMatrix);

//输入XY方向在图片上的误差
//返回机体坐标系下的单位向量
Eigen::Vector3d pixelerrorToVector(double errorX, double errorY, double length, int resolutionX, int resolutionY);

//输入机体坐标系下的向量
//输出世界坐标系下的向量
Eigen::Vector3d toVectorRefToWorld(Eigen::Vector3d currentOrientation, Eigen::Matrix3d rotationMatrix);

//输入世界坐标系下的物体向量及期望向量
//返回定义的方向的相对误差
Eigen::Vector3d getOrientationError(Eigen::Vector3d currentOrientation, Eigen::Vector3d desiredOrientation);

//输入世界坐标系下的当前与上一刻的物体向量及期望向量
//返回定义的一阶方向相对误差
Eigen::Vector3d
getOrientationErrorD(Eigen::Vector3d orientation, Eigen::Vector3d orientationD, Eigen::Vector3d desiredOrientation,
                     Eigen::Vector3d desiredOrientationD);

Eigen::Vector3d getForcePD(Eigen::Vector3d error, Eigen::Vector3d errorD, Eigen::Vector3d k_q, Eigen::Vector3d k_w);

Eigen::Vector3d
getForceFF(double massiveBody, double length, Eigen::Vector3d orientation, Eigen::Vector3d orientationD,
           Eigen::Vector3d desiredOrientation,
           Eigen::Vector3d desiredOrientationD, Eigen::Vector3d desiredOrientationDD);

Eigen::Vector4d getRevs(double Force, Eigen::Vector3d Moment);

Eigen::Vector3d getMoment(Eigen::Vector3d force, Eigen::Matrix3d rotationMatrix, Eigen::Vector3d omega);


#endif //TRACKER_ANTISWING_H
