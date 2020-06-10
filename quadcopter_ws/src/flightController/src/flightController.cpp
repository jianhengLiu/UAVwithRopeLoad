//
// Created by chrisliu on 2020/2/26.
//

#include "flightController.h"
#include "database.h"
#include "iostream"
using namespace std;

void PayloadController::initializeParameter(double inputMassQuadcopter,double inputMassPayload,double inputLength)
{
    massQuadcopter = inputMassQuadcopter;
    massPayload = inputMassPayload;
    length = inputLength;
    dt = 0.001;
    cDesiredPositionBody.Zero();
}

void PayloadController::updatePayloadStates(Eigen::Vector3d inputPositionPayload)
{
//TODO：没有初始化负载状态
    ppPositionPayload = pPositionPayload;
    pPositionPayload = cPositionPayload;
    cPositionPayload = inputPositionPayload;

    cVelocityPayload = getVectorD(cPositionPayload,pPositionPayload);
    cAccelerationPayload = getVectorDD(cPositionPayload,pPositionPayload,ppPositionPayload);
}

void PayloadController::updatePastStates()
{
    pDesiredPositionBody = cDesiredPositionBody;
}

void PayloadController::updateTargetStates( int cnt)
{
//    pDesiredVelocityBody = cDesiredVelocityBody;
//    cDesiredPositionBody = positionTarget;
//    cDesiredVelocityBody = getVectorD(cDesiredPositionBody, pDesiredPositionBody);
//    dAccelerationBody = getVectorD(cDesiredVelocityBody,pDesiredVelocityBody);
//    desiredQuaterniondBody = quaternionTarget;

    cDesiredPositionBody << planning[cnt][1],planning[cnt][2],planning[cnt][3];
    cDesiredVelocityBody << planning[cnt][4],planning[cnt][5],planning[cnt][6];
    dAccelerationBody  << planning[cnt][7],planning[cnt][8],planning[cnt][9];
    cDesiredJerk << planning[cnt][10],planning[cnt][11],planning[cnt][12];
//    cDesiredPositionBody<< 1,1,3;
//    cDesiredVelocityBody << 0,0,0;
//    dAccelerationBody  << 0,0,0;
//    cDesiredJerk <<0,0,0;
    cDesiredPsi = 0;
    cDesiredDotPsi = 0;

}


Eigen::Vector3d PayloadController::getVectorD(Eigen::Vector3d cVector, Eigen::Vector3d pVector)
{
    return (cVector - pVector)/(dt);
}

Eigen::Vector3d
PayloadController::getVectorDD(Eigen::Vector3d cVector, Eigen::Vector3d pVector, Eigen::Vector3d ppVector)
{
    return (cVector - 2 * pVector + ppVector)/dt/dt;
}

Eigen::Vector3d PayloadController::antisymmetricMatrixToVector(Eigen::Matrix3d antisymmetricMatrix)
{
    Eigen::Vector3d vector(antisymmetricMatrix(7), antisymmetricMatrix(2), antisymmetricMatrix(3));

    return vector;
}

Eigen::Matrix3d PayloadController::vectorToAntisymmetricMatrix(Eigen::Vector3d vector)
{
    Eigen::Matrix3d antisymmetricMatrix;
    antisymmetricMatrix << 0, -vector.z(), vector.y(),
            vector.z(), 0, -vector.x(),
            -vector.y(), vector.x(), 0;
    return antisymmetricMatrix;
}

Eigen::Vector4d PayloadController::getAllocatedRevs(double Force, Eigen::Vector3d Moment)
{
    Eigen::Matrix4d Minvese;
    double sqrt2 = sqrt(2);
    Minvese << 1, -sqrt2, sqrt2, 1,
            1, -sqrt2, -sqrt2, -1,
            1, sqrt2, -sqrt2, 1,
            1, sqrt2, sqrt2, -1;
    Minvese = 0.25 * Minvese;

    Eigen::Vector4d input(Force, Moment.x(), Moment.y(), Moment.z());
    Eigen::Vector4d revs = Minvese * input;
    if (revs.x() < 0)
    {
        revs.x() = 0;
    }
    if (revs.y() < 0)
    {
        revs.y() = 0;
    }
    if (revs.z() < 0)
    {
        revs.z() = 0;
    }
    if (revs.w() < 0)
    {
        revs.w() = 0;
    }
    revs.x() = sqrt(revs.x());
    revs.y() = sqrt(revs.y());
    revs.z() = sqrt(revs.z());
    revs.w() = sqrt(revs.w());
    return revs;
}

Eigen::Vector4d PayloadController::getRevs()
{

    // 位置环 100Hz
    if(cnt>5 || first_time)
    {
        if(time<1739 ){time ++;}
        updateTargetStates(time);

        first_time = false;
        cnt = 0;
        // 计算误差 (2)
        Eigen::Vector3d e_p = positionBody - cDesiredPositionBody;
        Eigen::Vector3d e_v = cVelocityBody - cDesiredVelocityBody;
        std::cout<<e_p.transpose()<<" / "<<e_v.transpose()<<std::endl;
        // 设置状态反馈误差增益 (3)
        const Eigen::Vector3d K_p = Eigen::Vector3d(0.65, 0.65, 5);
        const Eigen::Vector3d K_v = Eigen::Vector3d(1.7, 1.7, 5);

        // 计算期望力矩（3）
        Eigen::Vector3d zW(0, 0, -1);
        Eigen::Vector3d F_n = -e_p.cwiseProduct(K_p) - e_v.cwiseProduct(K_v)
                              +(massQuadcopter) * (9.8 * -zW ) + (massQuadcopter) * dAccelerationBody;;

        Eigen::Vector3d F = F_n;
//    保方向饱和函数
//        if (F.norm() > 50)
//        {
//            F = 50 * F / F.norm();
//        }

        // 计算无人机拉力方向 （5）
        Eigen::Vector3d zB_des = F / F.norm();
        // 计算中间坐标系（7）
        Eigen::Vector3d xC_des(cos(cDesiredPsi), sin(cDesiredPsi), 0);
        // 计算yB和xB旋转 （8）
        Eigen::Vector3d yB_des, xB_des;
        yB_des = zB_des.cross(xC_des);
        yB_des /= yB_des.norm();

        xB_des = yB_des.cross(zB_des);

        // 计算目标推力 （4）
        u1 = zB_des.transpose() * F;


        // 计算目标旋转矩阵 (9)
//    Eigen::Matrix3d desiredRotationMatrix;
        desiredRotationMatrix.col(0) = xB_des;
        desiredRotationMatrix.col(1) = yB_des;
        desiredRotationMatrix.col(2) = zB_des;


        // 计算目标角速度 (12)
//    Eigen::Vector3d zW;
//    zW << 0, 0, 1;
        Eigen::Vector3d hw = massQuadcopter / u1 * (cDesiredJerk  - (zB_des.dot(cDesiredJerk)) * zB_des);
        double wx = - hw.dot(yB_des) ;
        double wy =   hw.dot(xB_des) ;
        double wz = cDesiredDotPsi * zW.dot(zB_des) ;
        Eigen::Vector3d w_BW;
        w_BW << wx, wy, wz;
        pDesiredWd = cDesiredWd;
        cDesiredWd = w_BW;
        cDesiredDotWd = getVectorD(cDesiredDotWd,pDesiredWd);

    }
    cnt++;



    // 姿态环 1kHz

    // 计算旋转误差 （10）
    Eigen::Vector3d errorRotationMatrix;
    errorRotationMatrix = 0.5 * antisymmetricMatrixToVector((desiredRotationMatrix.transpose() * rotationMatrix_BuW -
                                                             rotationMatrix_BuW.transpose() * desiredRotationMatrix));

    // 计算角速度误差 (11)
    Eigen::Vector3d errorAngular =   cAngularVelocity_body -cDesiredWd;//小角度假设下可忽略： - rotationMatrix_BuW.transpose() * desiredRotationMatrix * desiredAngular;


    // 计算三轴动量
    const Eigen::Vector3d k_p = Eigen::Vector3d(3, 3, 2);
    const Eigen::Vector3d k_d = Eigen::Vector3d(0.8, 0.8, 1);
    Eigen::Vector3d J_q;
    J_q<<5,5,5;
    Eigen::Vector3d nonlin;
    nonlin = (cAngularVelocity_body.cross(rotationMatrix_BuW.transpose()* desiredRotationMatrix * cDesiredWd)  -
              rotationMatrix_BuW.transpose() * desiredRotationMatrix * cDesiredDotWd);

    Eigen::Vector3d moment =
            - errorRotationMatrix.cwiseProduct(k_p)
            + errorAngular.cwiseProduct(k_d);
//    + J_q.cwiseProduct(nonlin);

//            保方向饱和函数
//    if (moment.norm() > 5)
//    {
//        moment = 5 * moment / moment.norm();
//    }

    Eigen::Vector4d revs = getAllocatedRevs(u1, moment);

    return revs;
}

Eigen::Vector4d PayloadController::getRevs(Eigen::Vector3d inputDesiredPos,
                                           Eigen::Vector3d inputDesiredVel,
                                           Eigen::Vector3d inputDesiredAcc,
                                           Eigen::Vector3d inputDesiredJerk)
{
    // 位置环 100Hz
    if(cnt>5 || first_time)
    {
        cDesiredPositionBody = inputDesiredPos;
        cDesiredVelocityBody = inputDesiredVel;
        dAccelerationBody  = inputDesiredAcc;
        cDesiredJerk = inputDesiredJerk;
        cDesiredPsi = 0;
        cDesiredDotPsi = 0;

        first_time = false;
        cnt = 0;
        // 计算误差 (2)
        Eigen::Vector3d e_p = positionBody - cDesiredPositionBody;
        Eigen::Vector3d e_v = cVelocityBody - cDesiredVelocityBody;
//        std::cout<<e_p.transpose()<<" / "<<e_v.transpose()<<std::endl;
        // 设置状态反馈误差增益 (3)
        const Eigen::Vector3d K_p = Eigen::Vector3d(0.65, 0.65, 5);
        const Eigen::Vector3d K_v = Eigen::Vector3d(1.7, 1.7, 5);

        // 计算期望力矩（3）
        Eigen::Vector3d zW(0, 0, -1);
        Eigen::Vector3d F_n = -e_p.cwiseProduct(K_p) - e_v.cwiseProduct(K_v)
                              +(massQuadcopter) * (9.8 * -zW ) + (massQuadcopter) * dAccelerationBody;;

        Eigen::Vector3d F = F_n;
//    保方向饱和函数
//        if (F.norm() > 50)
//        {
//            F = 50 * F / F.norm();
//        }

        // 计算无人机拉力方向 （5）
        Eigen::Vector3d zB_des = F / F.norm();
        // 计算中间坐标系（7）
        Eigen::Vector3d xC_des(cos(cDesiredPsi), sin(cDesiredPsi), 0);
        // 计算yB和xB旋转 （8）
        Eigen::Vector3d yB_des, xB_des;
        yB_des = zB_des.cross(xC_des);
        yB_des /= yB_des.norm();

        xB_des = yB_des.cross(zB_des);

        // 计算目标推力 （4）
        u1 = zB_des.transpose() * F;


        // 计算目标旋转矩阵 (9)
//    Eigen::Matrix3d desiredRotationMatrix;
        desiredRotationMatrix.col(0) = xB_des;
        desiredRotationMatrix.col(1) = yB_des;
        desiredRotationMatrix.col(2) = zB_des;


        // 计算目标角速度 (12)
//    Eigen::Vector3d zW;
//    zW << 0, 0, 1;
        Eigen::Vector3d hw = massQuadcopter / u1 * (cDesiredJerk  - (zB_des.dot(cDesiredJerk)) * zB_des);
        double wx = - hw.dot(yB_des) ;
        double wy =   hw.dot(xB_des) ;
        double wz = cDesiredDotPsi * zW.dot(zB_des) ;
        Eigen::Vector3d w_BW;
        w_BW << wx, wy, wz;
        pDesiredWd = cDesiredWd;
        cDesiredWd = w_BW;
        cDesiredDotWd = getVectorD(cDesiredDotWd,pDesiredWd);

    }
    cnt++;



    // 姿态环 1kHz

    // 计算旋转误差 （10）
    Eigen::Vector3d errorRotationMatrix;
    errorRotationMatrix = 0.5 * antisymmetricMatrixToVector((desiredRotationMatrix.transpose() * rotationMatrix_BuW -
                                                             rotationMatrix_BuW.transpose() * desiredRotationMatrix));

    // 计算角速度误差 (11)
    Eigen::Vector3d errorAngular =   cAngularVelocity_body -cDesiredWd;//小角度假设下可忽略： - rotationMatrix_BuW.transpose() * desiredRotationMatrix * desiredAngular;


    // 计算三轴动量
    const Eigen::Vector3d k_p = Eigen::Vector3d(3, 3, 2);
    const Eigen::Vector3d k_d = Eigen::Vector3d(0.8, 0.8, 1);
    Eigen::Vector3d J_q;
    J_q<<5,5,5;
    Eigen::Vector3d nonlin;
    nonlin = (cAngularVelocity_body.cross(rotationMatrix_BuW.transpose()* desiredRotationMatrix * cDesiredWd)  -
              rotationMatrix_BuW.transpose() * desiredRotationMatrix * cDesiredDotWd);

    Eigen::Vector3d moment =
            - errorRotationMatrix.cwiseProduct(k_p)
            + errorAngular.cwiseProduct(k_d);
//    + J_q.cwiseProduct(nonlin);

//            保方向饱和函数
//    if (moment.norm() > 5)
//    {
//        moment = 5 * moment / moment.norm();
//    }


//    cout<<"u1:"<<u1<<endl;
//    cout<<"moment:"<<endl<<moment<<endl;
    Eigen::Vector4d revs = getAllocatedRevs(u1, moment);

    return revs;
}

Eigen::Vector4d PayloadController::getRevs_Payload(Eigen::Vector3d inputDesiredPos,
                                           Eigen::Vector3d inputDesiredVel,
                                           Eigen::Vector3d inputDesiredAcc,
                                           Eigen::Vector3d inputDesiredJerk)
{
    /**
     * payloadPosController
     */
    pVector = cVector;
    Eigen::Vector3d zI(0, 0, 1);
    cVector = -(cAccelerationPayload+9.8*zI)/(cAccelerationPayload+9.8*zI).norm();
    cVectorD = getVectorD(cVector,pVector);

    Eigen::Vector3d e_xL = cPositionPayload - inputDesiredPos;
    Eigen::Vector3d e_dxL = cVelocityPayload - inputDesiredVel;
    e_ixL += e_xL;

    const Eigen::Vector3d k_x = Eigen::Vector3d(3, 3, 2);
    const Eigen::Vector3d k_v = Eigen::Vector3d(0.8, 0.8, 1);
    const Eigen::Vector3d k_i = Eigen::Vector3d(0.8, 0.8, 1);
    Eigen::Vector3d A = (massPayload+massQuadcopter)*(-e_xL.cwiseProduct(k_x)-e_dxL.cwiseProduct(k_v)-e_ixL.cwiseProduct(k_i))
                        +(massPayload+massQuadcopter)*(cAccelerationPayload+9.8*zI)+massQuadcopter*length*cVectorD.dot(cVectorD)*cVector;

    pCalculateVector = cCalculateVector;
    cCalculateVector = -A/A.norm();
    cCalculateVectorD = getVectorD(cCalculateVector,pCalculateVector);

    /**
     * payloadAttitudeController
     */
    Eigen::Vector3d e_p = vectorToAntisymmetricMatrix(cVector)^2*cCalculateVector;
    Eigen::Vector3d e_dp = cVectorD - (cCalculateVector.cross(cCalculateVectorD).cross(cVector));

    const Eigen::Vector3d k_p = Eigen::Vector3d(3, 3, 2);
    const Eigen::Vector3d k_p_d = Eigen::Vector3d(0.8, 0.8, 1);
    Eigen::Vector3d F = massQuadcopter*length*(-vectorToAntisymmetricMatrix(cVector)^2*cCalculateVector);

    Eigen::Vector3d b_3c = F/F.norm();
//    中间坐标系
    Eigen::Vector3d c_1c(cos(eulerAngle(0)),sin(eulerAngle(0)));
    pCalculateR = cCalculateR;
    cCalculateR.col(0) = ((b_3c.cross(c_1c))/(b_3c.cross(c_1c)).norm()).cross(b_3c);
    cCalculateR.col(1) = ((b_3c.cross(c_1c))/(b_3c.cross(c_1c)).norm());
    cCalculateR.col(2) = b_3c;

    Eigen::Matrix3d cCalculateR_D = (cCalculateR-pCalculateR)/dt;

    Eigen::Matrix3d caculateOmega_head = cCalculateR.transpose()*cCalculateR_D;
    /**
     * QuadcopterAttitudeController
     */
}

void PayloadController::payloadPosController(Eigen::Vector3d inputDesiredPos,
                                                        Eigen::Vector3d inputDesiredVel)
{
    pVector = cVector;
    Eigen::Vector3d zI(0, 0, 1);
    cVector = -(cAccelerationPayload+9.8*zI)/(cAccelerationPayload+9.8*zI).norm();
    cVectorD = getVectorD(cVector,pVector);

    Eigen::Vector3d e_xL = cPositionPayload - inputDesiredPos;
    Eigen::Vector3d e_dxL = cVelocityPayload - inputDesiredVel;
    e_ixL += e_xL;

    const Eigen::Vector3d k_x = Eigen::Vector3d(3, 3, 2);
    const Eigen::Vector3d k_v = Eigen::Vector3d(0.8, 0.8, 1);
    const Eigen::Vector3d k_i = Eigen::Vector3d(0.8, 0.8, 1);
    Eigen::Vector3d A = (massPayload+massQuadcopter)*(-e_xL.cwiseProduct(k_x)-e_dxL.cwiseProduct(k_v)-e_ixL.cwiseProduct(k_i))
                        +(massPayload+massQuadcopter)*(cAccelerationPayload+9.8*zI)+massQuadcopter*length*cVectorD.dot(cVectorD)*cVector;

    pCalculateVector = cCalculateVector;
    cCalculateVector = -A/A.norm();
    cCalculateVectorD = getVectorD(cCalculateVector,pCalculateVector);

}

void payloadAttitudeController(){

}

void QuadcopterAttitudeController(){

}