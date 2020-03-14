//
// Created by chrisliu on 2020/2/26.
//

#include "antiSwing.h"

#include <iostream>

using namespace std;

void PayloadController::initializeParameter(double inputMassQuadcopter, double inputMassPayload, double inputLength)
{
    massQuadcopter = inputMassQuadcopter;
    massPayload = inputMassPayload;
    length = inputLength;

    cVector_world = Eigen::Vector3d(0, 0, 0);
    pVector_world = Eigen::Vector3d(0, 0, 0);
    pDesiredVector_world = Eigen::Vector3d(0, 0, 0);
    ppDesiredVector_world = Eigen::Vector3d(0, 0, 0);

    cDesiredPositionBody.Zero();
    desiredQuaterniondBody.Identity();
}

void PayloadController::updatePayloadStates(Eigen::Vector3d vector_camera)
{
    Eigen::Matrix3d rotationMatrix_CuB;
    //camera under body:alpha=-180,beta = zeta = 0
    //崔健P25
    rotationMatrix_CuB << 1, 0, 0,
            0, -1, 0,
            0, 0, -1;
    cVector_world = rotationMatrix_BuW * rotationMatrix_CuB * vector_camera;

    cVectorD_world = getVectorD(cVector_world, pVector_world);
    cOrientationD_world = cVectorD_world / cVectorD_world.norm();

    //get this from imu data
//    acceleration = getVectorDD(cVector_world, pVector_world, ppVector_world);

    cOrientation_world = cVector_world / cVector_world.norm();
}

void PayloadController::updateDesiredPayloadStates(Eigen::Vector3d inputDesiredVector_world)
{
    cDesiredVector_world = inputDesiredVector_world;
    cDesiredOrientation_world = cDesiredVector_world / cDesiredVector_world.norm();

    cDesiredVectorD_world = getVectorD(cDesiredVector_world, pDesiredVector_world);

    cDesiredVectorDD_world = getVectorDD(cDesiredVector_world, pDesiredVector_world, ppDesiredVector_world);
}

void PayloadController::updatePastStates()
{

    pDesiredPositionBody = cDesiredPositionBody;

    pVector_world = cVector_world;

    ppDesiredVector_world = pDesiredVector_world;
    pDesiredVector_world = cDesiredVector_world;

    ppDesiredOrientation_world = pDesiredOrientation_world;
    pDesiredOrientation_world = cDesiredOrientation_world;
}

void PayloadController::updateTargetStates(Eigen::Vector3d positionTarget, Eigen::Quaterniond quaternionTarget)
{
    cDesiredPositionBody = positionTarget;
    desiredVelocityBody = getVectorD(cDesiredPositionBody, pDesiredPositionBody);
    desiredQuaterniondBody = quaternionTarget;

}


Eigen::Vector3d PayloadController::getVectorD(Eigen::Vector3d cVector, Eigen::Vector3d pVector)
{
    return cVector - pVector;
}

Eigen::Vector3d
PayloadController::getVectorDD(Eigen::Vector3d cVector, Eigen::Vector3d pVector, Eigen::Vector3d ppVector)
{
    return cVector - 2 * pVector + ppVector;
}

Eigen::Matrix3d PayloadController::vectorToAntisymmetricMatrix(Eigen::Vector3d vector)
{
    Eigen::Matrix3d antisymmetricMatrix;
    antisymmetricMatrix << 0, -vector.z(), vector.y(),
            vector.z(), 0, -vector.x(),
            -vector.y(), vector.x(), 0;
    return antisymmetricMatrix;
}

Eigen::Vector3d PayloadController::antisymmetricMatrixToVector(Eigen::Matrix3d antisymmetricMatrix)
{
    Eigen::Vector3d vector(antisymmetricMatrix(7), antisymmetricMatrix(2), antisymmetricMatrix(3));

    return vector;
}

Eigen::Vector3d
PayloadController::pixelerrorToVector(double errorX, double errorY, double length, int resolutionX, int resolutionY)
{
    float u = errorX * 1 / resolutionX;
    float v = errorY * 1 / resolutionY;
    float det = sqrt(pow(u, 2) + pow(v, 2) + 1);
    det = det * 1 / length;
    Eigen::Vector3d orientation_body = Eigen::Vector3d(-u / det, -v / det, 1 / det);
    return orientation_body;
//
//    p_refToParent = Eigen::Vector3f(-u / det, -v / det, 1 / det);
}


Eigen::Vector3d
PayloadController::getOrientationError()
{
    Eigen::Matrix3d antisymmetricMatrix = vectorToAntisymmetricMatrix(cOrientation_world);
    Eigen::Vector3d orientationError = antisymmetricMatrix * antisymmetricMatrix * cDesiredOrientation_world;
    return orientationError;
}

Eigen::Vector3d PayloadController::getOrientationErrorD()
{
    Eigen::Vector3d orientationErrorD =
            cOrientationD_world - cDesiredOrientation_world.cross(cDesiredOrientationD_world).cross(cOrientation_world);
    return orientationErrorD;
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
//    cout << "1" <<endl<< input << endl;
    return revs;
}

Eigen::Vector3d
PayloadController::getMoment(Eigen::Vector3d force, Eigen::Matrix3d rotationMatrix, Eigen::Vector3d omega)
{
    Eigen::Vector3d b_3 = -force / force.norm();

    Eigen::Vector3d b_1(1, 0, 0);
    Eigen::Vector3d b_2 = b_3.cross(b_1) / b_3.cross(b_1).norm();
    b_1 = b_2.cross(b_3);
    Eigen::Matrix3d R_c;
    R_c.col(0) = b_1;
    R_c.col(1) = b_2;
    R_c.col(2) = b_3;

    Eigen::Vector3d e_R =
            0.5 * antisymmetricMatrixToVector((R_c.transpose() * rotationMatrix - rotationMatrix.transpose() * R_c));

    float k_omega = 1;
    Eigen::Vector3d omega_c = k_omega * e_R;
    Eigen::Vector3d e_omega = omega - rotationMatrix.transpose() * R_c * omega_c;
    Eigen::Vector3d moment = -e_omega + omega.cross(omega);

    return moment;

}

Eigen::Vector4d PayloadController::getRevs()
{
    Eigen::Vector3d errorPosition = positionBody - cDesiredPositionBody;
    Eigen::Vector3d errorVelocity = velocityBody - desiredVelocityBody;
//    cout << "C" << endl << positionBody << endl;
    Eigen::Vector3d errorPayloadPosition = cVector_world - cDesiredVector_world;
    Eigen::Vector3d errorPayloadVelocity = cVectorD_world - cDesiredVectorD_world;

    const Eigen::Vector3d k_bx = Eigen::Vector3d(2.0, 2.0, 1.5);
    const Eigen::Vector3d k_bv = Eigen::Vector3d(2.5, 2.5, 2.5);
    const Eigen::Vector3d k_px = Eigen::Vector3d(0, 0, 0);//(1.0, 1.0, 1.0);
    const Eigen::Vector3d k_pv = Eigen::Vector3d(0, 0, 0);//(1.5, 1.5, 2.5);

    Eigen::Vector3d e3(0, 0, -1);

    Eigen::Vector3d F_n = -errorPosition.cwiseProduct(k_bx) - errorVelocity.cwiseProduct(k_bv) -
                          errorPayloadPosition.cwiseProduct(k_px) -
                          errorPayloadVelocity.cwiseProduct(k_pv) +
                          (massQuadcopter + massPayload) * cDesiredVectorDD_world;
//    cout << "C" << endl << F_n << endl;
    F_n += (massQuadcopter + massPayload) * (-e3 * 9.8);

//    have been valued by function: updateDesiredPayloadStates
    cDesiredOrientation_world = cDesiredVector_world / cDesiredVector_world.norm();
    cDesiredOrientationD_world = getVectorD(cDesiredOrientation_world, pDesiredOrientation_world);
    cDesiredOrientationDD_world = getVectorDD(cDesiredOrientation_world, pDesiredOrientation_world,
                                              ppDesiredOrientation_world);

    Eigen::Vector3d orientationError = getOrientationError();
    Eigen::Vector3d orientationErrorD = getOrientationErrorD();


    const Eigen::Vector3d k_q(0, 0, 0);
    const Eigen::Vector3d k_w(0, 0, 0);  //p d controller parameters

    Eigen::Vector3d F_pd = - orientationError.cwiseProduct(k_q) -  orientationErrorD.cwiseProduct(k_w);
    Eigen::Vector3d F_ff =
            massQuadcopter * length * cOrientation_world.dot((cDesiredOrientation_world.cross(cDesiredVectorD_world))) *
            cOrientation_world.cross(cOrientationD_world) +
            massQuadcopter * length *
            (cDesiredOrientation_world.cross(cDesiredOrientationDD_world).cross(cOrientation_world));

    float k_ff = 0;
    Eigen::Vector3d F = F_n - F_pd - k_ff * F_ff;

    //保方向饱和函数
    if (F.norm() > 15)
    {
        F = 15 * F / F.norm();
    }

//    Eigen::Vector3d liftForce = F.cwiseProduct(rotationMatrix_BuW.inverse() * (-e3));
//    cout << "C" << endl << F << endl;

    //2,1,0->ZYX
    Eigen::Vector3d eulerAngle = desiredQuaterniondBody.matrix().eulerAngles(2, 1, 0);
    Eigen::Vector3d b1_des(cos(eulerAngle.x()), sin(eulerAngle.x()), 0);

    Eigen::Vector3d b3_des = F / F.norm();

    Eigen::Vector3d b2_des;
    b2_des = b3_des.cross(b1_des);
    b2_des /= b2_des.norm();

    Eigen::Matrix3d desiredRotationMatrix;
    desiredRotationMatrix.col(0) = b2_des.cross(b3_des);
    desiredRotationMatrix.col(1) = b2_des;
    desiredRotationMatrix.col(2) = b3_des;

    Eigen::Vector3d errorRotation =
            0.5 * antisymmetricMatrixToVector((desiredRotationMatrix.transpose() * rotationMatrix_BuW -
                                               rotationMatrix_BuW.transpose() * desiredRotationMatrix));
//    cout << "C" << endl << errorRotation << endl;
//    float k_omega = 1;
//    Eigen::Vector3d desiredAngular = k_omega * errorRotation;
    Eigen::Vector3d errorAngular = cAngularVelocity_body;//小角度假设下可忽略： - rotationMatrix_BuW.transpose() * desiredRotationMatrix * desiredAngular;

//    Eigen::Matrix3d J_q;
//    J_q << 0.1, 0, 0,
//            0, 0.1, 0,
//            0, 0, 0.1;
    const Eigen::Vector3d k_p = Eigen::Vector3d(3, 3, 1);
    const Eigen::Vector3d k_d = Eigen::Vector3d(0.8, 0.8, 1.5);
    Eigen::Vector3d moment =
            -errorRotation.cwiseProduct(k_p) +
            errorAngular.cwiseProduct(k_d);// + cAngularVelocity_body.cross(J_q * cAngularVelocity_body);

//            保方向饱和函数
    if (moment.norm() > 1)
    {
        moment = 1 * moment / moment.norm();
    }
//    cout << "C" << endl <<   << endl;
//    moment.x() = moment.x() / 10;
//    Eigen::Vector3d moment = getMoment(F, rotationMatrix_BuW, cAngularVelocity_body);
//    cout << liftForce << endl;

    double liftForce = b3_des.transpose() * F;

//    cout<<liftForce<<endl;
    Eigen::Vector4d revs = getAllocatedRevs(liftForce, moment);

//    moment = Eigen::Vector3d(0, 0, 0);
//     revs = getAllocatedRevs(liftForce.norm(), moment);

    return revs;
}