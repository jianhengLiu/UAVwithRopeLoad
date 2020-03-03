//
// Created by chrisliu on 2020/2/26.
//

#include "antiSwing.h"

#include <iostream>

using namespace std;

Eigen::Vector3d getOrientationD(Eigen::Vector3d cVector, Eigen::Vector3d pVector)
{
    return cVector - pVector;
}

Eigen::Vector3d getOrientationDD(Eigen::Vector3d cVector, Eigen::Vector3d pVector, Eigen::Vector3d ppVector)
{
    return cVector - 2 * pVector + ppVector;
}

Eigen::Matrix3d vectorToAntisymmetricMatrix(Eigen::Vector3d vector)
{
    Eigen::Matrix3d antisymmetricMatrix;
    antisymmetricMatrix << 0, -vector.z(), vector.y(),
            vector.z(), 0, -vector.x(),
            -vector.y(), vector.x(), 0;
    return antisymmetricMatrix;
}

Eigen::Vector3d antisymmetricMatrixToVector(Eigen::Matrix3d antisymmetricMatrix)
{
    Eigen::Vector3d vector(antisymmetricMatrix(7), antisymmetricMatrix(2), antisymmetricMatrix(3));

    return vector;
}

Eigen::Vector3d pixelerrorToVector(double errorX, double errorY, double length, int resolutionX, int resolutionY)
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


Eigen::Vector3d toVectorRefToWorld(Eigen::Vector3d orientation_body, Eigen::Matrix3d rotationMatrix)
{
    return rotationMatrix.inverse() * orientation_body;
}


Eigen::Vector3d getOrientationError(Eigen::Vector3d orientation_world, Eigen::Vector3d desiredOrientation)
{
    Eigen::Matrix3d antisymmetricMatrix = vectorToAntisymmetricMatrix(orientation_world);
    Eigen::Vector3d orientationError = antisymmetricMatrix * antisymmetricMatrix * desiredOrientation;
    return orientationError;
}

Eigen::Vector3d
getOrientationErrorD(Eigen::Vector3d orientation, Eigen::Vector3d orientationD, Eigen::Vector3d desiredOrientation,
                     Eigen::Vector3d desiredOrientationD)
{
    Eigen::Vector3d orientationErrorD =
            orientationD - desiredOrientation.cross(desiredOrientationD).cross(orientation);
    return orientationErrorD;
}

Eigen::Vector3d getForcePD(Eigen::Vector3d error, Eigen::Vector3d errorD, Eigen::Vector3d k_q, Eigen::Vector3d k_w)
{
    Eigen::Vector3d F_pd = -error.cwiseProduct(k_q) - errorD.cwiseProduct(k_w);
    return F_pd;
}

Eigen::Vector3d
getForceFF(double massiveBody, double length, Eigen::Vector3d orientation, Eigen::Vector3d orientationD,
           Eigen::Vector3d desiredOrientation,
           Eigen::Vector3d desiredOrientationD, Eigen::Vector3d desiredOrientationDD)
{
    Eigen::Vector3d forceFF = massiveBody * length * orientation.dot((desiredOrientation.cross(desiredOrientationD))) *
                              orientation.cross(orientationD) +
                              massiveBody * length *
                              (desiredOrientation.cross(desiredOrientationDD).cross(orientation));
//    cout<<forceFF<<endl;
    return forceFF;
}

Eigen::Vector4d getRevs(double Force, Eigen::Vector3d Moment)
{
    Eigen::Matrix4d Minvese;
    double sqrt2 = sqrt(2);
    Minvese << 1, sqrt2, sqrt2, 1,
            1, -sqrt2, sqrt2, -1,
            1, -sqrt2, -sqrt2, 1,
            1, sqrt2, -sqrt2, -1;
    Minvese = 0.25 * Minvese;

    Eigen::Vector4d input(Force, Moment.x(), Moment.y(), Moment.z());
    Eigen::Vector4d revs = Minvese * input;
//    cout<<"1"<<Minvese<<endl;
    return revs;
}

Eigen::Vector3d getForceA()
{
//    Eigen::Vector3d A = -k_xq*e_xq-k_vq*e_vq-k_xL*e_xL-k_vL*e_vL+(m_Q+m_L)*(desiredLoadPositionDD+gravity*e3)+m_Q*length*(orientationD.dot(orientationD)*cOrientation);
//    Eigen::Vector3d A = -(cVector_body - cDesiredVector_world).cwiseProduct(k_x) - (vectorD - desiredVectorD).cwiseProduct(k_v) +
//    (massiveQuadcopter + massivePayload) * desiredVectorDD +
//    massiveQuadcopter * length * (orientationD.dot(orientationD)) * cOrientation_world;
}

//输入无人机惯性矩阵J_Q
Eigen::Vector3d getMoment(Eigen::Vector3d force, Eigen::Matrix3d rotationMatrix, Eigen::Vector3d omega)
{
    Eigen::Vector3d b_3 = -force / force.norm();

    Eigen::Vector3d b_1(1, 0, 0);
    Eigen::Vector3d b_2 = b_3.cross(b_1) / b_3.cross(b_1).norm();
    b_1 = b_2.cross(b_3);
    Eigen::Matrix3d R_c;
    R_c.col(0) = b_1;
    R_c.col(1) = b_2;
    R_c.col(2) = b_3;
//    cout<<force<<endl;
//    cout<<"1"<<endl<<(R_c.transpose() * rotationMatrix - rotationMatrix.transpose() * R_c)<<endl;
    Eigen::Vector3d e_R =
            0.5 * antisymmetricMatrixToVector((R_c.transpose() * rotationMatrix - rotationMatrix.transpose() * R_c));

    float k_omega = 1;
    Eigen::Vector3d omega_c = k_omega * e_R;
    Eigen::Vector3d e_omega = omega - rotationMatrix.transpose() * R_c * omega_c;
    Eigen::Vector3d moment = -e_omega + omega.cross(omega);

    return moment;

}