#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <cmath>
#include <vector>


struct Quat
{
  double w, x, y, z;
};

struct EulerAngles
{
  double roll, pitch, yaw;
};


//MatrixXd blk3x3Diag(const Matrix3d& matrixA, const Matrix3d& matrixB, const Matrix3d& matrixC, const Matrix3d& matrixD);
Eigen::Matrix3d crossProductMatrix(const Eigen::Vector3d& n);
Eigen::Quaterniond quaternionHamiltonProduct(Eigen::Quaterniond quatLeft, Eigen::Quaterniond quatRight);
Eigen::Matrix3d quaternion2Rotationmatrix(const Eigen::Vector4d& quaternion);
//MatrixXd jacobianFdOfDVL(const VectorXd& fun, const Quaterniond& q, const double& step, const Vector3d& velWorld);
Eigen::Matrix3d eulerToRotationMatrix(const Eigen::Vector3d& eulerAngles);
EulerAngles fromQuaternionToEulerAngles(const Quat& q);
Eigen::Quaterniond fromRPYToQuaternion(const EulerAngles& angles);  // yaw (Z), pitch (Y), roll (X)
Eigen::Matrix3d quat2RotMat_fast(const Eigen::Quaterniond& quat);
