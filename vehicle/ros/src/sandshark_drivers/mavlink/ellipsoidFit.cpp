#include "ellipsoidFit.h"

using std::vector;
using namespace Eigen;

bool uniqueEllipsoidFit(vector<Vector3d> & inputData, Vector3d & center, Vector3d & radii, Matrix3d & evecs) {
  VectorXd B = VectorXd::Constant(inputData.size(), 1.0);
  MatrixXd A = MatrixXd::Constant(inputData.size(), 9, 0.0);
  for (unsigned int index = 0; index < inputData.size(); ++index) {
    Vector3d data = inputData[index];
    A(index, 0) = data.x() * data.x();
    A(index, 1) = data.y() * data.y();
    A(index, 2) = data.z() * data.z();
    A(index, 3) = 2.0 * data.x() * data.y();
    A(index, 4) = 2.0 * data.x() * data.z();
    A(index, 5) = 2.0 * data.y() * data.z();
    A(index, 6) = 2.0 * data.x();
    A(index, 7) = 2.0 * data.y();
    A(index, 8) = 2.0 * data.z();
  }

  VectorXd X = (A.transpose() * A).inverse() * A.transpose() * B;
  Matrix4d V;
  V(0, 0) = X(0);
  V(0, 1) = X(3);
  V(0, 2) = X(4);
  V(0, 3) = X(6);
  V(1, 0) = X(3);
  V(1, 1) = X(1);
  V(1, 2) = X(5);
  V(1, 3) = X(7);
  V(2, 0) = X(4);
  V(2, 1) = X(5);
  V(2, 2) = X(2);
  V(2, 3) = X(8);
  V(3, 0) = X(6);
  V(3, 1) = X(7);
  V(3, 2) = X(8);
  V(3, 3) = -1.0;

  Matrix3d A2 = -1.0 * V.block<3, 3>(0, 0);
  center = Vector3d(X(6), X(7), X(8));
  center = A2.inverse() * center;

  Matrix4d T = Matrix4d::Identity();
  T(3, 0) = center.x();
  T(3, 1) = center.y();
  T(3, 2) = center.z();

  Matrix4d R = T * V * T.transpose();
  Matrix3d R2 = R.block<3, 3>(0, 0) / (-1.0 * R(3, 3));
  SelfAdjointEigenSolver<Matrix3d> eigensolver(R2);
  if (eigensolver.info() != Success) {
    return false;
  }

  Vector3d evals = eigensolver.eigenvalues();
  evecs = eigensolver.eigenvectors();

  radii(0) = sqrt(1.0 / evals(0));
  radii(1) = sqrt(1.0 / evals(1));
  radii(2) = sqrt(1.0 / evals(2));
  return true;
}

bool alignedEllipsoidFit(std::vector<Eigen::Vector3d> & inputData, Eigen::Vector3d & center, Eigen::Vector3d & radii) {
  VectorXd B = VectorXd::Constant(inputData.size(), 1.0);
  MatrixXd A = MatrixXd::Constant(inputData.size(), 6, 0.0);
  for (unsigned int index = 0; index < inputData.size(); ++index) {
    Vector3d data = inputData[index];
    A(index, 0) = data.x() * data.x();
    A(index, 1) = data.y() * data.y();
    A(index, 2) = data.z() * data.z();
    A(index, 3) = 2.0 * data.x();
    A(index, 4) = 2.0 * data.y();
    A(index, 5) = 2.0 * data.z();
  }

  VectorXd X = (A.transpose() * A).inverse() * A.transpose() * B;

  center(0) = -1.0 * X(3) / X(0);
  center(1) = -1.0 * X(4) / X(1);
  center(2) = -1.0 * X(5) / X(2);

  double gamma = 1.0 + ((X(3) * X(3)) / X(0) + (X(4) * X(4)) / X(1) + (X(5) * X(5)) / X(2));

  radii(0) = sqrt(gamma / X(0));
  radii(1) = sqrt(gamma / X(1));
  radii(2) = sqrt(gamma / X(2));
  return true;
}

