#ifndef ELLIPSOIDFIT_H
#define ELLIPSOIDFIT_H

#include <Eigen>
#include <vector>

bool uniqueEllipsoidFit(std::vector<Eigen::Vector3d> & inputData, Eigen::Vector3d & center, Eigen::Vector3d & radii,
    Eigen::Matrix3d & evecs);
bool alignedEllipsoidFit(std::vector<Eigen::Vector3d> & inputData, Eigen::Vector3d & center, Eigen::Vector3d & radii);

#endif

