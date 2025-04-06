#ifndef HELPERMETHODS_H
#define HELPERMETHODS_H
#include <cmath>
#include <vector>

#include <Eigen/Geometry>
class HelperMethods {
public:

  static std::vector<double> getQuaternionFromEuler(double roll, double pitch, double yaw);
  static std::vector<double> getEulerFromQuaternion(Eigen::Quaternionf q);
};

#endif // HELPER_H
