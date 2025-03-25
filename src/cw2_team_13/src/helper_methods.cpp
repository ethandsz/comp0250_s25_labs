#include <cmath>
#include <vector>
#include <helper_methods.h>
#include <Eigen/Geometry>
std::vector<double>
HelperMethods::getQuaternionFromEuler(double roll, double pitch, double yaw){
  // Calculate trig values
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);

  double qx = sr * cp * cy - cr * sp * sy;
  double qy = cr * sp * cy + sr * cp * sy;
  double qz = cr * cp * sy - sr * sp * cy;
  double qw = cr * cp * cy + sr * sp * sy;

  return {qx, qy, qz, qw};
}


std::vector<double> HelperMethods::getEulerFromQuaternion(Eigen::Quaternionf q) {
  // roll (x-axis rotation)
  double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
  double r = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (q.w() * q.y() - q.x() * q.z());
  double p;
  // Clamp sinp to [-1, 1] to avoid NaNs from asin
  if (sinp >= 1)
    p = M_PI / 2;
  else if (sinp <= -1)
    p = -M_PI / 2;
  else
    p = std::asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
  double y = std::atan2(siny_cosp, cosy_cosp);

  return {r, p, y};
}
