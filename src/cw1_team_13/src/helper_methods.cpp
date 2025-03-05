#include <cmath>
#include <vector>
#include <helper_methods.h>

std::vector<double>
HelperMethods::getQuaternionFromEuler(double roll, double pitch, double yaw){
  // Calculate trig values
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);

  // Calculate quaternion components using the same formulas as the Python reference
  double qx = sr * cp * cy - cr * sp * sy;
  double qy = cr * sp * cy + sr * cp * sy;
  double qz = cr * cp * sy - sr * sp * cy;
  double qw = cr * cp * cy + sr * sp * sy;

  // Return as [qx, qy, qz, qw] to match the Python implementation
  return {qx, qy, qz, qw};
}
