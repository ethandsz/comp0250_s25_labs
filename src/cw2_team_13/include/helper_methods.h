#ifndef HELPERMETHODS_H
#define HELPERMETHODS_H
#include <cmath>
#include <vector>
class HelperMethods {
public:

  static std::vector<double> getQuaternionFromEuler(double roll, double pitch, double yaw);
};

#endif // HELPER_H
