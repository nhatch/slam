#include <random>
#include <chrono>
#include <eigen3/Eigen/LU>
#include "utils.h"

transform_t toTransform(double x, double y, double theta) {
  transform_t m;
  m << cos(theta),  sin(theta), -x,
       -sin(theta), cos(theta), -y,
                0,           0,  1;
  return m;
}

pose_t toPose(transform_t trf, double prev_theta) {
  pose_t s = trf.inverse() * pose_t(0, 0, 1);
  double cos_theta = trf(0,0);
  double sin_theta = -trf(1,0);
  double theta = atan2(sin_theta, cos_theta);
  while (theta < prev_theta - M_PI)
    theta += 2*M_PI;
  while (theta > prev_theta + M_PI)
    theta -= 2*M_PI;
  s(2) = theta;
  return s;
}

landmark_reading_t project(const landmark_t &landmark, const transform_t &transform) {
  landmark_t transformed_landmark = transform * landmark;
  // If we're projecting to a 1-D camera in front of the car:
  // using landmark_reading_t = double;
  // transformed_landmark(1)/transformed_landmark(0);
  return transformed_landmark;
}

std::normal_distribution<double> stdn_dist(0.0, 1.0);
unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
std::default_random_engine generator(seed);

double stdn() {
  return stdn_dist(generator);
}
