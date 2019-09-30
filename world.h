#ifndef WORLD_H
#define WORLD_H

#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include "graph.h"

using transform_t = Eigen::Matrix3d;
using landmark_t = Eigen::Vector3d; // the last coeff is all 1's
using landmarks_t = std::vector<landmark_t>;
using reading_t = landmark_t; // Assume we have a range-and-bearing sensor
using trajectory_t = std::vector<transform_t>;
using landmark_readings_t = std::vector<reading_t>;
using bag_t = std::vector<landmark_readings_t>;

class Car {
public:
  bag_t bag_;
  trajectory_t ground_truth_;
  trajectory_t odom_;

  Car(transform_t tf);

  void move(double d_theta, double d_x);

  void read(landmarks_t landmarks);
};

class World {
public:
  Car car_;
  landmarks_t landmarks_;

  World();

  void addLandmark(double x, double y);
};

#endif
