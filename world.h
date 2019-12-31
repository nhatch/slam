#ifndef WORLD_H
#define WORLD_H

#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include "graph.h"
#include "utils.h"

template <int N>
class World {
public:
  World(bool is_2D);

  void addLandmark(double x, double y);
  // T is the number of timesteps. Will generate data for a trajectory of length T+1
  void runSimulation(int T);

  // x0 returns a concatenation of the odometry poses and the t=0 landmark readings
  const values<N> x0();
  // groundTruth returns a concatenation of the ground truth poses and landmark locations
  const values<N> groundTruth();
  // bag returns a std::vector of the landmark readings for t=0..T
  const bag_t bag();

private:
  bool is_2D_;
  landmarks_t landmarks_;
  trajectory_t ground_truth_;
  trajectory_t odom_;
  bag_t bag_;

  void moveRobot(double d_theta, double d_x);
  void readLandmarks();
  values<N> toVector(trajectory_t &traj, landmark_readings_t &r);
};

#include "world.inl"

#endif
