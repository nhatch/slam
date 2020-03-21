#ifndef WORLD_H
#define WORLD_H

#include <vector>
#include <Eigen/Core>
#include <Eigen/LU>
#include "graph.h"
#include "utils.h"

template <int N>
class World {
public:
  World();

  void addLandmark(double x, double y);
  void setGoal(double x, double y);
  // T is the number of timesteps. Will generate data for a trajectory of length T+1
  void runSimulation(int T);
  void startSimulation();
  void renderTruth();
  void renderOdom();
  void moveRobot(double d_theta, double d_x);

  // x0 returns a concatenation of the odometry poses and the t=0 landmark readings
  const values<N> x0();
  // groundTruth returns a concatenation of the ground truth poses and landmark locations
  const values<N> groundTruth();
  // bag returns a std::vector of the landmark readings for t=0..T
  const bag_t bag();

private:
  landmarks_t landmarks_;
  landmark_t goal_;
  trajectory_t ground_truth_;
  trajectory_t odom_;
  trajectory_t gps_;
  bag_t bag_;

  void readLandmarks();
  void readGPS();
  landmark_readings_t transformReadings(const transform_t &tf);
  values<N> toVector(trajectory_t &traj, landmark_readings_t &r);
};

#include "world.inl"

#endif
