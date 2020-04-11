#ifndef WORLD_H
#define WORLD_H

#include <vector>
#include <Eigen/Core>
#include <Eigen/LU>
#include "utils.h"

class World {
public:
  World();

  void addObstacle(obstacle_t &obs);
  void addLandmark(double x, double y);
  // T is the number of timesteps. Will generate data for a trajectory of length T+1
  void runSimulation(int T);
  void startSimulation();
  void renderTruth();
  void renderRobotView(const transform_t &tf);
  void moveRobot(double d_theta, double d_x);

  // bag returns a std::vector of the landmark readings for t=0..T
  const traj_points_t lidar();
  const traj_points_t landmarks();
  const trajectory_t odom();
  const trajectory_t gps();

  // ground truth
  const trajectory_t truth();
  const points_t trueLandmarks();

private:
  obstacles_t obstacles_;
  points_t landmarks_;
  trajectory_t ground_truth_;
  trajectory_t odom_;
  trajectory_t gps_;
  traj_points_t landmark_readings_;
  traj_points_t lidar_readings_;

  void readSensors();
  void readLidar();
  void readLandmarks();
  void readGPS();
  void renderReadings(const transform_t &tf);
  points_t transformReadings(const points_t &ps, const transform_t &tf);
};

#endif
