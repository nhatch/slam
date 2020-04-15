#ifndef WORLD_H
#define WORLD_H

#include <vector>
#include <Eigen/Core>
#include <Eigen/LU>
#include "utils.h"

class World {
public:
  World();

  void addObstacle(const obstacle_t &obs);
  void addLandmark(double x, double y);
  void addDefaultObstacles();
  void addDefaultLandmarks();
  void startSimulation();
  void moveRobot(double d_theta, double d_x);

  // Sensor data. Each of these are a std::vector of length T+1
  const traj_points_t lidar();
  const traj_points_t landmarks();
  const trajectory_t odom();
  const trajectory_t gps();

  // ground truth
  const trajectory_t trueTrajectory();
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

  friend class WorldUI;
};

#endif
