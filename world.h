#ifndef WORLD_H
#define WORLD_H

#include <vector>
#include <Eigen/Core>
#include <Eigen/LU>
#include <thread>
#include "utils.h"
#include "graphics.h"

class World {
public:
  World();

  /* Obstacles trigger lidar hits and block landmarks from view.
   * If the robot hits an obstacle, this is reported to std::cout. */
  void addObstacle(const obstacle_t &obs);
  void addLandmark(double x, double y);

  /* A simple set of obstacles/landmarks which can be used as a basic example. */
  void addDefaultObstacles();
  void addDefaultLandmarks();

  void setCmdVel(double d_theta, double d_x);

  /* Sensor data. Each of these are a std::vector of length T+1,
   * where T is the number of times moveRobot has been called.
   * See utils.h for more details about data format.
   * Noisy: see settings in constants.h. */
  const traj_points_t lidar();
  const traj_points_t landmarks();
  const trajectory_t odom();
  const trajectory_t gps(); // Also includes theta readings taken from a "magnetometer"

  /* Ground truth */
  const trajectory_t trueTrajectory();
  const points_t trueLandmarks();

  void readSensors();
  void spawnWindow();

private:
  obstacles_t obstacles_;
  points_t landmarks_;
  trajectory_t ground_truth_;
  trajectory_t odom_;
  trajectory_t gps_;
  traj_points_t landmark_readings_;
  traj_points_t lidar_readings_;
  double cmd_vel_x_;
  double cmd_vel_theta_;
  transform_t current_transform_truth_;
  transform_t current_transform_odom_;
  std::thread spin_thread_;

  void readLidar();
  void readLandmarks();
  void readGPS();
  void readOdom();
  void readTrueTransform();
  void spinSim();
  void renderReadings(sf::RenderWindow &window, const transform_t &tf);
  void moveRobot(double d_theta, double d_x);

  friend class WorldUI;
};

#endif
