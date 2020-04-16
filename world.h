#ifndef WORLD_H
#define WORLD_H

#include <vector>
#include <Eigen/Core>
#include <Eigen/LU>
#include "utils.h"

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

  /* Call this once you are done creating obstacles / landmarks.
   * This saves the first set of sensor measurements.
   * The robot always starts at the pose (0,0,0). */
  void startSimulation();

  /* Move the robot forward d_x meters and turn d_theta radians clockwise.
   * After moving the robot, saves a new set of sensor measurements.
   * Noisy: see settings in constants.h. */
  void moveRobot(double d_theta, double d_x);

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
