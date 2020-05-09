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

  points_t readLidar();
  points_t readLandmarks();
  transform_t readGPS();
  transform_t readOdom();

  /* Ground truth */
  const points_t trueLandmarks();
  transform_t readTrueTransform();

  void start();

private:
  obstacles_t obstacles_;
  points_t landmarks_;
  double cmd_vel_x_;
  double cmd_vel_theta_;
  transform_t current_transform_truth_;
  transform_t current_transform_odom_;
  std::thread spin_thread_;

  void spinSim();
  void renderReadings(sf::RenderWindow &window);
  void moveRobot(double d_theta, double d_x);
};

#endif
