#include "world.h"
#include "utils.h"
#include "graphics.h"
#include <unistd.h>
#include <iostream>

constexpr double COLLISION_RADIUS = 0.2;

World::World() : obstacles_({}), landmarks_({}), ground_truth_({}), odom_({}),
                    gps_({}), landmark_readings_({}), lidar_readings_({}) {
}

void World::addObstacle(obstacle_t &obs) {
  obstacles_.push_back(obs);
}

void World::addLandmark(double x, double y) {
  point_t lm;
  lm << x, y, 1;
  landmarks_.push_back(lm);
}

void World::startSimulation() {
  ground_truth_.push_back(toTransformRotateFirst(0., 0., 0.));
  odom_.push_back(toTransformRotateFirst(0., 0., 0.));
  readSensors();
}

void World::runSimulation(int T) {
  startSimulation();
  for (int i = 1; i < T+1; i++) {
    moveRobot(0., 0.5);
    renderTruth();
    drawTraj(odom_, sf::Color::Blue);
    display();
    usleep(300*1000);
  }
}

points_t World::transformReadings(const points_t &ps, const transform_t &tf) {
  transform_t tf_inv = tf.inverse();
  points_t readings({});
  for (point_t p : ps) {
    readings.push_back(tf_inv * p);
  }
  return readings;
}

void World::renderReadings(const transform_t &tf) {
  points_t lidar = lidar_readings_.back();
  drawPoints(transformReadings(lidar, tf), sf::Color::Red);
  points_t lms = landmark_readings_.back();
  drawPoints(transformReadings(lms, tf), sf::Color::Blue);
}

void World::renderOdom() {
  renderReadings(odom_.back());
  drawRobot(odom_.back(), sf::Color::Blue);
}

void World::renderTruth() {
  drawObstacles(obstacles_);
  drawTraj(ground_truth_, sf::Color::Black);
  drawPoints(landmarks_, sf::Color::Black);
  renderReadings(ground_truth_.back());
}

void World::moveRobot(double d_theta, double d_x) {
  constexpr double WHEEL_BASE = 0.2;
  double noisy_x(0.), noisy_theta(0.);
  double true_wheel_std = 0.03; // m
  if (IS_2D) {
    double d_r = d_x + 0.5*WHEEL_BASE*d_theta;
    double d_l = d_x - 0.5*WHEEL_BASE*d_theta;
    // Larger distance means more noise
    double noisy_r = d_r + stdn() * true_wheel_std * sqrt(abs(d_r));
    double noisy_l = d_l + stdn() * true_wheel_std * sqrt(abs(d_l));
    noisy_x = 0.5*(noisy_r + noisy_l);
    noisy_theta = (noisy_r - noisy_l) / WHEEL_BASE;
  } else {
    noisy_x = d_x + stdn() * true_wheel_std * sqrt(abs(d_x));
  }
  ground_truth_.push_back(toTransformRotateFirst(noisy_x, 0., noisy_theta) * ground_truth_.back());
  if (collides(ground_truth_.back(), obstacles_)) {
    std::cout << "You crashed into an obstacle" << std::endl;
  }
  odom_.push_back(toTransformRotateFirst(d_x, 0., d_theta) * odom_.back());
  readSensors();
}

void World::readSensors() {
  readLandmarks();
  readGPS();
  readLidar();
}

// Currently this treats landmarks and lidar hits the same;
// presumably in the real world they should have different noise models.
void corrupt(point_t &p, double dist) {
  double true_x_std = 0.04; // m
  double true_y_std = 0.0; // m
  if (IS_2D)
    true_y_std = 0.04;
  // Add noise that increases with distance
  p(0) += stdn() * true_x_std * sqrt(dist);
  p(1) += stdn() * true_y_std * sqrt(dist);
  // Sometimes completely erase the data
  if (stdn() < -2.0) {
    p *= 0;
  }
}

void World::readLandmarks() {
  double MAX_RANGE = 10.0;
  points_t landmark_readings;
  transform_t tf = ground_truth_.back();
  point_t robot_location = tf.inverse()*point_t(0,0,1);
  for (point_t lm : landmarks_) {
    point_t reading = tf * lm;
    double dist = norm(robot_location - lm);
    corrupt(reading, dist);
    double t = obstacleIntersection(robot_location, lm, obstacles_);
    // t < 1.0 indicates there's an obstacle between the landmark and the robot
    if (dist > MAX_RANGE || t < 0.999999) {
      reading *= 0; // (0,0,0) indicates no data
    }
    landmark_readings.push_back(reading);
  }
  landmark_readings_.push_back(landmark_readings);
}

void World::readLidar() {
  double MAX_RANGE = 5.0;
  double MIN_RANGE = 0.3;
  int ANGULAR_RESOLUTION = 100; // number of scans per full rotation

  points_t hits({});
  for(int j = 0; j < ANGULAR_RESOLUTION; j++)
  {
    double angle = j * 2 * M_PI / ANGULAR_RESOLUTION;
    point_t r0, r1;
    r0 << 0, 0, 1;
    r1 << MAX_RANGE*cos(angle), MAX_RANGE*sin(angle), 1;
    transform_t tf_inv = ground_truth_.back().inverse();
    double t = obstacleIntersection(tf_inv*r0, tf_inv*r1, obstacles_);
    double dist = t*MAX_RANGE;
    if (dist < MAX_RANGE && dist > MIN_RANGE)
    {
      point_t hit;
      hit << dist*cos(angle), dist*sin(angle), 1;
      corrupt(hit, dist);
      if (hit(2) != 0.0) {
        hits.push_back(hit);
      }
    }
  }

  lidar_readings_.push_back(hits);
}

void World::readGPS() {
  double gps_x_std = 0.2; // m
  double gps_y_std = 0.2; // m
  double gps_theta_std = M_PI/24; // rad
  pose_t p = toPose(ground_truth_.back(), 0.);
  pose_t noise;
  noise << stdn()*gps_x_std, stdn()*gps_y_std, stdn()*gps_theta_std;
  p += noise;
  gps_.push_back(toTransform(p));
}

const traj_points_t World::landmarks() {
  return landmark_readings_;
}

const traj_points_t World::lidar() {
  return lidar_readings_;
}

const trajectory_t World::odom() {
  return odom_;
}

const trajectory_t World::gps() {
  return gps_;
}

const trajectory_t World::truth() {
  return ground_truth_;
}

const points_t World::trueLandmarks() {
  return landmarks_;
}
