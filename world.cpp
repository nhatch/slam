#include "world.h"
#include "utils.h"
#include "graphics.h"
#include "constants.h"
#include <unistd.h>
#include <iostream>

World::World() : obstacles_({}), landmarks_({}), ground_truth_({}), odom_({}),
                    gps_({}), landmark_readings_({}), lidar_readings_({}) {
}

using namespace NavSim;

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
  renderTruth();
  display();
  for (int i = 1; i < T+1; i++) {
    moveRobot(0., 0.5);
    renderTruth();
    drawTraj(odom_, sf::Color::Blue);
    display();
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

void World::renderRobotView(const transform_t &tf) {
  renderReadings(tf);
  drawRobot(tf, sf::Color::Blue);
}

void World::renderTruth() {
  drawObstacles(obstacles_);
  drawTraj(ground_truth_, sf::Color::Black);
  drawPoints(landmarks_, sf::Color::Black);
  renderReadings(ground_truth_.back());
}

void World::moveRobot(double d_theta, double d_x) {
  double move_dist;
  double noisy_x(0.), noisy_theta(0.);
  if (IS_2D) {
    double d_r = d_x + 0.5*ROBOT_WHEEL_BASE*d_theta;
    double d_l = d_x - 0.5*ROBOT_WHEEL_BASE*d_theta;
    move_dist = abs(d_r) > abs(d_l) ? abs(d_r) : abs(d_l);
    // Larger distance means more noise
    double noisy_r = d_r + stdn() * WHEEL_STD * sqrt(abs(d_r));
    double noisy_l = d_l + stdn() * WHEEL_STD * sqrt(abs(d_l));
    noisy_x = 0.5*(noisy_r + noisy_l);
    noisy_theta = (noisy_r - noisy_l) / ROBOT_WHEEL_BASE;
  } else {
    noisy_x = d_x + stdn() * WHEEL_STD * sqrt(abs(d_x));
    move_dist = abs(d_x);
  }
  usleep((size_t)(move_dist / ANIMATION_SPEED * 1000 * 1000));
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
  // Add noise that increases with distance
  p(0) += stdn() * CORRUPTION_STD * sqrt(dist);
  if (IS_2D) p(1) += stdn() * CORRUPTION_STD * sqrt(dist);
  // Sometimes completely erase the data
  if (stdn() < -2.0) {
    p *= 0;
  }
}

void World::readLandmarks() {
  points_t landmark_readings;
  transform_t tf = ground_truth_.back();
  point_t robot_location = tf.inverse()*point_t(0,0,1);
  for (point_t lm : landmarks_) {
    point_t reading = tf * lm;
    double dist = norm(robot_location - lm);
    corrupt(reading, dist);
    double t = obstacleIntersection(robot_location, lm, obstacles_);
    // t < 1.0 indicates there's an obstacle between the landmark and the robot
    if (dist > LANDMARK_MAX_RANGE || t < 0.999999) {
      reading *= 0; // (0,0,0) indicates no data
    }
    landmark_readings.push_back(reading);
  }
  landmark_readings_.push_back(landmark_readings);
}

void World::readLidar() {
  points_t hits({});
  for(int j = 0; j < LIDAR_RESOLUTION; j++)
  {
    double angle = j * 2 * M_PI / LIDAR_RESOLUTION;
    point_t r0, r1;
    r0 << 0, 0, 1;
    r1 << LIDAR_MAX_RANGE*cos(angle), LIDAR_MAX_RANGE*sin(angle), 1;
    transform_t tf_inv = ground_truth_.back().inverse();
    double t = obstacleIntersection(tf_inv*r0, tf_inv*r1, obstacles_);
    double dist = t*LIDAR_MAX_RANGE;
    if (dist < LIDAR_MAX_RANGE && dist > LIDAR_MIN_RANGE)
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
  pose_t p = toPose(ground_truth_.back(), 0.);
  pose_t noise;
  noise << stdn()*GPS_POS_STD, stdn()*GPS_POS_STD, stdn()*GPS_THETA_STD;
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
