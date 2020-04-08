#include "graph.h"
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
  landmark_t lm;
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

landmark_readings_t World::transformReadings(const landmarks_t &lms, const transform_t &tf) {
  transform_t tf_inv = tf.inverse();
  landmark_readings_t lms_readings({});
  for (landmark_reading_t lm : lms) {
    lms_readings.push_back(tf_inv * lm);
  }
  return lms_readings;
}

void World::renderReadings(const transform_t &tf) {
  landmarks_t lidar = lidar_readings_.back();
  drawLandmarks(transformReadings(lidar, tf), sf::Color::Red);
  landmarks_t lms = landmark_readings_.back();
  drawLandmarks(transformReadings(lms, tf), sf::Color::Blue);
}

void World::renderOdom() {
  renderReadings(odom_.back());
  drawRobot(odom_.back(), sf::Color::Blue);
}

void World::renderTruth() {
  drawObstacles(obstacles_);
  drawTraj(ground_truth_, sf::Color::Black);
  drawLandmarks(landmarks_, sf::Color::Black);
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

void World::readLandmarks() {
  double MAX_RANGE = 10.0;
  double true_x_std = 0.1; // m
  double true_y_std = 0.0; // m
  if (IS_2D)
    true_y_std = 0.1;
  landmark_readings_t landmark_readings;
  transform_t tf = ground_truth_.back();
  landmark_t robot_location = tf.inverse()*landmark_t(0,0,1);
  for (landmark_t lm : landmarks_) {
    landmark_reading_t reading = tf * lm;
    landmark_reading_t noise;
    noise << stdn()*true_x_std, stdn()*true_y_std, 0;
    reading += noise;
    double dist = norm(robot_location - lm);
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
  double FAILURE_PROBABILITY = 0.01; // probability of no hit even if obstacle is in range

  landmarks_t hits({});
  for(int j = 0; j < ANGULAR_RESOLUTION; j++)
  {
    double angle = j * 2 * M_PI / ANGULAR_RESOLUTION;
    landmark_t r0, r1;
    r0 << 0, 0, 1;
    r1 << MAX_RANGE*cos(angle), MAX_RANGE*sin(angle), 1;
    transform_t tf_inv = ground_truth_.back().inverse();
    double t = obstacleIntersection(tf_inv*r0, tf_inv*r1, obstacles_);
    double dist = t*MAX_RANGE;
    if (dist < MAX_RANGE && dist > MIN_RANGE)
    {
      landmark_t hit;
      hit << dist*cos(angle), dist*sin(angle), 1;
      // TODO: add noise
      hits.push_back(hit);
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

const bag_t World::landmarks() {
  return landmark_readings_;
}

const bag_t World::lidar() {
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

const values World::groundTruth() {
  return toVector(ground_truth_, landmarks_);
}
