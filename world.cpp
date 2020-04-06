#include "graph.h"
#include "world.h"
#include "utils.h"
#include "graphics.h"
#include <unistd.h>
#include <iostream>

constexpr double COLLISION_RADIUS = 0.2;

World::World() : obstacles_({}), landmarks_({}), tags_({}), ground_truth_({}), odom_({}),
                    gps_({}), bag_({}), tags_bag_({}) {
}

void World::addObstacle(obstacle_t &obs) {
  obstacles_.push_back(obs);
}

void World::addLandmark(double x, double y) {
  landmark_t lm;
  lm << x, y, 1;
  landmarks_.push_back(lm);
}

void World::addTag(double x, double y) {
  landmark_t lm;
  lm << x, y, 1;
  tags_.push_back(lm);
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
    renderOdom(true);
    display();
    usleep(300*1000);
  }
}

landmark_readings_t World::transformReadings(const transform_t &tf) {
  transform_t tf_inv = tf.inverse();
  landmark_readings_t lms_readings({});
  for (landmark_reading_t lm : bag_.back()) {
    lms_readings.push_back(tf_inv * lm);
  }
  return lms_readings;
}

void World::renderOdom(bool viz_landmark_noise) {
  transform_t tf = odom_.back();
  if (viz_landmark_noise)
    // Using the ground_truth frame makes the visualization more intuitive, I think.
    // (The intent is to show how noisy the sensor readings are.)
    // But if we were visualizing odom information only, we should use the odom frame.
    tf = ground_truth_.back();
  drawLandmarks(transformReadings(tf), sf::Color::Blue);
  drawTraj(odom_, sf::Color::Blue);
}

void World::renderTruth() {
  drawTraj(ground_truth_, sf::Color::Black);
  drawLandmarks(landmarks_, sf::Color::Black);
  drawObstacles(obstacles_);
  trajectory_t last({ground_truth_.back()});
  drawTraj(last, sf::Color::Magenta);
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
  if (collides(ground_truth_.back(), landmarks_, COLLISION_RADIUS)) {
    std::cout << "You crashed into a landmark" << std::endl;
  }
  odom_.push_back(toTransformRotateFirst(d_x, 0., d_theta) * odom_.back());
  readSensors();
}

void World::readSensors() {
  readLandmarks(landmarks_, bag_, 10.0); // TODO make small visibility radius work with SLAM
  readLandmarks(tags_, tags_bag_, 1.0);
  readGPS();
}

void World::readLandmarks(landmarks_t &lms, bag_t &b, double visibility_radius) {
  double true_x_std = 0.1; // m
  double true_y_std = 0.0; // m
  if (IS_2D)
    true_y_std = 0.1;
  landmark_readings_t landmark_readings;
  for (landmark_t lm : lms) {
    landmark_reading_t reading = project(lm, ground_truth_.back());
    if (norm(reading) < visibility_radius) {
      landmark_reading_t noise;
      noise << stdn()*true_x_std, stdn()*true_y_std, 0;
      reading += noise;
    } else {
      reading *= 0; // (0,0,0) indicates no data
    }
    landmark_readings.push_back(reading);
  }
  b.push_back(landmark_readings);
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

const bag_t World::bag() {
  return bag_;
}

const bag_t World::tags_bag() {
  return tags_bag_;
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
