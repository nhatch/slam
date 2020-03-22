#include "graph.h"
#include "world.h"
#include "utils.h"
#include "graphics.h"
#include <unistd.h>

World::World() : landmarks_({}), goal_(0., 0., 1.), ground_truth_({}), odom_({}),
                    gps_({}), bag_({}) {
}

void World::addLandmark(double x, double y) {
  landmark_t lm;
  lm << x, y, 1;
  landmarks_.push_back(lm);
}

void World::setGoal(double x, double y) {
  goal_ << x, y, 1;
}

void World::startSimulation() {
  ground_truth_.push_back(toTransformRotateFirst(0., 0., 0.));
  odom_.push_back(toTransformRotateFirst(0., 0., 0.));
  readGPS();
  readLandmarks();
}

void World::runSimulation(int T) {
  startSimulation();
  for (int i = 1; i < T+1; i++) {
    moveRobot(0., 0.5);
    renderTruth();
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

void World::renderOdom() {
  drawTraj(transformReadings(odom_.back()), odom_, sf::Color::Blue);
  landmark_t odom_goal = odom_.back().inverse() * (gps_.back() * goal_);
  drawGoal(odom_goal);
}

void World::renderTruth() {
  drawTraj(landmarks_, ground_truth_, sf::Color::Black);
  // Using the ground_truth frame makes the visualization more intuitive, I think.
  // (The intent is to show how noisy the sensor readings are.)
  // But if we were visualizing odom information only, we should use the odom frame.
  drawTraj(transformReadings(ground_truth_.back()), odom_, sf::Color::Blue);
  drawGoal(goal_);
}

void World::moveRobot(double d_theta, double d_x) {
  double true_dx_std = 0.1; // m
  double true_dtheta_std = 0.0; // rad
  if (IS_2D)
    true_dtheta_std = 0.1;
  double noisy_x = stdn()*true_dx_std + d_x;
  double noisy_theta = stdn()*true_dtheta_std + d_theta;
  ground_truth_.push_back(toTransformRotateFirst(noisy_x, 0., noisy_theta) * ground_truth_.back());
  odom_.push_back(toTransformRotateFirst(d_x, 0., d_theta) * odom_.back());
  readLandmarks();
  readGPS();
}

void World::readLandmarks() {
  double true_x_std = 0.1; // m
  double true_y_std = 0.0; // m
  if (IS_2D)
    true_y_std = 0.1;
  landmark_readings_t landmark_readings;
  for (landmark_t lm : landmarks_) {
    landmark_reading_t reading = project(lm, ground_truth_.back());
    landmark_reading_t noise;
    noise << stdn()*true_x_std, stdn()*true_y_std, 0;
    reading += noise;
    landmark_readings.push_back(reading);
  }
  bag_.push_back(landmark_readings);
}

void World::readGPS() {
  double gps_x_std = 0.3; // m
  double gps_y_std = 0.3; // m
  double gps_theta_std = M_PI/8; // rad
  pose_t p = toPose(ground_truth_.back(), 0.);
  pose_t noise;
  noise << stdn()*gps_x_std, stdn()*gps_y_std, stdn()*gps_theta_std;
  p += noise;
  gps_.push_back(toTransform(p));
}

const bag_t World::bag() {
  return bag_;
}

const trajectory_t World::odom() {
  return odom_;
}

const trajectory_t World::gps() {
  return gps_;
}

const values World::groundTruth() {
  return toVector(ground_truth_, landmarks_);
}
