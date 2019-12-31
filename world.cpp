#include <iostream>
#include <random>
#include <chrono>
#include "graph.h"
#include "world.h"

/* Left-multiplying a (map-frame) landmark_t by this matrix will give the
 * corresponding landmark_t in the frame that was rotated counterclockwise
 * by theta, then shifted by (x,y) along the new (x,y)-axes.
 *
 * For example, for (x, y, theta) = (1, 0, pi/2), the origin frame
 *
 *   . .
 *   > .
 *
 * becomes
 *
 *   ^ .
 *   . .
 */
transform_t toTransform(double x, double y, double theta) {
  transform_t m;
  m << cos(theta),  sin(theta), -x,
       -sin(theta), cos(theta), -y,
                0,           0,  1;
  return m;
}

reading_t project(const landmark_t &landmark, const transform_t &transform) {
  landmark_t transformed_landmark = transform * landmark;
  // If we're projecting to a 1-D camera in front of the car:
  // using reading_t = double;
  // transformed_landmark(1)/transformed_landmark(0);
  return transformed_landmark;
}

const double true_dx_std = 0.1; // m
const double true_dtheta_std = 0.1; // rad
const double true_sensor_std = 0.1; // m
std::normal_distribution<double> stdn_dist(0.0, 1.0);
unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
std::default_random_engine generator(seed);

double stdn() {
  return stdn_dist(generator);
}

Car::Car(transform_t tf) : bag_({}), ground_truth_({}), odom_({}) {
  ground_truth_.push_back(tf);
  odom_.push_back(toTransform(0., 0., 0.));
}

void Car::move(double d_theta, double d_x) {
  double noisy_x = stdn()*true_dx_std + d_x;
  double noisy_theta = stdn()*true_dtheta_std + d_theta;
  ground_truth_.push_back(toTransform(noisy_x, 0., noisy_theta) * ground_truth_.back());
  odom_.push_back(toTransform(d_x, 0., d_theta) * odom_.back());
}

void Car::read(landmarks_t landmarks) {
  landmark_readings_t landmark_readings;
  for (landmark_t lm : landmarks) {
    reading_t reading = project(lm, ground_truth_.back());
    reading_t noise;
    noise << stdn()*true_sensor_std, stdn()*true_sensor_std, 0;
    reading += noise;
    landmark_readings.push_back(reading);
  }
  bag_.push_back(landmark_readings);
}

World::World() : car_(toTransform(0., 0., 0.)), landmarks_({}) {
}

void World::addLandmark(double x, double y) {
  landmark_t lm;
  lm << x, y, 1;
  landmarks_.push_back(lm);
}
