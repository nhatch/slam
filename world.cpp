#include "world.h"
#include "utils.h"
#include "constants.h"
#include "graphics.h"
#include <unistd.h>
#include <iostream>

using namespace NavSim;

const sf::Color TRUTH_COLOR(0,0,0,128);
const sf::Color ODOM_COLOR(0,0,255,128);
const sf::Color LIDAR_COLOR(255,0,0,128);
const sf::Color LANDMARK_COLOR(0,0,255);

World::World() : obstacles_({}), landmarks_({}),
                    cmd_vel_x_(0), cmd_vel_theta_(0),
                    current_transform_truth_(toTransform({0,0,0})),
                    current_transform_odom_(toTransform({0,0,0})),
                    spin_thread_()
{
}

void World::addObstacle(const obstacle_t &obs) {
  obstacles_.push_back(obs);
}

void World::addLandmark(double x, double y) {
  if (!IS_2D) y = 0.0;
  point_t lm;
  lm << x, y, 1;
  landmarks_.push_back(lm);
}

void World::addDefaultObstacles() {
  obstacle_t o1(3,2);
  obstacle_t o2(4,2);
  obstacle_t o3(4,2);
  o1 << 3, 1,     4, 2,     2, 2.5;
  o2 << 5, -1,    5.5, -1,  5.5, 2,   5, 2;
  o3 << 5, 1.5,   6, 1.5,   6, 2,     5, 2;
  addObstacle(o1*ROBOT_LENGTH*3.3);
  addObstacle(o2*ROBOT_LENGTH*3.3);
  addObstacle(o3*ROBOT_LENGTH*3.3);
}

void World::addDefaultLandmarks() {
  double scale = ROBOT_LENGTH / 0.3;
  addLandmark(3*scale, 1*scale);
  addLandmark(6.*scale, -1*scale);
  addLandmark(-1.*scale, 0*scale);
  addLandmark(-0.*scale, -3*scale);
  addLandmark(3.1*scale, 1*scale);
  addLandmark(0.1*scale, 1*scale);
}

void World::spinSim() {
  sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH_PX, WINDOW_WIDTH_PX), "Simulator visualization");
  const double SIM_HZ = 30;
  const double dt = 1/SIM_HZ;
  while (true) {
    int c = 0;
    while (c != -1)
    {
      c = pollWindowEvent(window); // We ignore these events.
    }
    moveRobot(cmd_vel_theta_ * dt, cmd_vel_x_ * dt);
    drawObstacles(window, obstacles_);
    _drawPoints(window, landmarks_, TRUTH_COLOR, 4);
    renderReadings(window);
    drawRobot(window, current_transform_truth_, TRUTH_COLOR);
    display(window);
    usleep(1000 * 1000 / SIM_HZ);
  }
}

void World::renderReadings(sf::RenderWindow &window) {
  transform_t tf = current_transform_truth_;
  points_t lidar = readLidar();
  _drawPoints(window, transformReadings(lidar, tf), LIDAR_COLOR, 3);
  points_t lms = readLandmarks();
  _drawPoints(window, transformReadings(lms, tf), LANDMARK_COLOR, 4);
}

void World::spawnWindow() {
  spin_thread_ = std::thread( [this] {spinSim();} );
}

void World::setCmdVel(double d_theta, double d_x) {
  cmd_vel_theta_ = d_theta;
  cmd_vel_x_ = d_x;
}

void World::moveRobot(double d_theta, double d_x) {
  double noisy_x(0.), noisy_theta(0.);
  if (IS_2D) {
    double d_r = d_x + 0.5*ROBOT_WHEEL_BASE*d_theta;
    double d_l = d_x - 0.5*ROBOT_WHEEL_BASE*d_theta;
    // Larger distance means more noise
    double noisy_r = d_r + stdn() * WHEEL_STD * sqrt(abs(d_r));
    double noisy_l = d_l + stdn() * WHEEL_STD * sqrt(abs(d_l));
    noisy_x = 0.5*(noisy_r + noisy_l);
    noisy_theta = (noisy_r - noisy_l) / ROBOT_WHEEL_BASE;
  } else {
    noisy_x = d_x + stdn() * WHEEL_STD * sqrt(abs(d_x));
  }
  current_transform_truth_ = toTransformRotateFirst(noisy_x, 0., noisy_theta) * current_transform_truth_;
  if (collides(current_transform_truth_, obstacles_)) {
    std::cout << "You crashed into an obstacle" << std::endl;
  }
  current_transform_odom_ = toTransformRotateFirst(d_x, 0., d_theta) * current_transform_odom_;
}

// Currently this treats landmarks and lidar hits the same;
// presumably in the real world they should have different noise models.
void corrupt(point_t &p, double dist) {
  // Add noise that increases with distance
  p(0) += stdn() * CORRUPTION_STD * sqrt(dist);
  if (IS_2D) p(1) += stdn() * CORRUPTION_STD * sqrt(dist);
  // Sometimes completely erase the data
  if (stdn() < DATA_LOSS_THRESHOLD) {
    p *= 0;
  }
}

transform_t World::readTrueTransform() {
  return current_transform_truth_;
}

transform_t World::readOdom() {
  return current_transform_odom_;
}

points_t World::readLandmarks() {
  points_t landmark_readings;
  transform_t tf = current_transform_truth_;
  point_t robot_location = tf.inverse()*point_t(0,0,1);
  for (point_t lm : landmarks_) {
    point_t reading = tf * lm;
    double dist = (robot_location - lm).norm();
    corrupt(reading, dist);
    double t = obstacleIntersection(robot_location, lm, obstacles_);
    // t < 1.0 indicates there's an obstacle between the landmark and the robot
    if (dist > LANDMARK_MAX_RANGE || t < 0.999999) {
      reading *= 0; // (0,0,0) indicates no data
    }
    landmark_readings.push_back(reading);
  }
  return landmark_readings;
}

points_t World::readLidar() {
  points_t hits({});
  for(int j = 0; j < LIDAR_RESOLUTION; j++)
  {
    double angle = j * 2 * M_PI / LIDAR_RESOLUTION;
    point_t r0, r1;
    r0 << 0, 0, 1;
    r1 << LIDAR_MAX_RANGE*cos(angle), LIDAR_MAX_RANGE*sin(angle), 1;
    transform_t tf_inv = current_transform_truth_.inverse();
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
  return hits;
}

transform_t World::readGPS() {
  pose_t p = toPose(current_transform_truth_, 0.);
  pose_t noise;
  noise << stdn()*GPS_POS_STD, stdn()*GPS_POS_STD, stdn()*GPS_THETA_STD;
  p += noise;
  return toTransform(p);
}

const points_t World::trueLandmarks() {
  return landmarks_;
}
