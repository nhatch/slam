#include "world.h"
#include "utils.h"
#include "constants.h"
#include "graphics.h"
#include <Eigen/LU>
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
                    spin_thread_(), done_(false),
                    legs_({})
{
}

World::~World() {
  done_ = true;
  if (spin_thread_.joinable()) spin_thread_.join();
}

void World::addObstacle(const obstacle_t &obs) {
  obstacles_.push_back(obs);
}

int World::addLandmark(double x, double y) {
  if (!IS_2D) y = 0.0;
  point_t lm;
  lm << x, y, 1;
  landmarks_.push_back(lm);
  /* It would make sense to put an obstacle here so you don't crash into the landmark,
   * but the current perception code would mean that the landmark would be impossible to observe. */
  //obstacle_t obs(4,2);
  //double w = 0.02; // Posts are square, 4cm on a side
  //obs << x+w,y+w,  x-w,y+w,  x-w,y-w,  x+w,y-w;
  //addObstacle(obs); // meh
  return landmarks_.size()-1; // The ID of the new landmark
}

void World::addPost(double x, double y, double gps_x, double gps_y) {
  int id = addLandmark(x,y);
  URCLeg leg {id, -1, {gps_x, gps_y, 1}};
  legs_.push_back(leg);
}

void World::addGate(double x, double y, double theta, double width, double gps_x, double gps_y) {
  double x1 = x + sin(theta)*width/2;
  double y1 = y - cos(theta)*width/2;
  double x2 = x - sin(theta)*width/2;
  double y2 = y + cos(theta)*width/2;
  int right_id = addLandmark(x1, y1);
  int left_id = addLandmark(x2, y2);
  URCLeg leg {left_id, right_id, {gps_x, gps_y, 1}};
  legs_.push_back(leg);
}

double randf() {
  return double(std::rand())/RAND_MAX;
}

void World::addURCObstacles() {
  addPost(200, 0, 200, 0);
  addPost(100, 200, 100, 200);
  addPost(-97, 202, -100, 200); // Leg 3: GPS is off by 5 meters
  addGate(-200, 0, M_PI, 3.0, -200, 0);
  addGate(-101, -209, M_PI, 2.0, -100, -200);

  // Leg 6: The path is strewn with obstacles
  addGate(100, -200, 1./2.*M_PI, 2.0, 100, -200);
  size_t seed = 0;
  std::srand(seed);
  int n_obstacles = 64;
  int obs_verts = 5;
  double o_x(100), o_y(-200), obs_field_size(150), min_o(0.5), max_o(10.0);
  for (int i=0; i < n_obstacles; i++) {
    obstacle_t o(obs_verts,2);
    double d_x = (randf()-0.5)*obs_field_size;
    double d_y = (randf()-0.5)*obs_field_size;

    // Ensure the obstacle doesn't cover the gate
    double norm = sqrt(d_x*d_x + d_y*d_y);
    if (norm < max_o+2) {
      d_x = d_x*max_o/norm;
      d_y = d_y*max_o/norm;
    }

    for (int j=0; j<obs_verts; j++) {
      double theta = 2*M_PI * j / obs_verts;
      double dist = randf()*(max_o-min_o) + min_o;
      o(j,0) = o_x+d_x + cos(theta)*dist;
      o(j,1) = o_y+d_y + sin(theta)*dist;
    }
    addObstacle(o);
  }


  // Leg 7: Gate hidden by obstacles; search required
  addGate(-10, 10, 3./4.*M_PI, 2.0, -2, 5);
  obstacle_t o1(3,2);
  obstacle_t o2(4,2);
  obstacle_t o3(3,2);
  obstacle_t o4(4,2);
  obstacle_t o5(3,2);
  obstacle_t o6(4,2);
  obstacle_t o7(4,2);
  obstacle_t o8(3,2);
  o1 << -12.5, 9,    -12, 15,    -14, 13;
  o2 << -12.5, 14,    0, 14,    1, 15,    -13,16;
  o3 << 0, 15,        -1, 7,    1, 7;
  o4 << -11, 2,     0, 8,   0, 9,   -10, 3.5;
  o5 << -10, 2.5,   -15, 8,   -20, 2;
  o6 << -7, -4,   -6, -8,   -4, -9,   -4.5, -3.5;
  o7 << -11, -4,   -11, -5,   -10, -5,   -10, -4;
  o8 << 12, 12,   14, 13,   13, 16;
  addObstacle(o1);
  addObstacle(o2);
  addObstacle(o3);
  addObstacle(o4);
  addObstacle(o5);
  addObstacle(o6);
  addObstacle(o7);
  addObstacle(o8);
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
  MyWindow window("Simulator visualization");
  const double SIM_HZ = 30;
  const double dt = 1/SIM_HZ;
  while (!done_) {
    int c = 0;
    while (c != -1)
    {
      c = window.pollWindowEvent(); // We ignore these events.
    }
    moveRobot(cmd_vel_theta_ * dt, cmd_vel_x_ * dt);
    window.setOrigin(toPose(current_transform_truth_, 0.0));
    window.drawObstacles(obstacles_);
    window.drawPoints(landmarks_, TRUTH_COLOR, 4);
    renderReadings(window);
    window.drawRobot(current_transform_truth_, TRUTH_COLOR);
    window.display();
    usleep(1000 * 1000 / SIM_HZ);
  }
}

void World::renderReadings(MyWindow &window) {
  transform_t tf = current_transform_truth_;
  points_t lidar = readLidar();
  window.drawPoints(transformReadings(lidar, tf), LIDAR_COLOR, 3);
  points_t lms = readLandmarks();
  window.drawPoints(transformReadings(lms, tf), LANDMARK_COLOR, 4);
}

void World::start() {
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
  point_t robot_location = tf.inverse() * point_t(0,0,1);
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

URCLeg World::getLeg(int index) {
  if (index < 0 || index >= (int)legs_.size()) {
    return URCLeg {-1, -1, {0,0,0}};
  }
  return legs_[(size_t) index];
}

const points_t World::trueLandmarks() {
  return landmarks_;
}
