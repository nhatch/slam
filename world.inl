#include "graph.h"
#include "world.h"
#include "utils.h"

template <int N>
values<N> World<N>::toVector(trajectory_t &traj, landmark_readings_t &r) {
  int T = (int) traj.size()-1;
  int L = (int) r.size();
  int pose_size(1), lm_size(1);
  if (is_2D_) {
    pose_size = 3;
    lm_size = 2;
  }
  assert("uh oh" && (pose_size*T+lm_size*L == N));
  // We don't include a variable for T=0 since we *define* that to be the origin
  values<N> v = values<N>::Zero();
  double prev_theta = 0.;
  for (int i = 0; i < T; i++) {
    transform_t trf = traj[(size_t)i+1];
    pose_t p = toPose(trf, prev_theta);
    v.block(pose_size*i,0,pose_size,1) = p.topRows(pose_size);
    prev_theta = p(2);
  }
  for (int i = 0; i < L; i++) {
    v.block(pose_size*T+lm_size*i, 0, lm_size, 1) = r[(size_t)i].topRows(lm_size);
  }
  return v;
}

template <int N>
World<N>::World(bool is_2D) : is_2D_(is_2D), landmarks_({}), ground_truth_({}), odom_({}), bag_({}) {
}

template <int N>
void World<N>::addLandmark(double x, double y) {
  landmark_t lm;
  lm << x, y, 1;
  landmarks_.push_back(lm);
}

template <int N>
void World<N>::runSimulation(int T) {
  ground_truth_.push_back(toTransform(0., 0., 0.));
  odom_.push_back(toTransform(0., 0., 0.));
  readLandmarks();
  for (int i = 1; i < T+1; i++) {
    moveRobot(0., 0.5);
    readLandmarks();
  }
}

template <int N>
void World<N>::moveRobot(double d_theta, double d_x) {
  double true_dx_std = 0.1; // m
  double true_dtheta_std = 0.0; // rad
  if (is_2D_)
    true_dtheta_std = 0.1;
  double noisy_x = stdn()*true_dx_std + d_x;
  double noisy_theta = stdn()*true_dtheta_std + d_theta;
  ground_truth_.push_back(toTransform(noisy_x, 0., noisy_theta) * ground_truth_.back());
  odom_.push_back(toTransform(d_x, 0., d_theta) * odom_.back());
}

template <int N>
void World<N>::readLandmarks() {
  double true_x_std = 0.1; // m
  double true_y_std = 0.0; // m
  if (is_2D_)
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

template <int N>
const bag_t World<N>::bag() {
  return bag_;
}

template <int N>
const values<N> World<N>::x0() {
  return toVector(odom_, bag_[0]);
}

template <int N>
const values<N> World<N>::groundTruth() {
  return toVector(ground_truth_, landmarks_);
}
