#include <random>
#include <chrono>
#include <Eigen/LU>
#include "utils.h"
#include <iostream>

double norm(const landmark_t &lm)
{
  return sqrt(lm(0)*lm(0) + lm(1)*lm(1));
}

bool collides(const transform_t &tf, const obstacles_t &obss)
{
  for (const obstacle_t &obs : obss)
  {
    int n = obs.rows();
    bool inside = true;
    for(int i = 0; i < n; i++) {
      landmark_t p0, p1;
      p0 << obs(i,0), obs(i,1), 1;
      if (i < (n-1)) {
        p1 << obs(i+1,0), obs(i+1,1), 1;
      } else {
        p1 << obs(0,0), obs(0,1), 1;
      }
      p0 = tf*p0;
      p1 = tf*p1;
      // TODO handle actual robot footprints rather than single points
      if (p0(0)*p1(1) - p1(0)*p0(1) < 0) {
        inside = false;
        break;
      }
    }
    if (inside) return true;
  }
  return false;
}

bool collides(const transform_t &tf, const landmarks_t &lms, double radius)
{
  for (const landmark_t &lm : lms)
  {
    if (lm(2) == 0.0) {
      // No reading on this landmark
      continue;
    }
    landmark_t tf_lm = tf * lm;
    if (norm(tf_lm) < radius) return true;
  }
  return false;
}

values toVector(const trajectory_t &traj, const landmark_readings_t &r) {
  int T = (int) traj.size()-1;
  int L = (int) r.size();
  int pose_size(1), lm_size(1);
  if (IS_2D) {
    pose_size = 3;
    lm_size = 2;
  }
  // We don't include a variable for T=0 since we *define* that to be the origin
  int N = pose_size*T+lm_size*L;
  values v = values::Zero(N);
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

transform_t toTransformRotateFirst(double x, double y, double theta) {
  transform_t m;
  m << cos(theta),  sin(theta), -x,
       -sin(theta), cos(theta), -y,
                0,           0,  1;
  return m;
}

pose_t toPose(const transform_t &trf, double prev_theta) {
  pose_t s = trf.inverse() * pose_t(0, 0, 1);
  double cos_theta = trf(0,0);
  double sin_theta = -trf(1,0);
  double theta = atan2(sin_theta, cos_theta);
  while (theta < prev_theta - M_PI)
    theta += 2*M_PI;
  while (theta > prev_theta + M_PI)
    theta -= 2*M_PI;
  s(2) = theta;
  return s;
}

transform_t toTransform(const pose_t &pose) {
  return toTransformRotateFirst(0, 0, pose(2)) * toTransformRotateFirst(pose(0), pose(1), 0);
}

landmark_reading_t project(const landmark_t &landmark, const transform_t &transform) {
  landmark_t transformed_landmark = transform * landmark;
  // If we're projecting to a 1-D camera in front of the car:
  // using landmark_reading_t = double;
  // transformed_landmark(1)/transformed_landmark(0);
  return transformed_landmark;
}

std::normal_distribution<double> stdn_dist(0.0, 1.0);
unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
std::default_random_engine generator(seed);

double stdn() {
  return stdn_dist(generator);
}
