#include <random>
#include <chrono>
#include <Eigen/LU>
#include "utils.h"
#include "constants.h"
#include <iostream>

bool IS_2D { true };

points_t transformReadings(const points_t &ps, const transform_t &tf) {
  transform_t tf_inv = tf.inverse();
  points_t readings({});
  for (point_t p : ps) {
    readings.push_back(tf_inv * p);
  }
  return readings;
}

trajectory_t transformTraj(const trajectory_t &traj, const transform_t &tf) {
  trajectory_t tf_traj({});
  // TODO can probably simplify this method to use just one for loop
  for (const transform_t &tf_i : traj) {
    tf_traj.push_back(tf_i * tf);
  }
  return tf_traj;
}

// Computational optimization. With lots of obstacles things get expensive.
// To guarantee this is valid, don't make any obstacles larger than
// LIDAR_MAX_RANGE in diameter.
bool inRange(const point_t &p, const obstacle_t &obs) {
  point_t op;
  op << obs(0,0), obs(0,1), p(2);
  return ((p-op).norm() < 2*NavSim::LIDAR_MAX_RANGE);
}

bool collides(const transform_t &tf, const obstacles_t &obss)
{
  pose_t p = toPose(tf,0);
  for (const obstacle_t &obs : obss)
  {
    if (!inRange(p, obs)) continue;
    int n = obs.rows();
    bool inside = true;
    for(int i = 0; i < n; i++) {
      point_t p0, p1;
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

bool segmentIntersection(const point_t &r0, const point_t &r1,
                         const point_t &p0, const point_t &p1, double *t)
{
  Eigen::Matrix2d A;
  Eigen::Vector2d b;
  b = r0.topRows(2) - p0.topRows(2);
  A.col(0) = r0.topRows(2) - r1.topRows(2);
  A.col(1) = p1.topRows(2) - p0.topRows(2);
  if (A.determinant() == 0.0) return false;
  Eigen::Vector2d tt = A.inverse() * b;
  if (tt(1) >= 0.0 && tt(1) <= 1.0 && tt(0) >= 0.0 && tt(1) <= 1.0)
  {
    *t = tt(0);
    return true;
  }
  return false;
}

// returns: Fraction of distance along the segment r0->r1 where the segment hits an obstacle.
//          Returns 2.0 if the segment never hits an obstacle;
//          Otherwise `hit` will be populated with the exact location of the hit.
double obstacleIntersection(const point_t &r0, const point_t &r1, const obstacles_t &obss)
{
  double min_t = 2.0;
  for (const obstacle_t &obs : obss)
  {
    if (!inRange(r0, obs)) continue;
    int n = obs.rows();
    for(int i = 0; i < n; i++)
    {
      point_t p0, p1;
      p0 << obs(i,0), obs(i,1), 1;
      if (i < (n-1)) {
        p1 << obs(i+1,0), obs(i+1,1), 1;
      } else {
        p1 << obs(0,0), obs(0,1), 1;
      }

      double t;
      if (segmentIntersection(r0, r1, p0, p1, &t)) {
        if (t < min_t) {
          min_t = t;
        }
      }
    }
  }

  return min_t;
}

bool collides(const transform_t &tf, const points_t &ps, double radius)
{
  for (const point_t &p : ps)
  {
    if (p(2) == 0.0) {
      // This point is a "no data" point
      continue;
    }
    point_t tf_p = tf * p;
    tf_p(2) = 0;
    if (tf_p.norm() < radius) return true;
  }
  return false;
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

std::normal_distribution<double> stdn_dist(0.0, 1.0);
unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
std::default_random_engine generator(seed);

double stdn() {
  return stdn_dist(generator);
}
