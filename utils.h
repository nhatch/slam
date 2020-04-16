#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Core>
#include <vector>

extern const bool IS_2D;

/* Obstacles are specifed as a list of 2D vertices. Must be convex.
 * Do not repeat the starting vertex at the end. */
using obstacle_t = Eigen::ArrayX2d;
using obstacles_t = std::vector<obstacle_t>;

using pose_t = Eigen::Vector3d; // Robot pose: x, y, theta
using transform_t = Eigen::Matrix3d; // a pose in matrix form; see toPose below
using trajectory_t = std::vector<transform_t>;

using point_t = Eigen::Vector3d; // x, y, 1 (or 0, 0, 0 to represent "no data")
                                 // The 1 at the end makes calculating affine transforms easier.
using points_t = std::vector<point_t>;
using traj_points_t = std::vector<points_t>; // points_t for each time along a trajectory

/* Whether the robot in location given by `tf` collides with anything. */
bool collides(const transform_t &tf, const obstacles_t &obss);
bool collides(const transform_t &tf, const points_t &lms, double radius);

/* Returns the smallest number t in the unit interval [0,1] such that
 * the point r0 + t(r1-r0) lies on an obstacle boundary. If there is no such point,
 * returns t=2.0. */
double obstacleIntersection(const point_t &r0, const point_t &r1, const obstacles_t &obss);

/* Left-multiplying a (map-frame) point_t by this matrix will give the
 * corresponding point_t in the frame that was rotated counterclockwise
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
transform_t toTransformRotateFirst(double x, double y, double theta);

// Given a transform, gives the robot pose that would have that transform.
// That is, the x,y location and angle theta of the robot such that left-multiplying a
// world-frame landmark location by `trf` will give you the robot-frame landmark location.
//
// NOTE: THIS IS AN INVERSE OF toTransform, NOT toTransformRotateFirst.
// The parameters of toTransformRotateFirst are not a robot pose
// (rotation and translation operations do not commute).
pose_t toPose(const transform_t &trf, double prev_theta);

transform_t toTransform(const pose_t &pose);

// One sample from a standard normal distribution
double stdn();

#endif
