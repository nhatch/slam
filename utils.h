#ifndef UTILS_H
#define UTILS_H

#include <eigen3/Eigen/Core>

using landmark_reading_t = Eigen::Vector3d; // Assume we have a range-and-bearing sensor: x, y, 1 in sensor frame
using landmark_readings_t = std::vector<landmark_reading_t>;
using bag_t = std::vector<landmark_readings_t>;

using pose_t = Eigen::Vector3d; // x, y, theta
using transform_t = Eigen::Matrix3d; // a pose in matrix form; see toTransform
using trajectory_t = std::vector<transform_t>;
using landmark_t = Eigen::Vector3d; // x, y, 1
using landmarks_t = std::vector<landmark_t>;

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
transform_t toTransform(double x, double y, double theta);

// Inverse operation for toTransform
pose_t toPose(transform_t trf, double prev_theta);

landmark_reading_t project(const landmark_t &landmark, const transform_t &transform);

// One sample from a standard normal distribution
double stdn();

#endif
