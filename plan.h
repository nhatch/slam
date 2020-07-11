
#ifndef _PLAN_H_
#define _PLAN_H_

#include "utils.h"
#include <Eigen/Core>

// Sequence of controls. Each control is a (x distance, theta distance) pair.
using plan_t = Eigen::Matrix<double, Eigen::Dynamic, 2>;
using action_t = Eigen::Vector2d;

plan_t getPlan(const points_t &lidar_scan, const point_t &goal, double goal_radius);

#endif
