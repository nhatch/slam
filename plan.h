
#ifndef _PLAN_H_
#define _PLAN_H_

#include "utils.h"
#include "world.h"
#include <Eigen/Core>

// Sequence of controls. Each control is a (x distance, theta distance) pair.
using plan_t = Eigen::Matrix<double, Eigen::Dynamic, 2>;

plan_t getPlan(const World &w, const landmark_t &goal);
void drawPlan(const plan_t &p, const transform_t &transform);

#endif
