#ifndef SLAM_GRAPHICS_H
#define SLAM_GRAPHICS_H

#include "utils.h"

void draw(const landmarks_t &lms_gt, const trajectory_t &traj_gt,
          const landmarks_t &lms_odom, const trajectory_t &traj_odom);
void draw(const landmarks_t &lms_odom, const trajectory_t &traj_odom);
void drawSmoothed(const landmarks_t &lms, const trajectory_t &traj);
void drawGoal(const landmark_t &goal);
void spin();

#endif
