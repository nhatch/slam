
#ifndef __SLAM_UTILS_H__
#define __SLAM_UTILS_H__

#include "utils.h"
#include "graph.h"

values toVector(const trajectory_t &traj, const points_t &r, int max_num_poses, double prev_theta);
//void collectDataAndRunSLAM(Graph (*smooth)(const values &x0, const traj_points_t &readings));
void collectDataAndRunSLAM();

#endif
