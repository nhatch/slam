
#include "plan.h"
#include "utils.h"

void setGoal(const URCLeg &leg);
plan_t act(const points_t &lidar_scan, const points_t &landmarks, point_t *waypoint);
