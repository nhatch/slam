#include "world_interface.h"
#include "plan.h"
#include "constants.h"
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

using namespace NavSim;

const int THETA_DIVISIONS = 8;

// State for the search algorithm
int theta = 0;
bool tag_visible = false;
point_t gps_goal;
double waypoint_radius = GPS_WAYPOINT_RADIUS;
double search_radius = 0.0;

void setGoal(const point_t &goal)
{
  gps_goal = goal;
}

point_t nextWaypoint(const points_t &lms)
{
  point_t robot_goal;
  if (lms[0](2) != 0.)
  {
    tag_visible = true;
    waypoint_radius = LANDMARK_WAYPOINT_RADIUS;
    robot_goal = lms[0];
    //robot_goal(0) -= 0.3; // Don't crash into the AR tag
  }
  else
  {
    tag_visible = false;
    waypoint_radius = GPS_WAYPOINT_RADIUS;
    point_t search_goal = gps_goal;
    double a = (theta * 2 * M_PI) / THETA_DIVISIONS;
    search_goal(0) += cos(a)*search_radius;
    search_goal(1) += sin(a)*search_radius;
    robot_goal = readGPS() * search_goal;
  }
  return robot_goal;
}

plan_t act(const points_t &lidar_scan, const points_t &landmarks, point_t *chosen_waypoint)
{
  point_t waypoint = nextWaypoint(landmarks);
  plan_t plan = getPlan(lidar_scan, waypoint, waypoint_radius);
  while (!tag_visible && plan.size() == 0) {
    // Either we reached the goal, or the goal seems to be unreachable. Change the goal.
    if (theta % THETA_DIVISIONS == 0)
      search_radius += SEARCH_RADIUS_INCREMENT;
    theta += 1;
    waypoint = nextWaypoint(landmarks);
    plan = getPlan(lidar_scan, waypoint, waypoint_radius);
  }
  *chosen_waypoint = waypoint;
  return plan;
}
