#include "world.h"
#include "graphics.h"
#include "plan.h"
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

#define KEY_UP 72
#define KEY_DOWN 80
#define KEY_LEFT 75
#define KEY_RIGHT 77

const double GRID_RES = 0.5;
const double THETA_INCREMENT = 1.0;
const double SEARCH_RADIUS = 2.0;
const double WAYPOINT_RADIUS = 0.5;

// State for the search algorithm
double theta = 0.0;
landmark_t gps_goal;

// TODO: I think we need a map frame, otherwise this search radius thing will look stupid (the robot will barely move)

void setGoal(landmark_t &goal)
{
  gps_goal = goal;
}

landmark_t nextWaypoint(World &w)
{
  landmark_t search_goal = gps_goal;
  search_goal(0) += cos(theta)*SEARCH_RADIUS;
  search_goal(1) += sin(theta)*SEARCH_RADIUS;
  landmark_t robot_goal = w.gps().back() * search_goal;
  return robot_goal;
}

action_t act(World &w, transform_t &viz_tf)
{
  landmark_t waypoint = nextWaypoint(w);
  plan_t plan = getPlan(w, waypoint, WAYPOINT_RADIUS);
  while (plan.size() == 0) {
    // Either we reached the goal, or the goal seems to be unreachable. Change the goal.
    theta += THETA_INCREMENT;
    waypoint = nextWaypoint(w);
    plan = getPlan(w, waypoint, WAYPOINT_RADIUS);
  }
  drawPlan(plan, viz_tf);
  drawGoal(viz_tf.inverse() * waypoint);

  return plan.row(0);
}
