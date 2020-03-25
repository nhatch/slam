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

const int THETA_DIVISIONS = 8;
const double RADIUS_INCREMENT = 1.0;

// State for the search algorithm
int theta = 0;
bool tag_visible = false;
landmark_t gps_goal;
double waypoint_radius = 0.5;
double search_radius = 0.0;

// TODO: I think we need a map frame, otherwise this search radius thing will look stupid (the robot will barely move)

void setGoal(landmark_t &goal)
{
  gps_goal = goal;
}

landmark_t nextWaypoint(World &w)
{
  landmark_t robot_goal;
  landmark_readings_t lms = w.tags_bag().back();
  if (lms[0](2) != 0.)
  {
    tag_visible = true;
    waypoint_radius = 0.2;
    robot_goal = lms[0];
    //robot_goal(0) -= 0.3; // Don't crash into the AR tag
  }
  else
  {
    tag_visible = false;
    waypoint_radius = 0.5;
    landmark_t search_goal = gps_goal;
    double a = (theta * 2 * M_PI) / THETA_DIVISIONS;
    search_goal(0) += cos(a)*search_radius;
    search_goal(1) += sin(a)*search_radius;
    robot_goal = w.gps().back() * search_goal;
  }
  return robot_goal;
}

action_t act(World &w, transform_t &viz_tf)
{
  landmark_t waypoint = nextWaypoint(w);
  plan_t plan = getPlan(w, waypoint, waypoint_radius);
  while (!tag_visible && plan.size() == 0) {
    // Either we reached the goal, or the goal seems to be unreachable. Change the goal.
    if (theta % THETA_DIVISIONS == 0)
      search_radius += RADIUS_INCREMENT;
    theta += 1;
    waypoint = nextWaypoint(w);
    plan = getPlan(w, waypoint, waypoint_radius);
  }
  drawPlan(plan, viz_tf);
  drawGoal(viz_tf.inverse() * waypoint);
  if (tag_visible && plan.size() == 0) {
    std::cout << "Search complete.\n";
    return action_t::Zero();
  } else {
    return plan.row(0);
  }
}