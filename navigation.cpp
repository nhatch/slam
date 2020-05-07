#include "world_interface.h"
#include "plan.h"
#include "search.h"
#include "constants.h"
#include "graphics.h"
#include "utils.h"
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <X11/Xlib.h>
#include <Eigen/LU>

using namespace NavSim;
extern const bool IS_2D { true };
const double CONTROL_HZ = 5;
const double REFRESH_HZ = 30;
const int REFRESH_PER_CONTROL = REFRESH_HZ / CONTROL_HZ;

void drawPlan(sf::RenderWindow &plan_window, const transform_t &plan_odom,
    const plan_t &p, const point_t &goal, const points_t &lidar_hits)
{
  trajectory_t traj({});
  transform_t trans = toTransform({0,0,0});
  traj.push_back(trans);
  for (int i = 0; i < p.rows(); i++)
  {
    double theta = p(i,0);
    double x = p(i,1);
    // Doesn't matter if we rotate first because our plans never rotate
    // and move forward at the same time.
    trans = toTransformRotateFirst(x, 0, theta) * trans;
    traj.push_back(trans);
  }
  transform_t base = toTransform({WINDOW_CENTER_X,WINDOW_CENTER_Y,M_PI/2});
  transform_t plan_base = plan_odom * getOdom().inverse() * base;
  _drawTraj(plan_window, transformTraj(traj, plan_base), sf::Color::Red);
  _drawPoints(plan_window, transformReadings({goal}, plan_base), sf::Color::Green, 10);
  _drawPoints(plan_window, transformReadings(lidar_hits, plan_base), sf::Color::Red, 3);
  drawRobot(plan_window, base, sf::Color::Black);
  display(plan_window);
}

int main()
{
  XInitThreads();
  world_interface_init();
  sf::RenderWindow plan_window(sf::VideoMode(WINDOW_WIDTH_PX, WINDOW_WIDTH_PX), "Planning visualization");
  setGoal({13, 6, 1});
  std::cout << "Type 'q' to quit, 'wasd' to move around, 'r' to toggle autonomous mode.\n";

  bool autonomous = false;

  int c = 0;
  int iter = 0;
  bool done = false;
  point_t waypoint;
  transform_t plan_odom = getOdom();
  points_t lidar_hits = getLidarScan();
  plan_t plan = act(lidar_hits, &waypoint);
  drawPlan(plan_window, plan_odom, plan, waypoint, lidar_hits);
  while (true) {
    while ((c = pollWindowEvent(plan_window)) != -1) {
      switch (c) {
        case 22:
        case 73:
          setCmdVel(0.0, 2.0);
          break;
        case 18:
        case 74:
          setCmdVel(0.0, -2.0);
          break;
        case 0:
        case 71:
          setCmdVel(M_PI/2, 0.0);
          break;
        case 3:
        case 72:
          setCmdVel(-M_PI/2, 0.0);
          break;
        case -3:
          setCmdVel(0.0, 0.0);
          break;
        case 17:
          autonomous = !autonomous;
          break;
        case -2:
          done = true;
          break;
        default:
          break;
      }
    }
    if (done) break;

    if (iter++ % REFRESH_PER_CONTROL == 0) {
      plan_odom = getOdom();
      lidar_hits = getLidarScan();
      plan = act(lidar_hits, &waypoint);
      if (autonomous) {
        if (plan.size() == 0)
        {
          std::cout << "Disabling autonomous mode.\n";
          autonomous = false;
          setCmdVel(0.,0.);
        }
        else
        {
          action_t action = plan.row(0);
          setCmdVel(action(0)*CONTROL_HZ, action(1)*CONTROL_HZ);
        }
      }
    }
    drawPlan(plan_window, plan_odom, plan, waypoint, lidar_hits);

    usleep(1000*1000 / REFRESH_HZ);
  }

  return 0;
}
