#include "world_interface.h"
#include "plan.h"
#include "search.h"
#include "constants.h"
#include "graphics.h"
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <X11/Xlib.h>

using namespace NavSim;
extern const bool IS_2D { true };
const double CONTROL_RATE_HZ = 5;

int main()
{
  XInitThreads();
  world_interface_init();
  sf::RenderWindow plan_window(sf::VideoMode(WINDOW_WIDTH_PX, WINDOW_WIDTH_PX), "Planning visualization");
  setGoal({13, 6, 1});
  std::cout << "Type 'q' to quit, 'wasd' to move around, 'r' to toggle autonomous mode.\n";

  bool autonomous = false;

  int c = 0;
  bool done = false;
  action_t action = act(plan_window);
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

    if (autonomous)
    {
      action = act(plan_window);
      if (action == action_t::Zero())
      {
        std::cout << "Disabling autonomous mode.\n";
        autonomous = false;
      }
      setCmdVel(action(0)*CONTROL_RATE_HZ, action(1)*CONTROL_RATE_HZ);
    }

    usleep(1000*1000 / CONTROL_RATE_HZ);
  }

  return 0;
}
