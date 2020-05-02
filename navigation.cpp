#include "world_interface.h"
#include "plan.h"
#include "search.h"
#include "constants.h"
#include <iostream>
#include <stdio.h>
#include <unistd.h>

using namespace NavSim;
extern const bool IS_2D { true };
const double CONTROL_RATE_HZ = 0.5;

int main()
{
  init();
  setGoal({13, 6, 1});

  bool autonomous = true;

  int c = -1;
  action_t action = act();
  while (c != -2) {
    int c = -1;// todo read line from input, if "quit", break loop
      // Use interrupt to cancel autonomous mode

    if (((int) c) != -1 || autonomous)
    {
      action = act();
      if (action == action_t::Zero())
      {
        std::cout << "Disabling autonomous mode.\n";
        autonomous = false;
      }
    }

    if (autonomous) {
      setCmdVel(action(0), action(1));
    }

    usleep(CONTROL_RATE_HZ * 1000*1000);
  }

  return 0;
}
