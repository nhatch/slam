#include "world.h"
#include "plan.h"
#include "search.h"
#include "constants.h"
#include "world_ui.h"
#include <iostream>
#include <stdio.h>
#include <unistd.h>

using namespace NavSim;
extern const bool IS_2D { true };

int main()
{
  World w;
  point_t goal;
  goal << 9., 6., 1.;
  setGoal(goal);
  w.addDefaultObstacles();
  w.addLandmark(20, 3);
  WorldUI ui(w);

  bool autonomous = false;

  int c = -1;
  action_t action = act(ui);
  ui.show();
  while (c != -2) {
    c = ui.pollKeyPress();
    if (c == 17) autonomous = !autonomous;

    if (((int) c) != -1 || autonomous)
    {
      ui.show();
      action = act(ui);
      if (action == action_t::Zero())
      {
        std::cout << "Disabling autonomous mode.\n";
        autonomous = false;
      }
      ui.show();
    }

    if (autonomous) {
      w.moveRobot(action(0), action(1));
    }

    usleep(10*1000);
  }

  return 0;
}
