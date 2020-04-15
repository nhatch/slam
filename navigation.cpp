#include "world.h"
#include "graphics.h"
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
  WorldUI ui(w);
  point_t goal;
  goal << 9., 6., 1.;
  setGoal(goal);
  w.addDefaultObstacles();
  w.addLandmark(20, 3);
  w.startSimulation();
  ui.render();
  ui.show();

  bool autonomous = false;

  char c = -2;
  action_t action;
  while (c != 16) {
    c = ui.handleKeyPress();
    if (c == 17) autonomous = !autonomous;

    if (((int) c) != -1 || autonomous)
    {
      ui.render();
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
