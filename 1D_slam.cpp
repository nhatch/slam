
#include "graph.h"
#include "factors.h"
#include "world.h"
#include "world_ui.h"
#include "slam_utils.h"
#include "print_results.h"
#include "constants.h"
#include <unistd.h>
using namespace NavSim;

extern const bool IS_2D { false };

Graph smooth(const values& x0, const traj_points_t &readings) {
  int T = (int)readings.size()-1;
  int nLandmarks = (int)readings[0].size();
  assert("whoohoo" && (x0.size() == T + nLandmarks));
  Graph graph;
  for (int t=0; t < T+1; t++) {
    for (int i=0; i < nLandmarks; i++) {
      point_t l = readings[(size_t)t][(size_t)i];
      if (l(2) == 0.0) continue; // Landmark wasn't visible
      double l_dx = l(0);
      if (t==0) {
        graph.add(new GPSFactor(T+i, sqrt(SLAM_VAR), l_dx));
      } else {
        graph.add(new OdomFactor(t-1, T+i, sqrt(SLAM_VAR), l_dx));
      }
    }
    if (t==1)
      graph.add(new GPSFactor(0, sqrt(SLAM_VAR), x0(t-1)));
    if (t>1)
      graph.add(new OdomFactor(t-2, t-1, sqrt(SLAM_VAR), x0(t-1)-x0(t-2)));
  }
  graph.solve(x0, 0.001, 100000, 0.1);
  return graph;
}

void optimizeAndRender(WorldUI &ui) {
  const traj_points_t readings = ui.world.landmarks();
  values x0 = toVector(ui.world.odom(), readings[0]);
  Graph g = smooth(x0, readings);
  printResults(ui, g);
}

int main() {
  constexpr int T = 10;

  World w;
  WorldUI ui(w);
  w.addDefaultLandmarks();

  ui.start();
  ui.goForwardTSteps(T);
  ui.drawTraj(w.odom(), false, sf::Color::Blue);
  optimizeAndRender(ui);
  ui.show();

  int c;
  while ((c = ui.pollKeyPress()) != -2) {
    if (c != -1) {
      ui.drawTraj(w.odom(), false, sf::Color::Blue);
      if (c == 6) { // 'g'
        optimizeAndRender(ui);
      }
      ui.show();
    }
    usleep(10*1000);
  }

  return 0;
}
