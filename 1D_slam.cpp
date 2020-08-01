#include <Eigen/LU>
#include "graph.h"
#include "factors.h"
#include "utils.h"
#include "slam_utils.h"
#include "constants.h"
using namespace NavSim;

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
  graph.solve(x0);
  return graph;
}

int main() {
  IS_2D = false;
  collectDataAndRunSLAM(&smooth);
  return 0;
}
