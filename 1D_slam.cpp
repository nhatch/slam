
#include "graph.h"
#include "factors.h"
#include "world.h"
#include "print_results.h"

extern const bool IS_2D { false };

Graph smooth(const values& x0, const bag_t &bag) {
  int T = (int)bag.size()-1;
  int nLandmarks = (int)bag[0].size();
  double odom_std = 0.1;
  double sensor_std = 0.1;
  assert("whoohoo" && (x0.size() == T + nLandmarks));
  Graph graph;
  for (int t=0; t < T+1; t++) {
    for (int i=0; i < nLandmarks; i++) {
      double l_dx = bag[(size_t)t][(size_t)i](0);
      if (t==0) {
        graph.add(new GPSFactor(T+i, sensor_std, l_dx));
      } else {
        graph.add(new OdomFactor(t-1, T+i, sensor_std, l_dx));
      }
    }
    if (t==1)
      graph.add(new GPSFactor(0, odom_std, x0(t-1)));
    if (t>1)
      graph.add(new OdomFactor(t-2, t-1, odom_std, x0(t-1)-x0(t-2)));
  }
  graph.solve(x0, 0.001, 100000);
  return graph;
}

int main() {
  constexpr int nLandmarks = 6;
  constexpr int T = 10;

  World w;
  w.addLandmark(3., 0.);
  w.addLandmark(6., 0.);
  w.addLandmark(-1., 0.);
  w.addLandmark(-0., 0.);
  w.addLandmark(3.1, 0.);
  w.addLandmark(0.1, 0.);

  w.runSimulation(T);
  const bag_t bag = w.landmarks();
  values x0 = toVector(w.odom(), bag[0]);
  Graph g = smooth(x0, bag);
  printResults(w, g, T);

  return 0;
}
