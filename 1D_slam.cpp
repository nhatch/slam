
#include "graph.h"
#include "factors.h"
#include "world.h"
#include "print_results.h"

template <int N>
Graph<N> smooth(const values<N>& x0, const bag_t &bag) {
  int T = (int)bag.size()-1;
  int nLandmarks = (int)bag[0].size();
  double odom_std = 0.1;
  double sensor_std = 0.1;
  assert("whoohoo" && (N == T + nLandmarks));
  Graph<N> graph;
  for (int t=0; t < T+1; t++) {
    for (int i=0; i < nLandmarks; i++) {
      double l_dx = bag[(size_t)t][(size_t)i](0);
      if (t==0) {
        graph.add(new GPSFactor<N>(T+i, sensor_std, l_dx));
      } else {
        graph.add(new OdomFactor<N>(t-1, T+i, sensor_std, l_dx));
      }
    }
    if (t==1)
      graph.add(new GPSFactor<N>(0, odom_std, x0(t-1)));
    if (t>1)
      graph.add(new OdomFactor<N>(t-2, t-1, odom_std, x0(t-1)-x0(t-2)));
  }
  graph.solve(x0, 0.001, 100000);
  return graph;
}

int main() {
  constexpr int nLandmarks = 6;
  constexpr int T = 10;
  constexpr int N = T+nLandmarks;

  World<N> w(false);
  w.addLandmark(3., 0.);
  w.addLandmark(6., 0.);
  w.addLandmark(-1., 0.);
  w.addLandmark(-0., 0.);
  w.addLandmark(3.1, 0.);
  w.addLandmark(0.1, 0.);

  w.runSimulation(T);
  Graph<N> g = smooth<N>(w.x0(), w.bag());
  printResults<N>(w, g, false, T);

  return 0;
}
