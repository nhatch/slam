
#include "graph.h"
#include "factors.h"
#include "world.h"
#include "print_results.h"
#include "graphics.h"

template <int N>
Graph<N> smooth(const values<N> &x0, const bag_t &bag) {
  int T = (int) bag.size()-1;
  int nLandmarks = (int) bag[0].size();
  assert("whoohoo" && (N == T*3 + nLandmarks*2));
  covariance<3> odom_cov = covariance<3>::Zero();
  // TODO what are the right numbers here? Should y be correlated with theta?
  odom_cov << 0.1*0.1, 0, 0,
              0, 0.1*0.1, 0,
              0, 0, 0.1*0.1;
  covariance<3> odom_cov_inv = odom_cov.inverse();
  covariance<2> sensor_cov = covariance<2>::Zero();
  sensor_cov << 0.1*0.1, 0,
                0, 0.1*0.1;
  covariance<2> sensor_cov_inv = sensor_cov.inverse();
  Graph<N> graph;
  for (int t=0; t < T+1; t++) {
    for (int i = 0; i < nLandmarks; i++) {
      landmark_reading_t l = bag[(size_t)t][(size_t)i];
      measurement<2> lm = measurement<2> { l(0), l(1) };
      graph.add(new LandmarkFactor2D<N>(3*T+2*i, 3*(t-1), sensor_cov_inv, lm));
    }
    if (t>0) {
      int t2 = 3*(t-1);
      int t1 = 3*(t-2);
      measurement<3> om2 = x0.block(t2,0,3,1);
      measurement<3> om1 = measurement<3>::Zero();
      if (t1 >= 0)
        om1 = x0.block(t1,0,3,1);
      graph.add(new OdomFactor2D<N>(t2, t1, odom_cov_inv, om2-om1));
    }
  }
  graph.solve(x0, 0.0001, 10000);
  return graph;
}

int main() {
  constexpr int nLandmarks = 6;
  constexpr int T = 10;
  constexpr int N = 3*T+2*nLandmarks;

  World<N> w(true);
  w.addLandmark(3., 1.);
  w.addLandmark(6., -1.);
  w.addLandmark(-1., 0.);
  w.addLandmark(-0., -3.);
  w.addLandmark(3.1, 1.);
  w.addLandmark(0.1, 1.);

  w.runSimulation(T);
  Graph<N> g = smooth<N>(w.x0(), w.bag());
  printResults<N>(w, g, true, T);

  spin();
  return 0;
}
