
#include <math.h>
#include <iostream>
#include "graph.h"
#include "factors.h"
#include "world.h"

/* Input: odom trajectory of length T+1 (starting at the origin)
 *        list of landmark readings (of length T+1, with nLandmarks readings at each step)
 */
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

void pstr(Eigen::VectorXd v, bool newline) {
  std::cout << "(";
  int d = v.size();
  for (int i = 0; i < d-1; i++) {
    std::cout << v(i) << ", ";
  }
  std::cout << v(d-1) << ")";
  if (newline)
    std::cout << std::endl;
}

template <int N>
void printRange(Graph<N> &g, values<N> ground_truth, int start, int end, int size) {
  for (int i = start; i < end; i += size) {
    pstr(ground_truth.block(i,0,size,1), false);
    std::cout << "   Estimated: ";
    pstr(g.solution().block(i,0,size,1), false);
    std::cout << "   Difference: ";
    pstr((g.solution()-ground_truth).block(i,0,size,1), false);
    std::cout << "   Std: ";
    pstr(g.covariance().diagonal().block(i,0,size,1).cwiseSqrt(), true);
  }
}

int main() {
  std::cout.precision(3);
  std::cout << std::fixed;

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

  std::cout << std::showpos;
  values<N> ground_truth = w.groundTruth();
  std::cout << std::endl << "Trajectory:" << std::endl;
  printRange(g, ground_truth, 0, 3*T, 3);
  std::cout << std::endl << "Landmark locations:" << std::endl;
  printRange(g, ground_truth, 3*T, N, 2);
  std::cout << std::noshowpos;

  std::cout << std::endl << "Odom error: " << (ground_truth-g.x0()).norm() << std::endl;
  std::cout << "Smoothed error: " << (ground_truth-g.solution()).norm() << std::endl;
  std::cout << "Odom potential: " << g.eval(g.x0()) << std::endl;
  std::cout << "Smoothed potential: " << g.eval(g.solution()) << std::endl;
  std::cout << "Ground truth potential: " << g.eval(ground_truth) << std::endl;
  return 0;
}
