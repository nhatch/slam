
#include <iostream>
#include <cmath>
#include "graph.h"

constexpr int N = 3;
constexpr int D = 1;

int main() {
  Graph graph;
  Factor between1 { 0, 1, 0.2, 2.0 };
  Factor between2 { 1, 2, 0.2, 2.0 };
  Factor measurement0 { NULL_VAR, 0, 0.1, 0.0 };
  Factor measurement1 { NULL_VAR, 1, 0.1, 2.0 };
  Factor measurement2 { NULL_VAR, 2, 0.1, 4.0 };
  graph.push_back(between1);
  graph.push_back(between2);
  graph.push_back(measurement0);
  graph.push_back(measurement1);
  graph.push_back(measurement2);
  vec_t x0(N);
  x0 << 0.1, 2.0, 4.0;
  vec_t sol = findMAP(graph, x0, N);
  hess_t cov = findCov(graph, sol, N);
  std::cout << sol << std::endl;
  std::cout << cov << std::endl;

  Graphz<N> g;
  g.add(new GPSFactor<N>(0,     0.1, 0.0));
  g.add(new GPSFactor<N>(1,     0.1, 2.0));
  g.add(new GPSFactor<N>(2,     0.1, 4.0));
  g.add(new OdomFactor<N>(0, 1, 0.2, 2.0));
  g.add(new OdomFactor<N>(1, 2, 0.2, 2.0));
  g.solve(x0);
  std::cout << g.solution() << std::endl;
  std::cout << g.covariance() << std::endl;

  return 0;
}
