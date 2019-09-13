
#include <iostream>
#include <cmath>
#include "graph.h"

int main() {
  Graph graph;
  Factor prior { NULL_VAR, 0, 0.3, 0.0 };
  Factor between1 { 0, 1, 0.2, 2.0 };
  Factor between2 { 1, 2, 0.2, 2.0 };
  Factor measurement0 { NULL_VAR, 0, 0.1, 0.0 };
  Factor measurement1 { NULL_VAR, 1, 0.1, 2.0 };
  Factor measurement2 { NULL_VAR, 2, 0.1, 4.0 };
  //graph.push_back(prior);
  graph.push_back(between1);
  graph.push_back(between2);
  graph.push_back(measurement0);
  graph.push_back(measurement1);
  graph.push_back(measurement2);
  int N = 3;
  vec_t x0(N);
  x0 << 0.0, 2.0, 4.0;
  vec_t sol = findMAP(graph, x0, N);
  hess_t cov = findCov(graph, sol, N);
  std::cout << sol << std::endl;
  std::cout << cov << std::endl;
  return 0;
}
