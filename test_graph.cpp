
#include <iostream>
#include <cmath>
#include "graph.h"
#include "factors.h"

constexpr int N = 3;
constexpr int D = 1;

int main() {
  values<N> x0;
  x0 << 0.1, 2.0, 4.0;

  Graph<N> g;
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
