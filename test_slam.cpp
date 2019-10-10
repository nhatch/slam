
#include <math.h>
#include <iostream>
#include "graph.h"
#include "world.h"

template <int N>
values<N> toVector(trajectory_t &traj, landmark_readings_t &r) {
  int T = traj.size()-1;
  int L = r.size();
  assert("uh oh" && (T+L == N));
  // We don't include a variable for T=0 since we *define* that to be the origin
  values<N> v = values<N>::Zero();
  for (int i = 0; i < T; i++) {
    v(i) = -traj[(size_t)i+1](0,2);
  }
  for (int i = 0; i < L; i++) {
    v(T+i) = r[(size_t)i](0);
  }
  return v;
}

/* Input: odom trajectory of length T+1 (starting at the origin)
 *        list of landmark readings (of length T+1, with nLandmarks readings at each step)
 */
template <int N>
Graphz<N> smooth(trajectory_t &odom, bag_t &bag) {
  int T = odom.size()-1;
  int nLandmarks = bag[0].size();
  double odom_std = 0.1;
  double sensor_std = 0.1;
  landmark_readings_t r = bag[0];
  assert("whoohoo" && (N == T + nLandmarks));
  values<N> x0 = toVector<N>(odom, r);
  Graphz<N> graph;
  for (int t=0; t < T+1; t++) {
    for (int i = 0; i < nLandmarks; i++) {
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

void run_simulation(World &w, int T) {
  w.car_.read(w.landmarks_);
  for (int i = 1; i < T+1; i++) {
    w.car_.move(0., 0.5);
    w.car_.read(w.landmarks_);
    std::cout << "Moved to " << -w.car_.ground_truth_.back()(0,2) << std::endl;
  }
}

template <int N>
void printRange(Graphz<N> &g, int start, int end) {
  for (int i = start; i < end; i++) {
    std::cout << g.solution()(i) << '\t';
    std::cout << "(std: " << sqrt(g.covariance()(i,i)) << ")\n";
  }
}

int main() {
  std::cout.precision(3);
  std::cout << std::fixed;

  World w;
  w.addLandmark(3., 0.);
  w.addLandmark(6., 0.);
  w.addLandmark(-1., 0.);
  w.addLandmark(-0., 0.);
  w.addLandmark(3.1, 0.);
  w.addLandmark(0.1, 0.);
  constexpr int nLandmarks = 6;
  constexpr int T = 10;
  constexpr int N = T+nLandmarks;

  run_simulation(w, T);
  Graphz<N> g = smooth<N>(w.car_.odom_, w.car_.bag_);

  std::cout << std::endl << "Estimated trajectory:" << std::endl;
  printRange(g, 0, T);
  std::cout << std::endl << "Estimated landmark locations:" << std::endl;
  printRange(g, T, N);

  values<N> ground_truth = toVector<N>(w.car_.ground_truth_, w.landmarks_);
  std::cout << std::endl << "Odom error: " << (ground_truth-g.x0()).norm() << std::endl;
  std::cout << "Smoothed error: " << (ground_truth-g.solution()).norm() << std::endl;
  std::cout << "Odom potential: " << g.eval(g.x0()) << std::endl;
  std::cout << "Smoothed potential: " << g.eval(g.solution()) << std::endl;
  return 0;
}
