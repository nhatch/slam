
#include <math.h>
#include <iostream>
#include "graph.h"
#include "world.h"

struct smoothed_t {
  vec_t x0;
  vec_t sol;
  hess_t cov;
  Graph graph;
};

/* Input: landmark_readings of length T, each reading of length N (by 3)
 * Output: length N vector of landmark locations; length T vector of robot transforms
 */
smoothed_t smooth(trajectory_t &odom, bag_t &bag) {
  int T = odom.size()-1;
  int nLandmarks = bag[0].size();
  double landmark_std = 3.0;
  double odom_std = 0.1;
  double sensor_std = 0.1;
  landmark_readings_t r = bag[0];
  vec_t x0 = toVector(odom, r);
  Graph graph;
  graph.push_back(Factor { NULL_VAR, 0, 0.05, 0.0 }); // Enforce strong prior on start location
  for (int i = 0; i < nLandmarks; i++)
    graph.push_back(Factor { NULL_VAR, T+1+i, landmark_std, 0.0 });
  for (int t=0; t < T+1; t++) {
    for (int i = 0; i < nLandmarks; i++) {
      double l_dx = bag[(size_t)t][(size_t)i](0);
      graph.push_back(Factor { t, T+1+i, sensor_std, l_dx });
    }
    if (t>0)
      graph.push_back(Factor { t-1, t, odom_std, x0(t)-x0(t-1) });
  }
  int N = T+1+nLandmarks;
  vec_t sol = findMAP(graph, x0, N, 0.001, 100000);
  hess_t cov = findCov(graph, sol, N);
  return smoothed_t { x0, sol, cov, graph };
}

void run_simulation(World &w, int T) {
  w.car_.read(w.landmarks_);
  for (int i = 1; i < T+1; i++) {
    w.car_.move(0., 0.5);
    w.car_.read(w.landmarks_);
    std::cout << "Moved to " << -w.car_.ground_truth_.back()(0,2) << std::endl;
  }
}

void printRange(smoothed_t &smoothed, int start, int end) {
  for (int i = start; i < end; i++) {
    std::cout << smoothed.sol(i) << '\t';
    std::cout << "(std: " << sqrt(smoothed.cov(i,i)) << ")\n";
  }
}

int main() {
  World w;
  w.addLandmark(3., 0.);
  w.addLandmark(6., 0.);
  w.addLandmark(-1., 0.);
  w.addLandmark(-0., 0.);
  w.addLandmark(3.1, 0.);
  w.addLandmark(0.1, 0.);
  int nLandmarks = w.landmarks_.size();
  int T = 10;
  int N = T+1+nLandmarks;

  run_simulation(w, T);
  smoothed_t smoothed = smooth(w.car_.odom_, w.car_.bag_);

  std::cout << std::endl << "Estimated trajectory:" << std::endl;
  printRange(smoothed, 0, T+1);
  std::cout << std::endl << "Estimated landmark locations:" << std::endl;
  printRange(smoothed, T+1, N);

  vec_t ground_truth = toVector(w.car_.ground_truth_, w.landmarks_);
  std::cout << std::endl << "Odom error: " << (ground_truth-smoothed.x0).norm() << std::endl;
  std::cout << "Smoothed error: " << (ground_truth-smoothed.sol).norm() << std::endl;
  std::cout << "Odom potential: " << eval(smoothed.graph, smoothed.x0) << std::endl;
  std::cout << "Smoothed potential: " << eval(smoothed.graph, smoothed.sol) << std::endl;
  return 0;
}
