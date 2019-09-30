
#include <math.h>
#include <iostream>
#include "graph.h"
#include "world.h"

vec_t toVector(trajectory_t &traj, landmark_readings_t &r) {
  int T = traj.size()-1;
  int L = r.size();
  // We don't include a variable for T=0 since we *define* that to be the origin
  vec_t v = vec_t::Zero(T+L);
  for (int i = 0; i < T; i++) {
    v(i) = -traj[(size_t)i+1](0,2);
  }
  for (int i = 0; i < L; i++) {
    v(T+i) = r[(size_t)i](0);
  }
  return v;
}

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
  double odom_std = 0.1;
  double sensor_std = 0.1;
  landmark_readings_t r = bag[0];
  vec_t x0 = toVector(odom, r);
  Graph graph;
  for (int t=0; t < T+1; t++) {
    for (int i = 0; i < nLandmarks; i++) {
      double l_dx = bag[(size_t)t][(size_t)i](0);
      // Hack: -1 is NULL_VAR
      graph.push_back(Factor { t-1, T+i, sensor_std, l_dx });
    }
    if (t==1)
      graph.push_back(Factor { NULL_VAR, t-1, odom_std, x0(t-1) });
    if (t>1)
      graph.push_back(Factor { t-2, t-1, odom_std, x0(t-1)-x0(t-2) });
  }
  int N = T+nLandmarks;
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
  int N = T+nLandmarks;

  run_simulation(w, T);
  smoothed_t smoothed = smooth(w.car_.odom_, w.car_.bag_);

  std::cout << std::endl << "Estimated trajectory:" << std::endl;
  printRange(smoothed, 0, T);
  std::cout << std::endl << "Estimated landmark locations:" << std::endl;
  printRange(smoothed, T, N);

  vec_t ground_truth = toVector(w.car_.ground_truth_, w.landmarks_);
  std::cout << std::endl << "Odom error: " << (ground_truth-smoothed.x0).norm() << std::endl;
  std::cout << "Smoothed error: " << (ground_truth-smoothed.sol).norm() << std::endl;
  std::cout << "Odom potential: " << eval(smoothed.graph, smoothed.x0) << std::endl;
  std::cout << "Smoothed potential: " << eval(smoothed.graph, smoothed.sol) << std::endl;
  return 0;
}
