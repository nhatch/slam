
#include <iostream>
#include "graph.h"
#include "world.h"

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

template <int N>
void printResults(World<N> &w, Graph<N> &g, bool is_2D, int T) {
  int pose_size(1), lm_size(1);
  if (is_2D) {
    pose_size = 3;
    lm_size = 2;
  }

  std::cout.precision(3);
  std::cout << std::fixed;

  std::cout << std::showpos;
  values<N> ground_truth = w.groundTruth();
  std::cout << std::endl << "Trajectory:" << std::endl;
  printRange(g, ground_truth, 0, pose_size*T, pose_size);
  std::cout << std::endl << "Landmark locations:" << std::endl;
  printRange(g, ground_truth, pose_size*T, N, lm_size);
  std::cout << std::noshowpos;

  std::cout << std::endl << "Odom error: " << (ground_truth-g.x0()).norm() << std::endl;
  std::cout << "Smoothed error: " << (ground_truth-g.solution()).norm() << std::endl;
  std::cout << "Odom potential: " << g.eval(g.x0()) << std::endl;
  std::cout << "Smoothed potential: " << g.eval(g.solution()) << std::endl;
  std::cout << "Ground truth potential: " << g.eval(ground_truth) << std::endl;
}
