
#include <iostream>
#include "graph.h"
#include "world_ui.h"
#include "slam_utils.h"

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

void printRange(Graph &g, values ground_truth, int start, int end, int size) {
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

points_t toLandmarks(const values &x, int start, int lm_size) {
  int N = x.size();
  points_t lms({});
  for (int i = start; i < N; i += lm_size) {
    point_t lm ({0, 0, 1});
    lm.topRows(lm_size) = x.block(i, 0, lm_size, 1);
    lms.push_back(lm);
  }
  return lms;
}

trajectory_t toTraj(const values &x, int pose_size, int T) {
  trajectory_t tfs({});
  tfs.push_back(toTransformRotateFirst(0, 0, 0));
  for (int i = 0; i < T*pose_size; i += pose_size) {
    if (pose_size == 3) { // 2D
      tfs.push_back(toTransformRotateFirst(0, 0, x(i+2)) * toTransformRotateFirst(x(i), x(i+1), 0));
    } else { // 1D
      tfs.push_back(toTransformRotateFirst(x(i), 0, 0));
    }
  }
  return tfs;
}

void printResults(WorldUI &ui, Graph &g) {
  World &w = ui.world;
  int pose_size(1), lm_size(1);
  if (IS_2D) {
    pose_size = 3;
    lm_size = 2;
  }
  int T = w.trueTrajectory().size() - 1;

  std::cout.precision(3);
  std::cout << std::fixed;

  std::cout << std::showpos;
  values ground_truth = toVector(w.trueTrajectory(), w.trueLandmarks());
  std::cout << std::endl << "Trajectory:" << std::endl;
  printRange(g, ground_truth, 0, pose_size*T, pose_size);
  std::cout << std::endl << "Landmark locations:" << std::endl;
  printRange(g, ground_truth, pose_size*T, ground_truth.size(), lm_size);
  std::cout << std::noshowpos;

  std::cout << std::endl << "Odom error: " << (ground_truth-g.x0()).norm() << std::endl;
  std::cout << "Smoothed error: " << (ground_truth-g.solution()).norm() << std::endl;
  std::cout << "Odom potential: " << g.eval(g.x0()) << std::endl;
  std::cout << "Smoothed potential: " << g.eval(g.solution()) << std::endl;
  std::cout << "Ground truth potential: " << g.eval(ground_truth) << std::endl;

  trajectory_t smoothed_traj = toTraj(g.solution(), pose_size, T);
  points_t smoothed_lms = toLandmarks(g.solution(), T*pose_size, lm_size);
  ui.drawPoints(smoothed_lms, false, sf::Color::Green);
  ui.drawTraj(smoothed_traj, false, sf::Color::Green);
}
