
#include <iostream>
#include <unistd.h>
#include "print_results.h"
#include "graph.h"
#include "slam_utils.h"
#include "graphics.h"

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

void printResults(MyWindow &window, FriendlyGraph &fg, const trajectory_t &true_trajectory, const points_t &true_landmarks) {
  Graph &g = fg._graph;
  int pose_size(1), lm_size(1);
  if (IS_2D) {
    pose_size = 3;
    lm_size = 2;
  }
  int L = true_landmarks.size();

  std::cout.precision(3);
  std::cout << std::fixed;

  std::cout << std::showpos;
  values ground_truth = toVector(true_trajectory, true_landmarks);
  std::cout << std::endl << "Landmark locations:" << std::endl;
  printRange(g, ground_truth, 0, lm_size*L, lm_size);
  std::cout << std::endl << "Trajectory:" << std::endl;
  printRange(g, ground_truth, lm_size*L, ground_truth.size(), pose_size);
  std::cout << std::noshowpos;

  // TODO maybe we should compute error in a more sophisticated way?
  // E.g. we don't really care about absolute landmark location so much as
  // location relative to the robot.
  std::cout << std::endl << "Initial error: " << (ground_truth-g.x0()).norm() << std::endl;
  std::cout << "Smoothed error: " << (ground_truth-g.solution()).norm() << std::endl;
  std::cout << "Initial potential: " << g.eval(g.x0()) << std::endl;
  std::cout << "Smoothed potential: " << g.eval(g.solution()) << std::endl;
  std::cout << "Ground truth potential: " << g.eval(ground_truth) << std::endl;

  points_t smoothed_lms = fg.getLandmarkLocations();
  trajectory_t smoothed_traj = fg.getSmoothedTrajectory();
  window.drawPoints(smoothed_lms, sf::Color::Green, 3);
  window.drawTraj(smoothed_traj, sf::Color::Green);
  window.display();
  int c = 0;
  while (c != -2) {
    while ((c = window.pollWindowEvent()) != -1 && c != -2) {};
    usleep(100 * 1000);
  };
}
