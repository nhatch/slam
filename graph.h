
#ifndef GRAPH_H
#define GRAPH_H

#include <Eigen/Core>
#include <vector>

using values = Eigen::Matrix<double, Eigen::Dynamic, 1>;
using hessian = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

class AbstractFactor {
public:
  virtual ~AbstractFactor();
  virtual double eval(const values &/*x*/) = 0;
  virtual values gradient_at(const values &/*x*/) = 0;
  virtual hessian hessian_at(const values &/*x*/) = 0;
  // Used to handle graph trimming so graph size does not grow arbitrarily.
  // Shifts all factor indices related to robot poses down by `poseSize`. If this
  // moves the index outside the range used for poses, return false.
  // If this returns false, the factor should be removed from the graph.
  virtual bool shiftIndices(int poseSize, int firstPoseIdx) = 0;
};

class Graph {
private:
  values _x0;
  values _sol;
  hessian _sol_cov;
  std::vector<AbstractFactor *> _factors;

public:
  Graph();

  ~Graph();

  void add(AbstractFactor *f);
  double eval(const values &x);
  void solve(const values &x0, double alpha=1.0, int maxiters=1000, double tol=1e-8);
  values x0();
  values solution();
  hessian covariance();
  void shiftIndices(int poseSize, int firstPoseIdx);
};

#endif
