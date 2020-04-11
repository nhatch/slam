
#ifndef GRAPH_H
#define GRAPH_H

#include <Eigen/Core>
#include <vector>

using values = Eigen::Matrix<double, Eigen::Dynamic, 1>;
using hessian = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

class AbstractFactor {
public:
  virtual ~AbstractFactor();
  virtual double eval(const values &/*x*/);
  virtual values gradient_at(const values &/*x*/);
  virtual hessian hessian_at(const values &/*x*/);
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
  void solve(const values &x0, double alpha=0.01, int maxiters=10000, double tol=0.001);
  values x0();
  values solution();
  hessian covariance();
};

#endif
