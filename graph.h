
#ifndef GRAPH_H
#define GRAPH_H

#include <iostream>
#include <Eigen/Core>
#include <Eigen/LU>
#include <vector>

template <int N>
using values = Eigen::Matrix<double, N, 1>;

template <int N>
using hessian = Eigen::Matrix<double, N, N>;

template <int N>
class AbstractFactor {
public:
  virtual ~AbstractFactor() {};
  virtual double eval(const values<N> &/*x*/) { assert("abstract eval" && false); }
  virtual values<N> gradient_at(const values<N> &/*x*/) { assert("abstract grad" && false); }
  virtual hessian<N> hessian_at(const values<N> &/*x*/) { assert("abstract hess" && false); }
};

template <int N>
class Graph {
private:
  values<N> _x0;
  values<N> _sol;
  hessian<N> _sol_cov;
  std::vector<AbstractFactor<N> *> _factors;

public:
  Graph() : _x0(values<N>::Zero()), _sol(values<N>::Zero()), _sol_cov(hessian<N>::Zero()),
            _factors({}) {}

  ~Graph() {
    for (auto f : _factors) {
      free(f);
    }
  }

  void add(AbstractFactor<N> *f) {
    _factors.push_back(f);
  }

  double eval(const values<N> &x) {
    double sum = 0.0;
    for (auto f : _factors)
      sum += f->eval(x);
    return sum;
  }

  void solve(const values<N> &x0, double alpha=0.01, int maxiters=10000, double tol=0.001) {
    _x0 = x0;
    values<N> x = x0;
    double error = 2*tol;
    int i = 0;
    while (error > tol && i < maxiters) {
      values<N> grad = values<N>::Zero();
      for (auto f : _factors)
        grad += f->gradient_at(x);
      x -= alpha * grad;
      error = sqrt(grad.transpose() * grad);
      i += 1;
      if (i%100 == 0)
        std::cout << "Iteration " << i << ": " << error << std::endl;
    }
    std::cout << "MAP took " << i << " iterations." << std::endl;
    _sol = x;

    hessian<N> hess = hessian<N>::Zero();
    for (auto f : _factors)
      hess += f->hessian_at(_sol);
    _sol_cov = hess.inverse();
  }

  values<N> x0() {
    return _x0;
  }

  values<N> solution() {
    return _sol;
  }

  hessian<N> covariance() {
    return _sol_cov;
  }
};

#endif
