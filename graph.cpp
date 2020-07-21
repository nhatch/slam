
#include "graph.h"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/LU>
#include <vector>

AbstractFactor::~AbstractFactor() {}
double AbstractFactor::eval(const values &/*x*/) { assert("abstract eval" && false); }
values AbstractFactor::gradient_at(const values &/*x*/) { assert("abstract grad" && false); }
hessian AbstractFactor::hessian_at(const values &/*x*/) { assert("abstract hess" && false); }


Graph::Graph() : _x0(values::Zero(1)), _sol(values::Zero(1)), _sol_cov(hessian::Zero(1,1)),
    _factors({}) {}

Graph::~Graph() {
  for (auto f : _factors) {
    free(f);
  }
}

void Graph::add(AbstractFactor *f) {
  _factors.push_back(f);
}

double Graph::eval(const values &x) {
  double sum = 0.0;
  for (auto f : _factors)
    sum += f->eval(x);
  return sum;
}

void Graph::solve(const values &x0, double alpha, int maxiters, double tol) {
  _x0 = x0;
  values x = x0;
  double error = 2*tol;
  int i = 0;
  int N = x0.size();
  while (error > tol && i < maxiters) {
    values grad = values::Zero(N);
    hessian hess = hessian::Zero(N,N);
    for (auto f : _factors) {
      grad += f->gradient_at(x);
      hess += f->hessian_at(x);
    }
    x -= alpha * (hess.inverse() * grad);
    error = sqrt(grad.transpose() * grad);
    i += 1;
    if (i%100 == 0)
      std::cout << "Iteration " << i << ": " << error << std::endl;
  }
  std::cout << "MAP took " << i << " iterations." << std::endl;
  _sol = x;

  hessian hess = hessian::Zero(N,N);
  for (auto f : _factors)
    hess += f->hessian_at(_sol);
  _sol_cov = hess.inverse();
}

values Graph::x0() {
  return _x0;
}

values Graph::solution() {
  return _sol;
}

hessian Graph::covariance() {
  return _sol_cov;
}

