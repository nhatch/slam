
#include "graph.h"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/LU>
#include <vector>

values toVector(const trajectory_t &traj, const points_t &r) {
  int T = (int) traj.size()-1;
  int L = (int) r.size();
  int pose_size(1), lm_size(1);
  if (IS_2D) {
    pose_size = 3;
    lm_size = 2;
  }
  // We don't include a variable for T=0 since we *define* that to be the origin
  int N = pose_size*T+lm_size*L;
  values v = values::Zero(N);
  double prev_theta = 0.;
  for (int i = 0; i < T; i++) {
    transform_t trf = traj[(size_t)i+1];
    pose_t p = toPose(trf, prev_theta);
    v.block(pose_size*i,0,pose_size,1) = p.topRows(pose_size);
    prev_theta = p(2);
  }
  for (int i = 0; i < L; i++) {
    v.block(pose_size*T+lm_size*i, 0, lm_size, 1) = r[(size_t)i].topRows(lm_size);
  }
  return v;
}

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

