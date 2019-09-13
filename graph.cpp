
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <cmath>
#include <cassert>
#include "graph.h"

vec_t Factor::gradient(vec_t x, int N) {
  vec_t g = vec_t::Zero(N);
  if (_v1 != NULL_VAR) {
    g(_v2) = 1/_sigma/_sigma * (x(_v2) - x(_v1) - _measurement);
    g(_v1) = -g(_v2);
  } else {
    g(_v2) = 1/_sigma/_sigma * (x(_v2) - _measurement);
  }
  return g;
}

hess_t Factor::hessian(vec_t /* x */, int N) {
  hess_t h = hess_t::Zero(N,N);
  h(_v2,_v2) = 1/_sigma/_sigma;
  if (_v1 != NULL_VAR) {
    h(_v1,_v1) = 1/_sigma/_sigma;
    h(_v1,_v2) = -1/_sigma/_sigma;
    h(_v2,_v1) = -1/_sigma/_sigma;
  }
  return h;
}

double Factor::eval(vec_t x) {
  double diff = x(_v2) - _measurement;
  if (_v1 != NULL_VAR)
    diff -= x(_v1);
  return 0.5/_sigma/_sigma*diff*diff;
}

// Just uses gradient descent
vec_t findMAP(Graph graph, vec_t x0, int N,
              double alpha, int maxiters, double tol) {
  assert("Dimension mismatch" && (x0.rows() == N));
  double error = 1.0;
  int i = 0;
  while (error > tol && i < maxiters) {
    vec_t grad = vec_t::Zero(N);
    for (Factor f : graph)
      grad += f.gradient(x0, N);
    x0 -= alpha * grad;
    error = sqrt(grad.transpose() * grad);
    i += 1;
  }
  std::cout << "MAP took " << i << " iterations." << std::endl;
  return x0;
}

hess_t findCov(Graph graph, vec_t x, int N) {
  assert("Dimension mismatch" && (x.rows() == N));
  hess_t hess = hess_t::Zero(N, N);
  for (Factor f : graph)
    hess += f.hessian(x, N);
  return hess.inverse();
}

double eval(Graph graph, vec_t x) {
  double sum = 0.0;
  for (Factor f : graph)
    sum += f.eval(x);
  return sum;
}
