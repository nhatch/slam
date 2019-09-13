
#ifndef GRAPH_H
#define GRAPH_H

#include <eigen3/Eigen/Core>
#include <vector>

constexpr int NULL_VAR = -1;

// This is super inefficient so far; it explicitly represents the full Hessian etc.
using vec_t = Eigen::Matrix<double, Eigen::Dynamic, 1>;
using hess_t = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

// -log of an actual factor
class Factor {
public:
  int _v1;
  int _v2;
  double _sigma;
  double _measurement;

  vec_t gradient(vec_t x, int N);
  hess_t hessian(vec_t /* x */, int N);
  double eval(vec_t x);
};

using Graph = std::vector<Factor>;

// Just uses gradient descent
vec_t findMAP(Graph graph, vec_t x0, int N,
              double alpha = 0.01, int maxiters = 10000, double tol = 0.001);

hess_t findCov(Graph graph, vec_t x, int N);

double eval(Graph graph, vec_t x);

#endif
