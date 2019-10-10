
#ifndef GRAPH_H
#define GRAPH_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <vector>

template <int N, int D>
using jacobian = Eigen::Matrix<double, N, D>;
template <int N>
using values = Eigen::Matrix<double, N, 1>;
template <int D>
using measurement = Eigen::Matrix<double, D, 1>;
template <int D>
using covariance = Eigen::Matrix<double, D, D>;
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

template <int N, int D>
class Factor : public AbstractFactor<N> {
public:
  const covariance<D> _sigma_inv;
  const measurement<D> _measurement;

  Factor(const covariance<D> &sigma_inv, const measurement<D> &measurement) :
      _sigma_inv(sigma_inv), _measurement(measurement) {}

  // To be implemented by subclasses
  virtual measurement<D> f(const values<N>&) { assert("abstract f" && false); }
  virtual jacobian<N,D> jf(const values<N>&) { assert("abstract jf" && false); }

  virtual double eval(const values<N> &x) {
    measurement<D> diff = f(x) - _measurement;
    return 0.5 * (diff.transpose() * _sigma_inv * diff)(0,0);
  }

  virtual values<N> gradient_at(const values<N> &x) {
    return jf(x) * (_sigma_inv * (f(x) - _measurement));
  }

  virtual hessian<N> hessian_at(const values<N> &x) {
    jacobian<N,D> j = jf(x);
    return j * _sigma_inv * j.transpose();
  }
};

template<int N>
class OdomFactor : public Factor<N,1> {
  int _idx1, _idx2;

public:
  OdomFactor(int idx1, int idx2, double sigma, double m) : Factor<N,1>(
      covariance<1> { 1/sigma/sigma }, measurement<1> { m }
    ), _idx1(idx1), _idx2(idx2) { }

  virtual measurement<1> f(const values<N> &x) {
    return measurement<1> { x(_idx2) - x(_idx1) };
  }

  virtual jacobian<N,1> jf(const values<N> &/* x */) {
    jacobian<N,1> j = jacobian<N,1>::Zero();
    j(_idx2) = 1;
    j(_idx1) = -1;
    return j;
  }
};


template<int N>
class GPSFactor : public Factor<N,1> {
  int _idx;

public:
  GPSFactor(int idx, double sigma, double m) : Factor<N,1>(
      covariance<1> { 1/sigma/sigma }, measurement<1> { m }
    ), _idx(idx) { }

  virtual measurement<1> f(const values<N> &x) {
    return measurement<1> { x(_idx) };
  }

  virtual jacobian<N,1> jf(const values<N> &/* x */) {
    jacobian<N,1> j = jacobian<N,1>::Zero();
    j(_idx) = 1;
    return j;
  }
};

template <int N>
class Graph {
private:
  values<N> _x0;
  values<N> _sol;
  std::vector<AbstractFactor<N> *> _factors;

public:
  Graph() : _x0(values<N>::Zero()), _sol(values<N>::Zero()), _factors({}) {}

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
    double error = 1.0;
    int i = 0;
    while (error > tol && i < maxiters) {
      values<N> grad = values<N>::Zero();
      for (auto f : _factors)
        grad += f->gradient_at(x);
      x -= alpha * grad;
      error = sqrt(grad.transpose() * grad);
      i += 1;
    }
    std::cout << "MAP took " << i << " iterations." << std::endl;
    _sol = x;
  }

  values<N> x0() {
    return _x0;
  }

  values<N> solution() {
    return _sol;
  }

  hessian<N> covariance() {
    hessian<N> hess = hessian<N>::Zero();
    for (auto f : _factors)
      hess += f->hessian_at(_sol);
    return hess.inverse();
  }
};

#endif
