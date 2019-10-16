
#ifndef FACTORS_H
#define FACTORS_H

#include "graph.h"

template <int N, int D>
using jacobian = Eigen::Matrix<double, N, D>;

template <int D>
using measurement = Eigen::Matrix<double, D, 1>;

template <int D>
using covariance = Eigen::Matrix<double, D, D>;

template <int N, int D>
class Factor : public AbstractFactor<N> {
public:
  const covariance<D> _sigma_inv;
  const measurement<D> _measurement;

  Factor(const covariance<D> &sigma_inv, const measurement<D> &measurement) :
      _sigma_inv(sigma_inv), _measurement(measurement) {}

  // To be implemented by subclasses
  virtual measurement<D> f(const values<N> &/*x*/) { assert("abstract f" && false); }
  virtual jacobian<N,D> jf(const values<N> &/*x*/) { assert("abstract jf" && false); }

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

#endif
