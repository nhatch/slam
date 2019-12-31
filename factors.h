
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

template<int N>
class LandmarkFactor2D : public Factor<N,2> {
  int _lmPose, _sensorPose;

public:
  LandmarkFactor2D(int lmPose, int sensorPose, covariance<2> &sigma_inv, measurement<2> m) : Factor<N,2>(
      sigma_inv, m
      ), _lmPose(lmPose), _sensorPose(sensorPose) { }

  virtual measurement<2> f(const values<N> &x) {
    double px(0), py(0), theta(0);
    if (_sensorPose >= 0) {
      px = x(_sensorPose);
      py = x(_sensorPose+1);
      theta = x(_sensorPose+2);
    }
    return measurement<2> {
      (x(_lmPose) - px) * cos(theta)    + (x(_lmPose+1) - py) * sin(theta),
      (x(_lmPose) - px) * (-sin(theta)) + (x(_lmPose+1) - py) * cos(theta)
    };
  }

  virtual jacobian<N,2> jf(const values<N> &x) {
    jacobian<N,2> j = jacobian<N,2>::Zero();
    double px(0), py(0), theta(0);
    if (_sensorPose >= 0) {
      px = x(_sensorPose);
      py = x(_sensorPose+1);
      theta = x(_sensorPose+2);
    }
    j(_lmPose+0,0) = cos(theta);
    j(_lmPose+1,0) = sin(theta);
    j(_lmPose+0,1) = -sin(theta);
    j(_lmPose+1,1) = cos(theta);
    if (_sensorPose >= 0) {
      j(_sensorPose+0,0)   = -cos(theta);
      j(_sensorPose+1,0)   = -sin(theta);
      j(_sensorPose+2,0)   = (x(_lmPose) - px) * (-sin(theta)) + (x(_lmPose+1) - py) * cos(theta);
      j(_sensorPose+0,1)   = sin(theta);
      j(_sensorPose+1,1)   = -cos(theta);
      j(_sensorPose+2,1)   = (x(_lmPose) - px) * (-cos(theta)) + (x(_lmPose+1) - py) * (-sin(theta));
    }
    return j;
  }
};


template<int N>
class OdomFactor2D : public Factor<N,3> {
  int _pose1, _pose2;

public:
  OdomFactor2D(int pose2, int pose1, covariance<3> &sigma_inv, measurement<3> m) : Factor<N,3>(
      sigma_inv, m
      ), _pose1(pose1), _pose2(pose2) { }

  virtual measurement<3> f(const values<N> &x) {
    if (_pose1 >= 0)
      return measurement<3> { x(_pose2) - x(_pose1), x(_pose2+1) -x(_pose1+1), x(_pose2+2)-x(_pose1+2) };
    else
      return measurement<3> { x(_pose2), x(_pose2+1), x(_pose2+2) };
  }

  virtual jacobian<N,3> jf(const values<N> &/* x */) {
    jacobian<N,3> j = jacobian<N,3>::Zero();
    j(_pose2,0)   = 1;
    j(_pose2+1,1) = 1;
    j(_pose2+2,2) = 1;
    if (_pose1 >= 0) {
      j(_pose1,0)   = -1;
      j(_pose1+1,1) = -1;
      j(_pose1+2,2) = -1;
    }
    return j;
  }
};

#endif
