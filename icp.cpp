
#include "icp.h"
#include <Eigen/SVD>
#include <iostream>

transform_t do_icp(const points_t &cloud1, const points_t &cloud2) {
  transform_t tf = toTransform({0,0,0});
  int iters = 0;
  size_t n = cloud1.size();
  points_t tf_cloud1 = cloud1;
  // TODO use a smarter stopping criterion
  while (iters++ < 10) {
    points_t closest1({}), closest2({});
    std::vector<size_t> idxs1({}), idxs2({});
    idxs1.resize(n);
    idxs2.resize(n);
    for (size_t i = 0; i < n; i++) {
      double min_dist_1 = 100000;
      for (size_t j = 0; j < n; j++) {
        double dist = (tf_cloud1[i] - cloud2[j]).matrix().norm();
        if (dist < min_dist_1) {
          min_dist_1 = dist;
          idxs1[i] = j;
        }
      }
      double min_dist_2 = 100000;
      for (size_t j = 0; j < n; j++) {
        double dist = (tf_cloud1[j] - cloud2[i]).matrix().norm();
        if (dist < min_dist_2) {
          min_dist_2 = dist;
          idxs2[i] = j;
        }
      }
    }
    for (size_t i = 0; i < n; i++) {
      size_t j = idxs1[i];
      if (idxs2[j] == i) {
        // Mutually closest points
        closest1.push_back(tf_cloud1[i]);
        closest2.push_back(cloud2[j]);
      }
    }
    transform_t next_tf = associated_closed_form(closest1, closest2);
    for (size_t i = 0; i < n; i++) {
      tf_cloud1[i] = next_tf * tf_cloud1[i];
    }
    tf = next_tf * tf;
  }
  return tf;
}

transform_t associated_closed_form(const points_t &cloud1, const points_t &cloud2) {
  size_t n = cloud1.size();
  assert(n == cloud2.size());
  point_t m1 = {0,0,0};
  for (point_t p : cloud1) {
    m1 += p / n;
  }
  point_t m2 = {0,0,0};
  for (point_t p : cloud2) {
    m2 += p / n;
  }
  Eigen::Matrix2d w;
  w *= 0;
  for (size_t i = 0; i < n; i++) {
    w += ((cloud1[i] - m1) * (cloud2[i] - m2).transpose()).block(0,0,2,2);
  }
  Eigen::JacobiSVD<Eigen::Matrix2d> svd(w, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix2d r = svd.matrixV() * svd.matrixU().transpose();

  // TODO this transform is not correct (rotation seems right, but translation is wrong)
  transform_t pre_shift = toTransform({0,0,0});
  pre_shift.block(0,2,2,1) = ( - m1).topRows(2);
  transform_t tf_rot = toTransform({0,0,0});
  tf_rot.block(0,0,2,2) = r;
  transform_t post_shift = toTransform({0,0,0});
  post_shift.block(0,2,2,1) = m2.topRows(2);
  return post_shift * tf_rot * pre_shift;
}
