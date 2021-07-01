
#include "friendly_graph.h"
#include "utils.h"
#include "constants.h"

#include <iostream>
#include <Eigen/Core>
#include <Eigen/LU>
#include <vector>

using namespace NavSim;

constexpr int LM_SIZE = 2;
constexpr int POSE_SIZE = 3;

FriendlyGraph::FriendlyGraph(int num_landmarks) :
    _num_landmarks(num_landmarks), _num_poses(0), _current_guess(LM_SIZE*num_landmarks),
    _odom_cov_inv(), _sensor_cov_inv(), _gps_cov_inv(), _graph()
{
  covariance<3> odom_cov = covariance<3>::Zero();
  // TODO what are the right numbers here? Should y be correlated with theta?
  odom_cov << SLAM_VAR, 0, 0,
              0, SLAM_VAR, SLAM_VAR/2.0,
              0, SLAM_VAR/2.0, SLAM_VAR;
  _odom_cov_inv = odom_cov.inverse();
  covariance<2> sensor_cov = covariance<2>::Zero();
  sensor_cov << SLAM_VAR, 0,
                0, SLAM_VAR;
  _sensor_cov_inv = sensor_cov.inverse();
  // GPS has no heading measurements
  // which we represent using a large covariance
  covariance<3> gps_cov = covariance<3>::Zero();
  gps_cov << 3.0 * 3.0, 0, 0,
             0, 3.0 * 3.0, 0,
             0, 0, 50.0 * 50.0;
  _gps_cov_inv = gps_cov.inverse();
}

void FriendlyGraph::incrementNumPoses() {
    _num_poses++;
    _current_guess.conservativeResize(_num_landmarks * LM_SIZE + _num_poses * POSE_SIZE);
}

int FriendlyGraph::poseIdx(int pose_id) {
  if (pose_id == _num_poses) {
    incrementNumPoses();
  } else if (pose_id > _num_poses) {
    printf("Error: skipped a pose id (given %d, current %d)\n", pose_id, _num_poses);
    throw 1;
  }
  return _num_landmarks * LM_SIZE + pose_id * POSE_SIZE;
}

int FriendlyGraph::landmarkIdx(int lm_id) {
  assert(lm_id < _num_landmarks);
  return lm_id * LM_SIZE;
}

pose_t FriendlyGraph::getPoseEstimate(int pose_id) {
  pose_t p = _current_guess.block(poseIdx(pose_id), 0, POSE_SIZE, 1);
  return p;
}

void FriendlyGraph::addGPSMeasurement(int pose_id, const transform_t &gps_tf) {
  pose_t gps = toPose(gps_tf, 0); // heading doesn't matter
  _graph.add(new OdomFactor2D(poseIdx(pose_id), -1, _gps_cov_inv, gps));
}

void FriendlyGraph::addOdomMeasurement(int pose2_id, int pose1_id,
    const transform_t &pose2_tf, const transform_t &pose1_tf) {
  transform_t rel_tf = pose2_tf * pose1_tf.inverse();
  pose_t diff = toPose(rel_tf, 0.0);
  _graph.add(new OdomFactor2D(poseIdx(pose2_id), poseIdx(pose1_id), _odom_cov_inv, diff));
  pose_t pose1_est = getPoseEstimate(pose1_id);
  transform_t new_pose_tf = rel_tf * toTransform(pose1_est);
  pose_t pose2_est = toPose(new_pose_tf, pose1_est(2));
  _current_guess.block(poseIdx(pose2_id),0,POSE_SIZE,1) = pose2_est;
}

void FriendlyGraph::addLandmarkMeasurement(int pose_id, int lm_id, const point_t &bearing) {
  measurement<2> lm = measurement<2> { bearing(0), bearing(1) };
  _graph.add(new LandmarkFactor2D(landmarkIdx(lm_id), poseIdx(pose_id), _sensor_cov_inv, lm));
}

void FriendlyGraph::addLandmarkPrior(int lm_id, point_t location, double xy_std) {
  covariance<2> prior_cov = covariance<2>::Zero();
  prior_cov << xy_std * xy_std, 0,
               0, xy_std * xy_std;
  covariance<2> prior_cov_inv = prior_cov.inverse();
  measurement<2> lm = measurement<2> { location(0), location(1) };
  _graph.add(new LandmarkFactor2D(landmarkIdx(lm_id), -1, prior_cov_inv, lm));
  _current_guess.block(landmarkIdx(lm_id),0,LM_SIZE,1) = lm;
}

void FriendlyGraph::addPosePrior(int pose_id, const transform_t &pose_tf,
    double xy_std, double th_std) {
  covariance<3> prior_cov = covariance<3>::Zero();
  prior_cov << xy_std * xy_std, 0, 0,
               0, xy_std * xy_std, 0,
               0, 0, th_std * th_std;
  covariance<3> prior_cov_inv = prior_cov.inverse();
  pose_t pose = toPose(pose_tf, 0);
  _graph.add(new OdomFactor2D(poseIdx(pose_id), -1, prior_cov_inv, pose));
  _current_guess.block(poseIdx(pose_id),0,POSE_SIZE,1) = pose;
}

void FriendlyGraph::solve() {
  _graph.solve(_current_guess);
  _current_guess = _graph.solution();
}