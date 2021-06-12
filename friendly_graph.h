#ifndef FRIENDLY_GRAPH_H__
#define FRIENDLY_GRAPH_H__

#include "graph.h"
#include "factors.h"
#include "utils.h"

class FriendlyGraph {
private:
  int _num_landmarks;
  int _num_poses;

  covariance<3> _odom_cov_inv;
  covariance<2> _sensor_cov_inv;
  covariance<3> _gps_cov_inv;

  int poseIdx(int pose_id);
  int landmarkIdx(int lm_id);
  void incrementNumPoses();

public:
  Graph _graph;
  values _current_guess;

  FriendlyGraph(int num_landmarks);

  void addGPSMeasurement(int pose_id, const transform_t &gps_tf);
  void addOdomMeasurement(int pose2_id, int pose1_id,
    const transform_t &pose2_tf, const transform_t &pose1_tf);
  void addLandmarkMeasurement(int pose_id, int lm_id, const point_t &bearing);
  void addLandmarkPrior(int lm_id, point_t location, double xy_std);
  void addPosePrior(int pose_id, const transform_t &pose_tf,
    double xy_std, double th_std);

  void solve();
};

#endif
