#ifndef FRIENDLY_GRAPH_H__
#define FRIENDLY_GRAPH_H__

#include "graph.h"
#include "factors.h"
#include "utils.h"

class FriendlyGraph {
private:
  int _num_landmarks;
  int _max_pose_id;
  int _min_pose_id;
  int _max_num_poses;
  values _current_guess;

  covariance<3> _odom_cov_inv;
  covariance<2> _sensor_cov_inv;
  covariance<3> _gps_cov_inv;

  int nonincrementingPoseIdx(int pose_id);
  int poseIdx(int pose_id);
  int landmarkIdx(int lm_id);
  int numPoses();
  void incrementNumPoses();
  void trimToMaxNumPoses();

public:
  Graph _graph;

  /* num_landmarks: Currently we require you to pre-specify how many landmarks
   *                will be considered in the pose graph. TODO: Make it possible to
   *                add more landmarks over time.
   *
   * max_num_poses: To prevent the pose graph from growing arbitrarily over time,
   *                we automatically trim the oldest poses once we get enough newer ones.
   *                This parameter specifies the maximum number of poses in the graph.
   *
   * camera_std:    Standard deviation of landmark distance measurements, in meters
   *
   * gps_xy_std:    Standard deviation of GPS (x,y) measurements, in meters
   *
   * wheel_noise_rate:
   *      The variance of odom measurements will depend on how far the robot traveled.
   *      A WHEEL_NOISE_RATE of 0.05 means that when we rotate a wheel through a distance
   *      of 1 meter, the standard deviation of the true distance rotated will be 5 cm.
   */
  FriendlyGraph(int num_landmarks, int max_num_poses,
      float camera_std, float gps_xy_std, float wheel_noise_rate);

  void addGPSMeasurement(int pose_id, const transform_t &gps_tf);
  void addOdomMeasurement(int pose2_id, int pose1_id,
    const transform_t &pose2_tf, const transform_t &pose1_tf);
  void addLandmarkMeasurement(int pose_id, int lm_id, const point_t &bearing);
  void addLandmarkPrior(int lm_id, point_t location, double xy_std);
  void addPosePrior(int pose_id, const transform_t &pose_tf, covariance<3> &cov);

  pose_t getPoseEstimate(int pose_id);
  void solve();
  points_t getLandmarkLocations();
  /* This method will return the smoothed trajectory (assuming you've already called
   * `solve()`). Note that the length of this trajectory will be at most `max_num_poses`
   * (the oldest poses are discarded). */
  trajectory_t getSmoothedTrajectory();
  int getMaxNumPoses() const;

};

#endif
