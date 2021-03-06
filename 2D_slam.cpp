
#include <Eigen/LU>
#include "graph.h"
#include "factors.h"
#include "utils.h"
#include "slam_utils.h"
#include "constants.h"
using namespace NavSim;

Graph smooth(const values &x0, const traj_points_t &readings) {
  int T = (int) readings.size()-1;
  int nLandmarks = (int) readings[0].size();
  assert("whoohoo" && (x0.size() == T*3 + nLandmarks*2));
  covariance<3> odom_cov = covariance<3>::Zero();
  // TODO what are the right numbers here? Should y be correlated with theta?
  odom_cov << SLAM_VAR, 0, 0,
              0, SLAM_VAR, SLAM_VAR/2.0,
              0, SLAM_VAR/2.0, SLAM_VAR;
  covariance<3> odom_cov_inv = odom_cov.inverse();
  covariance<2> sensor_cov = covariance<2>::Zero();
  sensor_cov << SLAM_VAR, 0,
                0, SLAM_VAR;
  covariance<2> sensor_cov_inv = sensor_cov.inverse();
  Graph graph;
  for (int t=0; t < T+1; t++) {
    for (int i = 0; i < nLandmarks; i++) {
      point_t l = readings[(size_t)t][(size_t)i];
      if (l(2) == 0.0) continue; // Landmark wasn't visible
      measurement<2> lm = measurement<2> { l(0), l(1) };
      graph.add(new LandmarkFactor2D(3*T+2*i, 3*(t-1), sensor_cov_inv, lm));
    }
    if (t>0) {
      int t2 = 3*(t-1);
      int t1 = 3*(t-2);
      measurement<3> om2 = x0.block(t2,0,3,1);
      measurement<3> om1 = measurement<3>::Zero();
      if (t1 >= 0)
        om1 = x0.block(t1,0,3,1);
      pose_t diff = toPose(toTransform(om2) * toTransform(om1).inverse(), 0.0);
      graph.add(new OdomFactor2D(t2, t1, odom_cov_inv, diff));
    }
  }
  graph.solve(x0);
  return graph;
}

int main() {
  collectDataAndRunSLAM(&smooth);
  return 0;
}
