
#include "slam_utils.h"
#include "utils.h"
#include "graph.h"

values toVector(const trajectory_t &traj, const points_t &r) {
  int T = (int) traj.size()-1;
  int L = (int) r.size();
  int pose_size(1), lm_size(1);
  if (IS_2D) {
    pose_size = 3;
    lm_size = 2;
  }
  // We don't include a variable for T=0 since we *define* that to be the origin
  int N = pose_size*T+lm_size*L;
  values v = values::Zero(N);
  double prev_theta = 0.;
  for (int i = 0; i < T; i++) {
    transform_t trf = traj[(size_t)i+1];
    pose_t p = toPose(trf, prev_theta);
    v.block(pose_size*i,0,pose_size,1) = p.topRows(pose_size);
    prev_theta = p(2);
  }
  for (int i = 0; i < L; i++) {
    v.block(pose_size*T+lm_size*i, 0, lm_size, 1) = r[(size_t)i].topRows(lm_size);
  }
  return v;
}
