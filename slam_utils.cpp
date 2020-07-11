
#include <unistd.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include "print_results.h"
#include "slam_utils.h"
#include "utils.h"
#include "graph.h"
#include "graphics.h"
#include "world.h"
#include "constants.h"

using namespace NavSim;

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

void collectDataAndRunSLAM(Graph (*smooth)(const values &x0, const traj_points_t &readings)) {
  constexpr int T = 10;

  traj_points_t landmark_readings({});
  trajectory_t ground_truth({});
  trajectory_t odom({});

  World w;
  w.addDefaultLandmarks();
  w.start();
  for (int i = 0; i < T+1; i++) {
    landmark_readings.push_back(w.readLandmarks());
    odom.push_back(w.readOdom());
    ground_truth.push_back(w.readTrueTransform());
    if (i == 0) w.setCmdVel(0.0, ROBOT_LENGTH*8);
    usleep(200 * 1000);
  }
  w.setCmdVel(0.0, 0.0);

  MyWindow window("SLAM visualization");
  window.display();
  window.drawTraj(odom, sf::Color::Blue);
  window.drawPoints(landmark_readings[0], sf::Color::Blue, 3);
  window.drawTraj(ground_truth, sf::Color::Black);
  window.drawPoints(w.trueLandmarks(), sf::Color::Black, 3);
  window.display();
  window.drawTraj(odom, sf::Color::Blue);
  window.drawPoints(landmark_readings[0], sf::Color::Blue, 3);
  window.drawTraj(ground_truth, sf::Color::Black);
  window.drawPoints(w.trueLandmarks(), sf::Color::Black, 3);

  values x0 = toVector(odom, landmark_readings[0]);
  Graph g = smooth(x0, landmark_readings);
  printResults(window, g, ground_truth, w.trueLandmarks());
}

