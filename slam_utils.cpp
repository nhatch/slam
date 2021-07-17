
#include <unistd.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include "print_results.h"
#include "slam_utils.h"
#include "utils.h"
#include "graph.h"
#include "friendly_graph.h"
#include "graphics.h"
#include "world.h"
#include "constants.h"

using namespace NavSim;

values toVector(const trajectory_t &traj, const points_t &r, int max_num_poses, double prev_theta) {
  int num_extra_poses = (int) traj.size() - max_num_poses;
  int L = (int) r.size();
  int pose_size(1), lm_size(1);
  if (IS_2D) {
    pose_size = 3;
    lm_size = 2;
  }
  int N = lm_size*L + pose_size*max_num_poses;
  values v = values::Zero(N);
  printf("Vectorizing trajectory and landmarks to dimension %ld\n", v.size());
  for (int i = 0; i < L; i++) {
    v.block(lm_size*i, 0, lm_size, 1) = r[(size_t)i].topRows(lm_size);
  }
  for (int i = 0; i < max_num_poses; i++) {
    transform_t trf = traj[(size_t)(i+num_extra_poses)];
    pose_t p = toPose(trf, prev_theta);
    v.block(lm_size*L + pose_size*i,0,pose_size,1) = p.topRows(pose_size);
    prev_theta = p(2);
  }
  return v;
}

void collectDataAndRunSLAM() {
  constexpr int T = 30;
  points_t prior_landmarks({
      {0,0,1},
      {0,0,1},
      {0,0,1},
      {0,0,1},
      {0,0,1},
      {0,0,1}});
  int L = (int)prior_landmarks.size();

  traj_points_t landmark_readings({});
  trajectory_t ground_truth({});
  trajectory_t odom_traj({});
  trajectory_t gps_traj({});

  transform_t start_pose_guess = toTransform({13,-1,M_PI*2/3});
  transform_t odom_accumulated_guess = start_pose_guess;

  FriendlyGraph fg(L, 10, 0.3, 3.0, 0.05);
  float prior_xy_std = 3.0;
  float prior_th_std = 1.0;
  covariance<3> prior_cov = covariance<3>::Zero();
  prior_cov << prior_xy_std * prior_xy_std, 0, 0,
               0, prior_xy_std * prior_xy_std, 0,
               0, 0, prior_th_std * prior_th_std;
  fg.addPosePrior(0, start_pose_guess, prior_cov); // informed prior
  for (int l = 0; l < L; l++) {
    point_t location({0,0,1});
    fg.addLandmarkPrior(l, location, 20.0); // uninformative prior
  }

  World w;
  w.addDefaultLandmarks();
  w.start();
  transform_t prev_odom = w.readOdom();
  for (int pose_id = 0; pose_id < T+1; pose_id++) {
    points_t lm_reading = w.readLandmarks();
    landmark_readings.push_back(lm_reading);
    for (int lm_id = 0; lm_id < L; lm_id++) {
      point_t lm = lm_reading[(size_t)lm_id];
      if (lm(2) != 0.0) fg.addLandmarkMeasurement(pose_id, lm_id, lm);
    }
    if (pose_id > 0) {
      transform_t odom = w.readOdom();
      fg.addOdomMeasurement(pose_id, pose_id-1, odom, prev_odom);
      odom_accumulated_guess = odom * prev_odom.inverse() * odom_accumulated_guess;
      prev_odom = odom;
    }
    odom_traj.push_back(odom_accumulated_guess);
    transform_t gps = w.readGPS();
    if (gps.norm() != 0.0) {
      gps_traj.push_back(gps);
      fg.addGPSMeasurement(pose_id, gps);
    }

    ground_truth.push_back(w.readTrueTransform());
    if (pose_id == 0) w.setCmdVel(0.0, ROBOT_LENGTH);
    fg.solve();
    // Make this "relatively prime" with 1000 * 1000 to avoid randomness
    // due to context switching. (Less randomness == good for debugging.)
    usleep(498 * 1000);
  }
  w.setCmdVel(0.0, 0.0);

  MyWindow window("SLAM visualization");
  window.display();
  window.drawTraj(gps_traj, sf::Color::Red);
  window.drawTraj(odom_traj, sf::Color::Blue);
  window.drawPoints(landmark_readings[0], sf::Color::Blue, 3);
  window.drawTraj(ground_truth, sf::Color::Black);
  window.drawPoints(w.trueLandmarks(), sf::Color::Black, 3);
  window.display();
  window.drawTraj(gps_traj, sf::Color::Red);
  window.drawTraj(odom_traj, sf::Color::Blue);
  window.drawPoints(landmark_readings[0], sf::Color::Blue, 3);
  window.drawTraj(ground_truth, sf::Color::Black);
  window.drawPoints(w.trueLandmarks(), sf::Color::Black, 3);

  printResults(window, fg, ground_truth, w.trueLandmarks(), odom_traj, prior_landmarks);
}

