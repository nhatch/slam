
#include "graph.h"
#include "factors.h"
#include "world.h"
#include "utils.h"
#include "slam_utils.h"
#include "print_results.h"
#include "constants.h"
#include <unistd.h>
using namespace NavSim;

extern const bool IS_2D { true };

Graph smooth(const values &x0, const traj_points_t &readings) {
  int T = (int) readings.size()-1;
  int nLandmarks = (int) readings[0].size();
  assert("whoohoo" && (x0.size() == T*3 + nLandmarks*2));
  covariance<3> odom_cov = covariance<3>::Zero();
  // TODO what are the right numbers here? Should y be correlated with theta?
  odom_cov << SLAM_VAR, 0, 0,
              0, SLAM_VAR, 0,
              0, 0, SLAM_VAR;
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
      graph.add(new OdomFactor2D(t2, t1, odom_cov_inv, om2-om1));
    }
  }
  graph.solve(x0, 0.0001, 10000, 1.0);
  return graph;
}

void optimizeAndRender(sf::RenderWindow &window, const traj_points_t &landmark_readings, const trajectory_t &odom, const trajectory_t &true_trajectory, const points_t &true_landmarks) {
  values x0 = toVector(odom, landmark_readings[0]);
  Graph g = smooth(x0, landmark_readings);
  printResults(window, g, true_trajectory, true_landmarks);
}

int main() {
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

  sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH_PX, WINDOW_WIDTH_PX), "SLAM visualization");
  display(window);
  _drawTraj(window, odom, sf::Color::Blue);
  _drawPoints(window, landmark_readings[0], sf::Color::Blue, 3);
  _drawTraj(window, ground_truth, sf::Color::Black);
  _drawPoints(window, w.trueLandmarks(), sf::Color::Black, 3);
  display(window);
  _drawTraj(window, odom, sf::Color::Blue);
  _drawPoints(window, landmark_readings[0], sf::Color::Blue, 3);
  _drawTraj(window, ground_truth, sf::Color::Black);
  _drawPoints(window, w.trueLandmarks(), sf::Color::Black, 3);
  optimizeAndRender(window, landmark_readings, odom, ground_truth, w.trueLandmarks());

  return 0;
}
