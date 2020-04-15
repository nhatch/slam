#include <SFML/Graphics.hpp>
#include <Eigen/LU>
#include <unistd.h>
#include <iostream>
#include "world_ui.h"
#include "graphics.h"
#include "constants.h"

using namespace NavSim;

WorldUI::WorldUI(World &w) : world(w), diag_(false), truth_(true) {
  std::cout << "Type 'q' to quit, 'wasd' to move around, 't' to view ground truth, 'r' to start autonomous mode.\n";
}

void WorldUI::start() {
  world.startSimulation();
  render();
  show();
  render();
}

int WorldUI::pollKeyPress()
{
  char c = pollWindowEvent();
  double x_dist = diag_ ? 1.414*PLAN_RESOLUTION : PLAN_RESOLUTION;
  switch(c) {
  case 22:
  case 73:
    world.moveRobot(0.0, x_dist);
    break;
  case 18:
  case 74:
    world.moveRobot(0.0, -x_dist);
    break;
  case 0:
  case 71:
    world.moveRobot(M_PI/4, 0.0);
    diag_ = !diag_;
    break;
  case 3:
  case 72:
    world.moveRobot(-M_PI/4, 0.0);
    diag_ = !diag_;
    break;
  case 19:
    truth_ = !truth_;
    break;
  default:
    break;
  }
  if (c > 0) {
    show();
  }
  return (int) c;
}

void WorldUI::show() {
  display();
  render();
}

void WorldUI::goForwardTSteps(int T) {
  _drawTraj(world.odom_, sf::Color::Blue);
  display();
  for (int i = 1; i < T+1; i++) {
    world.moveRobot(0., ROBOT_LENGTH*1.5);
    render();
    _drawTraj(world.odom_, sf::Color::Blue);
    display();
  }
  render();
}

points_t transformReadings(const points_t &ps, const transform_t &tf) {
  transform_t tf_inv = tf.inverse();
  points_t readings({});
  for (point_t p : ps) {
    readings.push_back(tf_inv * p);
  }
  return readings;
}

void WorldUI::renderReadings(const transform_t &tf) {
  points_t lidar = world.lidar_readings_.back();
  _drawPoints(transformReadings(lidar, tf), sf::Color::Red);
  points_t lms = world.landmark_readings_.back();
  _drawPoints(transformReadings(lms, tf), sf::Color::Blue);
}

void WorldUI::render() {
  truth_ ? renderTruth() : renderRobotView();
}

const transform_t WorldUI::baseTF() {
  return truth_ ? world.ground_truth_.back() :
                  toTransform({WINDOW_CENTER_X,WINDOW_CENTER_Y,M_PI/2});
}

void WorldUI::renderRobotView() {
  transform_t tf = baseTF();
  renderReadings(tf);
  drawRobot(tf, sf::Color::Blue);
}

void WorldUI::renderTruth() {
  drawObstacles(world.obstacles_);
  _drawTraj(world.ground_truth_, sf::Color::Black);
  _drawPoints(world.landmarks_, sf::Color::Black);
  renderReadings(world.ground_truth_.back());
}

void WorldUI::drawTraj(const trajectory_t &traj, bool robot_frame, sf::Color c) {
  if (robot_frame) {
    transform_t base = baseTF();
    trajectory_t tf_traj({});
    for (const transform_t &tf_i : traj) {
      tf_traj.push_back(tf_i * base);
    }
    _drawTraj(tf_traj, c);
  } else {
    _drawTraj(traj, c);
  }
}

void WorldUI::drawPoints(const points_t &pp, bool robot_frame, sf::Color c) {
  transform_t base = robot_frame ? baseTF() : toTransform({0,0,0});
  _drawPoints(transformReadings(pp, base), c);
}
