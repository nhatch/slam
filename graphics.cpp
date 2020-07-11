#include <SFML/Graphics.hpp>
#include <Eigen/LU>
#include <unistd.h>
#include <iostream>
#include "graphics.h"
#include "constants.h"

using namespace NavSim;

void MyWindow::setOrigin(const pose_t &o) {
  window_center_x_ = o(0);
  window_center_y_ = o(1);
}

sf::Vector2f MyWindow::toWindowFrame(const point_t &p) {
  double scale_x = double(DEFAULT_WINDOW_WIDTH_PX) / window_width_;
  double scale_y = -double(DEFAULT_WINDOW_WIDTH_PX) / window_width_;
  double offset_x = double(DEFAULT_WINDOW_WIDTH_PX) / 2;
  double offset_y = double(DEFAULT_WINDOW_WIDTH_PX) / 2;
  return sf::Vector2f((p(0)-window_center_x_)*scale_x + offset_x, (p(1)-window_center_y_)*scale_y + offset_y);
}

MyWindow::MyWindow(const char *name) :
  sf_window_(sf::VideoMode(DEFAULT_WINDOW_WIDTH_PX, DEFAULT_WINDOW_WIDTH_PX), name),
  window_width_(DEFAULT_WINDOW_WIDTH),
  window_center_x_(DEFAULT_WINDOW_CENTER_X),
  window_center_y_(DEFAULT_WINDOW_CENTER_Y)
{
}

// Useful for 1D visualizations, which are too cluttered with everything on top of each other
sf::Vector2f MyWindow::toWindowFrame(const point_t &p, double vertOffset) {
  point_t p_shifted { p };
  p_shifted(1) += vertOffset;
  return toWindowFrame(p_shifted);
}

int MyWindow::pollWindowEvent() {
  sf::Event event;
  int code;
  if (sf_window_.pollEvent(event))
  {
    switch(event.type) {
      case sf::Event::Closed:
        sf_window_.close();
        return -2;
      case sf::Event::KeyPressed:
        code = (event.key.code == 16) ? -2 : event.key.code; // 16 is key press 'q' for quit
        if (code == -2) sf_window_.close();
        return code;
      case sf::Event::KeyReleased:
        return -3;
      case sf::Event::MouseWheelMoved:
        window_width_ *= exp(-0.25*event.mouseWheel.delta);
        if (window_width_ < 2.0) window_width_ = 2.0;
        if (window_width_ > 1000.0) window_width_ = 1000.0;
        return -4;
      default:
        return -4; // unhandled event type
    }
  }
  return -1; // no events in queue
}

void MyWindow::display() {
  // NB: sf_window_.display() actually just flips the double buffers (it's not idempotent!)
  sf_window_.display();
  sf_window_.clear(sf::Color::White);
}

double MyWindow::vertOffset(sf::Color c) {
  if (c.g > 0 && c.r == 0 && c.b == 0)
    return 0.0;
  if (!IS_2D && c.r == c.b && c.r == c.g)
    return -2*ROBOT_WHEEL_BASE;
  if (!IS_2D)
    return +2*ROBOT_WHEEL_BASE;
  return 0.0;
}

void MyWindow::drawLine(const point_t &l1, const point_t &l2, sf::Color c) {
  sf::Vertex line[2];
  line[0] = sf::Vertex(toWindowFrame(l2, vertOffset(c)), c);
  line[1] = sf::Vertex(toWindowFrame(l1, vertOffset(c)), c);
  sf_window_.draw(line, 2, sf::Lines);
}

void MyWindow::drawRobot(const transform_t &tf, sf::Color c) {
  transform_t tf_inv = tf.inverse();
  point_t tip =        tf_inv * point_t( ROBOT_LENGTH/2,  0.0, 1);
  point_t left_back =  tf_inv * point_t(-ROBOT_LENGTH/2,  ROBOT_WHEEL_BASE/2, 1);
  point_t right_back = tf_inv * point_t(-ROBOT_LENGTH/2, -ROBOT_WHEEL_BASE/2, 1);
  sf::ConvexShape shape;
  shape.setPointCount(3);
  shape.setFillColor(c);
  shape.setPoint(0, toWindowFrame(tip, vertOffset(c)));
  shape.setPoint(1, toWindowFrame(left_back, vertOffset(c)));
  shape.setPoint(2, toWindowFrame(right_back, vertOffset(c)));
  sf_window_.draw(shape);
}

void MyWindow::drawPoints(const points_t &ps, sf::Color c, int radius_px) {
  for (point_t p : ps) {
    if (p(2) == 0) continue;
    sf::CircleShape circle(radius_px);
    circle.setFillColor(c);
    sf::Vector2f pos = toWindowFrame(p, vertOffset(c));
    pos.x -= radius_px;
    pos.y -= radius_px;
    circle.setPosition(pos);
    sf_window_.draw(circle);
  }
}

void MyWindow::drawObstacles(const obstacles_t &obss) {
  for (obstacle_t obs : obss) {
    sf::ConvexShape shape;
    int n = obs.rows();
    shape.setPointCount((size_t)n);
    shape.setFillColor(sf::Color(128, 128, 128, 128));
    for (int i = 0; i < n; i++) {
      point_t l;
      l << obs(i,0), obs(i,1), 1;
      shape.setPoint((size_t)i, toWindowFrame(l));
    }
    sf_window_.draw(shape);
  }
}

void MyWindow::drawTraj(const trajectory_t &traj, sf::Color c) {
  for (size_t t = 0; t < traj.size(); t++) {
    if (t>0) {
      drawLine(toPose(traj[t-1], 0), toPose(traj[t], 0), c);
    }
    drawRobot(traj[t], c);
  }
}
