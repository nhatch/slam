#include <SFML/Graphics.hpp>
#include <Eigen/LU>
#include <unistd.h>
#include <iostream>
#include "graphics.h"
#include "constants.h"

using namespace NavSim;

const double scale_x = double(WINDOW_WIDTH_PX) / WINDOW_WIDTH;
const double scale_y = -double(WINDOW_WIDTH_PX) / WINDOW_WIDTH;
const double offset_x = double(WINDOW_WIDTH_PX) / 2;
const double offset_y = double(WINDOW_WIDTH_PX) / 2;

sf::Vector2f toWindowFrame(const point_t &p) {
  return sf::Vector2f((p(0)-WINDOW_CENTER_X)*scale_x + offset_x, (p(1)-WINDOW_CENTER_Y)*scale_y + offset_y);
}

// Useful for 1D visualizations, which are too cluttered with everything on top of each other
sf::Vector2f toWindowFrame(const point_t &p, double vertOffset) {
  point_t p_shifted { p };
  p_shifted(1) += vertOffset;
  return toWindowFrame(p_shifted);
}

char pollWindowEvent(sf::RenderWindow &window) {
  sf::Event event;
  if (window.pollEvent(event))
  {
    switch(event.type) {
      case sf::Event::Closed:
        window.close();
        return -2;
      case sf::Event::KeyPressed:
        return (event.key.code == 16) ? -2 : event.key.code; // 16 is key press 'q' for quit
      case sf::Event::KeyReleased:
        return -3;
      default:
        return -4; // unhandled event type
    }
  }
  return -1; // no events in queue
}

void display(sf::RenderWindow &window) {
  // NB: window.display() actually just flips the double buffers (it's not idempotent!)
  window.display();
  window.clear(sf::Color::White);
}

double vertOffset(sf::Color c) {
  if (c.g > 0 && c.r == 0 && c.b == 0)
    return 0.0;
  if (!IS_2D && c.r == c.b && c.r == c.g)
    return -2*ROBOT_WHEEL_BASE;
  if (!IS_2D)
    return +2*ROBOT_WHEEL_BASE;
  return 0.0;
}

void drawLine(sf::RenderWindow &window, const point_t &l1, const point_t &l2, sf::Color c) {
  sf::Vertex line[2];
  line[0] = sf::Vertex(toWindowFrame(l2, vertOffset(c)), c);
  line[1] = sf::Vertex(toWindowFrame(l1, vertOffset(c)), c);
  window.draw(line, 2, sf::Lines);
}

void drawRobot(sf::RenderWindow &window, const transform_t &tf, sf::Color c) {
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
  window.draw(shape);
}

void _drawPoints(sf::RenderWindow &window, const points_t &ps, sf::Color c, int radius_px) {
  for (point_t p : ps) {
    if (p(2) == 0) continue;
    sf::CircleShape circle(radius_px);
    circle.setFillColor(c);
    sf::Vector2f pos = toWindowFrame(p, vertOffset(c));
    pos.x -= radius_px;
    pos.y -= radius_px;
    circle.setPosition(pos);
    window.draw(circle);
  }
}

void drawObstacles(sf::RenderWindow &window, const obstacles_t &obss) {
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
    window.draw(shape);
  }
}

void _drawTraj(sf::RenderWindow &window, const trajectory_t &traj, sf::Color c) {
  for (size_t t = 0; t < traj.size(); t++) {
    if (t>0) {
      drawLine(window, toPose(traj[t-1], 0), toPose(traj[t], 0), c);
    }
    drawRobot(window, traj[t], c);
  }
}
