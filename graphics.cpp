#include <SFML/Graphics.hpp>
#include <Eigen/LU>
#include <unistd.h>
#include "graphics.h"

const int WINDOW_SIDE = 500;
const double BALL_RADIUS = 0.08;
sf::RenderWindow window(sf::VideoMode(WINDOW_SIDE, WINDOW_SIDE), "RRT visualization");
const double MAX_X(8), MIN_X(-2), MAX_Y(5), MIN_Y(-5);
const double ORIGIN_X(3), ORIGIN_Y(0);
const double scale_x = double(WINDOW_SIDE) / (MAX_X - MIN_X);
const double scale_y = double(WINDOW_SIDE) / -(MAX_Y - MIN_Y);
const double offset_x = double(WINDOW_SIDE) / 2;
const double offset_y = double(WINDOW_SIDE) / 2;

sf::Vector2f toWindowFrame(const landmark_t &lm) {
  return sf::Vector2f((lm(0)-ORIGIN_X)*scale_x + offset_x, (lm(1)-ORIGIN_Y)*scale_y + offset_y);
}

// Useful for 1D visualizations, which are too cluttered with everything on top of each other
sf::Vector2f toWindowFrame(const landmark_t &lm, double vertOffset) {
  landmark_t lm_shifted { lm };
  lm_shifted(1) += vertOffset;
  return toWindowFrame(lm_shifted);
}

void checkClosed() {
  if (!window.isOpen())
    exit(0);

  sf::Event event;
  while (window.pollEvent(event))
  {
    if (event.type == sf::Event::Closed)
        window.close();
  }
}

void clear() {
  checkClosed();
  window.clear(sf::Color::White);
}

void spin() {
  while (true) {
    checkClosed();
    usleep(10*1000);
  }
}

double vertOffset(sf::Color c) {
  if (!IS_2D && c == sf::Color::Blue)
    return -0.3;
  if (!IS_2D && c == sf::Color::Black)
    return +0.3;
  return 0.0;
}

void drawLine(const landmark_t &l1, const landmark_t &l2, sf::Color c) {
  sf::Vertex line[2];
  line[0] = sf::Vertex(toWindowFrame(l2, vertOffset(c)), c);
  line[1] = sf::Vertex(toWindowFrame(l1, vertOffset(c)), c);
  window.draw(line, 2, sf::Lines);
}

void drawRobot(const transform_t &tf, sf::Color c) {
  transform_t tf_inv = tf.inverse();
  landmark_t tip =        tf_inv * landmark_t( 0.2,  0.0, 1);
  landmark_t left_back =  tf_inv * landmark_t(-0.1,  0.1, 1);
  landmark_t right_back = tf_inv * landmark_t(-0.1, -0.1, 1);
  sf::ConvexShape shape;
  shape.setPointCount(3);
  shape.setFillColor(c);
  shape.setPoint(0, toWindowFrame(tip, vertOffset(c)));
  shape.setPoint(1, toWindowFrame(left_back, vertOffset(c)));
  shape.setPoint(2, toWindowFrame(right_back, vertOffset(c)));
  window.draw(shape);
}

void drawTraj(const landmarks_t &lms, const trajectory_t &traj, sf::Color c) {
  for (size_t t = 0; t < traj.size(); t++) {
    if (t>0) {
      drawLine(toPose(traj[t-1], 0), toPose(traj[t], 0), c);
    }
    drawRobot(traj[t], c);
  }
  for (size_t lm = 0; lm < lms.size(); lm++) {
    double pixelRadius = BALL_RADIUS * scale_x;
    sf::CircleShape circle(pixelRadius);
    circle.setFillColor(c);
    sf::Vector2f pos = toWindowFrame(lms[lm], vertOffset(c));
    pos.x -= pixelRadius;
    pos.y -= pixelRadius;
    circle.setPosition(pos);
    window.draw(circle);
  }
}

void draw(const landmarks_t &lms_gt, const trajectory_t &traj_gt,
          const landmarks_t &lms_odom, const trajectory_t &traj_odom) {
  clear();
  drawTraj(lms_gt, traj_gt, sf::Color::Black);
  drawTraj(lms_odom, traj_odom, sf::Color::Blue);
  window.display();
  usleep(300*1000);
}

void drawSmoothed(const landmarks_t &lms, const trajectory_t &traj) {
  drawTraj(lms, traj, sf::Color::Green);
  window.display();
}