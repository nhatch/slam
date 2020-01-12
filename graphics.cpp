#include <SFML/Graphics.hpp>
#include <Eigen/LU>
#include <unistd.h>
#include "graphics.h"

const int WINDOW_SIDE = 500;
sf::RenderWindow window(sf::VideoMode(WINDOW_SIDE, WINDOW_SIDE), "RRT visualization");
const double MAX_X(8), MIN_X(-2), MAX_Y(5), MIN_Y(-5);
const double ORIGIN_X(3), ORIGIN_Y(0);
const double scale_x = double(WINDOW_SIDE) / (MAX_X - MIN_X);
const double scale_y = double(WINDOW_SIDE) / -(MAX_Y - MIN_Y);
const double offset_x = double(WINDOW_SIDE) / 2;
const double offset_y = double(WINDOW_SIDE) / 2;

sf::Vector2f toWindowFrame(landmark_t lm) {
  return sf::Vector2f((lm(0)-ORIGIN_X)*scale_x + offset_x, (lm(1)-ORIGIN_Y)*scale_y + offset_y);
}

void clear() {
  if (!window.isOpen())
    exit(0);

  sf::Event event;
  while (window.pollEvent(event))
  {
    if (event.type == sf::Event::Closed)
        window.close();
  }

  window.clear(sf::Color::White);
}

void drawRobot(const pose_t &pose) {
  clear();

  double x(pose(0)), y(pose(1)), theta(pose(2));
  transform_t tf = toTransform(x, y, theta).inverse();
  landmark_t tip =        tf * landmark_t( 0.2,  0.0, 1);
  landmark_t left_back =  tf * landmark_t(-0.1,  0.1, 1);
  landmark_t right_back = tf * landmark_t(-0.1, -0.1, 1);
  sf::ConvexShape shape;
  shape.setPointCount(3);
  shape.setFillColor(sf::Color::Black);
  shape.setPoint(0, toWindowFrame(tip));
  shape.setPoint(1, toWindowFrame(left_back));
  shape.setPoint(2, toWindowFrame(right_back));
  window.draw(shape);

  window.display();
  usleep(300*1000);
}
