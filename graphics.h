#ifndef SLAM_GRAPHICS_H
#define SLAM_GRAPHICS_H

#include "utils.h"
#include <SFML/Graphics.hpp>

class MyWindow {
public:
  MyWindow(const char *name);
  void drawObstacles(const obstacles_t &obss);
  void drawTraj(const trajectory_t &traj, sf::Color c);
  void drawRobot(const transform_t &tf, sf::Color c);
  void drawPoints(const points_t &ps, sf::Color c, int radius_px);
  void display();
  void setOrigin(const pose_t &o);
  int pollWindowEvent();

private:
  sf::RenderWindow sf_window_;
  double window_width_;
  double window_center_x_;
  double window_center_y_;

  sf::Vector2f toWindowFrame(const point_t &p);
  sf::Vector2f toWindowFrame(const point_t &p, double vertOffset);
  void drawLine(const point_t &l1, const point_t &l2, sf::Color c);
  double vertOffset(sf::Color c);
};

#endif
