#ifndef SLAM_GRAPHICS_H
#define SLAM_GRAPHICS_H

#include "utils.h"
#include "world.h"
#include <SFML/Graphics.hpp>

void drawObstacles(const obstacles_t &obss);
// Underscore to differentiate from WorldUI::drawTraj. TODO clean this up
void _drawTraj(const trajectory_t &traj, sf::Color c);
void drawRobot(const transform_t &tf, sf::Color c);
void _drawPoints(const points_t &ps, sf::Color c, int radius_px);
void display();
char pollWindowEvent();

#endif
