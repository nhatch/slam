#ifndef SLAM_GRAPHICS_H
#define SLAM_GRAPHICS_H

#include "utils.h"
#include "world.h"
#include <SFML/Graphics.hpp>

void drawObstacles(const obstacles_t &obss);
void drawTraj(const trajectory_t &traj, sf::Color c);
void drawRobot(const transform_t &tf, sf::Color c);
void drawPoints(const points_t &ps, sf::Color c);
void display();
void spin();
char pollWindowEvent();

#endif
