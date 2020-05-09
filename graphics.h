#ifndef SLAM_GRAPHICS_H
#define SLAM_GRAPHICS_H

#include "utils.h"
#include "world.h"
#include <SFML/Graphics.hpp>

void drawObstacles(sf::RenderWindow &window, const obstacles_t &obss);
// Underscore to differentiate from WorldUI::drawTraj. TODO clean this up
void _drawTraj(sf::RenderWindow &window, const trajectory_t &traj, sf::Color c);
void drawRobot(sf::RenderWindow &window, const transform_t &tf, sf::Color c);
void _drawPoints(sf::RenderWindow &window, const points_t &ps, sf::Color c, int radius_px);
void display(sf::RenderWindow &window);
int pollWindowEvent(sf::RenderWindow &window);

#endif
