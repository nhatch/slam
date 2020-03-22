#ifndef SLAM_GRAPHICS_H
#define SLAM_GRAPHICS_H

#include "utils.h"
#include <SFML/Graphics.hpp>

void drawTraj(const landmarks_t &lms, const trajectory_t &traj, sf::Color c);
void drawGoal(const landmark_t &goal);
void display();
void spin();

#endif
