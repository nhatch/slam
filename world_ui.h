#ifndef NAVSIM_UI_H
#define NAVSIM_UI_H

#include "utils.h"
#include "world.h"
#include <SFML/Graphics.hpp>

class WorldUI {
public:
  /* Note: as a side effect, this constructor calls w.startSimulation() */
  WorldUI(World &w);

  void goForwardTSteps(int T);

  /* Returns event code. As a side effect, this function may call world.moveRobot
   * (if the arrow or WASD keys were pressed). Letters are numbered alphabetically
   * starting from 'a' = 0. */
  int pollKeyPress();

  /* Draw things into the UI. If `robot_frame` is true, the given points_t or
   * trajectory_t are assumed to be given in the robot frame, and will be transformed
   * into the window frame based on the current location of the robot.
   * TODO: Implement a more complete system for handling transformations. */
  void drawPoints(const points_t &pp, bool robot_frame, sf::Color c);
  void drawTraj(const trajectory_t &traj, bool robot_frame, sf::Color c);

  /* Call this to update the UI (usually after moving the robot and/or
   * drawing things using drawPoints or drawTraj). */
  void show();

  World &world;

private:
  bool diag_;
  bool truth_;

  void render();
  void renderRobotView();
  void renderTruth();
  void renderReadings(const transform_t &tf);
  const transform_t baseTF();
};

#endif
