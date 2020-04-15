#ifndef NAVSIM_UI_H
#define NAVSIM_UI_H

#include "utils.h"
#include "world.h"
#include <SFML/Graphics.hpp>

class WorldUI {
public:
  WorldUI(World &w);
  void runSimulation(int T);
  int handleKeyPress();
  void render();
  void show();
  void drawPoints(const points_t &pp, bool robot_frame, sf::Color c);
  void drawTraj(const trajectory_t &traj, bool robot_frame, sf::Color c);

  World &world;

private:
  bool diag_;
  bool truth_;

  void renderRobotView();
  void renderTruth();
  void renderReadings(const transform_t &tf);
  const transform_t baseTF();
};

#endif
