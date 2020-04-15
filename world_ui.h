#ifndef NAVSIM_UI_H
#define NAVSIM_UI_H

#include "utils.h"
#include "world.h"
#include <SFML/Graphics.hpp>

class WorldUI {
public:
  WorldUI(World &w);
  void runSimulation(int T);
  char handleKeyPress();
  void render();
  void show();
  void drawPointsP(const points_t &pp, sf::Color c);
  void drawTrajP(const trajectory_t &traj, sf::Color c);

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
