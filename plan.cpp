
#include "graphics.h"
#include "plan.h"

plan_t plan(const World &w, const landmark_t &goal)
{
  plan_t p(3,2);
  p << 0, 0.5, M_PI/2, 0, 0, 0.5;
  return p;
}

void drawPlan(const plan_t &p, const transform_t &initial_transform)
{
  trajectory_t traj({});
  transform_t trans = initial_transform;
  traj.push_back(trans);
  for (int i = 0; i < p.rows(); i++)
  {
    double theta = p(i,0);
    double x = p(i,1);
    // Doesn't matter if we rotate first because our plans never rotate
    // and move forward at the same time.
    trans = toTransformRotateFirst(x, 0, theta) * trans;
    traj.push_back(trans);
  }
  landmarks_t lms({}); // Just to conform with drawTraj interface
  drawTraj(lms, traj, sf::Color::Red);
}
