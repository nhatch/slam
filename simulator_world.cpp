#include "world_interface.h"
#include "world.h"

World world;

void init() {
  world.addDefaultObstacles();
  world.addLandmark(20, 3);
  world.startSimulation();
}

void setCmdVel(double dx, double dtheta) {
  world.moveRobot(dx, dtheta); // TODO switch to cmd_vel
}

points_t getLidarScan() {
  world.readLidar();
  return world.lidar().back();
}

points_t getLandmarks() {
  world.readLandmarks();
  return world.landmarks().back();
}

transform_t getGPS() {
  world.readGPS();
  return world.gps().back();
}
