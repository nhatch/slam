#include "world_interface.h"
#include "world.h"

World world;

void world_interface_init() {
  world.addDefaultObstacles();
  world.addLandmark(20, 3);
  world.readSensors();
}

// TODO clean this up. Right now we measure sensor data
// iff we set the cmd_vel.
void setCmdVel(double dtheta, double dx) {
  world.readSensors();
  world.setCmdVel(dtheta, dx);
}

points_t getLidarScan() {
  return world.lidar().back();
}

points_t getLandmarks() {
  return world.landmarks().back();
}

transform_t getGPS() {
  return world.gps().back();
}
