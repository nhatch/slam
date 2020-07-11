#include "world_interface.h"
#include "world.h"
#include <unistd.h>

World world;

void world_interface_init() {
  world.addURCObstacles();
  world.start();
  // Sleep to avoid situation where we're trying to launch two SFML windows
  // simultaneously (which seems to sometimes cause deadlock??)
  usleep(300 * 1000);
}

// TODO clean this up. Right now we measure sensor data
// iff we set the cmd_vel.
void setCmdVel(double dtheta, double dx) {
  world.setCmdVel(dtheta, dx);
}

points_t readLidarScan() {
  return world.readLidar();
}

points_t readLandmarks() {
  return world.readLandmarks();
}

transform_t readGPS() {
  return world.readGPS();
}

transform_t readOdom() {
  return world.readOdom();
}

URCLeg getLeg(int index) {
  return world.getLeg(index);
}
