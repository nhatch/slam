#include "utils.h"
#include <unistd.h>

void world_interface_init() {
}

bool setCmdVel(double /*dtheta*/, double /*dx*/) {
  return true;
}

points_t readLidarScan() {
  return {};
}

points_t readLandmarks() {
  return {};
}

transform_t readGPS() {
  return toTransform({0,0,0});
}

transform_t readOdom() {
  return toTransform({0,0,0});
}

URCLeg getLeg(int /*id*/) {
  return URCLeg { -1, -1, {0.,0.,0.}};
}
