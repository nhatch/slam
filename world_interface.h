
#include "utils.h"

void world_interface_init(); // Call this before trying to do anything else
void setCmdVel(double dtheta, double dx);

points_t getLidarScan();
points_t getLandmarks();
transform_t getGPS();
// TODO does the rover have wheel encoders?
