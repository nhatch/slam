
#include "utils.h"

void world_interface_init(); // Call this before trying to do anything else
void setCmdVel(double dtheta, double dx);

points_t readLidarScan();
points_t readLandmarks();
transform_t readGPS();
transform_t readOdom();
