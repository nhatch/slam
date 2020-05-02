
#include "utils.h"

void init(); // Call this before trying to do anything else
void setCmdVel(double dx, double dtheta);

points_t getLidarScan();
points_t getLandmarks();
transform_t getGPS();
// TODO does the rover have wheel encoders?
