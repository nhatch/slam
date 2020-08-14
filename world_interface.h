
#include "utils.h"

void world_interface_init(); // Call this before trying to do anything else
bool setCmdVel(double dtheta, double dx); // On the real robot this can fail (will return false)

points_t readLidarScan();
points_t readLandmarks();
transform_t readGPS();
transform_t readOdom();

// `index` must be in the range 0-6 (the URC competition will have 7 legs)
URCLeg getLeg(int index);
