
#ifndef _SIMULATOR_CONSTANTS_H_
#define _SIMULATOR_CONSTANTS_H_

namespace NavSim {

/* Robot footprint */

const double ROBOT_WHEEL_BASE = 0.2; // m
const double ROBOT_LENGTH = 0.3; // m

/* Sensors and actuators */

const double LANDMARK_MAX_RANGE = 10.0; // m
const double LIDAR_MAX_RANGE = 5.0; // m
const double LIDAR_MIN_RANGE = 0.3; // m
const int LIDAR_RESOLUTION = 100; // number of scans per full rotation
const double CORRUPTION_STD = 0.04; // std in meters at 1 meter distance
                                    // variance increases for measurements farther from robot
const double GPS_POS_STD = 0.2; // m
const double GPS_THETA_STD = M_PI/24; // rad (yes, I know this uses a magnetometer, not a GPS)
const double WHEEL_STD = 0.03; // std in meters for 1 meter distance traveled
                               // variance increases for longer distances

/* Graphics */

const double ANIMATION_SPEED = 4.0; // m/s
const double WINDOW_WIDTH = 10.0; // m
const double WINDOW_CENTER_X = 3.0; // m
const double WINDOW_CENTER_Y = 0.0; // m

/* Planning */

const double RADIAN_COST = ROBOT_WHEEL_BASE / 2.0; // Distance (m) we could have traveled forward in the time it takes to turn 1 radian
const double SAFE_RADIUS = 0.4; // Planner stays this far away from obstacles (m)
const int MAX_ITERS = 3000; // Max number of nodes expanded during A* search

}

#endif
