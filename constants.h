
#ifndef _SIMULATOR_CONSTANTS_H_
#define _SIMULATOR_CONSTANTS_H_

namespace NavSim {

/* Robot footprint */

const double ROBOT_WHEEL_BASE = 0.2; // m
const double ROBOT_LENGTH = 0.3; // m

/* Sensors and actuators */
// I define a lot of these constants in terms of ROBOT_LENGTH to make it easy to change the scale

const double LANDMARK_MAX_RANGE = ROBOT_LENGTH * 30; // m
const double LIDAR_MAX_RANGE = ROBOT_LENGTH * 20; // m
const double LIDAR_MIN_RANGE = ROBOT_LENGTH; // m
const int LIDAR_RESOLUTION = 100; // number of scans per full rotation
const double CORRUPTION_STD = 0.04; // std in meters at 1 meter distance
                                    // variance increases for measurements farther from robot
const double GPS_POS_STD = 0.2; // m
const double GPS_THETA_STD = M_PI/24; // rad (yes, I know this uses a magnetometer, not a GPS)
const double WHEEL_STD = 0.03; // std in meters for 1 meter distance traveled
                               // variance increases for longer distances

/* Graphics */

const double ANIMATION_SPEED = ROBOT_LENGTH * 15; // m/s
const double WINDOW_WIDTH = ROBOT_LENGTH * 30; // m
const double WINDOW_CENTER_X = ROBOT_LENGTH * 10; // m
const double WINDOW_CENTER_Y = 0.0; // m

/* Planning */

const double RADIAN_COST = ROBOT_WHEEL_BASE / 2.0; // Distance (m) we could have traveled forward in the time it takes to turn 1 radian
const double SAFE_RADIUS = ROBOT_LENGTH * 1.3; // Planner stays this far away from obstacles (m)
const int MAX_ITERS = 3000; // Max number of nodes expanded during A* search
const double PLAN_RESOLUTION = ROBOT_LENGTH; // m

}

#endif
