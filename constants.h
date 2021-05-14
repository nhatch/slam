
#ifndef _SIMULATOR_CONSTANTS_H_
#define _SIMULATOR_CONSTANTS_H_

#include <cmath>

namespace NavSim {

/* Robot footprint */

const double ROBOT_LENGTH = 1.0; // m
const double ROBOT_WHEEL_BASE = ROBOT_LENGTH*2/3; // m
const double MAX_SPEED = ROBOT_LENGTH * 10; // m/s

const double OBSTACLE_MAX_SIZE = ROBOT_LENGTH * 15; // m. used for efficient calculations

/* Sensors and actuators */
// I define a lot of these constants in terms of ROBOT_LENGTH to make it easy to change the scale

const double LANDMARK_MAX_RANGE = ROBOT_LENGTH * 15; // m
const double LIDAR_MAX_RANGE = ROBOT_LENGTH * 5.6; // m
const double LIDAR_MIN_RANGE = ROBOT_LENGTH; // m
const int LIDAR_RESOLUTION = 100; // number of scans in the lidar fov
const double LIDAR_FOV = 240.0 * M_PI / 180.0; // rad
const double LIDAR_FOV_DIR = 0; // radians, gives the direction relative to robot front of fov center
const double CORRUPTION_STD = 0.04; // std in meters at 1 meter distance
                                    // variance increases for measurements farther from robot
const double DATA_LOSS_THRESHOLD = -2.0; // if stdn() is less than this, erase data
const double GPS_POS_STD = 2.0; // m
const double GPS_THETA_STD = M_PI/24; // rad (yes, I know this uses a magnetometer, not a GPS)
const double WHEEL_STD = 0.04; // std in meters for 1 meter distance traveled
                               // variance increases for longer distances

/* Graphics */

const int DEFAULT_WINDOW_WIDTH_PX = 600;
const double DEFAULT_WINDOW_WIDTH = 2*LANDMARK_MAX_RANGE; // m
const double DEFAULT_WINDOW_CENTER_X = ROBOT_LENGTH*10; // m
const double DEFAULT_WINDOW_CENTER_Y = 2.0; // m

/* Planning */

const double RADIAN_COST = ROBOT_WHEEL_BASE / 2.0; // Distance (m) we could have traveled forward in the time it takes to turn 1 radian
const double SAFE_RADIUS = ROBOT_LENGTH * 1.3; // Planner stays this far away from obstacles (m)
const int MAX_ITERS = 3000; // Max number of nodes expanded during A* search
const double PLAN_RESOLUTION = ROBOT_LENGTH; // m
const double SEARCH_RADIUS_INCREMENT = ROBOT_LENGTH*3;
const double GPS_WAYPOINT_RADIUS = ROBOT_LENGTH * 1.5;
const double LANDMARK_WAYPOINT_RADIUS = ROBOT_LENGTH * 1.3;
const double EPS = 2.0; // heuristic weight for weighted A*

/* State estimation */

const double SLAM_VAR = (ROBOT_LENGTH / 3) * (ROBOT_LENGTH / 3);

}

#endif
