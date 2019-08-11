//
// Created by wenjj.
//

#ifndef CONSTANTS
#define CONSTANTS

#define VEHICLE_RADIUS 1.25              // meters
#define FOLLOW_DISTANCE 8.0              // distance to keep behind leading cars

#define PREVIOUS_PATH_POINTS_TO_KEEP 25
#define NUM_PATH_POINTS 50
#define PATH_DT 0.02                    // seconds

#define TRACK_LENGTH 6945.554           // meters

// number of waypoints to use for interpolation
#define COARSE_WAYPOINTS_BEHIND 5
#define COARSE_WAYPOINTS_FORWARD 10

// for trajectory generation/evaluation and non-ego car predictions
#define N_SAMPLES 20
#define DT 0.20                         // seconds

#define SPEED_LIMIT 21.5                // m/s
#define VELOCITY_INCREMENT_LIMIT 0.125

// cost function weights
#define COLLISION_COST_WEIGHT 99999
#define BUFFER_COST_WEIGHT 10
#define EFFICIENCY_COST_WEIGHT 10000

#define SENSOR_RANGE 250.0              // meters
#define INTERPLATED_DISTANCE 0.5

#define MAX_INSTANTANEOUS_ACCEL 10      // m/s/s

#endif
