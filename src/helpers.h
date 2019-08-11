#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include <map>
#include <iostream>
//#include "smoother.h"
#include "vehicle.h"
#include "spline.h"
#include "constants.h"
#include "Eigen-3.3/Eigen/Dense"

// for convenience
//using std::string;
//using std::vector;
using namespace std;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s);
double deg2rad(double x);
double rad2deg(double x);
//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) ;

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
                    const vector<double> &maps_y);

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
                 const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
                         const vector<double> &maps_x,
                         const vector<double> &maps_y,
                         const vector<double> &maps_s) ;

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s,
                     const vector<double> &maps_x,
                     const vector<double> &maps_y);
vector<double> interpolate_points(vector<double> pts_x, vector<double> pts_y,
                                  vector<double> eval_at_x);
vector<double> get_traj_coeffs(vector<double> start, vector<double> end, double T);

// Get nearby waypoints
void getNearbyWaypoints(int num_waypoints, int next_waypoints_index,
                        vector<double> &map_waypoints_s, vector<double> &coarse_waypoints_s,
                        vector<double> &map_waypoints_x, vector<double> &coarse_waypoints_x,
                        vector<double> &map_waypoints_y, vector<double> &coarse_wayopints_y,
                        vector<double> &map_waypoints_dx, vector<double> &coarse_waypoints_dx,
                        vector<double> &map_waypoints_dy, vector<double> &coarse_waypoints_dy);

void getNearbyInterpWaypoints(vector<double> &coarse_wayopints_s, vector<double> &interp_waypoints_s,
                              vector<double> &coarse_waypoints_x, vector<double> &interp_waypoints_x,
                              vector<double> &coarse_wayopints_y, vector<double> &interp_wayopints_y,
                              vector<double> &coarse_waypoints_dx, vector<double> &interp_waypoints_dx,
                              vector<double> &coarse_waypoints_dy, vector<double> &interp_waypoints_dy);

void getEgoCarInitialState(Vehicle &ego_car, vector<double> &previous_path_x, vector<double> &previous_path_y,
                           vector<double> &interp_waypoints_s, vector<double> &interp_waypoints_x, vector<double> &interp_waypoints_y,
                           vector<double> &interp_waypoints_dx, vector<double> &interp_waypoints_dy);

#endif  // HELPERS_H