//
// Created by rock on 8/2/19.
//
#include  "helpers.h"
using namespace std;
//using Eigen::MatrixXd;
//using Eigen::VectorXd;

string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x,
                    const vector<double> &maps_y) {
    double closestLen = 100000; //large number
    int closestWaypoint = 0;

    for (int i = 0; i < maps_x.size(); ++i) {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = distance(x,y,map_x,map_y);
        if (dist < closestLen) {
            closestLen = dist;
            closestWaypoint = i;
        }
    }

    return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x,
                 const vector<double> &maps_y) {
    int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

    double map_x = maps_x[closestWaypoint];
    double map_y = maps_y[closestWaypoint];

    double heading = atan2((map_y-y),(map_x-x));

    double angle = fabs(theta-heading);
    angle = std::min(2*pi() - angle, angle);

    if (angle > pi()/2) {
        ++closestWaypoint;
        if (closestWaypoint == maps_x.size()) {
            closestWaypoint = 0;
        }
    }

    return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta,
                         const vector<double> &maps_x,
                         const vector<double> &maps_y,
                         const vector<double> &maps_s) {
    int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

    int prev_wp;
    prev_wp = next_wp-1;
    if (next_wp == 0) {
        prev_wp  = maps_x.size()-1;
    }

    double n_x = maps_x[next_wp]-maps_x[prev_wp];
    double n_y = maps_y[next_wp]-maps_y[prev_wp];
    double x_x = x - maps_x[prev_wp];
    double x_y = y - maps_y[prev_wp];

    // find the projection of x onto n
    double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
    double proj_x = proj_norm*n_x;
    double proj_y = proj_norm*n_y;

    double frenet_d = distance(x_x,x_y,proj_x,proj_y);

    //see if d value is positive or negative by comparing it to a center point
    double center_x = 1000-maps_x[prev_wp];
    double center_y = 2000-maps_y[prev_wp];
    double centerToPos = distance(center_x,center_y,x_x,x_y);
    double centerToRef = distance(center_x,center_y,proj_x,proj_y);

    if (centerToPos <= centerToRef) {
        frenet_d *= -1;
    }

    // calculate s value
    double frenet_s = maps_s[0];
    for (int i = 0; i < prev_wp; ++i) {
        frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
    }

    frenet_s += distance(0,0,proj_x,proj_y);

    return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s,
                     const vector<double> &maps_x,
                     const vector<double> &maps_y) {
    int prev_wp = -1;

    while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
        ++prev_wp;
    }

    int wp2 = (prev_wp+1)%maps_x.size();

    double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                           (maps_x[wp2]-maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s-maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
    double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

    double perp_heading = heading-pi()/2;

    double x = seg_x + d*cos(perp_heading);
    double y = seg_y + d*sin(perp_heading);

    return {x,y};
}

vector<double> interpolate_points(vector<double> pts_x, vector<double> pts_y,
                                  vector<double> eval_at_x) {
    // uses the spline library to interpolate points connecting a series of x and y values
    // output is spline evaluated at each eval_at_x point

    if (pts_x.size() != pts_y.size()) {
        cout << "ERROR! SMOOTHER: interpolate_points size mismatch between pts_x and pts_y" << endl;
        return { 0 };
    }

    tk::spline s;
    s.set_points(pts_x,pts_y);    // currently it is required that X is already sorted
    vector<double> output;
    for (double x: eval_at_x) {
        output.push_back(s(x));
    }
    return output;
}

vector<double> get_traj_coeffs(vector<double> start, vector<double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS
    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]
    end   - the desired end state for vehicle. Like "start" this is a
        length three array.
    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT
    an array of length 6, each value corresponding to a coefficent in the polynomial
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
    */

    Eigen::MatrixXd a(3,3);
    double T2 =  T*T,
            T3 = T2*T,
            T4 = T3*T,
            T5 = T4*T;
    a <<  T3,    T4,    T5,
            3*T2,  4*T3,  5*T4,
            6*T, 12*T2, 20*T3;
    Eigen::MatrixXd aInv = a.inverse();

    Eigen::VectorXd b(3);
    b << end[0] - (start[0] + start[1]*T + 0.5*start[2]*T2),
            end[1] - (           start[1]   +     start[2]*T),
            end[2] - (                            start[2]);
    Eigen::VectorXd alpha = aInv * b;

    vector<double> output = {start[0], start[1], 0.5*start[2], alpha[0], alpha[1], alpha[2]};
    return output;
}

// Get nearby waypoints
void getNearbyWaypoints(int num_waypoints, int next_waypoints_index,
                        vector<double> &map_waypoints_s, vector<double> &coarse_waypoints_s,
                        vector<double> &map_waypoints_x, vector<double> &coarse_waypoints_x,
                        vector<double> &map_waypoints_y, vector<double> &coarse_waypoints_y,
                        vector<double> &map_waypoints_dx, vector<double> &coarse_waypoints_dx,
                        vector<double> &map_waypoints_dy, vector<double> &coarse_waypoints_dy){
    for (int i = -COARSE_WAYPOINTS_BEHIND; i < COARSE_WAYPOINTS_FORWARD; i++){
        int idx = (next_waypoints_index + i) % num_waypoints;
        if (idx < 0){
            // correct for track wrap
            idx += num_waypoints;
        }
        double cur_s = map_waypoints_s[idx];
        double base_s = map_waypoints_s[next_waypoints_index];
        // correct for track warp
        if(cur_s > base_s && i < 0){
            cur_s -= TRACK_LENGTH;
        }
        else if(cur_s < base_s && i > 0){
            cur_s += TRACK_LENGTH;
        }
        coarse_waypoints_s.push_back(cur_s);
        coarse_waypoints_x.push_back(map_waypoints_x[idx]);
        coarse_waypoints_y.push_back(map_waypoints_y[idx]);
        coarse_waypoints_dx.push_back(map_waypoints_dx[idx]);
        coarse_waypoints_dy.push_back(map_waypoints_dy[idx]);
    }
}

void getNearbyInterpWaypoints(vector<double> &coarse_waypoints_s, vector<double> &interp_waypoints_s,
                              vector<double> &coarse_waypoints_x, vector<double> &interp_waypoints_x,
                              vector<double> &coarse_wayopints_y, vector<double> &interp_waypoints_y,
                              vector<double> &coarse_waypoints_dx, vector<double> &interp_waypoints_dx,
                              vector<double> &coarse_waypoints_dy, vector<double> &interp_waypoints_dy){
    int num_interp_points = (coarse_waypoints_s[coarse_waypoints_s.size()-1] - coarse_waypoints_s[0]) / INTERPLATED_DISTANCE;
    double base_s = coarse_waypoints_s[0];
    for(int i = 0; i < num_interp_points ; i++){
        interp_waypoints_s.push_back(base_s + i * INTERPLATED_DISTANCE);
    }
    interp_waypoints_x = interpolate_points(coarse_waypoints_s, coarse_waypoints_x, interp_waypoints_s);
    interp_waypoints_y = interpolate_points(coarse_waypoints_s, coarse_wayopints_y, interp_waypoints_s);
    interp_waypoints_dx = interpolate_points(coarse_waypoints_s, coarse_waypoints_dx, interp_waypoints_s);
    interp_waypoints_dy = interpolate_points(coarse_waypoints_s, coarse_waypoints_dy, interp_waypoints_s);
}

void getEgoCarInitialState(Vehicle &ego_car, vector<double> &previous_path_x, vector<double> &previous_path_y,
                           vector<double> &interp_waypoints_s, vector<double> &interp_waypoints_x, vector<double> &interp_waypoints_y,
                           vector<double> &interp_waypoints_dx, vector<double> &interp_waypoints_dy){

    int previous_size_to_keep = min(PREVIOUS_PATH_POINTS_TO_KEEP, (int)previous_path_x.size());
    double path_start_time = previous_size_to_keep * PATH_DT;
    double pos_x, pos_y, angle, pos_x2, pos_y2, angle2, pos_x3, pos_y3, vel_x, vel_y, vel_x2, vel_y2, acc_x, acc_y;
    // get positions in x and y direction
    pos_x = previous_path_x[previous_size_to_keep - 1];
    pos_y = previous_path_y[previous_size_to_keep - 1];
    ego_car.x = pos_x;
    ego_car.y = pos_y;
    pos_x2 = previous_path_x[previous_size_to_keep - 2];
    pos_y2 = previous_path_y[previous_size_to_keep - 2];
    pos_x3 = previous_path_x[previous_size_to_keep - 3];
    pos_y3 = previous_path_y[previous_size_to_keep - 3];
    // calculate the velocity in x and y direction
    vel_x = (pos_x - pos_x2) / PATH_DT;
    vel_y = (pos_y - pos_y2) / PATH_DT;
    vel_x2 = (pos_x2 - pos_x3) / PATH_DT;
    vel_y2 = (pos_y2 - pos_y3) / PATH_DT;
    // calculate the acceleration in x and y direction
    acc_x = (vel_x - vel_x2) / PATH_DT;
    acc_y = (vel_y - vel_y2) / PATH_DT;

    angle = atan2(pos_y - pos_y2, pos_x-pos_x2);
    ego_car.angle = angle;
    vector<double> frenet = getFrenet(pos_x, pos_y, angle, interp_waypoints_x, interp_waypoints_y, interp_waypoints_s);
    // determine ego car's initial s and d
    ego_car.s = frenet[0];
    ego_car.d = frenet[1];

    int next_interp_waypoint_index = NextWaypoint(pos_x, pos_y, angle, interp_waypoints_x, interp_waypoints_y);
    double dx = interp_waypoints_dx[next_interp_waypoint_index - 1];
    double dy = interp_waypoints_dy[next_interp_waypoint_index - 1];
    // s vector is perpendicular to d vector
    double sx = -dy;
    double sy = dx;
    // determine ego car's initial s_d and d_d
    ego_car.s_d = vel_x * sx + vel_y * sy;
    ego_car.d_d = vel_x * dx + vel_y * dy;

    //determine ego car's initial s_dd and d_dd
    ego_car.s_dd = acc_x * sx + acc_y * sy;
    ego_car.d_dd = acc_x * dx + acc_y * dy;

}