#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <algorithm>
#include "helpers.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "constants.h"
#include "vehicle.h"
#include "costs.h"

// for convenience
using nlohmann::json;
//using std::string;
//using std::vector;
//using std::map;
using namespace std;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;   //waypoint's x
  vector<double> map_waypoints_y;   //waypoint's y
  vector<double> map_waypoints_s;   //waypoint's s
  vector<double> map_waypoints_dx;  //waypoint's d normal vector in x component
  vector<double> map_waypoints_dy;  //waypoint's d normal vector in y component

  Vehicle ego_car = Vehicle();

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &ego_car, &max_s]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          vector<double> previous_path_x = j[1]["previous_path_x"];
          vector<double> previous_path_y = j[1]["previous_path_y"];
//          std::cout<<previous_path_x<<std::endl;
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side
          //   of the road.
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
            /********************* 1.GET INTERPOLATED WAYPOINTS OF NEARBY TRACK **********************/
            // get nearby waypoints
            int num_waypoints = map_waypoints_x.size();
            int next_waypoint_index = NextWaypoint(car_x, car_y, car_yaw, map_waypoints_x, map_waypoints_y);
            vector<double> coarse_waypoints_s, coarse_waypoints_x, coarse_waypoints_y,
                    coarse_waypoints_dx, coarse_waypoints_dy;
            getNearbyWaypoints(num_waypoints, next_waypoint_index, map_waypoints_s, coarse_waypoints_s,
                               map_waypoints_x, coarse_waypoints_x, map_waypoints_y, coarse_waypoints_y,
                               map_waypoints_dx, coarse_waypoints_dx, map_waypoints_dy, coarse_waypoints_dy);
            // interpolate nearby track
            vector<double> interpolated_waypoints_s, interpolated_waypoints_x, interpolated_waypoints_y,
                    interpolated_waypoints_dx, interpolated_waypoints_dy;
            getNearbyInterpWaypoints(coarse_waypoints_s, interpolated_waypoints_s, coarse_waypoints_x, interpolated_waypoints_x,
                                     coarse_waypoints_y, interpolated_waypoints_y, coarse_waypoints_dx, interpolated_waypoints_dx,
                                     coarse_waypoints_dy, interpolated_waypoints_dy);

            /***************** 2.DETERMINE EGO CAR'S INITIAL STATE TO START A NEW PATH*******************/
            // ego car's initial state includes x, y, angle, s, s_d, s_dd, d, d_d, d_dd
            int previous_path_size = min(PREVIOUS_PATH_POINTS_TO_KEEP, (int)previous_path_x.size());
            double path_start_time = previous_path_size * PATH_DT;
            if(previous_path_size < 5){
                ego_car.x = car_x;
                ego_car.y = car_y;
                ego_car.angle = deg2rad(car_yaw);
                ego_car.s = car_s;
                ego_car.s_d = car_speed;
                ego_car.s_dd = 0;
                ego_car.d = car_d;
                ego_car.d_d = 0;
                ego_car.d_dd = 0;
            }else{
                getEgoCarInitialState(ego_car, previous_path_x, previous_path_y,
                                      interpolated_waypoints_s, interpolated_waypoints_x, interpolated_waypoints_y,
                                      interpolated_waypoints_dx, interpolated_waypoints_dy);
            }



            /****************** 3.PREDICT OTHER NEARBY CARS' SATES FROM SENSOR FUSION *********************/
            double duration = NUM_PATH_POINTS * PATH_DT  - previous_path_size * PATH_DT;
            vector<Vehicle> other_cars;
            map<int, vector<vector<double>>> predictions;
            for (auto sf: sensor_fusion) {
                double vx = sf[3], vy = sf[4];
                double other_car_s = sf[5], other_car_d = sf[6];
                double other_car_vel = sqrt(pow(vx, 2) + pow(vy, 2));
                //assume other cars don't change lanes and run in constant speed
                Vehicle other_car = Vehicle(other_car_s, other_car_vel, 0, other_car_d, 0, 0);
                other_cars.push_back(other_car);
                int v_id = sf[0];
                vector<vector<double>> preds = other_car.generate_predictions(path_start_time, duration);
                predictions[v_id] = preds;
            }

            /********************** 4.GENERATE POSSIBLE TARGET FOR EGO CAR ***************************/
            bool car_to_left = false, car_to_right = false, car_just_ahead = false;
            double max_s_diff = 0;
            for (Vehicle other_car: other_cars) {
                double s_diff = fabs(other_car.s - car_s);
                if(s_diff>max_s_diff){
                    max_s_diff = s_diff;
                }
                if (s_diff < FOLLOW_DISTANCE) {
                    double d_diff = other_car.d - car_d;
                    if (d_diff > 2 && d_diff < 6) {
                        car_to_right = true;
                    } else if (d_diff < -2 && d_diff > -6) {
                        car_to_left = true;
                    } else if (d_diff > -2 && d_diff < 2) {
                        car_just_ahead = true;
                    }
                }
            }

            ego_car.update_available_states(car_to_left, car_to_right);

            vector<vector<double>> best_frenet_traj, best_target;
            double best_cost = 99999;
            string best_traj_state = "";
            for (string state: ego_car.available_states) {
                vector<vector<double>> target_s_and_d = ego_car.get_target_for_state(state, predictions, duration, car_just_ahead);
                vector<vector<double>> possible_traj = ego_car.generate_traj_for_target(target_s_and_d, duration);
                /************ 5. CONSTRUCT POSSIBLE TRAJECTORY DUE TO LOWEST COST **************/
                double current_cost = calculate_total_cost(possible_traj[0], possible_traj[1], predictions);

                if (current_cost < best_cost) {
                    best_cost = current_cost;
                    best_frenet_traj = possible_traj;
                    best_traj_state = state;
                    best_target = target_s_and_d;
                }
            }

             /********************** 6. GET EGO CAR'S FUTURE NEW PATH ************************/
            vector<double> coarse_s_traj, coarse_x_traj, coarse_y_traj, interpolated_s_traj,
                    interpolated_x_traj, interpolated_y_traj;

            double prev_s = ego_car.s - ego_car.s_d * PATH_DT;

            // first two points of coarse trajectory are cloese to each other, make ensure spline begins smoothly
            if (previous_path_size >= 2) {
                coarse_s_traj.push_back(prev_s);
                coarse_x_traj.push_back(previous_path_x[previous_path_size-2]);
                coarse_y_traj.push_back(previous_path_y[previous_path_size-2]);
                coarse_s_traj.push_back(ego_car.s);
                coarse_x_traj.push_back(previous_path_x[previous_path_size-1]);
                coarse_y_traj.push_back(previous_path_y[previous_path_size-1]);
            } else {
                double prev_s = ego_car.s - 1;
                double prev_x = ego_car.x - cos(ego_car.angle);
                double prev_y = ego_car.y - sin(ego_car.angle);
                coarse_s_traj.push_back(prev_s);
                coarse_x_traj.push_back(prev_x);
                coarse_y_traj.push_back(prev_y);
                coarse_s_traj.push_back(ego_car.s);
                coarse_x_traj.push_back(ego_car.x);
                coarse_y_traj.push_back(ego_car.y);
            }

            // last two points of coarse trajectory, use target_d and current s + 30,60
            double target_s1 = ego_car.s + 30;
            // printf("best_target: %.2f, interpolated_s: %.2f, diff: %.2f\n", best_target[0][0], interpolated_waypoints_s[interpolated_waypoints_s.size()-1], best_target[0][0]-interpolated_waypoints_s[interpolated_waypoints_s.size()-1]);
            double target_d1 = best_target[1][0];
            vector<double> target_xy1 = getXY(target_s1, target_d1, interpolated_waypoints_s, interpolated_waypoints_x, interpolated_waypoints_y);
            double target_x1 = target_xy1[0];
            double target_y1 = target_xy1[1];
            coarse_s_traj.push_back(target_s1);
            coarse_x_traj.push_back(target_x1);
            coarse_y_traj.push_back(target_y1);
            double target_s2 = target_s1 + 30;
            double target_d2 = target_d1;
            vector<double> target_xy2 = getXY(target_s2, target_d2, interpolated_waypoints_s, interpolated_waypoints_x, interpolated_waypoints_y);
            double target_x2 = target_xy2[0];
            double target_y2 = target_xy2[1];
            coarse_s_traj.push_back(target_s2);
            coarse_x_traj.push_back(target_x2);
            coarse_y_traj.push_back(target_y2);


            // next s values
            double target_s_dot = best_target[0][1];
            double current_s = ego_car.s;
            double current_v = ego_car.s_d;
            double current_a = ego_car.s_dd;
            for (int i = 0; i < (NUM_PATH_POINTS - previous_path_size); i++) {
                double v_incr, a_incr;
                if (fabs(target_s_dot - current_v) < 2 * VELOCITY_INCREMENT_LIMIT) {
                    v_incr = 0;
                } else {
                    v_incr = (target_s_dot - current_v)/(fabs(target_s_dot - current_v)) * VELOCITY_INCREMENT_LIMIT;
                }
                current_v += v_incr;
                current_s += current_v * PATH_DT;
                interpolated_s_traj.push_back(current_s);
            }

            interpolated_x_traj = interpolate_points(coarse_s_traj, coarse_x_traj, interpolated_s_traj);
            interpolated_y_traj = interpolate_points(coarse_s_traj, coarse_y_traj, interpolated_s_traj);


            // add previous path, if any, to next path
            for(int i = 0; i < previous_path_size; i++) {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }
            // add xy points from newly generated path
            for (int i = 0; i < interpolated_x_traj.size(); i++) {
                //if (subpath_size == 0 && i == 0) continue; // maybe skip start position as a path point?
                next_x_vals.push_back(interpolated_x_traj[i]);
                next_y_vals.push_back(interpolated_y_traj[i]);
            }


            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}