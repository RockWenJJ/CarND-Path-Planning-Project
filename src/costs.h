#ifndef COSTS
#define COSTS

#include <iostream>
#include <cassert>
#include <vector>
#include <algorithm>
#include <cmath>
#include "constants.h"

using namespace std;

// UTILITY FUNCTIONS

double logistic(double x){
    // A function that returns a value between 0 and 1 for x in the range[0, infinity] and - 1 to 1 for x in
    // the range[-infinity, infinity]. Useful for cost functions.
    return 2.0 / (1 + exp(-x)) - 1.0;
}

double nearest_approach(vector<double> s_traj, vector<double> d_traj, vector<vector<double>> prediction) {
    double closest = 999999;
    for (int i = 0; i < N_SAMPLES; i++) {
        double current_dist = sqrt(pow(s_traj[i] - prediction[i][0], 2) + pow(d_traj[i] - prediction[i][1], 2));
        if (current_dist < closest) {
            closest = current_dist;
        }
    }
    return closest;
}

double nearest_approach_to_any_vehicle(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
    // Determines the nearest the vehicle comes to any other vehicle throughout a trajectory
    double closest = 99999;
    for (auto prediction : predictions) {
        double current_dist = nearest_approach(s_traj, d_traj, prediction.second);
        if (current_dist < closest) {
            closest = current_dist;
        }
    }
    return closest;
}

vector<double> velocities_for_trajectory(vector<double> traj) {
    // given a trajectory (a vector of positions), return the average velocity between each pair as a vector
    // also can be used to find accelerations from velocities, jerks from accelerations, etc.
    // (i.e. discrete derivatives)
    vector<double> velocities;
    for (int i = 1; i < traj.size(); i++) {
        velocities.push_back((traj[i] - traj[i-1]) / DT);
    }
    return velocities;
}


// COST FUNCTIONS
double collision_cost(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
    // Binary cost function which penalizes collisions.
    double nearest = nearest_approach_to_any_vehicle(s_traj, d_traj, predictions);
    if (nearest < 2 * VEHICLE_RADIUS) {
        return 1;
    } else {
        return 0;
    }
}

double nearby_cost(vector<double> s_traj, vector<double> d_traj, map<int, vector<vector<double>>> predictions) {
    // Penalizes getting close to other vehicles.
    double nearest = nearest_approach_to_any_vehicle(s_traj, d_traj, predictions);
    return logistic(2 * VEHICLE_RADIUS / nearest);
}

double efficiency_cost(vector<double> s_traj) {
    // Rewards high average speeds.
    vector<double> s_dot_traj = velocities_for_trajectory(s_traj);
    double final_s_dot, total = 0;

    final_s_dot = s_dot_traj[s_dot_traj.size() - 1];
    return logistic((SPEED_LIMIT - final_s_dot) / SPEED_LIMIT);
}

double calculate_total_cost(vector<double> s_traj, vector<double> d_traj, map<int,vector<vector<double>>> predictions) {
    double total_cost = 0;
    double coll = collision_cost(s_traj, d_traj, predictions) * COLLISION_COST_WEIGHT;
    double near = nearby_cost(s_traj, d_traj, predictions) * BUFFER_COST_WEIGHT;
    double effi = efficiency_cost(s_traj) * EFFICIENCY_COST_WEIGHT;

    total_cost += coll + near + effi;


    return total_cost;
}

#endif