//
// Created by happy on 2022-04-02.
//

#ifndef PATH_PLANNING_COST_FUNCTIONS_H
#define PATH_PLANNING_COST_FUNCTIONS_H

#include <vector>

using std::vector;

typedef vector<vector<double>> sensor_fusion_t;

class cost_functions {

};

float calc_lane_switch_cost(int new_lane, sensor_fusion_t sensor_fusion, double time_til_change, double front_speed, double curr_speed, double change_pos, double &min_new_speed);

float calc_lane_keep_cost(int curr_lane, double new_lane_speed, double curr_lane_speed);

#endif //PATH_PLANNING_COST_FUNCTIONS_H
