//
// Created by happy on 2022-04-02.
//

#include "cost_functions.h"
#include "common.h"
#include <math.h>

#define MAX_COST 1000000
#define SPEED_DIFF_WEIGHT   0.6
#define CENTER_LANE_WEIGHT  0.3
#define SWITCH_WEIGHT       0.1


float calc_lane_switch_cost(int new_lane, sensor_fusion_t sensor_fusion, double time_til_change, double curr_speed, double change_pos, double &min_new_speed) {
    float cost = 0;
    min_new_speed = 50; // max allowable speed (if no cars in the other lane we will be able to go at max speed)

    for (auto &vehicle : sensor_fusion) {
        // Check if the car is in the desired lane of the ego vehicle
        if (vehicle[VEHICLE_D_POS] < (2+4*new_lane+2) && vehicle[VEHICLE_D_POS] > (2+4*new_lane-2)){
            double vx = vehicle[VEHICLE_X_VEL];
            double vy = vehicle[VEHICLE_Y_VEL];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = vehicle[VEHICLE_S_POS];

            // Calculate the check_car's future location
            check_car_s += time_til_change* check_speed;

            // Check if check_car will collide during lane switch
            if (check_car_s > (change_pos - 15) && check_car_s < (change_pos + 50 / 2.24 * 1)) {
                return MAX_COST;
            }
            // No collision. Track min speed of the new lane
            else {
                if (check_car_s - (change_pos - 15)) { // We will be in front of the car so we aren't blocked
                    check_speed = 50;
                }

                if (check_speed < min_new_speed) {

                    min_new_speed = check_speed;
                }
            }
        }
    }

    // Calculate cost of switching
    cost += SWITCH_WEIGHT * 100;

    // cost of speed difference (decrements cost if new lane is faster)
    cost += (curr_speed - min_new_speed) / curr_speed * 100 * SPEED_DIFF_WEIGHT;

    // cost of not being in center lane
    cost += abs(new_lane - LANE_CENTER) * 100 * CENTER_LANE_WEIGHT;

    return cost;
}

float calc_lane_keep_cost(int curr_lane, double new_lane_speed, double curr_lane_speed) {
    float cost = 0;

    // cost of speed difference
    cost += (new_lane_speed - curr_lane_speed) / curr_lane_speed * 100 * SPEED_DIFF_WEIGHT;

    // cost of not being in center lane
    cost += abs(curr_lane - LANE_CENTER) * 100 * CENTER_LANE_WEIGHT;

    return cost;
}
