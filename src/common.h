//
// Created by happy on 2022-04-02.
//

#ifndef PATH_PLANNING_COMMON_H
#define PATH_PLANNING_COMMON_H

#define MILE_PH_TO_METER_PS 2.24
#define MAX_SPEED           49.5

typedef enum {
    LANE_KEEP,
    LANE_CHANGE_LEFT,
    LANE_CHANGE_RIGHT
} state_t;

typedef enum {
    VEHICLE_ID,
    VEHICLE_X_POS,
    VEHICLE_Y_POS,
    VEHICLE_X_VEL,
    VEHICLE_Y_VEL,
    VEHICLE_S_POS,
    VEHICLE_D_POS,
} vehicle_param_t;

typedef enum {
    LANE_LEFT,
    LANE_CENTER,
    LANE_RIGHT
} lane_t;

#endif //PATH_PLANNING_COMMON_H
