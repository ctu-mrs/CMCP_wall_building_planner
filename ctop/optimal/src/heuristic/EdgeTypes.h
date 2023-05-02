//
// Created by Michal Němec on 12/9/19.
//

#ifndef CTOP_PLANNER_EDGETYPES_H
#define CTOP_PLANNER_EDGETYPES_H

#include <string>
#include <cstring>
#include <iostream>

namespace ctop {

enum class TSP_EDGE_WEIGHT_TYPE {
#define TSP_EDGE_EUC_2D_STR "EUC_2D"
    EUC_2D,
#define TSP_EDGE_CEIL_2D_STR "CEIL_2D"
    CEIL_2D,
#define TSP_EDGE_CEIL_EXPLICIT_STR "EXPLICIT"
    EXPLICIT
};

/*
 BRICK_TYPES = {1:"RED",
 2:"GREEN",
 3:"BLUE",
 4:"ORANGE"}

 */

#define NO_BRICK "NO_BRICK"
#define RED_BRICK "RED_BRICK"
#define GREEN_BRICK "GREEN_BRICK"
#define BLUE_BRICK "BLUE_BRICK"
#define ORANGE_BRICK "ORANGE_BRICK"

//enum BRICK_TYPES {
//	NO_BRICK = 0, RED_BRICK = 1, GREEN_BRICK = 2, BLUE_BRICK = 3, ORANGE_BRICK = 4
//};

//extern int BRICK_REWARDS[5];
//extern int BRICK_DURATIONS_S[5];
//extern int BRICK_NUM_ROBOT_REQUIREMENTS[5];

TSP_EDGE_WEIGHT_TYPE parse_tsp_edge_type(const std::string& edge_type_string);

}

#endif //CTOP_PLANNER_EDGETYPES_H
