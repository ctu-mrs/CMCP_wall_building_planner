//
// Created by Michal Němec on 12/9/19.
//

#include "EdgeTypes.h"

using namespace ctop;

TSP_EDGE_WEIGHT_TYPE parse_tsp_edge_type(const std::string& edge_type_string) {

    if (edge_type_string == TSP_EDGE_EUC_2D_STR) {
        return TSP_EDGE_WEIGHT_TYPE::EUC_2D;
    } else if (edge_type_string == TSP_EDGE_CEIL_2D_STR) {
        return TSP_EDGE_WEIGHT_TYPE::CEIL_2D;
    } else if (edge_type_string == TSP_EDGE_CEIL_EXPLICIT_STR) {
        return TSP_EDGE_WEIGHT_TYPE::EXPLICIT;
    } else {
        std::cerr << "unknown tsp edge weight type " << edge_type_string << std::endl;
        exit(1);
    }
}