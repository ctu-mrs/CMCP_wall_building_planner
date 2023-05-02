//
// Created by Michal Němec on 31/03/2020.
//

#include "WallGraph.h"

using namespace ctop;

std::string
ctop::edge_type_2_string(WallEdgeType type)
{
    switch(type) {
        case WallEdgeType::none: return "none";
        case WallEdgeType::precedence: return "precedence";
        case WallEdgeType::distance: return "distance";
        case WallEdgeType::side: return "side";
        case WallEdgeType::side_activate: return "side_activate";
    }
    return "unknown";
}