//
// Created by Michal Němec on 16/03/2020.
//

#include "WallGraph.h"

using namespace ctop;

int64_t
WallEdge::start_brick_id()
{
    if(start != nullptr) {
        if(start->brick != nullptr) {
            return start->brick->id;
        }
    }
    return -1;
}

int64_t
WallEdge::end_brick_id()
{
    if(end != nullptr) {
        if(end->brick != nullptr) {
            return end->brick->id;
        }
    }
    return -1;
}
