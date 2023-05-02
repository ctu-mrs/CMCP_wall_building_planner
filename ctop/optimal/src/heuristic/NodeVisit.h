//
// Created by Michal Němec on 12/9/19.
//

#ifndef CTOP_PLANNER_NODEVISIT_H
#define CTOP_PLANNER_NODEVISIT_H

#include "GraphNode.h"

namespace ctop {

struct NodeVisit {
    unsigned int node_id = 0;
    int brick_id = 0;
    double duration = 0;

    double visit_start_time = 0;

    NodeVisit();
    explicit NodeVisit(const GraphNode& graphNode);
};

}

#endif //CTOP_PLANNER_NODEVISIT_H
