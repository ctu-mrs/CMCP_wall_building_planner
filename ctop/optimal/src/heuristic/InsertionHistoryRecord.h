//
// Created by Michal Němec on 12/9/19.
//

#ifndef CTOP_PLANNER_INSERTIONHISTORYRECORD_H
#define CTOP_PLANNER_INSERTIONHISTORYRECORD_H

#include <vector>
#include "GraphNode.h"

namespace ctop {

struct InsertionHistoryRecord {
    InsertionHistoryRecord(std::vector<GraphNode> graph_, GraphNode insertionNode_);
    std::vector<GraphNode> graph;
    GraphNode insertionNode;
};

}

#endif //CTOP_PLANNER_INSERTIONHISTORYRECORD_H
