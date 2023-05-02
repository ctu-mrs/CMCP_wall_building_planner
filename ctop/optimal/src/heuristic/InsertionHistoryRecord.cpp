//
// Created by Michal Němec on 12/9/19.
//

#include "InsertionHistoryRecord.h"

using namespace ctop;

InsertionHistoryRecord::InsertionHistoryRecord(std::vector<GraphNode> graph_, GraphNode insertionNode_)
: graph(std::move(graph_)), insertionNode(insertionNode_)
{}