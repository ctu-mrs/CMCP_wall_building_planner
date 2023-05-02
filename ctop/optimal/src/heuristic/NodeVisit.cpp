//
// Created by Michal Němec on 12/10/19.
//

#include "NodeVisit.h"

using namespace ctop;

NodeVisit::NodeVisit() = default;

NodeVisit::NodeVisit(const GraphNode& graphNode)
        :   brick_id(graphNode.brick_id),
            duration(graphNode.node_duration),
            node_id(graphNode.id)
{}

