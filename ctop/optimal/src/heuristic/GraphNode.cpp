//
// Created by Michal Němec on 12/9/19.
//


#include "GraphNode.h"

using namespace ctop;

GraphNode::GraphNode() = default;

GraphNode::GraphNode(double x_, double y_, double z_, double price_, unsigned int id_, double node_duration_, int robot_requirement_, unsigned int type_) {
    x = x_;
    y = y_;
    z = z_;
    reward = price_;
    id = id_;
    node_duration = node_duration_;
    robot_requirement = robot_requirement_;
    type = type_;
}

std::vector<double>
GraphNode::toVector() {
    return {x, y, reward};
}

double
GraphNode::distanceTo(const GraphNode& gn1) {
    if (!std::isnan(x) && !std::isnan(y)) {
        double dx = gn1.x - x;
        double dy = gn1.y - y;
        return std::sqrt(dx*dx + dy*dy);
    }
    return std::numeric_limits<double>::quiet_NaN();
}

bool operator==(const GraphNode &lhs, const GraphNode &rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y && lhs.reward == rhs.reward;
}

std::ostream& operator <<(std::ostream &o, GraphNode &p) {
    std::cout.precision(6);
    o << std::fixed << " " << std::setprecision(6) << p.x << " " << std::setprecision(6) << p.y << " " << std::setprecision(6) << p.reward;
    return o;
}

bool operator>(GraphNode &dt1, GraphNode &dt2) {
    return dt1.reward > dt2.reward;
}

bool operator<(GraphNode &dt1, GraphNode &dt2) {
    return dt1.reward < dt2.reward;
}
