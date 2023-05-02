//
// Created by Michal Němec on 12/9/19.
//

#ifndef CTOP_PLANNER_GRAPHNODE_H
#define CTOP_PLANNER_GRAPHNODE_H

#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>

namespace ctop {

struct GraphNode {

    double x = std::numeric_limits<double>::quiet_NaN();
    double y = std::numeric_limits<double>::quiet_NaN();
    double z = std::numeric_limits<double>::quiet_NaN();

    bool yaw_rotated = false;
    double reward = 0.0;
    double node_duration = 0.0;
    int robot_requirement = 0;
    unsigned int id = 0;
    int brick_id = 0;
    unsigned int type = 0;

    GraphNode();
    GraphNode(double x_, double y_, double z_, double price_, unsigned int id_, double node_duration_, int robot_requirement_, unsigned int type_);
    std::vector<double> toVector();
    double distanceTo(const GraphNode& gn1);

};

bool operator==(const GraphNode &lhs, const GraphNode &rhs);
std::ostream& operator <<(std::ostream &o, GraphNode &p);
bool operator>(GraphNode &dt1, GraphNode &dt2);
bool operator<(GraphNode &dt1, GraphNode &dt2);

}

#endif //CTOP_PLANNER_GRAPHNODE_H
