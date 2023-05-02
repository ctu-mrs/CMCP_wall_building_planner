//
// Created by Michal Němec on 12/9/19.
//

#ifndef CTOP_PLANNER_DATASETOP_H
#define CTOP_PLANNER_DATASETOP_H

#include <vector>

namespace ctop {

struct DatasetOP {
    std::vector<GraphNode> graph;
    double Tmax; //= available time budget per path
    unsigned int P; //= number of paths (=1)
    unsigned int startID;
    unsigned int goalID;
};

}

#endif //CTOP_PLANNER_DATASETOP_H
