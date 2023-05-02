//
// Created by Michal Němec on 12/9/19.
//

#ifndef CTOP_PLANNER_CLUSTERSOP_H
#define CTOP_PLANNER_CLUSTERSOP_H

#include <vector>

namespace ctop {

struct ClusterSOP {
    int cluster_id;
    double reward;
    std::vector<int> nodeIDs;
};

}

#endif //CTOP_PLANNER_CLUSTERSOP_H
