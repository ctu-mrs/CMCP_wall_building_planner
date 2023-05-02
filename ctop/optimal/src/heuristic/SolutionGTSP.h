//
// Created by Michal Němec on 12/9/19.
//

#ifndef CTOP_PLANNER_SOLUTIONGTSP_H
#define CTOP_PLANNER_SOLUTIONGTSP_H

#include <vector>

namespace ctop {

struct SolutionGTSP {
    std::vector<int> node_ids;
    double length;
    int num_nodes;
};

}

#endif //CTOP_PLANNER_SOLUTIONGTSP_H
