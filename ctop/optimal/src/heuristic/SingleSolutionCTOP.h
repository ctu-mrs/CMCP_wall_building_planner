//
// Created by Michal Němec on 12/9/19.
//

#ifndef CTOP_PLANNER_SINGLESOLUTIONCTOP_H
#define CTOP_PLANNER_SINGLESOLUTIONCTOP_H

#include <vector>

#include "NodeVisit.h"

namespace ctop {

struct SingleSolutionCTOP {
    std::vector<NodeVisit> node_sequence;
};

}

#endif //CTOP_PLANNER_SINGLESOLUTIONCTOP_H
