//
// Created by Michal Němec on 12/9/19.
//

#ifndef CTOP_PLANNER_PRECEDENCERULE_H
#define CTOP_PLANNER_PRECEDENCERULE_H

namespace ctop {

struct PrecedenceRule {
    int before = 0;
    int after = 0;

    PrecedenceRule();
    PrecedenceRule(int before_, int after_);
};

}

#endif //CTOP_PLANNER_PRECEDENCERULE_H
