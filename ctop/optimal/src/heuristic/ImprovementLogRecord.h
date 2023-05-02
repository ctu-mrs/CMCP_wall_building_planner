//
// Created by Michal Němec on 12/9/19.
//

#ifndef CTOP_PLANNER_IMPROVEMENTLOGRECORD_H
#define CTOP_PLANNER_IMPROVEMENTLOGRECORD_H

namespace ctop {

struct ImprovementLogRecord {
    double length;
    double reward;
    long timeMS;
};

}

#endif //CTOP_PLANNER_IMPROVEMENTLOGRECORD_H
