//
// Created by Michal Němec on 12/9/19.
//

#ifndef CTOP_PLANNER_TIMEDIMPROVEMENT_H
#define CTOP_PLANNER_TIMEDIMPROVEMENT_H

namespace ctop {

struct TimedImprovement {
    TimedImprovement(long timeMS_, double length_, double reward_, int iteration_) {
        this->timeMS = timeMS_;
        this->length = length_;
        this->reward = reward_;
        this->iteration = iteration_;
    }
    long timeMS;
    double length;
    double reward;
    int iteration;
};

}

#endif //CTOP_PLANNER_TIMEDIMPROVEMENT_H
