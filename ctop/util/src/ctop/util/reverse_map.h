//
// Created by Michal Němec on 12/12/19.
//

#ifndef CTOP_PLANNER_REVERSE_MAP_H
#define CTOP_PLANNER_REVERSE_MAP_H

namespace ctop {

template<typename T, typename U>
void reverse_map(const T& in, U& out) {
    for (const auto& val : in) {
        out[val.second] = val.first;
    }
}

}

#endif //CTOP_PLANNER_REVERSE_MAP_H
