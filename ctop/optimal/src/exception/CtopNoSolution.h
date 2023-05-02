//
// Created by Michal Němec on 12/10/19.
//

#ifndef CTOP_PLANNER_CTOPNOSOLUTION_H
#define CTOP_PLANNER_CTOPNOSOLUTION_H

#include <exception>
#include <stdexcept>

#if __cplusplus >= 201703L
#define DISCARD [[nodiscard]]
#else
#define DISCARD
#endif

namespace ctop {

struct CtopNoSolution : public std::runtime_error {
    DISCARD const char * what() const noexcept override;
};

}

#endif //CTOP_PLANNER_CTOPNOSOLUTION_H
