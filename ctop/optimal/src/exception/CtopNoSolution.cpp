//
// Created by Michal Němec on 12/10/19.
//

#include "CtopNoSolution.h"

using namespace ctop;

DISCARD const char * CtopNoSolution::what() const noexcept {
    return "CTOP could not find solution";
}