//
// Created by Michal Němec on 12/9/19.
//

#include "PrecedenceRule.h"

using namespace ctop;

PrecedenceRule::PrecedenceRule() = default;

PrecedenceRule::PrecedenceRule(int before_, int after_)
: before(before_), after(after_)
{}