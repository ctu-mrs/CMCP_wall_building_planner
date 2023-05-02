//
// Created by Michal Němec on 12/9/19.
//

#include "trim.h"

void ctop::trim(std::string &s, const char *t) {
    s.erase(0, s.find_first_not_of(t));
    s.erase(s.find_last_not_of(t) + 1);
}