//
// Created by Michal Němec on 12/9/19.
//

#ifndef CTOP_PLANNER_TRIM_H
#define CTOP_PLANNER_TRIM_H

#include <string>

namespace ctop {

void trim(std::string &s, const char *t = " \t\n\r\f\v");

}

#endif //CTOP_PLANNER_TRIM_H
