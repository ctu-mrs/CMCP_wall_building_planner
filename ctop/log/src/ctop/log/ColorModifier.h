//
// Created by Michal Němec on 03/02/2020.
//

#ifndef CTOP_PLANNERCOLORMODIFIER_H
#define CTOP_PLANNERCOLORMODIFIER_H

#include <ostream>

namespace ctop {
namespace log {

enum Code {
    FG_RED      = 31,
    FG_GREEN    = 32,
    FG_BLUE     = 34,
    FG_MAGENTA     = 35,
    FG_DEFAULT  = 39,
    BG_RED      = 41,
    BG_GREEN    = 42,
    BG_BLUE     = 44,
    BG_DEFAULT  = 49
};

class Modifier {
    Code code_ = FG_DEFAULT;
public:
    Modifier(Code pCode) : code_(pCode) {}
    Modifier(const Modifier& mod) = default;

    friend std::ostream&
    operator<<(std::ostream& os, const Modifier& mod) {
        return os << "\033[" << mod.code_ << "m";
    }
};

}
}

#endif //CTOP_PLANNERCOLORMODIFIER_H
