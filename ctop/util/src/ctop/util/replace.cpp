//
// Created by michalks on 8/5/18.
//

#include "replace.h"

namespace ctop {

int replace(std::string& subject, const std::string& search,
             const std::string& replace) {
    int count = 0;
    size_t pos = 0;
    while ((pos = subject.find(search, pos)) != std::string::npos) {
        subject.replace(pos, search.length(), replace);
        pos += replace.length();
        count++;
    }
    return count;
}

}
