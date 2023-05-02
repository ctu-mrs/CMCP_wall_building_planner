//
// Created by michalks on 11/21/17.
//

#ifndef CTOP_PLANNERSPLIT_H
#define CTOP_PLANNERSPLIT_H

#include <string>
#include <sstream>
#include <vector>
#include <iterator>

namespace ctop {

std::vector<std::string> split(const std::string& s, const std::string& delimiter);
std::vector<std::string> split(const std::string& s, const std::string& delimiter, std::size_t count);

}

#endif //CTOP_PLANNERSPLIT_H
