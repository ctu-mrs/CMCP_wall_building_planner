//
// Created by Michal Němec on 15/04/2018.
//

#ifndef CTOP_PLANNER_CSVREADER_H
#define CTOP_PLANNER_CSVREADER_H

#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>

namespace ctop {

bool parse_csv_line(const std::string &inputFileName, const std::function<void(int, const std::vector<std::string>&)>& fn) {
    std::ifstream inputFile(inputFileName);
    int l = 0;

    while (inputFile) {
        std::string s;
        if (!getline(inputFile, s)) break;
        if (s[0] != '#') {
            std::istringstream ss(s);
            std::vector<std::string> record;
            while (ss) {
                std::string line;
                if (!getline(ss, line, ' '))
                    break;
                try {
                    record.push_back(line);
                }
                catch (const std::invalid_argument& e) {
                    std::cout << "NaN found in file " << inputFileName << " line " << l << ", " << line << std::endl;
                    return false;
                }
            }
            fn(l, record);
        }
        l++;
    }
    return inputFile.eof();
}

}




#endif //UAV_LOCALIZATION_CORE_CSVREADER_H
