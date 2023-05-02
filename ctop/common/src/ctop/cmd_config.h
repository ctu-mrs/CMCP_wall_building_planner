//
// Created by Michal Němec on 12/13/19.
//

#ifndef CTOP_PLANNER_COMMON_CONFIG_H
#define CTOP_PLANNER_COMMON_CONFIG_H

#include <iostream>
#include <string>
#include <fstream>

#include <log4cxx/appender.h>
#include <log4cxx/fileappender.h>
#include <log4cxx/consoleappender.h>
#include <log4cxx/layout.h>
#include <log4cxx/patternlayout.h>
#include <log4cxx/level.h>

#include <crl/logging.h>
#include <crl/config.h>
#include <crl/boost_args_config.h>

#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>

#define GOP_VERSION "1.0"
#define LOGGER_NAME "lp_gop"

namespace ctop {

struct ConfigOptions {
    bool valid = false;
    std::string problemFile;
    std::string problemFileObj;
    std::string name;
    std::string problemSolution;

    crl::CConfig sopConfig;
    std::string canvasOutput;

    explicit operator bool() const {
        return valid;
    }

    static ConfigOptions parseArgs(int argc, char *argv[]);

private:
    static void addCommonConfig(crl::CConfig &config);

};

}


#endif //CTOP_PLANNER_COMMON_CONFIG_H
