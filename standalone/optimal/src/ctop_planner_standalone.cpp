//============================================================================
// Name        : gop.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description :
//============================================================================

#include <cstdlib>
#include <algorithm>
#include <unistd.h>
#include <iostream>
#include <string>

#include <log4cxx/appender.h>
#include <log4cxx/fileappender.h>
#include <log4cxx/consoleappender.h>
#include <log4cxx/layout.h>
#include <log4cxx/patternlayout.h>
#include <log4cxx/level.h>

#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>

#include "crl/logging.h"
#include "crl/exceptions.h"
#include "crl/config.h"
#include "crl/boost_args_config.h"

#include "ilp_ctop_alg_solver.h"
#include "common_config.h"

#define LOGGER_NAME "lp_gop"

void run_solver(ConfigOptions& config) {
    auto& sopConfig = config.sopConfig;
    const auto& problemFile = config.problemFile;

    //std::string path = std::string(argv[0]);
    //std::string filename = getFilename(path);
    //WINFO(filename);
    //snprintf(logName, sizeof(logName), "%s.%d.log", filename.c_str(), getpid());
    //crl::initLogger(LOGGER_NAME);

    //clog = new CLog(logName, std::cout);
    //INFO("starting " << filename);

    //std::string singleDatasetLoad = "set_64_1_15.txt";
    //datasetOP loadedDataset = DatasetLoader::loadDataset(dataset64Folder + singleDatasetLoad);
    //std::string singleDatasetLoad = "tsiligirides_problem_1_budget_05.txt";

    //MatlabImportExport::saveMatrixToMatFile("nodes.mat", "nodes", nodesToSave);

    auto sop_solver = sopConfig.get<std::string>("sop-solver");
    if (sop_solver == "lp") {
        INFO("using lp solver");
        ctop::IlpCtopAlgSolver solver(sopConfig, problemFile);
        solver.solve();
    } else {
        ERROR("unknown sop-solver="<<sop_solver);
        ERROR("choose either vns or lp");
        exit(1);
    }
}

int main(int argc, char **argv) {
    bool is_displayable = true;
	int ret = -1;
    auto config = ConfigOptions::parseArgs(argc, argv);
	if (config) {
		INFO("start Logging");
        INFO("problemFile: " << config.problemFile);
        try {
            INFO("executing solver");
            run_solver(config);
			INFO("after solve");
		} catch (crl::exception &e) {
			ERROR("crl::exception: " << e.what() << "!");
		} catch (std::exception &e) {
			ERROR("std::exception: " << e.what() << "!");
		} catch (...) {
            ERROR("unknown exception");
        }
		ret = EXIT_SUCCESS;
	}
	crl::shutdownLogger();
	return ret;
}

