//
// Created by Michal Němec on 12/13/19.
//

#include "common_config.h"

namespace po = boost::program_options;
namespace fs = boost::filesystem;

void ConfigOptions::addCommonConfig(crl::CConfig &config) {
    std::cout << "addCommonConfig" << std::endl;
    config.add<std::string>("output", "output directory to store particular results and outputs", "./");
    config.add<std::string>("results", "result log file, it will be placed in output directory", "results.log");
    config.add<std::string>("info", "information file, it will be placed in particular experiment directory", "info.txt");
    config.add<std::string>("settings", "store configurations in boost::program_options config file format ", "settings.txt");
    config.add<std::string>("result-path", "file name for the final found path (ring) as sequence of points", "path.txt");
    config.add<std::string>("result-canvas-output", "file name for final canvas output (eps,png,pdf,svg) are supported");
    config.add<std::string>("result-canvas-suffixes",
                            "comman separated list of exptensis e.g. png,pdf.  if specified  several result images are created, with particular suffix to the resultCanvasOutput");
    config.add<std::string>("name", "name used in result log as user identification if not set a default values (cities) is used");
    config.add<int>("iteration", "set particular interation, otherwise all interations (batch) are performed", -1);
    config.add<int>("batch", "number of iterations from 0 to batch (not included) ", -1);
    config.add<bool>("continue", "in batch mode, partial results are loaded and checked, only missing iterations are computed ", false);
    config.add<bool>("save-results", "disable/enable save results,configs and so on", true);
    config.add<bool>("save-info", "disable/enable save info", true);
    config.add<bool>("save-settings", "disable/enable save settings", true);

    config.add<bool>("save-visual", "disable/enable save visualization results, canvas be", true);
    config.add<bool>("verbose-result-log", "disable/enable printing results log into logger", false);
    // end basic config
    config.add<double>("learning-rate", "neuron adaptation parametr in activation function mi*exp(d)", 0.6); //0.6

    config.add<double>("number-neurons-multiplication", "multiplication parametr to get number of neurons as multiplication of number of cities",
                       2.5);
    config.add<double>("gain-decreasing-rate", "Decrasing rate, higher value means faster convergence", 1e-4);
    config.add<double>("neighborhood-factor",
                       "factor to compute number of neighborhood neurons as division of number of neurons, neighborhood are considered on both sides (left, right) of winning neuron so d*2 neurons are moved ",
                       5);
    config.add<int>("termination-max-steps", "Stop adaptation if number of steps is large than given", 180);
    config.add<double>("termination-minimal-learning-rate", "Stop adaptation if the current learning rate is below this threshold", 0.4);
    config.add<double>("termination-error", "Stop adaptation when the current error is less than this value", 0.001);

    config.add<bool>("best-path", "Enable/disable considering best path found during the evaluation", false);

    config.add<std::string>("pic-dir", "relative directory in result directory to store pictures from each iteration");
    config.add<std::string>("pic-ext", "extension of pic, eps, png, pdf, svg (supported by particular gui renderer rendered", "png");
    config.add<bool>("save-pic", "enable/disable saving pictures (after each refine)");

    config.add<bool>("draw-stations", "Enable/Disable drawing stations", true);
    config.add<bool>("draw-ring", "Enable/Disable drawing ring in the final shoot", true);
    config.add<bool>("draw-path", "Enable/Disable drawing ring in the final shoot", true);
    config.add<double>("canvas-border", "Free space around the canvas", 10);

    config.add<bool>("draw-neurons", "enable/disable drawing neurons", false);
    config.add<bool>("draw-winners", "enable/disable drawing winner using a different shape", false);
    config.add<bool>("draw-ring-iter", "enable/disable drawing ring at each iteration", false);
    config.add<bool>("draw-path-vertices", "enable/disable drawing path vertices(nodes)", true);
    config.add<bool>("draw-tour-represented-by-ring", "enable/disable drawing tour represented by ring", false);
    config.add<bool>("draw-cluster-points", "enable/disable drawing of cluster points", false);

    config.add<int>("dubins-resolution", "intial resolution of dop", 16);
    config.add<double>("dubins-radius", "radius of dubins vehicle", 0.5);
    config.add<bool>("save-targets", "disable/enable save targets", true);
    config.add<bool>("save-sampled-path", "disable/enable save sampled path", true);
    config.add<std::string>("targets", "file to save targets", "targets.txt");
    config.add<std::string>("sampled-path", "file to save sampled path", "sampled-path.txt");
    config.add<double>("sampled-path-distance", "distance between samples of path", 0.05);
    config.add<int>("num-iterations", "number of iteration used in VNS", 1000);
    config.add<int>("num-iterations-unimproved", "number of iteration used in VNS", 3000);
    config.add<bool>("use-rvns", "disable/enable randomized VNS", false);

    config.add<bool>("draw-neighborhood-points", "disable/enable draw points in neighborhoods", false);
    config.add<int>("maximal-calculation-time-sec", "maximal time for calculation", 600);
    config.add<double>("stop-gap", "GAP value before stopping the solver", -1.0);

    config.add<bool>("lower-bound-above-budget-then-skip", "disable/enable skip when lowerbound above budget", true);

    config.add<bool>("generate-examples-of-operations", "disable/enable creation of example operations", false);

    config.add<std::string>("sop-type", "type of SOP");
    config.add<std::string>("sop-solver", "type of SOP solver to use");
    config.add<std::string>("config-file", "yaml config file");

    //for lp solver
    config.add<bool>("lp-online-subtour-elimination", "disable/enable direct subtour elimination which may be slow", true);
    config.add<bool>("lp-adjust-milp-priorities", "disable/enable change of branch and cut priorities", true);
    config.add<std::string>("lp-model-file", "file where to save the model", "ctop-model.lp");
    config.add<int>("num-threads", "number of threads to use", 0);
    config.add<int>("maximal-memory-MB", "maximal memory usage MB", 1024);

    config.add<bool>("initial-greedy-solution", "whether to generate greedy initial solution instead of none or start-goal solution", true);

    //opn
    config.add<double>("neighborhood-radius", "radius of neighborhood in OPN , DOPN ...", 0.0);
    config.add<int>("neighborhood-resolution", "radius of neighborhood in OPN , DOPN ...", 1);

    config.add<bool>("draw-targets-reward", "enable/disable drawing targets in different color using penalty", false);
    config.add<std::string>("draw-targets-reward-palette", "File name with colors for the reward palette", "");
}


ConfigOptions ConfigOptions::parseArgs(int argc, char *argv[]) {

    ConfigOptions config;

    config.valid = false;

    //output
    auto& problemFile = config.problemFile;
    auto& guiConfig = config.guiConfig;
    auto& sopConfig = config.sopConfig;
    auto& canvasOutput = config.canvasOutput;

    bool ret = true;
    std::string configFile;
    std::string guiConfigFile;
    std::string loggerCfg;

    po::options_description desc("General options");
    desc.add_options()
            ("help,h", "produce help message")
            ("config,c",po::value<std::string>(&configFile)->default_value(std::string(argv[0]) + ".cfg"), "configuration file")
            ("logger-config,l",po::value<std::string>(&loggerCfg)->default_value(loggerCfg), "logger configuration file")
            ("config-gui",po::value<std::string>(&guiConfigFile)->default_value(std::string(argv[0]) + "-gui.cfg"), "dedicated gui configuration file")
            ("problem",po::value<std::string>(&problemFile), "problem file");

    try {
        po::options_description guiOptions("Gui options");

        guiConfig.add<double>("gui-add-x", "add the given value to the loaded goals x coord to determine the canvas size and transformation", 0);
        guiConfig.add<double>("gui-add-y", "add the given value to the loaded goals y coord to determine the canvas size and transformation", 0);
        crl::boost_args_add_options(guiConfig, "", guiOptions);
        guiOptions.add_options()("canvas-output", po::value<std::string>(&canvasOutput), "result canvas outputfile");

        po::options_description sopOptions("SOP solver options");
        //boost_args_add_options(SolverLP::getConfig(gopConfig), "", gopOptions);

        addCommonConfig(sopConfig);
        crl::boost_args_add_options(sopConfig, "", sopOptions);

        po::options_description cmdline_options;
        cmdline_options.add(desc).add(guiOptions).add(sopOptions);

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, cmdline_options), vm);
        po::notify(vm);

        std::ifstream ifs(configFile.c_str());
        store(parse_config_file(ifs, cmdline_options), vm);
        po::notify(vm);
        ifs.close();
        ifs.open(guiConfigFile.c_str());
        store(parse_config_file(ifs, cmdline_options), vm);
        po::notify(vm);
        ifs.close();

        if (vm.count("help")) {
            std::cerr << std::endl;
            std::cerr << "Solver of Generalized Orienteering Problem ver. " << GOP_VERSION << std::endl;
            std::cerr << cmdline_options << std::endl;
            ret = false;
        }
        if (ret && !loggerCfg.empty() && fs::exists(fs::path(loggerCfg))) {
            std::cout << "creating logger with config....." << loggerCfg.c_str() << std::endl;
            crl::initLogger(LOGGER_NAME, loggerCfg.c_str());
            std::cout << "done" << std::endl;
        } else {
            std::cout << "creating logger without config....." << std::endl;
            crl::logger = crl::getLogger(LOGGER_NAME);
            //crl::initLogger(LOGGER_NAME);
            std::cout << "created" << std::endl;
            log4cxx::LayoutPtr console_layout = new log4cxx::PatternLayout("%5p %d (%F:%L) - %m%n");

            std::cout << "console_layout created" << std::endl;
            log4cxx::AppenderPtr console_appender = new log4cxx::ConsoleAppender(console_layout);
            std::cout << "console_appender created" << std::endl;

            log4cxx::LayoutPtr file_layout = new log4cxx::PatternLayout("%5p %d (%F:%L) - %m%n");
            std::cout << "file_layout created" << std::endl;
            log4cxx::AppenderPtr file_appedner = new log4cxx::FileAppender(file_layout, "output.log");
            std::cout << "file_appedner created" << std::endl;

            //log4cxx::LevelPtr level = new log4cxx::Level();
            crl::logger->setLevel(log4cxx::Level::getAll());
            std::cout << "setLevel done" << std::endl;
            crl::logger->addAppender(console_appender);
            std::cout << "addAppender done" << std::endl;
            crl::logger->addAppender(file_appedner);
            std::cout << "addAppender done" << std::endl;
            std::cout << "done" << std::endl;
        }
        if (!fs::exists(fs::path(problemFile))) {
            ERROR("Problem file '" + problemFile + "' does not exists");
            ret = false;
        }
    } catch (std::exception &e) {
        std::cerr << std::endl;
        std::cerr << "Error in parsing arguments: " << e.what() << std::endl;
        ret = false;
    }

    config.valid = ret;
    return config;
}
