
#include "Eigen/Dense"

#include "ctop/cmd_config.h"
#include "ctop/log.h"
#include "ctop/util/csvreader.h"
#include "ctop/geometry/Point3D.h"
#include "ctop/log/Logger.h"
#include "ctop/util/split.h"

#define GRASP_ENABLE_LOG
#define GRASP_USE_DEBUG_LOG

#ifndef NDEBUG
#define GRASP_ASSERT(expr) \
    do { \
        if (!static_cast <bool>(expr)) { \
            ctop::log::a(#expr, __LINE__, __FILE__, CTOP_FUNCTION); \
        } \
        assert(expr); \
    } while (false)
#else
#define GRASP_ASSERT(tag, expr) assert(expr)
#endif

#ifdef GRASP_ENABLE_LOG
#ifdef GRASP_USE_DEBUG_LOG
#define GRASP_LOG_D(...) ctop::log::d_debug("", __LINE__, __FILE__, CTOP_FUNCTION, __VA_ARGS__)
#define GRASP_LOG_E(...) ctop::log::e_debug("", __LINE__, __FILE__, CTOP_FUNCTION, __VA_ARGS__)
#define GRASP_LOG_I(...) ctop::log::i_debug("", __LINE__, __FILE__, CTOP_FUNCTION, __VA_ARGS__)
#define GRASP_LOG_V(...) ctop::log::v_debug("", __LINE__, __FILE__, CTOP_FUNCTION, __VA_ARGS__)
#define GRASP_LOG_W(...) ctop::log::w_debug("", __LINE__, __FILE__, CTOP_FUNCTION, __VA_ARGS__)
#else
#define GRASP_LOG_D(...) ctop::log::d(__VA_ARGS__)
        #define GRASP_LOG_E(...) ctop::log::e(__VA_ARGS__)
        #define GRASP_LOG_I(...) ctop::log::i(__VA_ARGS__)
        #define GRASP_LOG_V(...) ctop::log::v(__VA_ARGS__)
        #define GRASP_LOG_W(...) ctop::log::w(__VA_ARGS__)
#endif
#else
#define GRASP_LOG_D(...)
    #define GRASP_LOG_E(...)
    #define GRASP_LOG_I(...)
    #define GRASP_LOG_V(...)
    #define GRASP_LOG_W(...)
#endif


#include <iostream>
#include <string>
#include <fstream>
#include <iomanip>
#include "ctop/WallSolution.h"
#include "ctop/WallInfo.h"
#include "ctop/Brick.h"

template<typename T>
static T string_to(const std::string &val) {
    T v;
    std::istringstream iss{val};
    auto valid = static_cast<bool>(iss >> v);
    if(!valid) {
        CTOP_LOG_D("failed on {}", val);
    }
    CTOP_ASSERT(valid);
    return v;
}

std::string read_last_line(const std::string& file) {
    std::string result = "";
    std::ifstream fin(file);

    if(fin.is_open()) {
        fin.seekg(0,std::ios_base::end);      //Start at end of file
        char ch = ' ';                        //Init ch not equal to '\n'
        while(ch != '\n'){
            fin.seekg(-2,std::ios_base::cur); //Two steps back, this means we
            //will NOT check the last character
            if((int)fin.tellg() <= 0){        //If passed the start of the file,
                fin.seekg(0);                 //this is the start of the line
                break;
            }
            fin.get(ch);                      //Check the next character
        }

        std::getline(fin,result);
        fin.close();

        std::cout << "final line length: " << result.size() <<std::endl;
        std::cout << "final line character codes: ";
        for(size_t i =0; i<result.size(); i++){
            std::cout << std::hex << (int)result[i] << " ";
        }
        std::cout << std::endl;
        std::cout << "final line: " << result <<std::endl;
    }

    return result;
}

int main(int argc, char** argv) {
    auto config = ctop::ConfigOptions::parseArgs(argc, argv);
    if (!config) return 1;

    auto outputFolder = config.sopConfig.get<std::string>("output");
    auto config_file = config.sopConfig.get<std::string>("config-file");

    GRASP_LOG_D("output {}", outputFolder);
    GRASP_LOG_D("config-file {}", config_file);
    GRASP_LOG_D("proble file {}", config.problemFile);

    ctop::WallInfo wall;
    wall.parse(config_file);

    std::ostringstream oss{};
    oss << outputFolder << "/" << config.name;
    GRASP_LOG_D("output file {}", oss.str());

    std::ofstream off{oss.str()};
    auto line = read_last_line(config.problemSolution);
    GRASP_LOG_D("plan {}", line);

    std::map<uint64_t, ctop::Brick> bricks;
    auto problem_file = config.problemFile;
    GRASP_LOG_D("problem {}", problem_file);
    ctop::parse_csv_line(problem_file, [&](int line, const std::vector<std::string>& vals){
        if(!vals.empty()) {
            GRASP_ASSERT(vals.size() == 6);
            auto b = ctop::Brick::from_string_vector(wall, vals);
            bricks[b.id] = b;
        }
    });

    ctop::WallSolution solution;

    auto types = ctop::split(line, ";");
    for(auto&& type : types) {
        if(type.empty()) continue;
        auto vals = ctop::split(type, ":");
        GRASP_ASSERT(vals.size() == 2);

        auto& tp = vals[0];
        std::map<uint64_t, std::vector<uint64_t>>* saving_map = nullptr;
        if(tp == "RESULT_TARGET_IDS") {
            saving_map = &solution.target_brick_ids;
        } else if(tp == "RESULT_START_TIMES") {
            saving_map = &solution.start_brick_times;
        } else if(tp == "RESULT_TARGET_DURATIONS") {
            saving_map = &solution.target_durations;
        } else if(tp == "BUDGET"){
            auto budget = string_to<int>(vals[1]);
            solution.budget = budget;
            continue;
        } else {
            continue;
        }
        GRASP_ASSERT(saving_map != nullptr);

        auto robots_ids = ctop::split(vals[1], ",");
        auto robot_id = 0;
        for(auto&& robot_value : robots_ids) {
            auto robots_values = ctop::split(robot_value, "|");
            auto& save_arr = (*saving_map)[robot_id];
            for(auto&& vv : robots_values) {
                auto v = string_to<int>(vv);
                save_arr.push_back(v);
            }
            robot_id++;
        }
    }

    auto max_time = 0;
    for(auto&& a : solution.target_brick_ids) {
        auto r_id = a.first;
        auto &placement_bricks = solution.target_brick_ids[r_id];
        auto total_bricks = placement_bricks.size();
        auto &start_times = solution.start_brick_times[r_id];
        auto &durs = solution.target_durations[r_id];
        auto prev_time = 0;
        for (int i = 0; i != total_bricks; i++) {
            auto brick_id = placement_bricks[i];
            if (brick_id == 0) continue;
            auto start_time = start_times[i];
            auto duration = durs[i];
            auto diff_start = start_time - prev_time;
            prev_time = start_time + duration;
            if(prev_time > max_time) {
                max_time = prev_time;
            }
        }
    }
    solution.end_time = max_time;

    off << R"(\begin{tikzpicture}[xscale=1,transform shape])" << "\n";

    off << "\\draw [-latex](-0.5,0) coordinate(dd)-- (0,0) coordinate (O1) -- ("
    << fmt::format("{:.2f}", (solution.budget+10)/10.0)
    << ",0)coordinate(ff) node[above]{$t[s]$};" << "\n";
    //off << R"(\draw [dashed,thick] (O1) -- (0,1) coordinate(S3) -- (0,3) coordinate(S2) -- (0,5) coordinate(S1) -- ++(0,1)coordinate(ff2);)" << "\n";
    off << R"(\draw [dashed,thick] (O1) )" << "\n";

    std::map<uint64_t, std::string> robot_names;
    int k_y_max = solution.target_brick_ids.size();
    int k_y = k_y_max-1;
    for(auto&& a : solution.target_brick_ids) {
        auto r_id = a.first;
        auto nm = fmt::format("R{}", r_id);
        robot_names[r_id] = nm;

        auto idx_y = 2*k_y+1;
        off << "-- (0, "<<idx_y<<") coordinate(" << nm << ") ";
        k_y--;
    }
    off << "-- ++(0," << 2*k_y_max-0.3 << ")coordinate(ff2);" << "\n";

    std::ostringstream oss_robots{};
    int k = 0;
    for(auto& names : robot_names) {
        oss_robots << names.second;
        k++;
        if(k != robot_names.size()) {
            oss_robots << ",";
        }
    }

    off << R"(\foreach \nn in{)" << oss_robots.str() << R"(}{)" << "\n";
    off << R"(    \draw [thick] (dd|-\nn) node[above]{\nn}-- (\nn-|ff);)" << "\n";
    off << R"(})" << "\n";

    auto end_coor = fmt::format("{}", solution.budget/10);
    auto end_coor_title = fmt::format("{}", solution.budget);

    off << "\\foreach \\xx in{1,2,...," << end_coor << "}{" << "\n";
    off << R"(    \draw[dashed] (\xx,0) -- (\xx,0|- ff2);)" << "\n";
    off << R"(})" << "\n";

    off << "    \\draw[thick, color=red] (" << end_coor << ",0|- ff2) -- (" << end_coor << ",0) node[below]{$T_{max}=" << end_coor_title << "$};" << "\n";


    off << "\\foreach \\xx in{0,4,8,...," << end_coor << "}{" << "\n";
    off << R"(  \ifthenelse{\xx=0}{)" << "\n";
    off << R"(      \draw[dashed] (\xx,0.2) -- (\xx,-0.2) node[below]{\xx};)" << "\n";
    off << R"(  }{)" << "\n";
    off << R"(      \draw[dashed] (\xx,0.2) -- (\xx,-0.2) node[below]{\xx 0};)" << "\n";
    off << R"(  })" << "\n";
    off << R"(})" << "\n";

    for(auto&& a : solution.target_brick_ids) {
        auto r_id = a.first;

        auto& name = robot_names[r_id];

        off << R"(\begin{scope}[shift={()" << name << R"()}])" << "\n";
        auto& placement_bricks = solution.target_brick_ids[r_id];
        auto total_bricks = placement_bricks.size();

        auto& start_times = solution.start_brick_times[r_id];
        auto& durs = solution.target_durations[r_id];

        auto prev_time = 0;
        std::string prev_b_name;
        for(int i = 0; i != total_bricks; i++) {
            auto brick_id = placement_bricks[i];
            if(brick_id == 0) continue;
            auto start_time = start_times[i];
            auto duration = durs[i];

            auto diff_start = start_time - prev_time;

            prev_time = start_time + duration;

            auto& brick_name = wall.brick_ids_map_reversed[bricks[brick_id].type];
            std::string color_name;

            // colors must be defined inside latex project eg.
            // \definecolor{brick_red_color}{rgb}{0.973, 0.808, 0.80}
            // \definecolor{brick_blue_color}{rgb}{0.855, 0.91, 0.988}
            // \definecolor{brick_green_color}{rgb}{0.835, 0.91, 0.831}
            // \definecolor{brick_orange_color}{rgb}{1.0,0.902,0.8}

            if(brick_name == "RED_BRICK") {
                color_name = "brick_red_color";
            } else if(brick_name == "GREEN_BRICK") {
                color_name = "brick_green_color";
            } else if(brick_name == "BLUE_BRICK") {
                color_name = "brick_blue_color";
            } else if(brick_name == "ORANGE_BRICK") {
                color_name = "brick_orange_color";
            } else {
                color_name = "white";
            }

            auto b_name = fmt::format("b{}{}", r_id, brick_id);
            if(prev_b_name.empty()) {
                off << R"(\coordinate()" << name << ") at (" << fmt::format("{:.2f}", start_time/10.0) << ",0.6);" << "\n";
                off << "\\node[ "
                    << "right=0cm and 0cm of " << name;
            } else {
                off << "\\node[ "
                    << "right=" << fmt::format("{:.2f}", diff_start/10.0) << "cm"
                    << " of "<< prev_b_name;
            }
            off<< ", right,draw, minimum "
               << "width=" << fmt::format("{:.2f}", duration/10.0) << "cm"
               << ",minimum height=1.2cm,fill=" << color_name << "!60](" << b_name <<") "
               << "{$\\mathbf{" << brick_id << "}$};" << "\n";
            prev_b_name = b_name;
        }

        off << R"(\end{scope})" << "\n";
    }

    off << R"(\end{tikzpicture})" << "\n";

    std::cout << solution.print_res() << std::endl;
    return 0;
}
