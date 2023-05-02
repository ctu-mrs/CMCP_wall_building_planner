/*
 * VNSDOP.cpp
 *
 *  Created on: 22. 3. 2016
 *      Author: Robert Penicka
 */

#include "crl/alg/text_result_log.h"
#include "ilp_ctop_alg_solver.h"

#define MAX_FLOAT (std::numeric_limits<double>::max())

using crl::logger;

using namespace ctop;
using namespace crl;

#define TIME_PRECISION 0.001

#define MIN_CHANGE_EPS 0.00001
#define DD " , "
#define DEBUG_DOP_TRY_OPERATIONS false

#define NUMBER_PRECISION 9

IlpCtopAlgSolver::IlpCtopAlgSolver(
        crl::CConfig &config,
        const std::string &problemFile
)
:   config(config),
    SAVE_SETTINGS(config.get<bool>("save-settings")),
    SAVE_RESULTS(config.get<bool>("save-results")),
    SAVE_INFO(config.get<bool>("save-info")),
    yaml_config_file(config.get<std::string>("config-file"))
{

    iter = config.get<int>("iteration");
    batchNbr = config.get<int>("batch");
    batchContinue = config.get<bool>("continue");
    saveResults = config.get<bool>("save-results");
    output = config.get<std::string>("output");


    saveVisual = config.get<bool>("save-visual");
    verboseLog = config.get<bool>("verbose-result-log");

	assert(!yaml_config_file.empty());

	loadYamlConfig(yaml_config_file);

	DatasetSOP problem = DatasetLoaderWall::loadDataset(problemFile);

	//problem.set_params(num_robots, brick_ids_map, brick_ids_map_reversed, brick_rewards_map, brick_durations_map, brick_num_robot_requirements_map);
	//CTOPLoader::getSOPDefinition(config, problemFile, true);

	precendence_rules = problem.precendence_rules;

	for (int var = 0; var < current_brick_ids.size(); ++var) {
		INFO("current_brick_ids[" << var << "] " << current_brick_ids[var]);
		INFO("times_finish_actual_s[" << var << "] " << times_finish_actual_s[var])
	}

	DatasetSOP updated_problem = problem.get_updated_problem(
	        num_robots,
	        brick_ids_map_reversed,
	        brick_rewards_map,
	        brick_durations_map,
	        brick_num_robot_requirements_map,
	        brick_reservoir_time_from_wall_s_map,
	        times_finish_actual_s,
	        current_brick_ids);

	nodesAll = updated_problem.nodesAll;
	allDistances = updated_problem.distance_matrix;
	goal_index = updated_problem.goalID;
	start_indexes = updated_problem.start_indexes;

	//here starts change to starts using num robots

	INFO("after setting params");
	for (int var = 0; var < nodesAll.size(); ++var) {
		auto& node = nodesAll[var];
		INFO(var << " node " << node.id << " duration " << node.node_duration << " robot requirement " << node.robot_requirement);
	}

	//allDistances = problem.distance_matrix;

	maximalRewardAll = 0;
	for (auto & node : nodesAll) {
		maximalRewardAll += node.reward;
	}

	for (int var = 0; var < start_indexes.size(); ++var) {
		INFO("start_indexes[" << var << "] " << start_indexes[var]);
	}

	INFO_VAR(goal_index)

	num_threads = config.get<int>("num-threads");
	maximal_memory_MB = config.get<int>("maximal-memory-MB");
	maximal_calculation_time_sec = config.get<int>("maximal-calculation-time-sec");
    stop_gap = config.get<double>("stop-gap");

    INFO("using budget:" << budget_s);
	INFO("using num_threads:" << num_threads);
	INFO("using maximal_memory_MB:" << maximal_memory_MB);
	INFO("using maximal_calculation_time_sec:" << maximal_calculation_time_sec);
    INFO("using stop_gap:" << stop_gap);

	if (!entire_layer_first) {
		precendence_rules = DatasetLoaderWall::getPrecedenceRulesAccurate(problem, brick_size_length_depth_height_cm_map,brick_ids_map_reversed);
	}

	concurrence_exclusion_rules = DatasetLoaderWall::getNextToEachOther(problem,min_distance_concurrent_brick_placing_cm);

	bool current_brick_concurrently_excluded = DatasetLoaderWall::areCurentBricksExcluded(
	        concurrence_exclusion_rules,
	        current_brick_ids);

	if (current_brick_concurrently_excluded) {
		ERROR("can not optimize with concurrently excluded current bricks....");
		exit(1);
	}

	solver = std::make_shared<IlpCtopSolver>(
	        allDistances,
	        nodesAll,
	        budget_s,
	        num_robots,
	        start_indexes,
	        goal_index,
	        precendence_rules,
			concurrence_exclusion_rules,
			current_brick_ids,
			builded_brick_ids,
			num_threads,
			maximal_memory_MB,
			maximal_calculation_time_sec,
			minimize_total_time,
            stop_gap);

	solver->set_params(
	        brick_ids_map,
	        brick_rewards_map,
	        brick_durations_map,
	        brick_num_robot_requirements_map);

}

IlpCtopAlgSolver::~IlpCtopAlgSolver() {

}

void IlpCtopAlgSolver::loadYamlConfig(const std::string& file) {
	INFO("loading yaml file " << file);

	auto config = YAML::LoadFile(file);
	num_robots = config["num_robots"].as<int>();
	budget_s = config["time_budget_s"].as<int>();

	// add starting offset
    budget_s += 5;

	min_distance_concurrent_brick_placing_cm = config["min_distance_concurrent_brick_placing_cm"].as<int>();
	entire_layer_first = config["entire_layer_first"].as<bool>();
    minimize_total_time = config["minimize_total_time"].as<bool>();

    parseBrickYamlParam(config, "brick_types", brick_types);
    parseBrickYamlParam(config, "brick_ids", brick_ids_map);
	parseBrickYamlParam(config, "brick_rewards", brick_rewards_map);
	parseBrickYamlParam(config, "brick_durations", brick_durations_map);
	parseBrickYamlParam(config, "brick_num_robot_requirements", brick_num_robot_requirements_map);
	parseBrickYamlParam(config, "brick_reservoir_time_from_wall_s", brick_reservoir_time_from_wall_s_map);
	parseBrickYamlParam(config, "brick_size_length_depth_height_cm", brick_size_length_depth_height_cm_map);
    parseBrickYamlParam(config, "builded_brick_ids", builded_brick_ids);
    parseBrickYamlParam(config, "current_brick_ids", current_brick_ids);
    parseBrickYamlParam(config, "times_finish_actual_s", times_finish_actual_s);

	if (current_brick_ids.size() != num_robots) {
		ERROR("current_brick_ids must be of size num_robots");
		exit(1);
	}

	if (times_finish_actual_s.size() != num_robots) {
		ERROR("times_finish_actual_s must be of size num_robots");
		exit(1);
	}

	reverse_map(brick_ids_map, brick_ids_map_reversed);

	INFO("loading yaml file " << file << " end");
}

void IlpCtopAlgSolver::parseBrickYamlParam(YAML::Node &config, const std::string& param, std::map<std::string, int> &map_to_fill) {
	YAML::Node brick_ids = config[param.c_str()];
	if (brick_ids.Type() != YAML::NodeType::Null) {
		for (const auto& type : brick_types) {
		    const auto& brick_id_node = brick_ids[type];
            if (brick_id_node.Type() != YAML::NodeType::Null) {
                auto brick_id = brick_id_node.as<int>();
				map_to_fill[type] = brick_id;
				INFO("brick " << type << " has " << param << " " << brick_id);
			} else {
				INFO(param.c_str() << " for " << type << " not defined");
				exit(1);
			}
		}
	} else {
		INFO(param.c_str() << " not defined");
		exit(1);
	}
}

void IlpCtopAlgSolver::parseBrickYamlParam(YAML::Node &config, const std::string& param, std::map<std::string, std::vector<int>> &map_to_fill) {
	auto brick_ids = config[param.c_str()];
	if (brick_ids.Type() != YAML::NodeType::Null) {
		for (const auto& type : brick_types) {
			const auto& brick_id_node = brick_ids[type];
            if (brick_id_node.Type() != YAML::NodeType::Null) {
                auto& vec = map_to_fill[type];
                for(const auto& value_in_list : brick_id_node) {
                    //INFO("value_in_list " << value_in_list);
                    auto val = value_in_list.as<int>();
                    vec.push_back(val);
                    INFO("brick " << type
                                  << " has " << param
                                  << " " << vec.size()
                                  << " val " << val);
                }
			} else {
				INFO(param << " for " << type << " not defined");
				exit(1);
			}
		}
	} else {
		INFO(param << " not defined");
		exit(1);
	}
}

void IlpCtopAlgSolver::solve() {
    defineResultLog();
    tLoad.reset().start();
    load();
    tLoad.stop();
    tInit.reset().start();
    initialize();
    tInit.stop();
    after_init();
    tSolve.reset().start();
    iterate(0);
    tSolve.stop();
    tRelease.reset().start();
    release();
    tRelease.stop();
    tSave.reset().start();
    save();
    tSave.stop();
    appendToLog();

}

std::string IlpCtopAlgSolver::getOutputPath(const std::string filename,
                                      std::string &dir)
{
    if (!output.empty()) {
        dir = (output[output.size() - 1] == '/') ? output : output + "/";
        return dir + filename;
    } else {
        dir = "./";
        return filename;
    }
}


void IlpCtopAlgSolver::appendToLog()
{ // append last record
    std::string dir;
    std::string log = getOutputPath(config.get<std::string>("results"), dir);
    assert_io(createDirectory(dir), "Can not create directory '" + dir + "'");
    std::fstream out(log.c_str(), std::ios::app | std::ios::out);
    CTextLog t(resultLog);
    t.printRecord(resultLog.last(), out);
    if (verboseLog) {
        std::stringstream str;
        t.printRecord(resultLog.last(), str, false);
        INFO("result log: " << str.str());
    }
    out.close();
    if (!resultLog) {
        WARN("Error in resultLog:" << resultLog.getError());
        resultLog.clear();
    }
}

/// - protected method ---------------------------------------------------------
std::string IlpCtopAlgSolver::getOutputIterPath(const std::string filename,
                                          std::string &dir)
{
    std::string iter_sep = iter >= 0 ? crl::string_format<int>(iter) + "/" : "/";
    if (output.size() > 0) {
        dir = (output[output.size() - 1] == '/' and iter_sep[0] == '/')
              ? output + iter_sep
              : output + "/" + iter_sep;
        return dir + filename;
    } else {
        dir = iter_sep;
        return dir + filename;
    }
}

void IlpCtopAlgSolver::my_problem_definition(IloEnv &env, IloModel &model) {

}

void IlpCtopAlgSolver::iterate(int iter) {

	//INFO("start " << this->startNode.x << " " << this->startNode.y);
	//INFO("goal " << this->goalNode.x << " " << this->goalNode.y);

	int numNodesTotally = nodesAll.size();

	long comp_time[3] = { 0, 0, 0 };

	double max_reward = 0;
	for (auto& var : nodesAll) {
		max_reward += var.reward;
	}

	ctop_solution = solver->solve();

    INFO("max reward is " << max_reward);

    tSolve.stop();
	tSolve.addTime(comp_time);
	INFO("computational time ms = " << comp_time[0]);

	INFO("max reward is " << max_reward);

	drawPath();

	double final_length = 0;
	double final_reward = 0;

	std::unordered_set<int> visited_targets;
	for (auto& single_solution : ctop_solution.solution_vector) {
        for (auto& node : single_solution.node_sequence) {
			visited_targets.insert(node.node_id);
		}
	}

	for (auto& node_id : visited_targets) {
		final_reward += nodesAll[node_id].reward;
	}

	for (int solution_idx = 0; solution_idx < ctop_solution.solution_vector.size(); ++solution_idx) {
		std::stringstream ss;
		for (auto & node : ctop_solution.solution_vector[solution_idx].node_sequence) {
				ss << node.node_id << "(" << node.visit_start_time << ", " << node.brick_id << ") ";
		}
		INFO("solution " << solution_idx << " is: " << ss.str())
	}

	if (std::abs(ctop_solution.objective_value - final_reward) > ctop_solution.tolerance) {
		ERROR("the lp objective value ( " << ctop_solution.objective_value << " ) and the calculated reward " << final_reward << " are different");
		//exit(1);
	}

	if (std::abs(ctop_solution.solution_length - final_length) > ctop_solution.tolerance) {
		ERROR("the lp solution length ( " << ctop_solution.solution_length << " ) and the calculated length " << final_length << " are different");
		exit(1);
	}

//change ids of final point to original one
	INFO_CYAN("TODO uncomment change ids");

	//checkSolution(finalTourOP);
	fillResultRecord(ctop_solution.num_iters, final_length, final_reward, ctop_solution.optimal_solution, ctop_solution.gap_perc,
			ctop_solution.numItersLastImprovement, ctop_solution.timeLastImprovement, ctop_solution.improvementLog, comp_time[0]);
//fillResultRecord(act_iter, final_length, final_reward, optimal_solution);
	INFO("write result log");
    resultLog << crl::result::endrec;
	INFO("set finalTourDOP");
	INFO_GREEN("found tour with reward "<<final_reward<<" and length "<<final_length<<" out of "<<budget_s<<" budget within " <<comp_time[0] <<" ms");
}

void IlpCtopAlgSolver::checkSolution(std::vector<SingleSolutionCTOP> solution) {
	INFO_RED("TODO: implement checkSolution")
	/*
	 for (int robotidx = 0; robotidx < solution.size(); ++robotidx) {
	 //check start times
	 SingleSolutionCTOP single_solution = solution[robotidx];
	 double last_start_time = single_solution.node_sequence[0].visit_start_time;
	 int last_node = start_index;
	 for (int var = 1; var < single_solution.node_sequence.size(); ++var) {
	 int new_node = single_solution.node_sequence[var].node_id;
	 double new_start_time = single_solution.node_sequence[var].visit_start_time;
	 double &distance = allDistances[last_node][new_node];
	 if ((last_start_time + nodesAll[last_node].node_duration + distance) - new_start_time < TIME_PRECISION) {
	 //all OK
	 } else {
	 ERROR("bad sequence order not fullfilling time requirements");
	 INFO_VAR(last_start_time);
	 INFO_VAR(nodesAll[last_node].node_duration);
	 INFO_VAR(distance);
	 INFO_VAR(new_start_time);
	 INFO_VAR(last_start_time + nodesAll[last_node].node_duration + distance)
	 exit(1);
	 }
	 last_node = new_node;
	 last_start_time = new_start_time;
	 }

	 }
	 */
}

double IlpCtopAlgSolver::getTourLength(std::vector<IndexSOP> indexedSolution) {
	double distance = 0;
	for (int var = 1; var < indexedSolution.size(); ++var) {
		int ni1 = indexedSolution[var - 1].nodeIndex;
		int ni2 = indexedSolution[var].nodeIndex;
		distance += allDistances[ni1][ni2];
	}
	return distance;
}

double IlpCtopAlgSolver::getTourReward(std::vector<GraphNode> tour) {
	double price = 0;
	for (int var = 0; var < tour.size(); ++var) {
		price += tour[var].reward;
	}
	return price;
}

void IlpCtopAlgSolver::load() {

}

void IlpCtopAlgSolver::drawPath(int usleepTime, std::vector<IndexSOP> *toShow) {

}

void IlpCtopAlgSolver::initialize() {

}

void IlpCtopAlgSolver::after_init() {

}

void IlpCtopAlgSolver::save() {
	std::string dir;
    //updateResultRecordTimes(); //update timers as load and initilization is outside class
    //DEBUG("LOAD_TIME_CPU: " << tLoad.cpuTime());
    //DEBUG("INIT_TIME_CPU: " << tInit.cpuTime());
    //DEBUG("SAVE_TIME_CPU: " << tSave.cpuTime());
    //DEBUG("SOLVE_TIME_CPU: " << tSolve.cpuTime());
    if (SAVE_SETTINGS) {
        //saveSettings(getOutputIterPath(config.get<std::string>("settings"), dir));
    }
    if (SAVE_INFO) {
        //saveInfo(getOutputIterPath(config.get<std::string>("info"), dir));
    }
	if (SAVE_RESULTS) {
		std::string file = getOutputIterPath(config.get<std::string>("result-path"), dir);
		crl::assert_io(createDirectory(dir), "Can not create file in path'" + file + "'");
		std::ofstream ofs(file.c_str());
		crl::assert_io(ofs.good(), "Cannot create path '" + file + "'");
		ofs << std::setprecision(14);

		SolutionCTOP finalPath = ctop_solution;
		for (int var = 0; var < finalPath.solution_vector.size(); ++var) {
			SingleSolutionCTOP &single_solution = finalPath.solution_vector[var];
			ofs << "robot " << var << std::endl;
			for (int var2 = 0; var2 < single_solution.node_sequence.size(); ++var2) {
				ofs << " " << single_solution.node_sequence[var2].node_id << "," << single_solution.node_sequence[var2].visit_start_time << std::endl;
			}
		}

		crl::assert_io(ofs.good(), "Error occur during path saving");
		ofs.close();
	}
}

void IlpCtopAlgSolver::release() {

}

void IlpCtopAlgSolver::visualize() {

}

void IlpCtopAlgSolver::defineResultLog() {
	static bool resultLogInitialized = false;
	if (!resultLogInitialized) {
		resultLog << result::newcol << "NAME";
		resultLog << result::newcol << "METHOD";
		resultLog << result::newcol << "CTIME";
		resultLog << result::newcol << "NUM_ITERS";
		resultLog << result::newcol << "BUDGET";
		resultLog << result::newcol << "REWARDS";
		resultLog << result::newcol << "IS_OPTIMAL";
		resultLog << result::newcol << "GAP_PERCENT";
		resultLog << result::newcol << "MAX_ACHIEVABLE_REWARDS";
		resultLog << result::newcol << "LENGTH";
		resultLog << result::newcol << "NUM_ITERS_LAST_IMPR";
		resultLog << result::newcol << "CTIME_LAST_IMPR";
		resultLog << result::newcol << "MAX_ALLOWED_CALC_TIME_MS";
		resultLog << result::newcol << "RESULT_TARGET_IDS";
		resultLog << result::newcol << "RESULT_START_TIMES";
		resultLog << result::newcol << "RESULT_TARGET_DURATIONS";
		resultLog << result::newcol << "SOLUTION_TIME_REWARD_LENGTH_IMPROVEMENTS";
		resultLog << result::newcol << "LOWER_BOUND_ABOVE_BUDGET_THEN_SKIP";
        resultLog << result::newcol << "RUNTIME";
		resultLogInitialized = true;
	}
}

void IlpCtopAlgSolver::fillResultRecord(
        int num_iters,
        double length,
        double reward,
        int optimal_solution,
        double gap_prec,
		int num_iters_last_improvement,
		long timeLastImprovement,
		const std::vector<ImprovementLogRecord>& improvement_log,
		long time) {
	long t[3] = { 0, 0, 0 };

	double final_reward = reward;

	//std::vector<SingleSolutionCTOP> finalTourOP_copy = finalTourOP;

	std::stringstream brickNodes;
	std::stringstream toursStartTimes;
	std::stringstream nodeDurations;

	for (int robot_id = 0; robot_id < ctop_solution.solution_vector.size(); ++robot_id) {
        const auto& robot_sol = ctop_solution.solution_vector[robot_id];
		if (robot_id != 0) {
			brickNodes << ",";
			toursStartTimes << ",";
			nodeDurations << ",";
		}
		for (int var = 0; var < robot_sol.node_sequence.size(); ++var) {
            const auto& node = robot_sol.node_sequence[var];
			if (var != 0) {
				brickNodes << "|";
				toursStartTimes << "|";
				nodeDurations << "|";
			}
			brickNodes << node.brick_id;
			toursStartTimes << node.visit_start_time;
			nodeDurations << node.duration;
		}

	}

	INFO("fillResultRecord:")
	INFO("tour node IDs:      " << brickNodes.str());
	INFO("tour node start times:      " << toursStartTimes.str());

	std::stringstream tourImprovements;
	for (int var = 0; var < improvement_log.size(); ++var) {
		if (var != 0) {
			tourImprovements << ",";
		}
		tourImprovements << improvement_log[var].timeMS << "|" << improvement_log[var].reward << "|" << improvement_log[var].length;
	}

	double final_length = length;

	resultLog << result::newrec << name << getMethod() << t[0] << num_iters << this->budget_s << final_reward << optimal_solution << gap_prec
              << maximalRewardAll << final_length << num_iters_last_improvement << timeLastImprovement << (maximal_calculation_time_sec * 1000)
              << brickNodes.str() << toursStartTimes.str() << nodeDurations.str() << tourImprovements.str() << false << time;
}

crl::CConfig& IlpCtopAlgSolver::getConfig(crl::CConfig &config) {
	return config;
}

std::string
IlpCtopAlgSolver::getName() {
    return ILP_CTOP_ALG_SOLVER_NAME;
}

std::string
IlpCtopAlgSolver::getMethod() {
    return getName();
}

