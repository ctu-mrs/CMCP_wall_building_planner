/*
 * VNSDOP.h
 *
 *  Created on: 22. 3. 2016
 *      Author: Robert Penicka
 */

#ifndef SRC_ILP_CTOP_ALG_SOLVER_H_
#define SRC_ILP_CTOP_ALG_SOLVER_H_

#include <vector>
#include <list>
#include <cfloat>
#include <limits>
#include <cassert>
#include <unordered_set>

#include <yaml-cpp/yaml.h>

#include <boost/foreach.hpp>

#include "crl/config.h"
#include "crl/loader/text_format_loader.h"
#include "crl/random.h"
#include "crl/timerN.h"
#include "crl/perf_timer.h"
#include "crl/logging.h"
#include "crl/stringconversions.h"
#include "crl/loader/nodeloader.h"
#include "crl/file_utils.h"
#include "crl/assert.h"
#include "crl/alg/result_log.h"
#include <ilcplex/ilocplex.h>

#include <ctop/defines.h>
#include <ctop/heuristic_types.h>
#include <ctop/IlpCtopDefinition.h>
#include <ctop/IlpCtopSolver.h>
#include "ctop/util/math_common.h"
#include <ctop/loader/dataset_loader_wall.h>
#include <ctop/heuristic/SolutionCTOP.h>
#include "ctop/util/reverse_map.h"
#include "ctop/heuristic/PrecedenceRule.h"

namespace ctop {

#define ILP_CTOP_ALG_SOLVER_NAME "ilp_ctop"

class IlpCtopAlgSolver {
	typedef std::vector<int> IntVector;

public:
    int iter;
    int batchNbr;
    bool batchContinue;
    bool saveResults;
    bool saveVisual;
    bool verboseLog;
    std::string output;
    std::string citiesFilename;
    std::string resultCanvasOutput;
    std::string name;
    crl::CResultLog resultLog;

    crl::CConfig config;
	IlpCtopAlgSolver(crl::CConfig &config, const std::string &problemFile);
	virtual ~IlpCtopAlgSolver();
	//Tour find(DatasetOP dataset);
	//Tour find(std::vector<GraphNode> nodes, double budget, int startIndex_, int goalIndex_);

	//std::vector<State> getPathSampled(std::vector<GraphNode> finalPath, double sampled_path_distance);
	//CAlgorithm functions
	std::string getMethod();
	void drawPath(int usleepTime = 0, std::vector<IndexSOP> *toShow = nullptr);

    static crl::CConfig& getConfig(crl::CConfig &config);
    static std::string getName();
    void solve();
    void appendToLog();
    std::string getOutputPath(const std::string filename, std::string &dir);
protected:
    std::string getOutputIterPath(const std::string filename, std::string &dir);
	void load();
	void initialize();
	void after_init();
	void iterate(int step);

	void my_problem_definition(IloEnv &env, IloModel &model);

	void save();
	void release();
	void visualize();
	void defineResultLog();
	void fillResultRecord(
	        int num_iters,
	        double length,
	        double reward,
	        int optimal_solution,
	        double gap_prec,
	        int num_iters_last_improvement,
			long timeLastImprovement,
			const std::vector<ImprovementLogRecord>& improvement_log, long time);

	const bool SAVE_RESULTS;
	const bool SAVE_SETTINGS;
	const bool SAVE_INFO;

private:

	void loadYamlConfig(const std::string& file);
	void parseBrickYamlParam(YAML::Node& config, const std::string& param, std::map<std::string, int>& map_to_fill);
	void parseBrickYamlParam(YAML::Node& config, const std::string& param, std::map<std::string, std::vector<int>>& map_to_fill);

	template<typename T>
	void parseBrickYamlParam(YAML::Node &config, const std::string& param, std::vector<T> &vector_to_fill)
    {
        auto read = config[param.c_str()].as<std::vector<T>>();
        vector_to_fill.insert(vector_to_fill.end(), read.begin(), read.end());
    }

    double getTourLength(std::vector<IndexSOP> indexedSolution);
	void checkSolution(std::vector<SingleSolutionCTOP> solution);

	//double getTourLength(std::vector<GraphNode> tour);
	double getTourReward(std::vector<GraphNode> tour);

	//default params
	std::vector<std::vector<double>> allDistances;
	std::vector<GraphNode> nodesAll;

	int budget_s;
	int num_robots;
	std::vector<int> start_indexes;
	int goal_index;
	std::vector<PrecedenceRule> precendence_rules;
	int min_distance_concurrent_brick_placing_cm;
	bool entire_layer_first;
    bool minimize_total_time;

	double maximalRewardAll;
	//std::vector<SingleSolutionCTOP> finalTourOP;
	SolutionCTOP ctop_solution;

	//cplex settings
	int num_threads;
	int maximal_memory_MB;
	int maximal_calculation_time_sec;
    double stop_gap;

	std::string yaml_config_file;
    std::vector<std::string> brick_types;
	std::map<std::string, int> brick_ids_map;
	std::map<int, std::string> brick_ids_map_reversed;
	std::map<std::string, int> brick_rewards_map;
	std::map<std::string, int> brick_durations_map;
	std::map<std::string, int> brick_num_robot_requirements_map;
	std::map<std::string, int> brick_reservoir_time_from_wall_s_map;
	std::map<std::string, std::vector<int>> brick_size_length_depth_height_cm_map;

    std::vector<PrecedenceRule> concurrence_exclusion_rules;

	std::vector<short int> builded_brick_ids;
	std::vector<short int> current_brick_ids;
	std::vector<short int> times_finish_actual_s;

	std::shared_ptr<IlpCtopSolver> solver;

    typedef crl::CTimerN Timer;

    Timer tLoad;
    Timer tInit;
    Timer tSolve;
    Timer tSave;
    Timer tRelease;
};

}
#endif /* SRC_ILP_CTOP_ALG_SOLVER_H_ */
