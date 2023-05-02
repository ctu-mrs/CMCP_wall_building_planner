/*
 * ilp_ctop_solver.h
 *
 *  Created on: Sep 17, 2019
 *      Author: robert
 */

#ifndef SRC_ILP_CTOP_SOLVER_H_
#define SRC_ILP_CTOP_SOLVER_H_

#include <vector>
#include <map>

#include "defines.h"
#include "heuristic_types.h"
#include "IlpCtopDefinition.h"

namespace ctop {

class IlpCtopSolver {
	//default params
	std::vector<std::vector<double>> allDistances;
	std::vector<GraphNode> nodesAll;
	double budget;
	int num_robots;
	std::vector<int> start_indexes;
	int goal_index;

	bool minimize_total_time = false;

	std::vector<PrecedenceRule> precendence_rules;
	std::vector<PrecedenceRule> concurrence_exclusion_rules;

	std::vector<short int> current_brick_ids;
	std::vector<short int> builded_brick_ids;

	//cplex config
	int num_threads;
	int maximal_memory_MB;
	int maximal_calculation_time_sec;

	std::map<std::string, int> brick_ids_map;
	std::map<std::string, int> brick_rewards_map;
	std::map<std::string, int> brick_durations_map;
	std::map<std::string, int> brick_num_robot_requirements_map;

	double stop_gap;

    void apply_params(IloCplex& cplex);

public:
	IlpCtopSolver(
            const std::vector<std::vector<double>>& allDistances,
            const std::vector<GraphNode>& nodesAll,
            double budget,
            int num_robots,
            const std::vector<int>& start_indexes,
            int goal_index,
            const std::vector<PrecedenceRule>& precendence_rules,
            const std::vector<PrecedenceRule>& concurrence_exclusion_rules,
            const std::vector<short int>& current_brick_ids,
            const std::vector<short int>& builded_brick_ids,
            int num_threads,
            int maximal_memory_MB,
            int maximal_calculation_time_sec,
            bool minimize_total_time,
			double stop_gap);

	virtual ~IlpCtopSolver();

	SolutionCTOP solve();

	void set_params(const std::map<std::string, int>& brick_ids_map_,
                    const std::map<std::string, int>& brick_rewards_map_,
                    const std::map<std::string, int>& brick_durations_map,
                    const std::map<std::string, int>& brick_num_robot_requirements_map);

	void export_lp_definition(const IloCplex& cplex, std::string filename);
};

} /* namespace op */

#endif /* SRC_ILP_CTOP_SOLVER_H_ */
