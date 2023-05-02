/*
 * fischetti_op_milp.h
 *
 *  Created on: Jun 28, 2017
 *      Author: penicrob
 */

#ifndef SRC_ILP_CTOP_DEFINITION_H_
#define SRC_ILP_CTOP_DEFINITION_H_

#include <limits>
#include <vector>
#include <list>
#include <cfloat>
#include <limits>
#include <unordered_map>
#include <map>
#include <set>

//#include "DatasetLoaderOP.h"
#include "defines.h"
#include <ctop/util/math_common.h>
#include "heuristic_types.h"

namespace ctop {

class IlpCtopDefinition {

    IloNumVarMatrix3D x;
    IloNumVarMatrix2D z;
    IloNumVarArray s;
    IloNumVarArray y;

    IloNum lastObjVal;
    IloNum lastIncumbentLC;
    IloNum lastTime;

    IloObjective workerObj;
    int lastImprovementIter;
    long lastImprovementTime;
    std::vector<ImprovementLogRecord> improvementLog;

    IloEnv env;
    IloModel model;
    std::vector<GraphNode> nodes_all;
    std::vector<GraphNode> unvisited_nodes;
    std::map<int, int> brick_ids__unvisited_idx;
    std::map<int, int> unvisited_idx__brick_ids;
    std::map<int, int> all_node_ids__unvisited_idx;
    std::map<int, int> unvisited_idx__all_node_ids;

    bool initial_solution_set;
    std::vector<std::vector<double>> distances;
    int num_robots;
    std::vector<int> start_indexes;
    std::set<int> start_indexes_set;
    int goalIndex;
    double budget;
    double max_distance;
    double max_node_duration;
    bool minimize_total_time;
    std::vector<PrecedenceRule> precendence_rules;
    std::vector<PrecedenceRule> precendence_rules_applyable;
    std::vector<PrecedenceRule> concurrence_exclusion_rules;
    std::vector<PrecedenceRule> concurrence_exclusion_rules_applyable;
    std::vector<short int> current_brick_ids;
    std::vector<short int> builded_brick_ids;

    static void printAllSubsets(std::list<std::list<int>> &list_to_fill);
    static void getAllSubsets(int n, std::list<std::list<int>> &list_to_fill);

    void find_max_values();

    void init_start_index_set();
    void init_applyable_rules();
    void init_unvisited_ids();

    void add_variable_y_sop();
    void add_variable_x_z();

    void constraint_start_end();
    void constraint_x_flow();
    void constraint_robot_number();
    void constraint_z_and_x();
    void constraint_start_times();
    void constraint_visit_precedence();
    void constraint_side_by_side();

public:
	IlpCtopDefinition(
            IloEnv& env_,
            IloModel& model_,
            const std::vector<GraphNode>& nodes_all_,
            const std::vector<std::vector<double>>& distances_,
            const std::vector<int>& start_indexes_,
            int goalIndex_,
            double budget_,
            int num_robots_,
            const std::vector<PrecedenceRule>& precendence_rules,
            const std::vector<PrecedenceRule>& concurrence_exclusion_rules,
            const std::vector<short int>& current_brick_ids_,
            const std::vector<short int>& builded_brick_ids_,
            bool minimize_total_time_);

	virtual ~IlpCtopDefinition();
	//void adjustPriorities(IloCplex & cplex);
	std::vector<SingleSolutionCTOP> parseSolutionVector(IloCplex &cplex);
	//void setInitialSolution(IloCplex & cplex, std::vector<IndexSOP> initialSolution);
	int getLastImprovementIter();
	void setLastImprovementIter(int lastImprovementIter_);
	long getLastImprovementTime();
	void setLastImprovementTime(long lastImprovementTime_);
	std::vector<ImprovementLogRecord> getImprovementLog();
	void setImprovementLog(std::vector<ImprovementLogRecord> improvementLog_);

	IloInt numAddedConstraints;
};

} /* namespace op */

#endif /* SRC_ILP_CTOP_DEFINITION_H_ */
