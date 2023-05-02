/*
 * ilp_ctop_solver.cpp
 *
 *  Created on: Sep 17, 2019
 *      Author: robert
 */

#include "IlpCtopSolver.h"
#include <ctop/util/timestamp.h>

using namespace ctop;

IlpCtopSolver::IlpCtopSolver(
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
        double stop_gap)
:   allDistances(allDistances),
    nodesAll(nodesAll),
    budget(budget),
    goal_index(goal_index),
    num_robots(num_robots),
    start_indexes(start_indexes),
    precendence_rules(precendence_rules),
    concurrence_exclusion_rules(concurrence_exclusion_rules),
    builded_brick_ids(builded_brick_ids),
    current_brick_ids(current_brick_ids),
    num_threads(num_threads),
    maximal_memory_MB(maximal_memory_MB),
    maximal_calculation_time_sec(maximal_calculation_time_sec),
    minimize_total_time(minimize_total_time),
    stop_gap(stop_gap)
{

}

IlpCtopSolver::~IlpCtopSolver() = default;

void IlpCtopSolver::set_params(
        const std::map<std::string, int>& brick_ids_map_,
        const std::map<std::string, int>& brick_rewards_map_,
        const std::map<std::string, int>& brick_durations_map_,
        const std::map<std::string, int>& brick_num_robot_requirements_map_) {
	brick_ids_map = brick_ids_map_;
	brick_rewards_map = brick_rewards_map_;
	brick_durations_map = brick_durations_map_;
	brick_num_robot_requirements_map = brick_num_robot_requirements_map_;
}

SolutionCTOP IlpCtopSolver::solve() {
	SolutionCTOP solution;
	solution.success = false;
	solution.message = std::string("solver exception");

	int numNodesTotally = nodesAll.size();
	double objective_value = 0;
	double lp_solution_length = 0;
	int act_iter = 0;
	int numItersLastImprovement = 0;
	long timeLastImprovement = 0;
	int optimal_solution = 0;
	double gap_perc = -1;

	std::vector<ImprovementLogRecord> improvementLog;

	IloEnv env;
	try {
		IloModel model(env);

		INFO("dataset with " << nodesAll.size() << " clusters");
		INFO("dataset with total number of nodes "<<numNodesTotally);
		for (int start_index : start_indexes) {
			INFO("startID " << start_index);
		}

		INFO("goalID " <<goal_index);
		//int num_nodes = 10;

		auto gen_start = std::chrono::high_resolution_clock::now();
        INFO("generating cplex definitions");

        //op::IlpCtopDefinition *my_problem_definition;
		//in preparation for alternative definitions
		ctop::IlpCtopDefinition my_problem_definition(
		        env,
		        model,
		        nodesAll,
		        allDistances,
		        start_indexes,
		        goal_index,
		        budget,
		        num_robots,
				precendence_rules,
				concurrence_exclusion_rules,
				current_brick_ids,
				builded_brick_ids,
				minimize_total_time);
        auto gen_end = std::chrono::high_resolution_clock::now();
        INFO("generation IloCplex took " << ctop::duration_to_string(gen_end - gen_start));

        auto mod_start = std::chrono::high_resolution_clock::now();
        INFO("creating IloCplex model");
		IloCplex cplex(model);
        auto mod_end = std::chrono::high_resolution_clock::now();
        INFO("cplex created took "<< ctop::duration_to_string(mod_end - mod_start));
        apply_params(cplex);
        cplex.exportModel("ctop-model.lp");
		if (!cplex.solve()) {
			ERROR("Failed to optimize LP");
			return solution;
		}

		objective_value = cplex.getObjValue();

		numItersLastImprovement = my_problem_definition.getLastImprovementIter();
		timeLastImprovement = my_problem_definition.getLastImprovementTime();
		improvementLog = my_problem_definition.getImprovementLog();

		act_iter = cplex.getNiterations();
		if (cplex.getStatus() == IloAlgorithm::Status::Optimal) {
			optimal_solution = 1;
			gap_perc = cplex.getMIPRelativeGap() * 100.0;
		} else {
			gap_perc = cplex.getMIPRelativeGap() * 100.0;
		}

		INFO("Solution status = " << cplex.getStatus());
		INFO("optimal_solution "<< optimal_solution);
		INFO("Gap value  = " << cplex.getMIPRelativeGap());
		INFO("Solution value  = " << cplex.getObjValue());

		// in version 12.6.1 works
		// version 12.10 throws IloAlgorithm::CannotExtractException
		//INFO("Maximum bound violation = " << cplex.getQuality(IloCplex::Quality::DualObj));

		INFO("solution is:");

		//change
		solution.status = ilo2ctop_status(cplex.getStatus());
		solution.solution_vector = my_problem_definition.parseSolutionVector(cplex);
		solution.objective_value = objective_value;
		solution.solution_length = 0;
		solution.gap_perc = gap_perc;
		solution.num_iters = act_iter;
		solution.optimal_solution = optimal_solution;
		solution.numItersLastImprovement = numItersLastImprovement;
		solution.timeLastImprovement = timeLastImprovement;
		solution.improvementLog = improvementLog;
		solution.success = true;

		std::stringstream ss;
		ss << "solution found with reward " << objective_value << " and gap " << gap_perc << "%";
		solution.message = ss.str();
	} catch (IloAlgorithm::CannotExtractException &e) {
		ERROR("CannoExtractException: " << e);
		IloExtractableArray failed = e.getExtractables();
		for (IloInt i = 0; i < failed.getSize(); ++i) {
			ERROR("\t" << failed[i]);
			// Handle exception ...
		}
	} catch (IloException &e) {
		ERROR("Concert exception caught: " << e);
	} catch (const std::exception &e) {
		ERROR("Error " << e.what());
	} catch (const std::string &e) {
		ERROR("Error " << e);
	} catch (...) {
		ERROR("Unknown exception caught");
	}

	env.end();

	return solution;
}

void IlpCtopSolver::apply_params(IloCplex& cplex) {
    INFO("set num threads to " << num_threads);
    cplex.setParam(IloCplex::Param::Threads, num_threads);

    if(stop_gap >= 0.0) {
        cplex.setParam(IloCplex::EpGap, stop_gap);
    }

    //0 	No display until optimal solution has been found
    //1 	Display integer feasible solutions
    //2 	Display integer feasible solutions plus an entry at a frequency set by MIP node log interval; default
    //3 	Display the number of cuts added since previous display; information about the processing of each successful MIP start; elapsed time in seconds and elapsed time in deterministic ticks for integer feasible solutions
    //4 	Display information available from previous options and information about the LP subproblem at root
    //5 	Display information available from previous options and information about the LP subproblems at root and at nodes
    //cplex.setParam(IloCplex::MIPDisplay, 2);

    //0 	Automatic: let CPLEX choose; default
    //1 	Moderate probing level
    //2 	Aggressive probing level
    //3 	Very aggressive probing level
    //cplex.setParam(IloCplex::Probe, 1);

    //cplex.setOut(logger);
    //default
    //cplex.setParam(IloCplex::Param::RootAlgorithm, IloCplex::AutoAlg);

    //primal simplex
    //cplex.setParam(IloCplex::Param::RootAlgorithm, IloCplex::Primal);

    //dual   simplex
    //cplex.setParam(IloCplex::Param::RootAlgorithm, IloCplex::Dual);

    //barrier without crossover
    //cplex.setParam(IloCplex::Param::RootAlgorithm, IloCplex::Barrier);
    //cplex.setParam(IloCplex::Param::Barrier::Crossover, IloCplex::NoAlg);

    //barrier with crossover
    //cplex.setParam(IloCplex::Param::RootAlgorithm, IloCplex::Barrier);

    //network with dual simplex cleanup
    //cplex.setParam(IloCplex::Param::RootAlgorithm, IloCplex::Network);

    //sifting
    //cplex.setParam(IloCplex::Param::RootAlgorithm, IloCplex::Sifting);

    //concurrent
    //cplex.setParam(IloCplex::Param::RootAlgorithm, IloCplex::Concurrent);

    //custom logging and listing of incumbent solution

    //cplex.setOut(logger);

    //exit(1);

    // Turn off CPLEX logging
    //cplex.setParam(IloCplex::Param::MIP::Display, 0);

    int automatic = IloCplex::MIPsearch::AutoSearch;
    int branch_and_cut = IloCplex::MIPsearch::Traditional;
    cplex.setParam(IloCplex::MIPSearch, automatic);

    int node_file_on_disk = 3;		//Node file on disk and compressed
    cplex.setParam(IloCplex::MemoryEmphasis, true);	//Directs CPLEX that it should conserve memory where possible.
    cplex.setParam(IloCplex::WorkMem, maximal_memory_MB*3); //maximal working memory
    cplex.setParam(IloCplex::NodeFileInd, node_file_on_disk);
    cplex.setParam(IloCplex::TiLim, maximal_calculation_time_sec);
}

void IlpCtopSolver::export_lp_definition(const IloCplex& cplex, std::string filename) {
    //std::string dir;
    //std::string file = getOutputIterPath(config.get < std::string > ("lp-model-file"), dir);
    //INFO("saving file to "<<file.c_str());
    //imr::assert_io(createDirectory(dir), "Can not create file in path'" + file + "'");

    cplex.exportModel(filename.c_str());
}
