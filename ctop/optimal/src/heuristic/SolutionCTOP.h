//
// Created by Michal Němec on 12/9/19.
//

#ifndef CTOP_PLANNER_SOLUTIONCTOP_H
#define CTOP_PLANNER_SOLUTIONCTOP_H

#include <ilconcert/iloalg.h>
#include "SingleSolutionCTOP.h"
#include "ImprovementLogRecord.h"

namespace ctop {

enum class SolutionStatus {
    Unknown,
    Feasible,
    Optimal,
    Infeasible,
    Unbounded,
    InfeasibleOrUnbounded,
    Error
};

std::string ctop_status_str(SolutionStatus status);
SolutionStatus ilo2ctop_status(IloAlgorithm::Status status);

struct SolutionCTOP {
    SolutionStatus status = SolutionStatus::Unknown;
    std::vector<SingleSolutionCTOP> solution_vector;
    double objective_value;
    double solution_length;
    double gap_perc;
    int num_iters;
    int optimal_solution;
    int numItersLastImprovement = 0;
    long timeLastImprovement = 0;
    std::vector<ImprovementLogRecord> improvementLog;
    bool success;
    std::string message;

    double tolerance = 10e-3;
};

}

#endif //CTOP_PLANNER_SOLUTIONCTOP_H
