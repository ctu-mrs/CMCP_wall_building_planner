//
// Created by Michal Němec on 12/10/19.
//

#include "SolutionCTOP.h"

using namespace ctop;

std::string ctop::ctop_status_str(SolutionStatus status)
{
    switch(status) {
        case SolutionStatus::Feasible: return "Feasible";
        case SolutionStatus::Optimal: return "Optimal";
        case SolutionStatus::Infeasible: return "Infeasible";
        case SolutionStatus::Unbounded: return "Unbounded";
        case SolutionStatus::InfeasibleOrUnbounded: return "InfeasibleOrUnbounded";
        case SolutionStatus::Error: return "Error";
        case SolutionStatus::Unknown:
        default:
            return "Unknown";
    }
}

SolutionStatus ctop::ilo2ctop_status(IloAlgorithm::Status status)
{
    switch(status) {
        case IloAlgorithm::Feasible: return SolutionStatus::Feasible;
        case IloAlgorithm::Optimal: return SolutionStatus::Optimal;
        case IloAlgorithm::Infeasible: return SolutionStatus::Infeasible;
        case IloAlgorithm::Unbounded: return SolutionStatus::Unbounded;
        case IloAlgorithm::InfeasibleOrUnbounded: return SolutionStatus::InfeasibleOrUnbounded;
        case IloAlgorithm::Error: return SolutionStatus::Error;
        case IloAlgorithm::Unknown:
        default:
            return SolutionStatus::Unknown;
    }
}