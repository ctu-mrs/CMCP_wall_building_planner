//
// Created by Michal Němec on 12/9/19.
//

#ifndef CTOP_PLANNER_DATASETSOP_H
#define CTOP_PLANNER_DATASETSOP_H

#include <vector>
#include <map>
#include <string>

#include "GraphNode.h"
#include <ctop/heuristic/PrecedenceRule.h>
#include "EdgeTypes.h"

namespace ctop {

struct DatasetSOP {
    std::vector<GraphNode> nodesAll;
    std::vector<std::vector<double>> distance_matrix;
    bool is_with_nodes;
    std::string name;
    std::string type;
    std::string comment;
    TSP_EDGE_WEIGHT_TYPE edge_weight_type;
    int dimension;
    int goalID;
    int startID;
    std::vector<int> start_indexes;
    //unsigned int num_robots;
    //double Tmax; //= available time budget per path
    std::vector<PrecedenceRule> precendence_rules;

    /*
    void set_params(
            int num_robots,
            const std::map<std::string, int>& brick_ids_map,
            const std::map<int, std::string>& brick_ids_map_reversed,
            const std::map<std::string, int>& brick_rewards_map,
            const std::map<std::string, int>& brick_durations_map,
            const std::map<std::string, int>& brick_num_robot_requirements_map)
    {
        std::cout << "set_params" << std::endl;
        std::cout << "nodesAll.size() " << nodesAll.size() << std::endl;

        for (int var = 0; var < this->nodesAll.size(); ++var) {
            std::string brick_type = brick_ids_map_reversed[nodesAll[var].type];
            std::cout << "brick_type " << brick_type << std::endl;
            nodesAll[var].reward = brick_rewards_map[brick_type];
            nodesAll[var].node_duration = brick_durations_map[brick_type];
            nodesAll[var].robot_requirement = brick_num_robot_requirements_map[brick_type];

            //if (nodesAll[var].type == 0) {
            //	nodesAll[var].robot_requirement = num_robots;
            //}
        }
    }
    */

    DatasetSOP get_updated_problem(
            int num_robots,
            const std::map<int, std::string>& brick_types_map_reversed,
            const std::map<std::string, int>& brick_types_rewards_map,
            const std::map<std::string, int>& brick_types_durations_map,
            const std::map<std::string, int>& brick_num_robot_requirements_map,
            const std::map<std::string, int>& brick_reservoir_time_from_wall_s_map,
            const std::vector<short int>& times_finish_actual_s,
            const std::vector<short int>& current_brick_ids);

private:
    void set_params_end(DatasetSOP& new_problem, int num_robots);

    void set_params_begin(
            DatasetSOP& new_problem,
            const std::map<int, std::string>& brick_types_map_reversed,
            const std::map<std::string, int>& brick_types_rewards_map,
            const std::map<std::string, int>& brick_types_durations_map,
            const std::map<std::string, int>& brick_num_robot_requirements_map);

    void reset_indexed(DatasetSOP& new_problem, int num_robots);

    void set_durations(DatasetSOP& new_problem,
                       int num_robots,
                       const std::map<int, std::string>& brick_types_map_reversed,
                       const std::vector<short int>& times_finish_actual_s,
                       const std::vector<short int>& current_brick_ids);

    void renumber_nodes(DatasetSOP& new_problem);

    void resize_distance_matrix(DatasetSOP& new_problem);

    void set_distances_reservoir(DatasetSOP& new_problem,
                                 const std::map<int, std::string>& brick_types_map_reversed,
                                 const std::map<std::string, int>& brick_reservoir_time_from_wall_s_map);

    void set_distance_start(DatasetSOP& new_problem,
                            int num_robots,
                            const std::map<int, std::string>& brick_types_map_reversed,
                            const std::map<std::string, int>& brick_reservoir_time_from_wall_s_map,
                            const std::vector<short int>& times_finish_actual_s);

};

}

#endif //CTOP_PLANNER_DATASETSOP_H
