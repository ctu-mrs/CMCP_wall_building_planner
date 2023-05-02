//
// Created by Michal Němec on 12/9/19.
//

#include <cassert>
#include "DatasetSOP.h"

using namespace ctop;

DatasetSOP
DatasetSOP::get_updated_problem(
        int num_robots,
        const std::map<int, std::string>& brick_types_map_reversed,
        const std::map<std::string, int>& brick_types_rewards_map,
        const std::map<std::string, int>& brick_types_durations_map,
        const std::map<std::string, int>& brick_num_robot_requirements_map,
        const std::map<std::string, int>& brick_reservoir_time_from_wall_s_map,
        const std::vector<short int>& times_finish_actual_s,
        const std::vector<short int>& current_brick_ids)
{
    //make copy
    DatasetSOP new_problem = *this;

    set_params_begin(new_problem, brick_types_map_reversed, brick_types_rewards_map, brick_types_durations_map, brick_num_robot_requirements_map);
    set_params_end(new_problem, num_robots);
    reset_indexed(new_problem, num_robots);
    set_durations(new_problem, num_robots, brick_types_map_reversed, times_finish_actual_s, current_brick_ids);
    renumber_nodes(new_problem);

    resize_distance_matrix(new_problem);
    set_distances_reservoir(new_problem, brick_types_map_reversed, brick_reservoir_time_from_wall_s_map);
    set_distance_start(new_problem, num_robots, brick_types_map_reversed, brick_reservoir_time_from_wall_s_map, times_finish_actual_s);

    return new_problem;
}

void
DatasetSOP::set_params_begin(
        DatasetSOP& new_problem,
        const std::map<int, std::string>& brick_types_map_reversed,
        const std::map<std::string, int>& brick_types_rewards_map,
        const std::map<std::string, int>& brick_types_durations_map,
        const std::map<std::string, int>& brick_num_robot_requirements_map)
{
    /* set params part begin */
    std::cout << "set_params" << std::endl;
    std::cout << "nodesAll.size() " << new_problem.nodesAll.size() << std::endl;
    std::cout << "brick_types_map_reversed.size() " << brick_types_map_reversed.size() << std::endl;

    if(brick_types_map_reversed.empty()) return;

    for (auto& node : new_problem.nodesAll) {
        std::cout << "node.type " << node.type << std::endl;
        auto it = brick_types_map_reversed.find(node.type);
        if(it == brick_types_map_reversed.end()) continue;
        const auto& brick_type = (*it).second;
        std::cout << "brick_type " << brick_type << std::endl;
        node.reward = brick_types_rewards_map.at(brick_type);
        node.node_duration = brick_types_durations_map.at(brick_type);
        node.robot_requirement = brick_num_robot_requirements_map.at(brick_type);
        //if (nodesAll[var].type == 0) {
        //  nodesAll[var].robot_requirement = num_robots;
        //}
    }
}

void
DatasetSOP::set_params_end(DatasetSOP& new_problem, int num_robots)
{

    /* set params part end */
    assert(!new_problem.nodesAll.empty());
    // set start requirements to 1 and add equal number of starts as num robots
    new_problem.nodesAll[0].robot_requirement = 1;
    //add num_robots-1 start nodes
    for (int var = 1; var < num_robots; ++var) {
        GraphNode new_other_start;
        new_other_start.id = 0;
        new_other_start.type = 0;
        new_other_start.x = 0;
        new_other_start.y = 0;
        new_other_start.z = 0;
        new_other_start.reward = 0;
        new_other_start.node_duration = 0;
        new_other_start.robot_requirement = 1;
        new_problem.nodesAll.insert(new_problem.nodesAll.begin(), new_other_start);
        //INFO("insert start "<<var)
    }

    new_problem.goalID = static_cast<int>(new_problem.nodesAll.size()) - 1;
    //INFO("added all robot starts");
}

void
DatasetSOP::reset_indexed(DatasetSOP& new_problem, int num_robots) {
    //fill start indexes inside nodesAll
    new_problem.start_indexes.clear();
    new_problem.start_indexes.reserve(num_robots);
    for (int var = 0; var < num_robots; ++var) {
        new_problem.start_indexes.push_back(var);
    }
}

void
DatasetSOP::set_durations(DatasetSOP& new_problem,
                   int num_robots,
                   const std::map<int, std::string>& brick_types_map_reversed,
                   const std::vector<short int>& times_finish_actual_s,
                   const std::vector<short int>& current_brick_ids) {

    for (int var = 0; var < current_brick_ids.size(); ++var) {
        auto& current_brick_id = current_brick_ids[var];

        std::cout << "current_brick_ids[" << var << "] is " << current_brick_id
                  << " count " << brick_types_map_reversed.count((int)current_brick_id)
                  << std::endl;

        if (current_brick_id > 0 && current_brick_id < new_problem.nodesAll.size()) {	//current_brick_ids[var] has to be brick id

            auto new_start_index = current_brick_id + num_robots - 1;

            std::cout << "set start_indexes[" << var << "] to " << new_start_index << std::endl;
            new_problem.start_indexes[var] = new_start_index;			//start index is the brick index + num_robots - 1

            auto& finish_time = times_finish_actual_s[var];
            if (finish_time < 0) {
                std::cerr << "times_finish_actual_s is smaller than zero for current brick id " << current_brick_id << std::endl;
            }

            std::cout << "set times_finish_actual_s for " << new_start_index << " to " << finish_time << std::endl;
            new_problem.nodesAll[new_start_index].node_duration = finish_time;
        }
    }
}

void
DatasetSOP::renumber_nodes(DatasetSOP& new_problem) {
    // renumber the node ids
    for (int var = 0; var < new_problem.nodesAll.size(); ++var) {
        new_problem.nodesAll[var].id = var;
    }
}

void
DatasetSOP::resize_distance_matrix(DatasetSOP& new_problem) {
    // resize allDistances
    // make new_problem.distance_matrix NxN, N = new_problem.nodesAll.size()
    new_problem.distance_matrix.clear();
    new_problem.distance_matrix.resize(new_problem.nodesAll.size());
    for (int var = 0; var < new_problem.nodesAll.size(); ++var) {
        new_problem.distance_matrix[var].resize(new_problem.nodesAll.size());
    }
    // INFO("resized distances");
}

void
DatasetSOP::set_distances_reservoir(DatasetSOP& new_problem,
                                    const std::map<int, std::string>& brick_types_map_reversed,
                                    const std::map<std::string, int>& brick_reservoir_time_from_wall_s_map)
{
    // set all distances in seconds between wall and brick reservoiars
    auto nodes_size = new_problem.nodesAll.size();
    for (int var = 0; var < nodes_size; ++var) {
        const auto& node = new_problem.nodesAll[var];

        assert(brick_types_map_reversed.find(node.type) != brick_types_map_reversed.end());
        auto brick_type = brick_types_map_reversed.at(node.type);

        assert(brick_reservoir_time_from_wall_s_map.find(brick_type) != brick_reservoir_time_from_wall_s_map.end());
        auto time_from_wall = brick_reservoir_time_from_wall_s_map.at(brick_type);

        //INFO("time_from_wall " << time_from_wall)
        for (int var_from = 0; var_from < nodes_size; ++var_from) {
            new_problem.distance_matrix[var_from][var] = time_from_wall; //distance from wall to var brick
        }
    }
    // INFO("filled distances");
}


void
DatasetSOP::set_distance_start(DatasetSOP& new_problem,
                                    int num_robots,
                                    const std::map<int, std::string>& brick_types_map_reversed,
                                    const std::map<std::string, int>& brick_reservoir_time_from_wall_s_map,
                                    const std::vector<short int>& times_finish_actual_s)
{
    // set distances for all starts
    for (int var_from = 0; var_from < num_robots; ++var_from) {
        int remaining_time_finish_actual = times_finish_actual_s[var_from];
        for (int var = 0; var < new_problem.nodesAll.size(); ++var) {
            //INFO_VAR(nodesAll[var].type);
            std::string brick_type = brick_types_map_reversed.at(new_problem.nodesAll[var].type);
            //INFO_VAR(brick_type);
            int time_from_wall = brick_reservoir_time_from_wall_s_map.at(brick_type);
            new_problem.distance_matrix[var_from][var] = time_from_wall;  //distance from wall to reservoar
            //+ remaining_time_finish_actual; //distance from wall to reservoar + finish current job
        }
    }
    //INFO("filled start distances");
}