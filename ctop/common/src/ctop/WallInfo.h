//
// Created by Michal Němec on 2/02/2020.
//

#ifndef CTOP_PLANNER_WALLINFO_H
#define CTOP_PLANNER_WALLINFO_H

#include <string>
#include <map>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <ctop/log.h>

#include <ctop/heuristic/PrecedenceRule.h>

#include <ctop/util/reverse_map.h>


namespace ctop {

struct WallInfo {

    int budget_s;
    int num_robots;
    int num_robots2;
    std::vector<int> start_indexes;
    int goal_index;
    std::vector<PrecedenceRule> precendence_rules;
    int min_distance_concurrent_brick_placing_cm;
    bool entire_layer_first;

    double maximalRewardAll;

    std::string yaml_config_file;
    std::vector<std::string> brick_types;
    std::map<std::string, int> brick_ids_map;
    std::map<int, std::string> brick_ids_map_reversed;
    std::map<std::string, int> brick_rewards_map;
    std::map<std::string, int> brick_num_robot_requirements_map;
    std::map<std::string, std::vector<int>> brick_size_length_depth_height_cm_map;

    // first robot type
    std::map<std::string, int> brick_durations_map;
    std::map<std::string, int> brick_reservoir_time_from_wall_s_map;
    std::vector<short int> builded_brick_ids;
    std::vector<short int> current_brick_ids;
    std::vector<short int> times_finish_actual_s;
    std::vector<short int> battery_budgets_s;
    std::vector<short int> battery_recharge_s;

    // second robot type
    std::map<std::string, int> brick_durations2_map;
    std::map<std::string, int> brick_reservoir_time_from_wall_s2_map;
    std::vector<short int> builded_brick_ids2;
    std::vector<short int> current_brick_ids2;
    std::vector<short int> times_finish_actual_s2;
    std::vector<short int> battery_budgets_s2;
    std::vector<short int> battery_recharge_s2;

    std::vector<PrecedenceRule> concurrence_exclusion_rules;

public:
    WallInfo() {

    }

    void loadYamlConfig(const std::string& file)
    {
        CTOP_LOG_I("loading yaml file {}", file);

        auto config = YAML::LoadFile(file);
        num_robots = config["num_robots"].as<int>();
        num_robots2 = config["num_robots2"].as<int>();

        budget_s = config["time_budget_s"].as<int>();
        min_distance_concurrent_brick_placing_cm = config["min_distance_concurrent_brick_placing_cm"].as<int>();
        entire_layer_first = config["entire_layer_first"].as<bool>();

        CTOP_LOG_I("num_robots: {}", num_robots);
        CTOP_LOG_I("budget_s: {}", budget_s);
        CTOP_LOG_I("min_distance_concurrent_brick_placing_cm: {}", min_distance_concurrent_brick_placing_cm);
        CTOP_LOG_I("entire_layer_first: {}", entire_layer_first);

        parseBrickYamlParam(config, "brick_types", brick_types);
        parseBrickYamlParam(config, "brick_ids", brick_ids_map);
        parseBrickYamlParam(config, "brick_rewards", brick_rewards_map);
        parseBrickYamlParam(config, "brick_num_robot_requirements", brick_num_robot_requirements_map);
        parseBrickYamlParam(config, "brick_size_length_depth_height_cm", brick_size_length_depth_height_cm_map);


        parseBrickYamlParam(config, "brick_durations", brick_durations_map);
        parseBrickYamlParam(config, "brick_reservoir_time_from_wall_s", brick_reservoir_time_from_wall_s_map);
        parseBrickYamlParam(config, "builded_brick_ids", builded_brick_ids);
        parseBrickYamlParam(config, "current_brick_ids", current_brick_ids);
        parseBrickYamlParam(config, "times_finish_actual_s", times_finish_actual_s);
        parseBrickYamlParam(config, "battery_budgets_s", battery_budgets_s);
        parseBrickYamlParam(config, "battery_recharge_s", battery_recharge_s);

        if (current_brick_ids.size() != num_robots) {
            CTOP_LOG_E("current_brick_ids must be of size num_robots");
            exit(1);
        }

        if (times_finish_actual_s.size() != num_robots) {
            CTOP_LOG_E("times_finish_actual_s must be of size num_robots");
            exit(1);
        }

        if (battery_budgets_s.size() != num_robots) {
            CTOP_LOG_E("battery_budgets_s must be of size num_robots");
            exit(1);
        }

        parseBrickYamlParam(config, "brick_durations2", brick_durations2_map);
        parseBrickYamlParam(config, "brick_reservoir_time_from_wall_s2", brick_reservoir_time_from_wall_s2_map);
        parseBrickYamlParam(config, "builded_brick_ids2", builded_brick_ids2);
        parseBrickYamlParam(config, "current_brick_ids2", current_brick_ids2);
        parseBrickYamlParam(config, "times_finish_actual_s2", times_finish_actual_s2);
        parseBrickYamlParam(config, "battery_budgets_s2", battery_budgets_s2);
        parseBrickYamlParam(config, "battery_recharge_s2", battery_recharge_s2);

        if (current_brick_ids2.size() != num_robots2) {
            CTOP_LOG_E("current_brick_ids must be of size num_robots2");
            exit(1);
        }

        if (times_finish_actual_s2.size() != num_robots2) {
            CTOP_LOG_E("times_finish_actual_s must be of size num_robots2");
            exit(1);
        }

        if (battery_budgets_s2.size() != num_robots2) {
            CTOP_LOG_E("battery_budgets_s must be of size num_robots2");
            exit(1);
        }

        reverse_map(brick_ids_map, brick_ids_map_reversed);
        CTOP_LOG_I("loading yaml file {} end", file);
    }

    template<typename T>
    void parseBrickYamlParam(YAML::Node &config, const std::string& param, std::vector<T> &vector_to_fill)
    {
        auto read = config[param.c_str()].as<std::vector<T>>();
        vector_to_fill.insert(vector_to_fill.end(), read.begin(), read.end());
    }

    void parseBrickYamlParam(YAML::Node &config, const std::string& param, std::map<std::string, int> &map_to_fill) {
        YAML::Node brick_ids = config[param.c_str()];
        if (brick_ids.Type() != YAML::NodeType::Null) {
            for (const auto& type : brick_types) {
                const auto& brick_id_node = brick_ids[type];
                if (brick_id_node.Type() != YAML::NodeType::Null) {
                    auto brick_id = brick_id_node.as<int>();
                    map_to_fill[type] = brick_id;
                    CTOP_LOG_I("brick {} has {} {}", type, param, brick_id);
                } else {
                    CTOP_LOG_I("{} for {} not defined", param, type);
                    exit(1);
                }
            }
        } else {
            CTOP_LOG_I("{} not defined", param);
            exit(1);
        }
    }

    void parseBrickYamlParam(YAML::Node &config, const std::string& param, std::map<std::string, std::vector<int>> &map_to_fill) {
        auto brick_ids = config[param.c_str()];
        if (brick_ids.Type() != YAML::NodeType::Null) {
            for (const auto& type : brick_types) {
                const auto& brick_id_node = brick_ids[type];
                if (brick_id_node.Type() != YAML::NodeType::Null) {
                    auto& vec = map_to_fill[type];
                    for(const auto& value_in_list : brick_id_node) {
                        auto val = value_in_list.as<int>();
                        vec.push_back(val);
                        CTOP_LOG_I("brick {} has {} size={} val {}", type, param, vec.size(), val);
                    }
                } else {
                    CTOP_LOG_I("{} for {} not defined", param, type);
                    exit(1);
                }
            }
        } else {
            CTOP_LOG_I("{} not defined", param);
            exit(1);
        }
    }

    void parse(const std::string& file) {
        yaml_config_file = file;
        loadYamlConfig(file);
    }

};

}

#endif //CTOP_PLANNER_WALLINFO_H
