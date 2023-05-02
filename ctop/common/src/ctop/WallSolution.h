//
// Created by Michal Němec on 23/04/2020.
//

#ifndef CTOP_PLANNER_WALLSOLUTION_H
#define CTOP_PLANNER_WALLSOLUTION_H

#include <map>
#include <sstream>

namespace ctop {

// NAME:;
// METHOD:ilp_ctop;
// CTIME:608;
// NUM_ITERS:8891;
// BUDGET:180;
// REWARDS:28;
// IS_OPTIMAL:1;
// GAP_PERCENT:0;
// MAX_ACHIEVABLE_REWARDS:42;
// LENGTH:0;
// NUM_ITERS_LAST_IMPR:0;
// CTIME_LAST_IMPR:0;
// MAX_ALLOWED_CALC_TIME_MS:600000;
// RESULT_TARGET_IDS:0|3|6|4|0,0|7|0,0|2|1|5|0;
// RESULT_START_TIMES:0|45|95|140|180,0|140|180,0|5|50|100|180;
// RESULT_TARGET_DURATIONS:0|40|40|40|0,0|40|0,0|40|40|40|0;
// SOLUTION_TIME_REWARD_LENGTH_IMPROVEMENTS:;
// LOWER_BOUND_ABOVE_BUDGET_THEN_SKIP:0;
//    solution_ids = data_ctop[-1]['RESULT_TARGET_IDS'][robot_id]
//    solution_start_times = data_ctop[-1]['RESULT_START_TIMES'][robot_id]
//    solution_durations = data_ctop[-1]['RESULT_TARGET_DURATIONS'][robot_id]
struct WallSolution {
  uint64_t budget = 0;
  std::string ctime_str = "";
  uint64_t reward = 0;
  uint64_t end_time = 0;
  int placed_bricks = 0;
  std::map<uint64_t, std::vector<uint64_t>> target_brick_ids{};
  std::map<uint64_t, std::vector<uint64_t>> start_brick_times{};
  std::map<uint64_t, std::vector<uint64_t>> target_durations{};

  static void map2string(std::ostream &oss,
                         const std::map<uint64_t, std::vector<uint64_t>> &m) {
    uint64_t k = 0;
    for (const auto &p : m) {
      auto id = p.first;
      const auto &vals = p.second;
      for (uint64_t i = 0; i != vals.size(); ++i) {
        oss << vals[i];
        if (i + 1 != vals.size()) {
          oss << "|";
        }
      }
      k++;
      if (k != m.size()) {
        oss << ",";
      }
    }
  }

  static void
  map_key2string(std::ostream &oss,
                 const std::map<uint64_t, std::vector<uint64_t>> &m) {
    uint64_t k = 0;
    for (const auto &p : m) {
      auto id = p.first;
      oss << id;
      k++;
      if (k != m.size()) {
        oss << ",";
      }
    }
  }

  std::string print() {
    std::ostringstream oss;
    oss << "RESULT_REWARD:";
    oss << reward;
    oss << ";";
    oss << "BUDGET:";
    oss << budget;
    oss << ";";
    oss << "RESULT_TOTAL_TIME:";
    oss << end_time;
    oss << ";";
    oss << "RESULT_ROBOT_IDS:";
    map_key2string(oss, target_brick_ids);
    oss << ";";
    oss << "RESULT_TARGET_IDS:";
    map2string(oss, target_brick_ids);
    oss << ";";
    oss << "RESULT_START_TIMES:";
    map2string(oss, start_brick_times);
    oss << ";";
    oss << "RESULT_TARGET_DURATIONS:";
    map2string(oss, target_durations);
    oss << ";";
    oss << "CTIME:";
    oss << ctime_str;
    oss << ";";
    return oss.str();
  }

  std::string print_res() {
    std::ostringstream oss_out;
    auto num_robots = target_brick_ids.size();
    {
      std::ostringstream oss_o;
      std::ostringstream oss;
      uint64_t time = 0;
      while (time < end_time) {
        oss << fmt::format("{:<10}", time);
        time += 10;
      }
      oss << fmt::format("{}", end_time);
      oss_o << fmt::format("{:<10}", "time") << "  " << oss.str() << std::endl;
      auto timeline_str = oss_o.str();
      oss_out << timeline_str; // << fmt::format("{:->{}}", "",
                               // timeline_str.size()) << std::endl;
    }
    for (auto &&a : target_brick_ids) {
      auto r_id = a.first;
      auto &placement_bricks = target_brick_ids[r_id];
      auto total_bricks = placement_bricks.size();

      auto &start_times = start_brick_times[r_id];
      auto &durs = target_durations[r_id];

      auto prev_time = 0;

      std::ostringstream oss;
      for (int i = 0; i != total_bricks; i++) {

        auto brick_id = placement_bricks[i];
        auto start_time = start_times[i];
        auto duration = durs[i];

        auto diff_start = start_time - prev_time;

        auto spacing_str = fmt::format("{:>{}}", "", diff_start);
        auto brick_str =
            fmt::format("{:*^{}}", fmt::format(" {} ", brick_id), duration);

        oss << spacing_str << brick_str;

        prev_time = start_time + duration;
      }
      oss_out << fmt::format("{:<10}", fmt::format("robot{}", r_id)) << ": "
              << oss.str() << std::endl;
    }
    return oss_out.str();
  }

  bool operator==(const WallSolution &rhs) {
    auto num_robots = target_brick_ids.size();
    if (target_brick_ids.size() != rhs.target_brick_ids.size())
      return false;

    std::vector<uint64_t> robot_ids;
    robot_ids.reserve(num_robots);
    for (auto &&p : target_brick_ids) {
      robot_ids.push_back(p.first);
    }

    std::vector<uint64_t> robot_ids_other;
    robot_ids_other.reserve(num_robots);
    for (auto &&p : rhs.target_brick_ids) {
      robot_ids_other.push_back(p.first);
    }

    for (int i = 0; i < num_robots; i++) {
      if (robot_ids_other[i] != robot_ids[i]) {
        return false;
      }
    }

    for (auto &&r_id : robot_ids) {
      auto &placement_bricks = target_brick_ids.at(r_id);
      auto &placement_bricks_2 = rhs.target_brick_ids.at(r_id);

      auto total_bricks = placement_bricks.size();
      auto total_bricks_2 = placement_bricks_2.size();
      if (total_bricks != total_bricks_2)
        return false;

      auto &start_times = start_brick_times.at(r_id);
      auto &start_times_2 = rhs.start_brick_times.at(r_id);
      if (start_times.size() != start_times_2.size())
        return false;

      auto &durs = target_durations.at(r_id);
      auto &durs_2 = rhs.target_durations.at(r_id);
      if (durs.size() != durs_2.size())
        return false;

      for (int i = 0; i != total_bricks; i++) {
        auto brick_id = placement_bricks[i];
        auto brick_id_2 = placement_bricks_2[i];
        if (brick_id != brick_id_2)
          return false;

        auto start_time = start_times[i];
        auto start_time_2 = start_times_2[i];
        if (start_time != start_time_2)
          return false;

        auto duration = durs[i];
        auto duration_2 = durs_2[i];
        if (duration != duration_2)
          return false;
      }
    }

    return true;
  }
};

} // namespace ctop

#endif // CTOP_PLANNER_WALLSOLUTION_H
