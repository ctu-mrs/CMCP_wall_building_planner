
#include <Eigen/Dense>

#include <ctop/cmd_config.h>
#include <ctop/geometry/Point3D.h>
#include <ctop/log.h>
#include <ctop/log/Logger.h>
#include <ctop/util/csvreader.h>
#include <ctop/util/split.h>

#include <ctop/Brick.h>

#include "Grasp.h"
#include "WallGraph.h"
#include <ctop/WallInfo.h>

class File {

  std::string base_;

public:
  File(const std::string &base) : base_(base) {}

  std::string file_path(const std::string &base) const {
    return base_ + "/" + base;
  }
};

bool isPointInside(double px, double py,
                   const std::vector<ctop::Point3D> &corners) {
  int count = 0;
  // Cast a horizontal ray and see how many times it intersects all the edges
  auto sz = corners.size();
  for (int i = 0; i < sz; ++i) {

    const auto &corner1 = corners[i];
    const auto &corner2 = corners[(i + 1) % sz];

    double v1x = corner1.x;
    double v1y = corner1.y;
    double v2x = corner2.x;
    double v2y = corner2.y;

    if (v1y > py && v2y > py)
      continue;
    if (v1y <= py && v2y <= py)
      continue;

    double intersect_x =
        (v1y == v2y) ? v1x : (py - v1y) * (v2x - v1x) / (v2y - v1y) + v1x;
    bool does_intersect = intersect_x > px;
    if (does_intersect) {
      ++count;
    }
  }

  return count % 2;
}

float clamp(float v, float min = 0.0f, float max = 1.0f) {
  return std::max(std::min(v, max), min);
}

double solve_closest_points_on_planes(ctop::Plane &b1, ctop::Plane &b2) {
  Eigen::Matrix4f A{};
  Eigen::Vector4f b{};

  std::array<Eigen::Vector3f *, 4> w = {&b1.w1, &b1.w2, &b2.w1, &b2.w2};

  Eigen::Vector3f dp = b1.p0 - b2.p0;
  for (int i = 0; i != 4; ++i) {
    auto &wj = *w[i];
    for (int j = 0; j != 4; ++j) {
      auto &wi = *w[j];
      double dot = wi.adjoint() * wj;
      if (j >= 2) {
        dot *= -1;
      }
      A(i, j) = dot;
    }
    double bp = -dp.adjoint() * wj;
    b(i) = bp;
  }

  Eigen::Vector4f x = A.colPivHouseholderQr().solve(b);
  Eigen::Vector2f uv{};
  Eigen::Vector2f uv_prime{};

  uv(0) = clamp(x(0));
  uv(1) = clamp(x(1));
  uv_prime(0) = clamp(x(2));
  uv_prime(1) = clamp(x(3));

  std::array<Eigen::Vector3f, 2> out;
  // CTOP_LOG_E("[u v] = [{}, {}], [u' v'] = [{}, {}]", uv(0), uv(1),
  // uv_prime(0), uv_prime(1)); CTOP_LOG_E("[u v] = [{}, {}], [u' v'] = [{},
  // {}]", x(0), x(1), x(2), x(3));

  return (b1.eval_at(uv) - b2.eval_at(uv_prime)).norm();

  uv(0) = x(0);
  uv(1) = x(1);
  uv_prime(0) = x(2);
  uv_prime(1) = x(3);
  return (b1.eval_at(uv) - b2.eval_at(uv_prime)).norm();
}

struct CloseWallInfo {
  double dist;
  std::set<int> i;
  std::set<int> j;
};

CloseWallInfo find_closest_planes_top_bottom(ctop::Brick &b1, ctop::Brick &b2) {

  auto distMin = std::numeric_limits<double>::max();
  std::set<int> minI;
  std::set<int> minJ;

  for (int i = 4; i != 6; ++i) {
    auto &p1 = b1.walls.walls[i];
    for (int j = 4; j != 6; ++j) {
      auto &p2 = b2.walls.walls[j];

      double dist = solve_closest_points_on_planes(p1, p2);
      // CTOP_LOG_E("test b1[id={}] wall={} b2[id={}] wall={} dist={}", b1.id,
      // ctop::BrickWalls::wall_pos2string(i), b2.id,
      // ctop::BrickWalls::wall_pos2string(j), dist);

      bool clear = false;
      bool add = false;
      if (dist < distMin) {
        add = true;
        clear = std::abs(dist - distMin) >= 0.01;
        distMin = dist;
      } else {
        if (std::abs(dist - distMin) <= 0.01) {
          add = true;
        }
      }

      if (add) {
        if (clear) {
          minI.clear();
          minJ.clear();
        }
        minI.emplace(i);
        minJ.emplace(j);
      }
    }
  }

  return {distMin, minI, minJ};
}

CloseWallInfo find_closest_planes_sides(ctop::Brick &b1, ctop::Brick &b2) {

  auto distMin = std::numeric_limits<double>::max();
  std::set<int> minI;
  std::set<int> minJ;

  for (int i = 0; i != 4; ++i) {
    auto &p1 = b1.walls.walls[i];
    for (int j = 0; j != 4; ++j) {
      auto &p2 = b2.walls.walls[j];

      double dist = solve_closest_points_on_planes(p1, p2);
      // CTOP_LOG_E("test b1[id={}] wall={} b2[id={}] wall={} dist={}", b1.id,
      // ctop::BrickWalls::wall_pos2string(i), b2.id,
      // ctop::BrickWalls::wall_pos2string(j), dist);

      bool clear = false;
      bool add = false;
      if (dist < distMin) {
        add = true;
        clear = std::abs(dist - distMin) >= 0.01;
        distMin = dist;
      } else {
        if (std::abs(dist - distMin) <= 0.01) {
          add = true;
        }
      }

      if (add) {
        if (clear) {
          minI.clear();
          minJ.clear();
        }
        minI.emplace(i);
        minJ.emplace(j);
      }
    }
  }

  return {distMin, minI, minJ};
}

void eval_distances(std::vector<ctop::Brick> &bricks) {
  for (int i = 0; i != bricks.size(); ++i) {
    auto &b1 = bricks[i];
    for (int j = i + 1; j != bricks.size(); ++j) {
      auto &b2 = bricks[j];
      auto info = find_closest_planes_sides(b1, b2);

      bool hasTopBottomSide = false;
      bool hasSide = false;
      bool hasCross = false;

      if (info.dist <= 0.11) {
        hasSide = true;
        CTOP_LOG_I("min_plane b1[id={}]: {}, b2[id={}]: {}, dist: {}", b1.id,
                   ctop::BrickWalls::wall_pos2string_set(info.i), b2.id,
                   ctop::BrickWalls::wall_pos2string_set(info.j), info.dist);
      } else {
        // CTOP_LOG_D("min_plane b1[id={}]: {}, b2[id={}]: {}, dist: {}", b1.id,
        // ctop::BrickWalls::wall_pos2string_set(info.i), b2.id,
        // ctop::BrickWalls::wall_pos2string_set(info.j), info.dist);
      }
      info = find_closest_planes_top_bottom(b1, b2);
      if (info.dist <= 0.01) {
        hasTopBottomSide = true;
        CTOP_LOG_I("min_plane b1[id={}]: {}, b2[id={}]: {}, dist: {}", b1.id,
                   ctop::BrickWalls::wall_pos2string_set(info.i), b2.id,
                   ctop::BrickWalls::wall_pos2string_set(info.j), info.dist);
      }
      if (hasTopBottomSide) {
        CTOP_LOG_E("min_plane b1[id={}] b2[id={}] TOP-BOTTOM", b1.id, b2.id);
      } else if (hasSide) {
        CTOP_LOG_E("min_plane b1[id={}] b2[id={}] SIDE", b1.id, b2.id);
      }
    }
  }
}

int main(int argc, char **argv) {
  std::random_device dev;

#ifdef CTOP_ENABLE_LOG
  // setup multithread safe logging to have consistent output
  std::mutex log_mutex;
  ctop::Logger logger;
  logger.set_on_log_listener([&](ctop::LogItem &item) {
    std::lock_guard<decltype(log_mutex)> ll{log_mutex};
    ctop::Logger::default_log(item);
  });
  CTOP_LOG_I("setting logger");
  ctop::log::set_logger(&logger);
#endif

  auto config = ctop::ConfigOptions::parseArgs(argc, argv);
  if (!config)
    return 1;

  auto outputFolder = config.sopConfig.get<std::string>("output");
  auto config_file = config.sopConfig.get<std::string>("config-file");

  GRASP_LOG_D("output {}", outputFolder);
  GRASP_LOG_D("config-file {}", config_file);

  ctop::WallInfo wall;
  wall.parse(config_file);

  std::vector<ctop::Brick> bricks;

  double min_z_brick = std::numeric_limits<double>::max();
  std::vector<int> start_bricks;

  if (!config.problemFileObj.empty()) {
    auto problem_file = config.problemFileObj;
    GRASP_LOG_D("problem {}", problem_file);

    int parsed_vertex = 0;
    bool parse_brick = false;

    int brick_id = 0;
    int brick_type = 0;
    double brick_x_pos = 0.0;
    double brick_y_pos = 0.0;
    double brick_z_pos = 0.0;
    int brick_yaw_rotated = 0;

    ctop::parse_csv_line(
        problem_file, [&](int line, const std::vector<std::string> &vals) {
          if (vals.empty()) {
            return;
          }

          if (vals[0] == "v") {
            // v x y z -> y is directing up so we need to swi
            GRASP_ASSERT(vals.size() >= 4);

            // in centimeters, used as meters
            brick_x_pos += ctop::Brick::string_to<double>(vals[1]) / 100.0;
            brick_y_pos += ctop::Brick::string_to<double>(vals[3]) / 100.0;
            brick_z_pos += ctop::Brick::string_to<double>(vals[2]) / 100.0;
            parsed_vertex++;

            // each brick has 8 verticies
            if (parsed_vertex == 8) {
              brick_x_pos /= parsed_vertex;
              brick_y_pos /= parsed_vertex;
              brick_z_pos /= parsed_vertex;
              parsed_vertex = 0;
            }
          } else if (vals[0] == "o") {
            GRASP_ASSERT(vals.size() >= 2);
            auto name = vals[1];
            // Cube.<id>, can also be just "Cube" without id
            auto ids = ctop::split(name, ".");
            auto idx = 0;
            if (ids.size() >= 2) {
              idx = ctop::Brick::string_to<int>(ids[1]);
            }
            brick_id = idx + 1;
          } else if (vals[0] == "usemtl") {
            GRASP_ASSERT(vals.size() >= 2);
            auto name = vals[1];
            ctop::replace(name, "\r", "");
            brick_type = wall.brick_ids_map[name];
            parse_brick = true;
          }

          if (parse_brick) {
            ctop::Brick b;
            b.x = brick_x_pos;
            b.y = brick_y_pos;
            b.z = brick_z_pos;
            b.yaw_rotated = brick_yaw_rotated;
            b.id = brick_id;
            b.type = brick_type;

            b.name = wall.brick_ids_map_reversed[brick_type];

            b.corners = b.get_corners(wall);
            b.walls = b.get_walls(wall);
            bricks.push_back(b);
            parse_brick = false;

            auto dz = min_z_brick - b.z;
            if (std::abs(dz) < 0.01) {
              start_bricks.push_back(bricks.size() - 1);
            } else if (dz > 0) {
              min_z_brick = b.z;
              start_bricks.clear();
              start_bricks.push_back(bricks.size() - 1);
            }

            // reset values
            brick_id = 0;
            brick_type = 0;
            brick_x_pos = 0.0;
            brick_y_pos = 0.0;
            brick_z_pos = 0.0;
            brick_yaw_rotated = 0;
          }
        });
  } else {
    auto problem_file = config.problemFile;
    GRASP_LOG_D("problem {}", problem_file);
    ctop::parse_csv_line(
        problem_file, [&](int line, const std::vector<std::string> &vals) {
          if (vals.empty()) {
            return;
          }
          GRASP_ASSERT(vals.size() == 6);
          auto b = ctop::Brick::from_string_vector(wall, vals);
          b.print();
          CTOP_LOG_D("type={}", wall.brick_ids_map_reversed[b.type]);
          bricks.push_back(b);

          auto dz = min_z_brick - b.z;
          if (std::abs(dz) < 0.01) {
            start_bricks.push_back(bricks.size() - 1);
          } else if (dz > 0) {
            min_z_brick = b.z;
            start_bricks.clear();
            start_bricks.push_back(bricks.size() - 1);
          }
        });
  }

  auto gptr = std::make_shared<ctop::WallGraph>(wall, dev);

  auto &graph = *gptr;
  graph.add_brick_nodes(bricks);

  auto b_size = bricks.size();
  for (int i = 0; i < b_size; i++) {
    auto &b1 = bricks[i];
    for (int j = i + 1; j < b_size; j++) {
      auto &b2 = bricks[j];
      auto dz = b2.z - b1.z;
      if (std::abs(std::abs(dz) - 0.2) < 0.01) {
        auto &bottom_brick = b1.z > b2.z ? b2 : b1;
        auto &top_brick = b1.z > b2.z ? b1 : b2;

        if (wall.entire_layer_first ||
            ctop::Brick::is_on_top_of_each_other(&bottom_brick, &top_brick)) {
          graph.add_dir_edge(bottom_brick, top_brick,
                             ctop::WallEdgeType::precedence);
        }
      }
      if (std::abs(dz) < 0.01) {
        graph.add_both_dir_edge(b1, b2, ctop::WallEdgeType::side_activate);
        auto layer =
            static_cast<int>(std::floor((std::abs(b1.z) - 0.1) / 0.2 + 0.5));
        if (layer < 0)
          layer = 0;
        if (b1.layer == 0)
          b1.layer = layer;
        if (b2.layer == 0)
          b2.layer = layer;
        GRASP_ASSERT(b1.layer == layer);
        GRASP_ASSERT(b2.layer == layer);

        // if((std::abs(b1.z) - 0.1) > 0.01) {
        graph.add_both_dir_edge(b1, b2, ctop::WallEdgeType::distance);
        //}

        auto didx = std::abs(b1.id - b2.id);
        if (didx == 1) {
          graph.add_both_dir_edge(b1, b2, ctop::WallEdgeType::side);
        }
      }
    }
  }

  GRASP_LOG_D("starting bricks {}", start_bricks.size());
  for (auto &b : start_bricks) {
    // starting nodes
    graph.add_to_start(bricks[b]);
  }

  auto brick_id0 = std::make_shared<ctop::BrickPlacementData>();
  auto brick_id1 = std::make_shared<ctop::BrickPlacementData>();

  for (auto &&n : wall.brick_ids_map) {
    auto brick_type = n.second;
    {
      auto &data = brick_id0->brick_2_data[brick_type];
      data.duration = wall.brick_durations_map[n.first];
      data.reservoir_time = wall.brick_reservoir_time_from_wall_s_map[n.first];
    }
    {
      auto &data = brick_id1->brick_2_data[brick_type];
      data.duration = wall.brick_durations2_map[n.first];
      data.reservoir_time = wall.brick_reservoir_time_from_wall_s2_map[n.first];
    }
  }

  std::unordered_map<uint64_t, std::vector<std::shared_ptr<ctop::Resource>>>
      resource_bucket;
  int rid = 0;
  for (int i = 0; i < wall.num_robots; i++) {
    auto res = std::make_shared<ctop::Resource>();
    res->id = rid++;
    res->type = 0;
    res->brick_placement = brick_id0;
    res->battery_state = wall.battery_budgets_s[res->id];
    res->battery_budget = wall.battery_budgets_s[res->id];
    res->battery_recharge = wall.battery_recharge_s[res->id];
    resource_bucket[0].push_back(res);
  }
  for (int i = 0; i < wall.num_robots2; i++) {
    auto res = std::make_shared<ctop::Resource>();
    res->id = rid++;
    res->type = 0;
    res->brick_placement = brick_id1;
    res->battery_state = wall.battery_budgets_s2[i];
    res->battery_budget = wall.battery_budgets_s2[i];
    res->battery_recharge = wall.battery_recharge_s2[i];
    resource_bucket[0].push_back(res);
  }
  wall.num_robots = rid;
  gptr->resource_bucket = resource_bucket;

  File output_folder{outputFolder};
  {
    GRASP_LOG_D("saving graph dot file");
    std::ofstream of{output_folder.file_path("output.dot")};
    graph.print_dot(wall, of);
  }

  const std::size_t sanapshot_samples = 100;

  auto grasp_start = std::chrono::high_resolution_clock::now();

  ctop::Grasp grasp;
  grasp.max_threads = std::numeric_limits<int>::max();
  grasp.max_iterations = 1000;
  grasp.max_did_not_improve = 100;
  grasp.snapshot_coef = 0.1;

  GRASP_LOG_D("grasp starting with coeff: {}", grasp.snapshot_coef);

  auto sol_graph = grasp.run(gptr);
  auto grasp_end = std::chrono::high_resolution_clock::now();
  auto runtime = ctop::duration_to_string(grasp_end - grasp_start);
  auto runtime_raw = ctop::duration_to_string(grasp.duration_raw);

  GRASP_LOG_D("grasp runtime: {}", runtime);

  std::ofstream of{output_folder.file_path("run.output.txt")};
  std::ofstream of_csv{output_folder.file_path("run.output.csv")};

  of << "runtime_raw: " << runtime_raw << "\n";
  of << "runtime: " << runtime << "\n";

  auto &g = sol_graph.graph;

  std::cout << g->sol.print() << std::endl;
  of << g->sol.print() << std::endl;
  for (auto rew : grasp.rewards) {
    std::cout << rew.first << "/" << rew.second << " ";
    of << rew.first << "/" << rew.second << " ";
    of_csv << rew.first << "," << rew.second << "\n";
  }

  std::cout << "\n";
  std::cout << "placed " << g->sol.placed_bricks << "/" << bricks.size()
            << "\n";
  std::cout << "reward " << g->sol.reward << "\n";
  std::cout << "time " << g->sol.end_time << "\n";
  std::cout << "runtime " << runtime << "\n";

  {
    std::ofstream out;
    // white to the end of file
    out.open(output_folder.file_path("results.log"), std::ios::app);
    g->sol.budget = wall.budget_s;
    g->sol.ctime_str = runtime;
    out << g->sol.print() << std::endl;
  }
  std::cout << g->sol.print_res() << std::endl;

  of << "\n";
  auto done = (100.0 * g->sol.placed_bricks) / bricks.size();
  of << "placed " << g->sol.placed_bricks << "/" << bricks.size() << " "
     << fmt::format("{:.2f}%", done) << "\n";
  of << g->sol.print_res() << std::endl;

  // find all distinct solutions
  std::vector<std::shared_ptr<ctop::WallGraph>> dist_sol;
  dist_sol.push_back(sol_graph.graph);
  for (auto &grasp_sol : grasp.solutions) {
    bool distinct = true;
    for (uint64_t i = 0, max = dist_sol.size(); i != max; i++) {
      auto &sol = dist_sol[i];
      if (grasp_sol.graph->sol == sol->sol) {
        distinct = false;
        break;
      }
    }
    if (distinct) {
      dist_sol.push_back(grasp_sol.graph);
    }
  }

  GRASP_LOG_D("dist_sol={}", dist_sol.size());
  for (auto &grasp_sol : dist_sol) {
    if (dist_sol.size() < 5) {
      std::cout << grasp_sol->sol.print() << std::endl;
      std::cout << grasp_sol->sol.print_res() << std::endl;
    }
    of << grasp_sol->sol.print() << std::endl;
    of << grasp_sol->sol.print_res() << std::endl;
  }

  return 0;
}
