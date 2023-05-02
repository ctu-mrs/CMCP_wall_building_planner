//
// Created by Michal Němec on 09/03/2020.
//

#ifndef CTOP_PLANNER_WALLGRAPH_H
#define CTOP_PLANNER_WALLGRAPH_H

#include <deque>
#include <limits>
#include <ctop/log.h>

#include <ctop/util/has_unique_id.h>
#include <unordered_map>
#include <random>
#include <ctop/util/timestamp.h>

#include <ctop/Brick.h>
#include <ctop/WallSolution.h>
namespace ctop {

struct WallNode;

struct Resource {
    uint64_t type = 0;
    uint64_t id = 0;
    std::shared_ptr<WallNode> node = nullptr;
    uint64_t start_time = 0;
    uint64_t end_time = 0;
    uint64_t available_time = 0;
    uint64_t battery_budget = 0;
    uint64_t battery_state = 0;
    uint64_t battery_recharge = 0;
    double max_z = std::numeric_limits<double>::max();
    std::shared_ptr<BrickPlacementData> brick_placement = nullptr;


    Resource()
    {}

    Resource(const Resource& rhs) :
        id(rhs.id),
        node(rhs.node),
        start_time(rhs.start_time),
        end_time(rhs.end_time),
        available_time(rhs.available_time),
        max_z(rhs.max_z),
        brick_placement(rhs.brick_placement),
        battery_budget(rhs.battery_budget),
        battery_state(rhs.battery_state),
        battery_recharge(rhs.battery_recharge)
    {}
};


struct NodeData {
    uint64_t visited = 0;
};

struct EdgeData {
    uint64_t visited = 0;
};

enum class WallEdgeType {
    none,
    precedence,
    distance,
    side,
    side_activate
};

std::string edge_type_2_string(WallEdgeType type);

struct WallEdge : has_unique_id<WallEdge> {
    std::shared_ptr<WallNode> start = nullptr;
    std::shared_ptr<WallNode> end = nullptr;
    bool used = false;
    bool distance_valid = false;
    uint64_t available_time = 0;
    int weight = 1;
    std::shared_ptr<EdgeData> data = nullptr;

    WallEdgeType type = WallEdgeType::none;

    void visited() {
        used = true;
        if(data != nullptr) {
            data->visited++;
        }
    }

    int64_t start_brick_id();
    int64_t end_brick_id();
};


struct WallNode : has_unique_id<WallNode> {
    Brick* brick = nullptr;
    bool done = false;
    bool in_use = false;
    int visited = 0;
    double mininal_visit_time = 0;
    std::shared_ptr<NodeData> data = nullptr;
};

struct WallGraph {

    const WallGraph* prev = nullptr;
    WallInfo& info;

    int start_index = -1;

    std::vector<std::shared_ptr<WallNode>> start_nodes;

    // setup variables
    std::unordered_map<Brick::uid, std::shared_ptr<WallNode>> brick_nodes;

    std::unordered_map<WallNode::uid, std::shared_ptr<WallNode>> nodes;
    std::unordered_map<WallEdge::uid, std::shared_ptr<WallEdge>> edges;

    std::unordered_map<WallNode::uid, std::vector<std::shared_ptr<WallEdge>>> node_in_edge;
    std::unordered_map<WallNode::uid, std::vector<std::shared_ptr<WallEdge>>> node_out_edge;

    //algorithm variables
    std::vector<std::shared_ptr<WallNode>> active;
    int64_t no_process_count = 0;

    std::vector<std::shared_ptr<WallNode>> unvisited;
    std::vector<std::shared_ptr<WallEdge>> unvisited_edges;
    std::vector<std::shared_ptr<WallNode>> placed_nodes;

    std::mt19937 rng;

    std::unordered_map<uint64_t, std::vector<std::shared_ptr<Resource>>> resource_bucket;

    bool ignore_time_contraint = false;

    uint64_t actualTime = 0;
    bool done = false;
    uint64_t iter = 0;
    bool start_used = false;

    WallSolution sol;

    // for each iteration
    uint64_t added_active_nodes = 0;
    uint64_t max_available_time_glob = 0;
    std::vector<std::shared_ptr<WallEdge>> ignored;

    bool ignored_empty = false;

    uint64_t assinged_res = 0;
    uint64_t ava_res = 0;
    std::vector<WallNode::uid > remove_active;
    uint64_t active_count = 0;

    std::vector<std::shared_ptr<Resource>> finish_resources;
    std::vector<std::shared_ptr<Resource>> waiting_resources;

    WallGraph(WallInfo& inf, std::random_device& dev)
    : info(inf), rng() {

    }

    WallGraph(const WallGraph& inf)
            : info(inf.info), rng(inf.rng) {
        //
        prev = &inf;

        start_index = inf.start_index;
        no_process_count = inf.no_process_count;
        actualTime = inf.actualTime;
        done = inf.done;
        iter = inf.iter;
        start_used = inf.start_used;
        sol = inf.sol;

        CTOP_LOG_D("copying wall graph actualTime={} iter={} done={}", actualTime, iter, done);
        sol.print_res();

        // copy nodes
        std::unordered_map<WallNode::uid, WallNode::uid> nodes_mapping_old2new;
        for(auto& e : inf.nodes) {
            auto ec = std::make_shared<WallNode>(*e.second);
            nodes[ec->unique_id()] = ec;
            if(ec->brick != nullptr) {
                brick_nodes[ec->brick->id] = ec;
            }
            nodes_mapping_old2new[e.first] = ec->unique_id();
        }
        CTOP_LOG_D("inf.nodes.size={} nodes.size={}", inf.nodes.size(), nodes.size());
        CTOP_ASSERT(inf.nodes.size() == nodes.size());
        placed_nodes.reserve(nodes.size());

        // copy edges
        std::unordered_map<WallEdge::uid, WallEdge::uid> edges_mapping_old2new;

        for(auto& e : inf.edges) {
            auto ec = std::make_shared<WallEdge>(*e.second);
            // rewire shared_pointers
            if(e.second->start != nullptr) {
                ec->start = nodes[nodes_mapping_old2new[e.second->start->unique_id()]];
            } else {
                ec->start = nullptr;
            }
            if(e.second->end != nullptr) {
                ec->end = nodes[nodes_mapping_old2new[e.second->end->unique_id()]];
            } else {
                ec->end = nullptr;
            }
            edges[ec->unique_id()] = ec;
            edges_mapping_old2new[e.first] = ec->unique_id();
        }
        CTOP_ASSERT(inf.edges.size() == edges.size());

        start_nodes.reserve(inf.start_nodes.size());
        for(auto& e : inf.start_nodes) {
            start_nodes.push_back(nodes[nodes_mapping_old2new[e->unique_id()]]);
        }
        CTOP_ASSERT(inf.start_nodes.size() == start_nodes.size());


        for(auto& e : inf.node_out_edge) {
            auto uid = nodes_mapping_old2new[e.first];
            auto& edges_copy = e.second;
            auto& copy_to = node_out_edge[uid];
            copy_to.reserve(edges_copy.size());
            for(auto& ec : edges_copy) {
                auto ecc = edges[edges_mapping_old2new[ec->unique_id()]];
                copy_to.push_back(ecc);
            }
            CTOP_ASSERT(copy_to.size() == edges_copy.size());
        }
        CTOP_ASSERT(inf.node_out_edge.size() == node_out_edge.size());

        for(auto& e : inf.node_in_edge) {
            auto uid = nodes_mapping_old2new[e.first];
            auto& edges_copy = e.second;
            auto& copy_to = node_in_edge[uid];
            copy_to.reserve(edges_copy.size());
            for(auto& ec : edges_copy) {
                auto ecc = edges[edges_mapping_old2new[ec->unique_id()]];
                copy_to.push_back(ecc);
            }
            CTOP_ASSERT(copy_to.size() == edges_copy.size());
        }
        CTOP_ASSERT(inf.node_in_edge.size() == node_in_edge.size());

        active.reserve(inf.active.size());
        for(auto& e : inf.active) {
            active.push_back(nodes[nodes_mapping_old2new[e->unique_id()]]);
        }
        CTOP_ASSERT(inf.active.size() == active.size());

        unvisited.reserve(inf.unvisited.size());
        for(auto& e : inf.unvisited) {
            unvisited.push_back(nodes[nodes_mapping_old2new[e->unique_id()]]);
        }
        CTOP_ASSERT(inf.unvisited.size() == unvisited.size());

        placed_nodes.reserve(inf.placed_nodes.size());
        for(auto& e : inf.placed_nodes) {
            placed_nodes.push_back(nodes[nodes_mapping_old2new[e->unique_id()]]);
        }
        CTOP_ASSERT(inf.placed_nodes.size() == placed_nodes.size());

        unvisited_edges.reserve(inf.unvisited_edges.size());
        for(auto& e : inf.unvisited_edges) {
            unvisited_edges.push_back(edges[edges_mapping_old2new[e->unique_id()]]);
        }
        CTOP_ASSERT(inf.unvisited_edges.size() == unvisited_edges.size());


        for(auto& e : inf.resource_bucket) {
            auto uid = e.first;
            auto& res_array = e.second;
            auto& arr = resource_bucket[uid];
            arr.reserve(res_array.size());
            for(auto& ress : res_array) {
                // copy resource
                auto new_res = std::make_shared<Resource>(*ress);
                // rewire node reference
                if(ress->node != nullptr) {
                    new_res->node = nodes[nodes_mapping_old2new[ress->node->unique_id()]];
                }
                arr.push_back(new_res);
            }
            CTOP_ASSERT(arr.size() == res_array.size());
        }
        CTOP_ASSERT(inf.resource_bucket.size() == resource_bucket.size());
    }

    void reset_node_data() {
        for(auto& n : nodes) {
            n.second->data = std::make_shared<NodeData>();
        }
        for(auto& n : edges) {
            n.second->data = std::make_shared<EdgeData>();
        }
    }

    void add_brick_nodes(std::vector<Brick>& bricks) {
        for(auto& b : bricks) {
            auto n = std::make_shared<WallNode>();
            n->brick = &b;
            nodes[n->unique_id()] = n;
            brick_nodes[b.id] = n;
            CTOP_LOG_D("add node {} [{}, {}, {}]", b.id, b.x, b.y, b.z);
        }
    }

    void add_to_start(Brick& b) {
        b.print();
        auto n = brick_nodes[b.id];
        CTOP_ASSERT(n != nullptr);
        start_nodes.push_back(n);
    }

    std::shared_ptr<WallEdge> add_edge(const std::shared_ptr<WallNode>& n1, const std::shared_ptr<WallNode>& n2) {
        CTOP_ASSERT(n1 != nullptr);
        CTOP_ASSERT(n2 != nullptr);

        auto n = std::make_shared<WallEdge>();
        CTOP_ASSERT(n != nullptr);

        n->start = n1;
        n->end = n2;
        node_out_edge[n1->unique_id()].push_back(n);
        node_in_edge[n2->unique_id()].push_back(n);

        edges[n->unique_id()] = n;
        return n;
    }

    std::shared_ptr<WallEdge> add_dir_edge(Brick& b1, Brick& b2, WallEdgeType type) {
        auto n1 = brick_nodes[b1.id];
        CTOP_ASSERT(n1 != nullptr);
        auto n2 = brick_nodes[b2.id];
        CTOP_ASSERT(n2 != nullptr);
        auto n = add_edge(n1, n2);
        n->type = type;
        CTOP_LOG_D("add edge {} -> {} type={}", b1.id, b2.id, edge_type_2_string(n->type));
        return n;
    }

    void add_both_dir_edge(Brick& b1, Brick& b2, WallEdgeType type) {
        auto n1 = add_dir_edge(b1, b2, type);
        auto n2 = add_dir_edge(b2, b1, type);
    }

    bool is_start_node(const std::shared_ptr<WallNode>& node) const noexcept {
        for(const auto& n : start_nodes) {
            if(n->unique_id() == node->unique_id()) {
                return true;
            }
        }
        return false;
    }

    bool update_time(int min_needed_res, std::vector<std::shared_ptr<Resource>>& finish_resources, std::vector<std::shared_ptr<Resource>>& waiting_resources) {
        // find first resource that will finish
        uint64_t minTime = std::numeric_limits<uint64_t>::max();
        uint64_t waitTime = std::numeric_limits<uint64_t>::max();

        std::map<uint64_t, std::unordered_map<int,std::vector<std::shared_ptr<Resource>>>> res_wait_map;
        std::map<uint64_t, std::unordered_map<int,std::vector<std::shared_ptr<Resource>>>> res_min_map;
        std::map<uint64_t, std::unordered_map<int,std::vector<std::shared_ptr<Resource>>>> res_map;

        CTOP_LOG_I("min_needed_res={}", min_needed_res);

        for(auto&& a : resource_bucket) {
            for(auto&& b : a.second) {
                auto node = b->node;
                if(node != nullptr) {
                    // resources assigned to task
                    if(b->end_time == 0) continue;
                    res_min_map[b->available_time][b->type].push_back(b);
                    res_map[b->available_time][b->type].push_back(b);
                    if(b->end_time < minTime) {
                        finish_resources.clear();
                        minTime = b->end_time;
                        finish_resources.push_back(b);
                    }else if(b->end_time == minTime) {
                        finish_resources.push_back(b);
                    }
                } else {
                    res_wait_map[b->available_time][b->type].push_back(b);
                    res_map[b->available_time][b->type].push_back(b);
                    // resources waiting for task
                    if(b->available_time == 0) {
                        waiting_resources.push_back(b);
                        continue;
                    }
                    if(b->available_time < waitTime) {
                        waiting_resources.clear();
                        waitTime = b->available_time;
                        waiting_resources.push_back(b);
                    }else if(b->available_time == waitTime) {
                        waiting_resources.push_back(b);
                    }
                }
            }
        }
        CTOP_LOG_I("waiting_resources: {}", waiting_resources.size());

        if(waiting_resources.size() < min_needed_res) {

            auto it_res_wait_map = res_wait_map.begin();
            auto it_res_min_map = res_min_map.begin();

            bool found = false;
            uint64_t res_sum = 0;
            for(auto&& a : res_map) {
                auto time = a.first;
                auto& r_map = a.second;
                CTOP_LOG_I("res_map[{}].size={}", time, r_map.size());
                for(auto&& b : r_map) {
                    auto res_type = b.first;
                    auto& resources = b.second;
                    auto res_size = resources.size();

                    res_sum += res_size;
                    CTOP_LOG_I("wait type={} time: {} res: {} ressum: {}", res_type, time, res_size, res_sum);
                    if(res_sum >= min_needed_res) {
                        found = true;
                        waiting_resources = resources;
                        waitTime = time;
                        break;
                    }
                }
            }

            if(!found) {
                for(auto&& a : res_wait_map) {
                    res_sum = 0;
                    auto time = a.first;
                    auto& r_map = a.second;

                    for(auto&& b : r_map) {
                        auto res_type = b.first;
                        auto& resources = b.second;
                        auto res_size = resources.size();

                        res_sum += res_size;
                        CTOP_LOG_I("wait type={} time: {} res: {}", res_type, time, res_size);
                        if(res_sum >= min_needed_res) {
                            found = true;
                            waiting_resources = resources;
                            waitTime = time;
                            break;
                        }
                    }
                }
            }
            if(!found) {
                auto needed = min_needed_res - waiting_resources.size();
                for(auto&& a : res_min_map) {
                    auto& time = a.first;
                    auto& r_map = a.second;
                    for(auto&& b : r_map) {
                        auto& res_type = b.first;
                        auto& resources = b.second;
                        auto res_size = resources.size();
                        res_sum += res_size;
                        CTOP_LOG_I("min type={} time: {} res: {}", res_type, time, res_size);
                        if(res_sum >= needed) {
                            found = true;
                            minTime = time;
                            break;
                        }
                    }
                }
            }


            if(!found) {
                CTOP_LOG_E("deadlock, cannot process further");
                CTOP_ASSERT(false);
                done = true;
                return true;
            }
        }


        auto prevTime = actualTime;
        uint64_t reqTime = 0;
        if(waitTime <= minTime) {
            // all resources are available
            CTOP_LOG_I("time moved wait");
            reqTime = waitTime;
        } else {
            CTOP_LOG_I("time moved min");
            reqTime = minTime;
        }
        if(reqTime >= actualTime) {
            bool same = actualTime == reqTime;
            actualTime = reqTime;
            CTOP_LOG_I("time moved {} -> {}, min={}, wait={}", prevTime, actualTime, minTime, waitTime);
            return !same;
        } else {
            CTOP_LOG_I("req time is lower then actual min={}, wait={}, req={} actual={}", minTime, waitTime,reqTime, actualTime);
            return false;
        }
    };

    uint64_t available_res() {
        uint64_t count_max = 0;
        for(auto&& a : resource_bucket) {
            uint64_t count = 0;
            for(auto&& b : a.second) {
                if(b->node == nullptr) {
                    count++;
                }
            }
            if(count > count_max) {
                count_max = count;
            }
        }
        return count_max;
    };

    std::vector<std::shared_ptr<Resource>> get_available_res(uint64_t count, const std::shared_ptr<WallNode>& node) {
        std::vector<std::shared_ptr<Resource>> out;
        out.reserve(count);
        for(auto&& a : resource_bucket) {

            // when we heve multiple types of robots and we need to assing multiple of them we want only one type of the robot
            out.clear();

            for(auto&& res : a.second) {
                if(node->brick->z <= res->max_z) {
                    if(res->node == nullptr && res->available_time <= actualTime) {
                        out.push_back(res);
                        if(out.size() == count) return out;
                    }
                }
            }
        }
        if(out.size() != count) out.clear();
        return out;
    }

    void add_solution(int id, int time, const std::shared_ptr<Resource>& res, const std::shared_ptr<WallNode>& node) {
        auto& data = res->brick_placement->brick_2_data[node->brick->type];

        auto end_time = time + data.duration;
        if(!ignore_time_contraint) {
            if(end_time > info.budget_s) {
                return;
            }
        }

        sol.target_brick_ids[id].push_back(node->brick->id);
        sol.start_brick_times[id].push_back(time);
        sol.target_durations[id].push_back(data.duration);

        if(sol.end_time < end_time) {
            sol.end_time = end_time;
        }
        if(!node->done){
            sol.reward += node->brick->reward;
            sol.placed_bricks++;
            CTOP_LOG_W("reward={} +{}", sol.reward, node->brick->reward);
        } else {
            CTOP_LOG_W("reward={} +0 already processed", sol.reward);
        }

    };


    void init(int start_brick_id = -1) {
        sol.target_brick_ids.clear();
        sol.target_durations.clear();
        sol.start_brick_times.clear();
        sol.reward = 0;

        rng.seed(time(nullptr));

        placed_nodes.reserve(nodes.size());
        if(start_brick_id < 0 || start_brick_id >= start_nodes.size()) {
            std::uniform_int_distribution<std::mt19937::result_type> dist6(0,start_nodes.size()-1); // distribution in range [1, 6]
            CTOP_LOG_I("rng seed {}", rng.default_seed);
            start_brick_id = dist6(rng);
        }

        auto n = start_nodes[start_brick_id];
        CTOP_LOG_I("starting brick {}", n->brick->id);

        auto start_edge = std::make_shared<WallEdge>();
        start_edge->end = n;
        start_edge->available_time = 0;
        start_edge->start = nullptr;
        edges[start_edge->unique_id()] = start_edge;
        unvisited_edges.emplace_back(start_edge);

        if(resource_bucket.empty()) {
            for(int i = 0; i<info.num_robots; i++) {
                auto res = std::make_shared<Resource>();
                res->id = i;
                resource_bucket[0].push_back(res);
            }
        }
    }

    void print_iteration_info() {
        CTOP_LOG_V("******* iter {} time {} done {} *******", iter, actualTime, done);
        for(auto&& a : resource_bucket) {
            for(auto&& res : a.second) {
                auto node = res->node;
                bool has_brick = node != nullptr;
                std::string state_str;
                if(actualTime >= res->end_time) {
                    if(actualTime >= res->available_time) {
                        state_str = "IDLE";
                    } else {
                        state_str = "RESTING";
                    }
                } else {
                    state_str = "WORKING";
                }
                CTOP_LOG_V("type={} res id={} brick={} start_time={} end_time={} available_time={} ...{}", a.first, res->id, has_brick ? node->brick->id : -1, res->start_time, res->end_time, res->available_time, state_str);
            }
            CTOP_LOG_V("");
        }
    }

    void print_res_info() {
        for(auto&& a : resource_bucket) {
            for(auto&& res : a.second) {
                auto node = res->node;
                bool has_brick = node != nullptr;
                std::string state_str;
                if(actualTime >= res->end_time) {
                    if(actualTime >= res->available_time) {
                        state_str = "IDLE";
                    } else {
                        state_str = "RESTING";
                    }
                } else {
                    state_str = "WORKING";
                }
                CTOP_LOG_W("type={} res id={} brick={} start_time={} end_time={} available_time={} ...{}", a.first, res->id, has_brick ? node->brick->id : -1, res->start_time, res->end_time, res->available_time, state_str);
            }
            CTOP_LOG_W("");
        }
    }

    bool iter_process_edges() {
        added_active_nodes = 0;
        max_available_time_glob = 0;
        ignored.clear();

        CTOP_LOG_D("available unvisited_edges={}", unvisited_edges.size());
        for(auto& edge : unvisited_edges) {
            if(edge == nullptr) continue;

            CTOP_ASSERT(edge != nullptr);

            if(edge->available_time > max_available_time_glob) {
                max_available_time_glob = edge->available_time;
            }

            if(edge->available_time > actualTime) {
                CTOP_LOG_D("edge ignored available={} from {} -> {}", edge->available_time, edge->start_brick_id(), edge->end_brick_id());
                ignored.push_back(edge);
                continue;
            }
            auto node = edge->end;
            if (node->done || node->in_use) {
                //ignore
                continue;
            }
            // mark this edge as active
            edge->used = true;
            unvisited.push_back(node);
        }
        ignored_empty = ignored.empty();
        CTOP_LOG_D("available ignored_edges={}", ignored.size());
        unvisited_edges = std::move(ignored);
        if(!ignored_empty && unvisited.empty() && max_available_time_glob > actualTime && active.empty()) {
            CTOP_LOG_E("no nodes added, update time {} -> {}", actualTime, max_available_time_glob);
            actualTime = max_available_time_glob;
            iter++;
            return true;
        }
        return false;
    }

    bool iter_process_nodes() {
        std::set<WallNode::uid> processed;
        // process nodes after edge was used
        for(auto& node : unvisited) {
            // find new nodes for building wall
            {
                // check duplicate elements
                auto is_in = processed.find(node->unique_id()) != processed.end();
                if(is_in) continue;
                processed.insert(node->unique_id());
                bool ignore =false;
                for(auto&& a : placed_nodes) {
                    if(a->brick != nullptr && node->brick != nullptr) {
                        if(a->brick->id == node->brick->id) {
                            ignore = true;
                            break;
                        }
                    }
                }
                if(ignore) {
                    continue;
                }
                for(auto&& r : resource_bucket) {
                    for(auto&& v : r.second) {
                        if(v->node == nullptr) continue;
                        if(v->node->unique_id() == node->unique_id()) {
                            ignore =true;
                            break;
                        }
                    }
                }
                if(ignore) {
                    continue;
                }
            }


            if(!node->done) {
                node->visited++;
                auto& node_in_edges = node_in_edge[node->unique_id()];
                auto req_visits = 0;

                uint64_t used_edges = 0;
                uint64_t max_available_time = 0;

                uint64_t precedence_max = 0;
                uint64_t precedence_used = 0;

                uint64_t side_max = 0;
                uint64_t side_used = 0;

                uint64_t distance_max = 0;
                uint64_t distance_used = 0;

                uint64_t side_activate_max = 0;
                uint64_t side_activate_used = 0;

                for(auto&& e : node_in_edges) {
                    //CTOP_ASSERT(e->type != WallEdgeType::none);
                    switch(e->type) {
                        case WallEdgeType::precedence: {
                            precedence_max++;
                            if(e->used) {
                                precedence_used++;
                            }
                            break;
                        }
                        case WallEdgeType::distance: {
                            distance_max++;
                            if(e->used && e->distance_valid) {
                                if(info.min_distance_concurrent_brick_placing_cm > 0) {
                                    auto distance = Brick::brick_distance(e->start->brick, e->end->brick)*100.0; //cm
                                    if(distance <= info.min_distance_concurrent_brick_placing_cm) {
                                        distance_used++;
                                    }
                                }
                            }
                            break;
                        }
                        case WallEdgeType::side: {
                            side_max++;
                            if(e->used) {
                                side_used++;
                            }
                            break;
                        }
                        case WallEdgeType::side_activate: {
                            side_activate_max++;
                            if(e->used) {
                                side_activate_used++;
                            }
                            break;
                        }
                        case WallEdgeType::none:
                        default: {
                            break;
                        }
                    }

                    auto w = e->type == WallEdgeType::precedence ? e->weight : 0;
                    if(e->used) {
                        used_edges += w;
                    }
                    req_visits += w;
                    if(e->available_time > max_available_time) {
                        max_available_time = e->available_time;
                    }
                }
                // should be true from previous phase with edges
                //CTOP_ASSERT(max_available_time <= actualTime);

                CTOP_LOG_W("edges: precedence {}/{} side {}/{} distance {}/{} side_act {}/{}",
                           precedence_used, precedence_max,
                           side_used, side_max,
                           distance_used, distance_max,
                           side_activate_used, side_activate_max);

                if(is_start_node(node)) {
                    if(!start_used) {
                        start_used = true;
                        req_visits = 0;
                    } else {
                        req_visits = 1;
                    }
                } else {
                    if(node->brick != nullptr) {
                        --req_visits;
                    }
                }
                CTOP_LOG_D("actual_time={} visiting node id={} brick[{:p}, id={}] visited={} req={} used_edges={} max_available_time={}",
                           actualTime,
                           node->unique_id(),
                           (void*)node->brick,
                           node->brick == nullptr ? -1 : node->brick->id,
                           node->visited,
                           req_visits,
                           used_edges,
                           max_available_time_glob);

                bool can_activate = false;

                // precedence rules met
                if(precedence_max == precedence_used) {
                    if(distance_used == 0) {
                        can_activate = true;
                    }
                }

                if(can_activate && can_be_assigned(node)) {
                    if(placed_nodes.size() == brick_nodes.size()) {
                        CTOP_LOG_D("done");
                        done = true;
                        break;
                    }
                    // end node
                    // brick can be placed, all previous conditions are met
                    CTOP_LOG_D("brick id={} visited", node->brick->id);

                    // move to active
                    active.push_back(node);
                    added_active_nodes++;
                }
            }
        }
        unvisited.clear();
        if(done) {
            return true;
        }

        CTOP_LOG_D("added_active_nodes={}, total={}, max_available_time_glob={}", added_active_nodes, active.size(), max_available_time_glob);

        return false;
    }

    void select_next_nodes(std::vector<std::shared_ptr<WallNode>>& selected_nodes) {
        // pick the the best available nodes
        auto max_available = ava_res;

        CTOP_LOG_D("available active={}", active.size());


        std::map<uint64_t, std::vector<std::shared_ptr<WallNode>>> max_reward_nodes;
        auto max_reward = 0;

        for(auto& active_node : active) {
            if(!can_be_assigned(active_node)) {
                print_res_info();
                CTOP_LOG_D("could not assign brick={} min_concurrentplacement={}", active_node->brick->id, info.min_distance_concurrent_brick_placing_cm);
                continue;
            }
            bool used = false;
            for(auto&& b : selected_nodes) {
                if(active_node->unique_id() == b->unique_id()) {
                    used = true;
                    break;
                }
            }
            if(used) continue;

            auto req_res = active_node->brick->req_robots;
            // find maximum that still can be fitted to available bucket
            if(max_available >= req_res) {
                auto reward = active_node->brick->reward;
                if(reward > max_reward) {
                    max_reward = reward;
                }
                max_reward_nodes[reward].push_back(active_node);
            }
            CTOP_LOG_D("brick id={} max_reward_nodes size={}, max_reward={} max_available={}", active_node->brick == nullptr ? -1 : active_node->brick->id, max_reward_nodes.size(), max_reward, max_available);
        }
        CTOP_LOG_D("max_reward_nodes size={}, max_reward={} max_available={}", max_reward_nodes.size(), max_reward, max_available);
        if(max_reward_nodes.empty()) {
            return;
        }

        // pick random node to place
        do {
            // pick random reward
            std::uniform_int_distribution<std::mt19937::result_type> dist(0,max_reward_nodes.size()-1);
            auto random_reward_index = dist(rng);
            auto reward_it = max_reward_nodes.begin();
            // move reward iterator by random position
            std::advance(reward_it, random_reward_index);
            //for(int i =0; i<random_reward_index; i++, reward_it++);
            CTOP_LOG_D("picking reward={}", reward_it->first);
            auto& rewards = reward_it->second;

            // pick random node from reward
            std::uniform_int_distribution<std::mt19937::result_type> dist2(0,rewards.size()-1);
            auto random_node_index = dist2(rng);
            auto node_it = rewards.begin()+random_node_index;
            auto node = *node_it;

            // add to selected brick to be placed
            max_available -= node->brick->req_robots;
            selected_nodes.push_back(node);

            // remove node from array
            rewards.erase(node_it);
            // remove reward array if empty
            if(rewards.empty()) {
                max_reward_nodes.erase(reward_it);
                if(max_reward_nodes.empty()) {
                    return;
                }
            }
        }while(true);

        CTOP_LOG_D("selected_nodes size={}", selected_nodes.size());
    }

    bool can_be_assigned(std::shared_ptr<WallNode>& node) {
        uint64_t processed_nodes = 0;
        for(auto&& a : resource_bucket) {
            for (auto &&res : a.second) {
                if (res->node == nullptr) continue;
                processed_nodes++;

                auto brick_dist = Brick::brick_distance(node->brick, res->node->brick);
                CTOP_LOG_D("node brick={} res id={} distance={}", node->brick->id, res->node->brick->id, brick_dist);
                if (brick_dist * 100.0 <= info.min_distance_concurrent_brick_placing_cm) {
                    return false;
                }



                auto& out_edges = node_out_edge[res->node->unique_id()];
                for(auto& e : out_edges) {
                    if(e->type == WallEdgeType::side) {
                        CTOP_LOG_D("res brick={} side to brick id={}", res->node->brick->id, e->end->brick->id);
                        if(e->end->brick->id == node->brick->id) {
                            return false;
                        }
                    }
                }
            }
        }
        return true;
    }

    int iter_process_assign_nodes() {
        // check nodes that are available for placing
        assinged_res = 0;
        ava_res = available_res();
        remove_active.clear();
        active_count = active.size();

        // pick the the best available nodes
        auto max_available = ava_res;
        std::vector<std::shared_ptr<WallNode>> selected_nodes;
        select_next_nodes(selected_nodes);
        auto min_req_resources = std::numeric_limits<uint64_t>::max();

        bool stop = false;
        CTOP_LOG_D("selected_nodes.size = {}", selected_nodes.size());
        CTOP_LOG_E("active={}", active.size());

        if(!selected_nodes.empty()) {
            no_process_count = 0;
            for(auto& active_node : selected_nodes) {

                auto req_res = active_node->brick->req_robots;
                auto ress = get_available_res(req_res, active_node);
                CTOP_LOG_D("get_available_res()={}, req_res={}", ress.size(), req_res);
                if(req_res < min_req_resources) {
                    min_req_resources = req_res;
                }
                if(ress.size() == req_res) {
                    assinged_res += req_res;
                    active_node->in_use = true;
                    for(int i = 0; i != req_res; ++i) {
                        // assign brick to resource
                        auto res = ress[i];
                        res->node = active_node;

                        auto& placement_info = res->brick_placement->brick_2_data[active_node->brick->type];

                        // compute working times
                        res->start_time = actualTime;
                        res->end_time = res->start_time + placement_info.duration;
                        res->available_time = res->end_time + placement_info.reservoir_time;


                        auto diff = res->available_time - res->start_time;
                        if(res->battery_state <= diff) {
                            res->available_time += res->battery_recharge;
                            res->battery_state = res->battery_budget;
                        } else {
                            res->battery_state -= diff;
                        }

                        auto& node_in_edges = node_in_edge[active_node->unique_id()];

                        bool has_side_placed = true;
                        for(auto&& e : node_in_edges) {
                            //CTOP_ASSERT(e->type != WallEdgeType::none);
                            switch (e->type) {
                                case WallEdgeType::side: {
                                    if (e->used) {
                                        has_side_placed = true;
                                    }
                                    break;
                                }
                                default: {
                                    break;
                                }
                            }
                            if(has_side_placed) {
                                break;
                            }
                        }
                        auto& node_out_edges = node_out_edge[active_node->unique_id()];
                        for (auto &&e : node_out_edges) {
                            switch(e->type) {
                                case WallEdgeType::side_activate: {
                                    e->available_time = actualTime;
                                    unvisited_edges.emplace_back(e);
                                }
                                case WallEdgeType::distance: {
                                    e->distance_valid = true;
                                    e->available_time = actualTime;
                                    unvisited_edges.emplace_back(e);
                                    break;
                                }
                                default: {
                                    //ignore
                                }
                            }
                        }


                        CTOP_LOG_E("assigned brick id={} robot_id={} start={} end={} available={}", res->node->brick->id, res->id, res->start_time, res->end_time, res->available_time);
                        //clear nodes and check boundary again
                    }

                    auto auid = active_node->unique_id();
                    // reset all other nodes and check contraints
                    for(auto&& a : active) {
                        if(auid != a->unique_id()) {
                            unvisited.push_back(a);
                        }
                    }
                    active.clear();
                    break;
                } else {
                    CTOP_LOG_E("not enough resources", ress.size(), req_res);
                }
            }

            /*for(auto&& id : remove_active) {
                active.erase(id);
            }*/
            CTOP_LOG_D("active_bef={} assigned={} active_aft={} available_res={} max_glob_time={}", active_count, assinged_res, active.size(), available_res(), max_available_time_glob);
        } else {
            no_process_count++;
            CTOP_LOG_E("selected nodes are empty={}", no_process_count);
            if(no_process_count > 10) {
                return -1;
            }
            return 0;
        }
        CTOP_LOG_I("min_req_resources={}", min_req_resources);
        return min_req_resources;
    }

    void iter_process_finished_nodes() {
        uint64_t end_time = std::numeric_limits<uint64_t>::max();
        for(auto&& b : finish_resources) {
            auto node = b->node;
            if(node == nullptr) continue;

            b->node = nullptr;
            node->in_use = false;
            CTOP_LOG_W("brick id={} placed by robot id={} start={} end={} available={}", node->brick->id, b->id,
                       b->start_time, b->end_time, b->available_time);
            add_solution(b->id, b->start_time, b, node);
            if(b->end_time < end_time) {
                end_time = b->end_time;
            }
            auto already_placed = false;
            for(auto&& n : placed_nodes) {
                if(n->unique_id() == node->unique_id()) {
                    already_placed = true;
                    break;
                }
            }
            if(!already_placed) {
                placed_nodes.push_back(node);
            }

            CTOP_LOG_W("progress {}/{}", placed_nodes.size(), brick_nodes.size());

            node->done = true;
            auto& node_out_edges = node_out_edge[node->unique_id()];
            CTOP_LOG_D("out edges size={}", node_out_edges.size());
            for (auto &&e : node_out_edges) {
                e->available_time = b->end_time;
                if(e->type == WallEdgeType::distance) {
                    e->used = false;
                    e->distance_valid = false;
                    continue;
                }
                CTOP_LOG_I("add edge from brick id={} to id={} available_time={} actual_time={}", e->start_brick_id(), e->end_brick_id(), b->end_time, actualTime);
                unvisited_edges.emplace_back(e);
            }
        }
        if(active.empty() && end_time != std::numeric_limits<uint64_t>::max()) {
            CTOP_LOG_E("nodes assigned, update time {} -> {}", actualTime, end_time);
            actualTime = end_time;
        }
    }

    void iterate_step() {
        CTOP_LOG_D("active={}", active.size());
        CTOP_LOG_D("unvisited={}", unvisited.size());
        // check available edge connection between nodes
        if(iter_process_edges()) {
            return;
        }
        CTOP_LOG_D("active={}", active.size());
        CTOP_LOG_D("unvisited={}", unvisited.size());
        // check all nodes that can be accessed from edges
        if(iter_process_nodes()) {
            return;
        }
        CTOP_LOG_D("active={}", active.size());
        CTOP_LOG_D("unvisited={}", unvisited.size());
        // assign bricks that can be places to free resources
        auto needed_res = iter_process_assign_nodes();
        if(needed_res == -1) {
            CTOP_LOG_E("contraints could not be met");
            done = true;
            return;
        }
        // when no resources was assigned to brick
        if(assinged_res == 0) {
            // update time to find what resources finished
            finish_resources.clear();
            waiting_resources.clear();

            if(!update_time(needed_res, finish_resources, waiting_resources)) {

                if(!ignore_time_contraint && actualTime > info.budget_s) {
                    CTOP_LOG_I("budget exhaused, done.");
                    done = true;
                    return;
                }

                // place finished brick and free resource
                iter_process_finished_nodes();
                CTOP_LOG_E("max_available_time_glob {}", max_available_time_glob);
                if(added_active_nodes == 0 && max_available_time_glob > actualTime) {
                    CTOP_LOG_E("no nodes added, update time {} -> {}", actualTime, max_available_time_glob);
                    actualTime = max_available_time_glob;
                }
            }
        }


        if(!ignore_time_contraint && actualTime > info.budget_s) {
            CTOP_LOG_I("budget exhaused, done.");
            done = true;
            return;
        }

    }

    void iterate() {
        auto start = std::chrono::high_resolution_clock::now();
        print_iteration_info();
        if(iter == 0) {
            init(start_index);
        } else {
            iterate_step();
        }
        auto end = std::chrono::high_resolution_clock::now();
        CTOP_LOG_D("iter {} took {}", iter, ctop::duration_to_string(end - start));
        iter++;
    }

    bool is_done() {
        return iter >= 100000 || done;
    }

    WallSolution solve() {
        init(start_index);

        auto start = std::chrono::high_resolution_clock::now();
        do {
            iterate();
        } while(!is_done());
        auto end = std::chrono::high_resolution_clock::now();

        CTOP_LOG_D("done iter={} reward={} took={}", iter, sol.reward, duration_to_string(end-start));
        return sol;
    }


    void print_dot(WallInfo& info, std::ostream& out) const {
        out << "digraph {\n";
        out << "rankdir=BT;\n";
        out << "splines=false;\n";
        out << "clusterrank=local;\n";
        out << "ranksep=2;\n";
        std::unordered_map<int, std::vector<std::shared_ptr<WallNode>>> layers;

        for(const auto& n : nodes) {
            layers[n.second->brick->layer].push_back(n.second);
        }

        //out << "subgraph cluster_0{\n";
        for(auto&& l : layers) {
            out << "subgraph cluster_" << (l.first+1) << " {\n";
            out << "style=invis\n";
            out << "rank=same\n";
            for(auto& n : l.second) {

                bool color = false;
                if(color) {
                    out << n->brick->id << " [shape=box, style=filled, fillcolor=";
                    auto& type = info.brick_types[n->brick->type];
                    if(type == "BLUE_BRICK") {
                        out << "blue";
                    } else if(type == "GREEN_BRICK") {
                        out << "green";
                    } else if(type == "RED_BRICK") {
                        out << "red";
                    } else if(type == "ORANGE_BRICK") {
                        out << "orange";
                    }
                    out << "]" << "\n";
                } else {
                    out << n->brick->id << " [shape=box]\n";
                }
            }
            for(const auto& n : edges) {
                if(n.second->type == WallEdgeType::side) {
                    if(n.second->start->brick->layer == l.first) {
                        out << n.second->start_brick_id() << "->" << n.second->end_brick_id() << " [arrowhead=vee color=red constraint=false]" << "\n";
                    }
                }
            }
            out << "}\n";
        }
        //out << "}\n";

        for(const auto& n : edges) {
            if(n.second->type == WallEdgeType::precedence) {
                out << n.second->start_brick_id() << "->" << n.second->end_brick_id() << " [arrowhead=vee]" << "\n";
            }
        }

        out << "splines=true;\n";
        for(const auto& n : edges) {
            if(n.second->type == WallEdgeType::distance) {
                auto e = n.second;
                if(Brick::brick_distance(e->start->brick, e->end->brick)*100.0 <= info.min_distance_concurrent_brick_placing_cm) {
                    out << n.second->start_brick_id() << "->" << n.second->end_brick_id() << " [arrowhead=vee color=blue constraint=false]" << "\n";
                }
            }
            if(n.second->type == WallEdgeType::side_activate) {
                out << n.second->start_brick_id() << "->" << n.second->end_brick_id() << " [arrowhead=vee color=green constraint=false]" << "\n";
            }
        }

        out << "}";
    }

};

}

#endif //CTOP_PLANNER_WALLGRAPH_H
