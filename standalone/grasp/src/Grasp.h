//
// Created by Michal Němec on 09/02/2020.
//

#ifndef CTOP_PLANNER_GRASP_H
#define CTOP_PLANNER_GRASP_H

#include <stdint.h>
#include <memory>
#include <utility>
#include <vector>
#include <memory>
#include <thread>
#include <ctop/util/QueueBuffer.h>
//#include <ctop/util/gate.h>

#include "WallGraph.h"

#define GRASP_ENABLE_LOG
#define GRASP_USE_DEBUG_LOG

#ifndef NDEBUG
#define GRASP_ASSERT(expr) \
    do { \
        if (!static_cast <bool>(expr)) { \
            ctop::log::a(#expr, __LINE__, __FILE__, CTOP_FUNCTION); \
        } \
        assert(expr); \
    } while (false)
#else
#define GRASP_ASSERT(tag, expr) assert(expr)
#endif

#ifdef GRASP_ENABLE_LOG
#ifdef GRASP_USE_DEBUG_LOG
#define GRASP_LOG_D(...) ctop::log::d_debug("", __LINE__, __FILE__, CTOP_FUNCTION, __VA_ARGS__)
#define GRASP_LOG_E(...) ctop::log::e_debug("", __LINE__, __FILE__, CTOP_FUNCTION, __VA_ARGS__)
#define GRASP_LOG_I(...) ctop::log::i_debug("", __LINE__, __FILE__, CTOP_FUNCTION, __VA_ARGS__)
#define GRASP_LOG_V(...) ctop::log::v_debug("", __LINE__, __FILE__, CTOP_FUNCTION, __VA_ARGS__)
#define GRASP_LOG_W(...) ctop::log::w_debug("", __LINE__, __FILE__, CTOP_FUNCTION, __VA_ARGS__)
#else
#define GRASP_LOG_D(...) ctop::log::d(__VA_ARGS__)
        #define GRASP_LOG_E(...) ctop::log::e(__VA_ARGS__)
        #define GRASP_LOG_I(...) ctop::log::i(__VA_ARGS__)
        #define GRASP_LOG_V(...) ctop::log::v(__VA_ARGS__)
        #define GRASP_LOG_W(...) ctop::log::w(__VA_ARGS__)
#endif
#else
#define GRASP_LOG_D(...)
    #define GRASP_LOG_E(...)
    #define GRASP_LOG_I(...)
    #define GRASP_LOG_V(...)
    #define GRASP_LOG_W(...)
#endif

namespace ctop {

struct GraspJob {
    std::function<void()> fn;
};

struct GraspSolution {

    int iter;
    std::chrono::nanoseconds took;
    std::vector<GraspSolution> states;
    std::shared_ptr<WallGraph> graph = nullptr;

    explicit GraspSolution() = default;

    explicit GraspSolution(std::shared_ptr<WallGraph> g)
    : graph(std::move(g)) {

    }

    void copy_graph() {
        if(graph != nullptr) {
            auto start = std::chrono::high_resolution_clock::now();
            auto graph_new = std::make_shared<WallGraph>(*graph);
            auto end = std::chrono::high_resolution_clock::now();
            GRASP_LOG_D("took {}", ctop::duration_to_string(end - start));
            CTOP_ASSERT(graph_new != nullptr);
            graph = graph_new;
        }
    }

    uint64_t reward() {
        if(graph == nullptr) return 0;
        return graph->sol.reward;
    }
};

struct GraspJobData {
    std::chrono::high_resolution_clock::time_point start;
    std::mutex candidates_mutex;
    std::vector<GraspSolution> candidates;
    uint32_t generated;
};

struct Grasp {

    std::atomic<bool> stop_workers{false};
    int max_iterations = 1000;
    int max_did_not_improve = 30;
    double snapshot_coef = 0;
    int max_threads = 1;//std::numeric_limits<int>::max();

    uint64_t seed = 0;
    std::chrono::nanoseconds duration_raw;

    std::vector<std::pair<uint64_t, uint64_t>> rewards;
    std::vector<GraspSolution> solutions;

    GraspSolution initial_solution;
    GraspSolution best_solution;

    std::mutex init_idx_mutex;
    std::vector<int> init_idx;
    std::vector<int> used_idx;

    QueueBuffer<std::shared_ptr<GraspJob>> pool_buffer;
    bool tpool_used = false;
    std::vector<std::thread> tpool;

    std::mutex tpool_mutex;
    std::map<int, uint64_t> tpool_utilization;
    std::map<int, std::chrono::high_resolution_clock::time_point> tpool_start;
    std::map<int, std::chrono::high_resolution_clock::time_point> tpool_end;

    std::mutex placed_mutex;
    int max_number_of_placed_bricks = 0;

    void select_random_solution(std::vector<GraspSolution>& candidates, std::function<void(GraspSolution)>&& on_done) {
        CTOP_LOG_D("picking random solution");
        CTOP_ASSERT(!candidates.empty());

        int max_reward = 0;
        int min_time = std::numeric_limits<int>::max();
        std::vector<GraspSolution> pick_solutions;
        for(auto&& candidate : candidates) {
            auto& sol = candidate.graph->sol;
            auto rew = sol.reward;
            auto end_time = sol.end_time;
            if(rew > max_reward) {
                max_reward = rew;
                min_time = end_time;
                pick_solutions.clear();
            }
            if(rew == max_reward) {
                if(end_time < min_time) {
                    min_time = end_time;
                    pick_solutions.clear();
                }
                if(end_time == min_time) {
                    pick_solutions.push_back(candidate);
                }
            }
        }
        std::uniform_int_distribution<std::mt19937::result_type> dist(0,pick_solutions.size()-1);
        auto randoms_index = dist(candidates[0].graph->rng);
        CTOP_LOG_V("random index={}", randoms_index);
        on_done(pick_solutions[randoms_index]);
    }

    // Fisher–Yates_shuffle
    std::vector<int> generate_uniform_numbers(std::size_t size, std::size_t max_size, std::mt19937& gen)
    {
        assert(size <= max_size);
        std::vector<int> b(max_size);
        for(std::size_t i = 0; i != max_size; ++i) {
            b[i] = i;
        }
        std::vector<int> out;
        out.reserve(size);
        for(std::size_t i = 0; i != size; ++i) {
            std::uniform_int_distribution<int> dis(0, b.size()-1);
            auto it = b.begin()+dis(gen);
            out.emplace_back(*it);
            b.erase(it);
        }
        return out;
    }

    void iterate_to_end(GraspSolution& solution) {
        auto start = std::chrono::high_resolution_clock::now();
        solution.states.reserve(solution.graph->nodes.size());
        auto k = 0;

        int save_placements_idx = 0;

        std::unique_lock<std::mutex> ll{placed_mutex};
        auto expected_placed = solution.graph->nodes.size();
        if(max_number_of_placed_bricks > 0) {
            expected_placed = max_number_of_placed_bricks;
        }
        ll.unlock();

        auto number_of_elements = static_cast<int>(expected_placed*snapshot_coef);
        int actual_save;
        std::vector<int> save_placements;
        if(number_of_elements == 0) {
            actual_save = -1;
        } else {
            auto max = expected_placed;
            save_placements = generate_uniform_numbers(number_of_elements, max, solution.graph->rng);
            std::sort(save_placements.begin(), save_placements.end());
            actual_save = save_placements[save_placements_idx];
        }

        while(!solution.graph->is_done() && !stop_workers) {
            // iterate until brick is placed
            auto rew = solution.reward();
            while(!solution.graph->is_done() && solution.reward() == rew && !stop_workers) {
                solution.graph->iterate();
            }
            GRASP_LOG_D("iteration {}", k);
            if(k == actual_save) {
                auto cp = solution;
                cp.copy_graph();
                solution.states.push_back(cp);
                save_placements_idx++;
                actual_save = save_placements[save_placements_idx];
            }
            k++;
        }
        auto end = std::chrono::high_resolution_clock::now();
        CTOP_LOG_D("took {}", ctop::duration_to_string(end - start));

        ll.lock();
        auto placed = solution.graph->placed_nodes.size();
        if(placed > max_number_of_placed_bricks) {
            max_number_of_placed_bricks = placed;
        }
        ll.unlock();
    }

    void iterate_to_end_only(GraspSolution& solution) {
        auto start = std::chrono::high_resolution_clock::now();
        solution.states.reserve(solution.graph->nodes.size());
        auto k = 0;
        while(!solution.graph->is_done() && !stop_workers) {
            // iterate until brick is placed
            auto rew = solution.reward();
            while(!solution.graph->is_done() && solution.reward() == rew && !stop_workers) {
                solution.graph->iterate();
            }
        }
        auto end = std::chrono::high_resolution_clock::now();
        CTOP_LOG_D("took {}", ctop::duration_to_string(end - start));
    }

    void pick_initial_brick(GraspSolution& solution) {
        std::lock_guard<std::mutex> ll{init_idx_mutex};
        auto start_max = solution.graph->start_nodes.size();
        if(init_idx.empty()) {
            if(used_idx.empty()) {
                init_idx.reserve(start_max);
                used_idx.reserve(start_max);
                for(int j = 0; j<3; j++) {
                    for(int i = 0; i<start_max; i++) {
                        init_idx.push_back(i);
                    }
                }
            } else {
                init_idx = std::move(used_idx);
                used_idx.clear();
            }
        }

        std::uniform_int_distribution<std::mt19937::result_type> dist(0,init_idx.size()-1);
        auto idx = dist(solution.graph->rng);
        auto it = init_idx.begin()+idx;
        auto index = *it;
        solution.graph->start_index = index;
        init_idx.erase(it);
        used_idx.push_back(index);
    }

    void greedy_randomized_construction(const std::function<void(bool, GraspSolution)>& on_done) {
        auto start = std::chrono::high_resolution_clock::now();
        auto stop = false;

        auto solution = initial_solution;
        solution.copy_graph();

        pick_initial_brick(solution);
        CTOP_LOG_D("pick index took {}", ctop::duration_to_string(std::chrono::high_resolution_clock::now() - start));
        iterate_to_end(solution);
        CTOP_LOG_D("total took {}", ctop::duration_to_string(std::chrono::high_resolution_clock::now() - start));
        run_job([=](){
            on_done(stop, solution);
        });
    }

    void local_search(const std::shared_ptr<GraspJobData>& job_data, GraspSolution& solution, const std::function<void(GraspSolution)>& on_done) {
        bool stop = false;
        make_rlc(job_data, solution, [=](std::vector<GraspSolution>& candidates){
            select_random_solution(candidates, [=](GraspSolution solution){
                on_done(std::move(solution));
            });
        });
    }

    void make_rlc(const std::shared_ptr<GraspJobData>& job_data, GraspSolution& solution, const std::function<void(std::vector<GraspSolution>&)>& on_done) {
        CTOP_LOG_D("generating restricted candidate list");

        job_data->candidates.clear();
        if(solution.states.empty()) {
            job_data->candidates.push_back(solution);
            on_done(job_data->candidates);
            return;
        }

        // pick random positions on the solution and iterate new solution to the end
        auto generate_candidates = solution.states.size();
        job_data->candidates.reserve(generate_candidates);
        job_data->generated = 0;
        for(int i =0; i<generate_candidates; i++) {
            run_job([=](){
                auto new_sol = solution.states[i];
                //new_sol.copy_graph();
                iterate_to_end_only(new_sol);
                if(stop_workers) return;
                {
                    std::lock_guard<std::mutex> ll{job_data->candidates_mutex};
                    job_data->candidates.push_back(new_sol);
                    job_data->generated++;
                    if(job_data->generated == generate_candidates) {
                        on_done(job_data->candidates);
                    }
                }
            });
        }
    }

    bool update_solution(GraspSolution& solution) {
        auto did_improve = false;
        CTOP_LOG_V("solution update best={} found={}", best_solution.reward(), solution.reward());
        if(solution.reward() > best_solution.reward()) {
            // update 'best_solution'
            best_solution = solution;
            did_improve = true;
        } else if(solution.reward() == best_solution.reward()) {
            if(best_solution.graph == nullptr || solution.graph->sol.end_time < best_solution.graph->sol.end_time) {
                // update 'best_solution'
                best_solution = solution;
                best_solution.states.clear();
                did_improve = true;
            }
        }
        //TODO just for monitoring, it is not needed for problem as is
        //solutions.push_back(solution);
        return did_improve;
    }

    void tpool_init() {
        auto n = static_cast<int>(std::thread::hardware_concurrency());
        n = std::max(1, std::min(n, max_threads));
        tpool_used = true;

        if(tpool_used) {
            GRASP_LOG_D("spawning {} worker threads", n);
            for(int i = 0; i != n; ++i) {
                tpool.emplace_back([&, i](){
                    std::unique_lock<std::mutex> ll{tpool_mutex};
                    tpool_start[i] = std::chrono::high_resolution_clock::now();
                    ll.unlock();
                    do {
                        auto val = pool_buffer.remove();
                        if(val == nullptr) break;
                        auto start = std::chrono::high_resolution_clock::now();
                        val->fn();
                        auto duration = std::chrono::high_resolution_clock::now() - start;

                        ll.lock();
                        tpool_utilization[i] += duration.count();
                        ll.unlock();

                    } while(true);
                    ll.lock();
                    tpool_end[i] = std::chrono::high_resolution_clock::now();
                    ll.unlock();
                });
            }
        }
    }

    void tpool_stop() {
        if(tpool_used) {
            for(int i = 0; i<tpool.size(); i++) {
                pool_buffer.add(nullptr);
            }
            for(auto & th : tpool) {
                if(th.joinable()) {
                    th.join();
                }
            }
            tpool_print();
        }
    }

    void tpool_print() {
        auto total_util = 0.0;
        for(int i = 0; i<tpool.size(); i++) {
            auto& start = tpool_start[i];
            auto& end = tpool_end[i];
            auto& util = tpool_utilization[i];
            auto runtime = end-start;
            auto util_perc = 100.0*static_cast<double>(util)/runtime.count();
            GRASP_LOG_D("thread {} runtime={} busy={} utilization={:.2f}%", i, duration_to_string(runtime), duration_to_string(std::chrono::nanoseconds{util}), util_perc);
            total_util += util_perc;
        }
        GRASP_LOG_D("total utilization: {:.2f}%", total_util);
    }

    void run_job(const std::function<void()>& fn) {
        if(stop_workers) return;
        if(tpool_used) {
            auto job = std::make_shared<GraspJob>();
            job->fn = fn;
            pool_buffer.add(job);
        } else {
            fn();
        }
    }

    void run_job(std::function<void()>&& fn) {
        if(stop_workers) return;
        if(tpool_used) {
            auto job = std::make_shared<GraspJob>();
            job->fn = std::forward<std::function<void()>>(fn);
            pool_buffer.add(job);
        } else {
            fn();
        }
    }

    GraspSolution run(std::shared_ptr<WallGraph> init) {
        initial_solution = GraspSolution();
        initial_solution.graph = std::move(init);

        auto tpool_start_init = std::chrono::high_resolution_clock::now();
        tpool_init();
        auto t_pool_end_init = std::chrono::high_resolution_clock::now();
        GRASP_LOG_D("tpool init took {}", duration_to_string(t_pool_end_init - tpool_start_init));

        auto stop = false;
        int did_not_improve = 0;
        rewards.reserve(max_iterations);
        QueueBuffer<GraspSolution> buf;

        std::atomic<int> k{0};
        std::function<void()> run_grasp_iter;
        std::function<void()> run_again = [&](){
            if(k < max_iterations) {
                run_job(run_grasp_iter);
            }
            k++;
        };

        run_grasp_iter = [&](){
            auto data = std::make_shared<GraspJobData>();
            data->start = std::chrono::high_resolution_clock::now();
            greedy_randomized_construction([&, data](bool stop, GraspSolution sol){
                if(stop) {
                    buf.add(sol);
                    return;
                }
                local_search(data, sol, [&, data](GraspSolution sol){
                    auto runtime = std::chrono::high_resolution_clock::now() - data->start;
                    sol.took = runtime;
                    buf.add(sol);
                    run_again();
                });
            });
        };

        auto start = std::chrono::high_resolution_clock::now();
        auto runs = std::max(static_cast<int>(tpool.size()), 1);

        for(int i = 0; i<runs; i++) {
            run_again();
        }

        std::chrono::nanoseconds took_total{0};
        int l = 0;
        for(; l < max_iterations && did_not_improve < max_did_not_improve;) {
            // process solution in batches so we dont end up ignoring solutions after we hit the end
            auto sols = buf.remove_all();
            for(auto& sol : sols) {
                if(update_solution(sol)) {
                    did_not_improve = 0;
                } else {
                    did_not_improve++;
                }
                took_total += sol.took;

                auto& s = sol.graph->sol;
                GRASP_LOG_I("GRASP iter={} k={} did_not_improve={} reward={} time={} placed={} took={}",
                        l, k, did_not_improve,
                        s.reward, s.end_time, s.placed_bricks,
                        duration_to_string(sol.took));
                rewards.emplace_back(best_solution.reward(), best_solution.graph->sol.end_time);

                // process rest, but stop computation of new solutions
                if(++l >= max_iterations) {
                    stop_workers = true;
                }
            }
        }

        auto avg_per_iter = took_total / l;

        auto end = std::chrono::high_resolution_clock::now();
        duration_raw = end - start;

        GRASP_LOG_D("grasp done in {}, cpu time {}, worker threads {} avg iter {}", duration_to_string(duration_raw), duration_to_string(took_total), tpool.size(), duration_to_string(avg_per_iter));
        auto tpool_start_stop = std::chrono::high_resolution_clock::now();
        stop_workers = true;
        tpool_stop();
        auto t_pool_end_stop = std::chrono::high_resolution_clock::now();
        GRASP_LOG_D("tpool stop took {}", duration_to_string(t_pool_end_stop - tpool_start_stop));

        return best_solution;
    }

};

}

#endif //CTOP_PLANNER_GRASP_H
