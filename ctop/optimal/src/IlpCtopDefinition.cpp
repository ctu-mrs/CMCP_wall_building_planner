/*
 *
 *  Created on: Jun 28, 2017
 *      Author: penicrob
 */

#include "IlpCtopDefinition.h"

#include <utility>
#include <fmt/format.h>

using namespace ctop;

const double M = std::numeric_limits<double>::max();

#define FLAT_X(fromClusterNode,toClusterNode,num_to_cluster_nodes)  (num_to_cluster_nodes * fromClusterNode + toClusterNode )

IlpCtopDefinition::IlpCtopDefinition(
        IloEnv& env_,
        IloModel& model_,
        const std::vector<GraphNode>& nodes_all_,
        const std::vector<std::vector<double>>& distances_,
        const std::vector<int>& start_indexes_,
        int goalIndex_,
        double budget_,
        int num_robots_,
        const std::vector<PrecedenceRule>& precendence_rules,
        const std::vector<PrecedenceRule>& concurrence_exclusion_rules,
        const std::vector<short int>& current_brick_ids_,
        const std::vector<short int>& builded_brick_ids_,
        bool minimize_total_time_)
:   initial_solution_set(false),
    env(env_),
    model(model_),
    nodes_all(nodes_all_),
    start_indexes(start_indexes_),
    goalIndex(goalIndex_),
    budget(budget_),
    distances(distances_),
    num_robots(num_robots_),
    unvisited_nodes(nodes_all_),
    max_distance(0),
    max_node_duration(0),
    minimize_total_time(minimize_total_time_),
    precendence_rules(precendence_rules),
    precendence_rules_applyable(),
    concurrence_exclusion_rules(concurrence_exclusion_rules),
    concurrence_exclusion_rules_applyable(),
    current_brick_ids(current_brick_ids_),
    builded_brick_ids(builded_brick_ids_),
    numAddedConstraints(0),
    lastObjVal(-IloInfinity),
    lastIncumbentLC(-1),
    lastTime(0),
    lastImprovementIter(0),
    lastImprovementTime(0)
{
	PRINT_DEF("this->num_robots " << num_robots)
	for (int var = 0; var < nodes_all.size(); ++var) {
		PRINT_DEF("node " << var << " reward " << nodes_all[var].reward)
	}

    find_max_values();
    init_unvisited_ids();
    init_start_index_set();
    init_applyable_rules();

    add_variable_y_sop();
	add_variable_x_z();

    constraint_start_end();
    constraint_x_flow();
    constraint_robot_number();
    constraint_z_and_x();
    constraint_start_times();
	constraint_visit_precedence();
	constraint_side_by_side();
}

void IlpCtopDefinition::find_max_values() {
    //calc max_distance and max_node_duration
    int sz = nodes_all.size();
    max_node_duration = 0;
    max_distance = 0;
    for (int id1 = 0; id1 < sz; ++id1) {
        if (nodes_all[id1].node_duration > max_node_duration) {
            max_node_duration = nodes_all[id1].node_duration;
        }
        for (int id2 = 0; id2 < sz; ++id2) {
            if (distances[id1][id2] > max_distance) {
                max_distance = distances[id1][id2];
            }
        }
    }
}

void IlpCtopDefinition::init_applyable_rules() {

    concurrence_exclusion_rules_applyable = concurrence_exclusion_rules;
    precendence_rules_applyable = precendence_rules;

    PRINT_DEF("already visited nodes are:")

    auto sz = builded_brick_ids.size();
    for (int var = 0; var < sz; ++var) {
        auto& builded_brick_id = builded_brick_ids[var];

        {
            auto end = unvisited_nodes.size() - 1;
            for (int node_idx = end; node_idx >= 0; --node_idx) {
                auto& node = unvisited_nodes[node_idx];
                if (builded_brick_id == node.brick_id) {
                    auto erase_pos = unvisited_nodes.begin() + node_idx;
                    unvisited_nodes.erase(erase_pos);
                    PRINT_DEF("already builded node" << var << " is " << builded_brick_id << " removed from possible nodes")
                }
            }
        }

        {
            auto end = precendence_rules_applyable.size() - 1;
            for (int pr_idx = end; pr_idx >= 0; --pr_idx) {
                auto& rule = precendence_rules_applyable[pr_idx];
                if (builded_brick_id == rule.before || builded_brick_id == rule.after) {
                    //remove no more applyable rule
                    auto erase_pos = precendence_rules_applyable.begin() + pr_idx;
                    precendence_rules_applyable.erase(erase_pos);
                } else {
                    //renumber indexes to the one applyable
                }
            }
        }

        {
            auto end = concurrence_exclusion_rules_applyable.size() - 1;
            for (int cr_idx = end; cr_idx >= 0; --cr_idx) {
                auto& rule = concurrence_exclusion_rules_applyable[cr_idx];
                if (builded_brick_id == rule.before || builded_brick_id == rule.after) {
                    //remove no more applyable rule
                    auto erase_pos = concurrence_exclusion_rules_applyable.begin() + cr_idx;
                    precendence_rules_applyable.erase(erase_pos);
                } else {
                    //renumber indexes to the one applyable
                }
            }
        }
    }
}

void IlpCtopDefinition::init_start_index_set() {
    for (int start_index : start_indexes) {
        start_indexes_set.insert(start_index);
    }
}

void IlpCtopDefinition::init_unvisited_ids() {
    PRINT_DEF("planning for bricks:");
    for (int node_idx = 0; node_idx < unvisited_nodes.size(); ++node_idx) {

        auto& unvisited_node = unvisited_nodes[node_idx];
        auto unvisited_node_id = unvisited_node.id;
        auto unvisited_node_brick_id = unvisited_node.brick_id;

        PRINT_DEF("brick id " << unvisited_node_brick_id)
        brick_ids__unvisited_idx[unvisited_node_brick_id] = node_idx;
        unvisited_idx__brick_ids[node_idx] = unvisited_node_brick_id;

        all_node_ids__unvisited_idx[unvisited_node_id] = node_idx;
        unvisited_idx__all_node_ids[node_idx] = unvisited_node_id;
    }
}

void IlpCtopDefinition::add_variable_y_sop() {
    y = IloNumVarArray(env); //whether the set was visited
    s = IloNumVarArray(env); //visit start times

    // variables_y (10) and sop_objective_ilp (2)
    IloExpr objectiveExpression(env);
    for (int fromNode = 0; fromNode < unvisited_nodes.size(); ++fromNode) {

        // add y variable
        auto varname = fmt::format("y_{}", fromNode);
        IloBoolVar new_var_y(env, varname.c_str());
        y.add(new_var_y);

        // add s variable
        auto varname_s = fmt::format("s_{}", fromNode);
        IloNumVar new_var_s(env, 0, budget, varname_s.c_str());
        s.add(new_var_s);

        double p_i = unvisited_nodes[fromNode].reward;
        objectiveExpression += p_i * new_var_y;
    }

    if (minimize_total_time) {
        objectiveExpression -= s[unvisited_nodes.size() - 1] / budget;
    }
    model.add(IloMaximize(env, objectiveExpression));
    PRINT_DEF_DEBUG("objective created:");
    PRINT_DEF_DEBUG(objectiveExpression);
}

void IlpCtopDefinition::add_variable_x_z() {
    x = IloNumVarMatrix3D(env); //int edges between nodes equal to number of robots
    z = IloNumVarMatrix2D(env); //binary edges betweeen nodes

    //objective function and add variables x_from_to
    for (int fromNode = 0; fromNode < unvisited_nodes.size() - 1; ++fromNode) {
        PRINT_DEF("fromNode "<<fromNode)
        x.add(IloNumVarMatrix2D(env));
        z.add(IloNumVarArray(env));

        auto& from_x =  x[fromNode];
        auto& from_z =  z[fromNode];

        for (int toNode = 1, toNodeIndex = 0; toNode < unvisited_nodes.size(); ++toNode, ++toNodeIndex) {
            PRINT_DEF("toNode "<<toNode)
            from_x.add(IloNumVarArray(env));

            auto& from_to_x = from_x[toNodeIndex];

            PRINT_DEF("added")
            for (int robot_id = 0; robot_id < num_robots; ++robot_id) {
                auto varname = fmt::format(VARIABLE_FROM_TO_ROBOT, fromNode, toNode, robot_id);
                IloBoolVar new_var_x(env, varname.c_str());
                from_to_x.add(new_var_x);
            }

            auto varname_z = fmt::format(VARIABLE_FROM_TO_Z, fromNode, toNode);
            IloBoolVar new_var_z(env, varname_z.c_str());
            from_z.add(new_var_z);
            //INFO("fromNode "<<fromNode<<" toNode "<<toNode<<" var "<<newvar)
        }
    }

    PRINT_DEF_DEBUG("variables x:");
    PRINT_DEF_DEBUG(x);
    PRINT_DEF_DEBUG("variables z:");
    PRINT_DEF_DEBUG(z);
}

void IlpCtopDefinition::constraint_start_end() {

    /*
    // constraint_start and end
    for (int robot_id = 0; robot_id < num_robots; ++robot_id) {
        //for (int var = 0; var < start_indexes.size(); ++var) {
            IloExpr constraint_start_expr(env);
            int &start_index = start_indexes[robot_id];
            int &unvisited_start_index = all_node_ids__unvisited_idx[start_index];
            for (int toNode = num_robots; toNode < unvisited_nodes.size(); ++toNode) {
                if (start_indexes_set.count(toNode) == 0) {
                constraint_start_expr += x[unvisited_start_index][toNode - 1][robot_id];
                }
                //PRINT_DEF("start " << x[start_index][toNode - 1]<<" start_index "<<start_index);
            }
            IloConstraint constraint_single_start(constraint_start_expr == 1);
            std::stringstream varname;
            varname << "constraint_robot" << robot_id << "_start_" << unvisited_start_index << "_at_zero_time";
            constraint_single_start.setName(varname.str().c_str());
            model.add(constraint_single_start);
            PRINT_DEF_DEBUG("constraint_single_start");
            PRINT_DEF_DEBUG(constraint_single_start);
        //}
    }
    */

    IloExpr constraint_goal_expr(env);

    auto endNode = static_cast<int>(unvisited_nodes.size()) - 1;
    auto endNodeIndex = endNode-1;

    for (int robot_id = 0; robot_id < num_robots; ++robot_id) {
        for (int fromNode = 0; fromNode < endNode; ++fromNode) {
            constraint_goal_expr += x[fromNode][endNodeIndex][robot_id];
        }
    }

    //IloConstraint constraint_start_equal_goal(constraint_start_expr == constraint_goal_expr);
    //constraint_start_equal_goal.setName("constraint_start_equal_goal");
    //model.add(constraint_start_equal_goal);
    //PRINT_DEF_DEBUG("constraint_start_equal_goal:");
    //PRINT_DEF_DEBUG(constraint_start_equal_goal);

    IloConstraint constraint_goal(constraint_goal_expr == num_robots);
    constraint_goal.setName("constraint_goal");
    model.add(constraint_goal);
    PRINT_DEF_DEBUG("constraint_goal:");
    PRINT_DEF_DEBUG(constraint_goal);

    // set start s and y variables	of start
    for (auto&& start_index : start_indexes) {
        // set start to start at 0
        auto& unvisited_start_index = all_node_ids__unvisited_idx[start_index];
        IloConstraint constraint_start_at_zero_time(s[unvisited_start_index] == 0);

        auto varname = fmt::format("constraint_start_{}_at_zero_time", unvisited_start_index);
        constraint_start_at_zero_time.setName(varname.c_str());
        model.add(constraint_start_at_zero_time);
        PRINT_DEF_DEBUG(constraint_start_at_zero_time);

        // set start y to be used
        IloConstraint constraint_start_y(y[unvisited_start_index] == 1);
        auto varname_y = fmt::format("constraint_start_{}_y", unvisited_start_index);
        constraint_start_y.setName(varname_y.c_str());
        model.add(constraint_start_y);
        PRINT_DEF_DEBUG(constraint_start_y);
    }

    // set end y to be used
    IloConstraint constraint_end_y(y[y.getSize() - 1] == 1);
    constraint_end_y.setName("constraint_end_y");
    PRINT_DEF_DEBUG(constraint_end_y);
    model.add(constraint_end_y);

}

void IlpCtopDefinition::constraint_x_flow() {
    // constraint x flow
    for (int robot_id = 0; robot_id < num_robots; ++robot_id) {

        // flow of starting nodes
        for (int start_id = 0; start_id < start_indexes.size(); ++start_id) {
            /* use or not use outgoing edges from start_indexes begin */
            IloExpr constraint_flow_expr(env);
            int &start_index = start_indexes[start_id];
            int &unvisited_start_index = all_node_ids__unvisited_idx[start_index];
            for (int toNode = 1; toNode < unvisited_nodes.size(); ++toNode) {
                if (unvisited_start_index != toNode) {
                    constraint_flow_expr += x[unvisited_start_index][toNode - 1][robot_id];
                }
            }

            int has_to_be_used = 0;
            std::string varname;
            if (start_id == robot_id) {
                has_to_be_used = 1;
                varname = fmt::format("start_r{}_in_{}", robot_id, unvisited_start_index);
            } else {
                varname = fmt::format("not_start_r{}_in_{}", robot_id, unvisited_start_index);
            }
            IloConstraint constraint_flow(constraint_flow_expr == has_to_be_used);
            constraint_flow.setName(varname.c_str());
            model.add(constraint_flow);
            PRINT_DEF_DEBUG("constraint_flow:");
            PRINT_DEF_DEBUG(constraint_flow);

            /* use or not use outgoing edges from start_indexes end */
            if (start_id != start_index) {
                //also omit its original starting node if start_index is a brick
                IloExpr constraint_flow_old_start_expr(env);
                for (int toNode = 1; toNode < unvisited_nodes.size(); ++toNode) {
                    if (start_id != toNode) {
                        constraint_flow_old_start_expr += x[start_id][toNode - 1][robot_id];
                    }
                }
                IloConstraint constraint_flow_old(constraint_flow_old_start_expr == 0);
                auto varname2 = fmt::format("not_start_r{}_in_old_start_{}", robot_id, start_id);
                constraint_flow_old.setName(varname2.c_str());
                model.add(constraint_flow_old);
                PRINT_DEF_DEBUG("constraint_flow_old:");
                PRINT_DEF_DEBUG(constraint_flow_old);
            }

        }

        //flow of non-starting nodes
        for (int flowNode = num_robots; flowNode < unvisited_nodes.size() - 1; ++flowNode) {
            IloExpr constraint_flow_expr(env);

            int flow_node_brick_id = unvisited_nodes[flowNode].id;
            if (start_indexes_set.count(flow_node_brick_id) > 0) { //do not ensure flow in node that is the starting one
                PRINT_DEF("omiting start node " << flow_node_brick_id << " to be considered for the flow")
                continue;
            }

            for (int fromNode = 0; fromNode < unvisited_nodes.size() - 1; ++fromNode) {
                if (flowNode != fromNode) {
                    constraint_flow_expr += x[fromNode][flowNode - 1][robot_id];
                }
            }

            for (int toNode = 1; toNode < unvisited_nodes.size(); ++toNode) {
                if (flowNode != toNode) {
                    constraint_flow_expr -= x[flowNode][toNode - 1][robot_id];
                }
            }

            IloConstraint constraint_flow(constraint_flow_expr == 0);
            auto varname = fmt::format("flow_r{}_in_{}", robot_id, flowNode);
            constraint_flow.setName(varname.c_str());
            model.add(constraint_flow);
            PRINT_DEF_DEBUG("constraint_flow:");
            PRINT_DEF_DEBUG(constraint_flow);
        }
    }
}

void IlpCtopDefinition::constraint_robot_number() {
    /***        robot number requirement in node         ***/

    auto sz_unvisited = unvisited_nodes.size();
    auto sz_unvisited_1 = sz_unvisited - 1;
    for (int requirementNode = num_robots; requirementNode < sz_unvisited_1; ++requirementNode) {

        auto& flow_node = unvisited_nodes[requirementNode];
        auto flow_node_brick_id = flow_node.id;

        if (start_indexes_set.count(flow_node_brick_id) > 0) { //do not ensure flow in node that is the starting one
            PRINT_DEF("omitting start node "<<flow_node_brick_id<<" to be considered for number requirement in")
            continue;
        }

        IloExpr sum_from_expr(env);
        for (int robot_id = 0; robot_id < num_robots; ++robot_id) {
            for (int toNode = 1; toNode < sz_unvisited; ++toNode) {
                if (requirementNode != toNode) {
                    sum_from_expr += x[requirementNode][toNode - 1][robot_id];
                }
            }
        }

        IloConstraint constraint_sum_from_robots(
            flow_node.robot_requirement * y[requirementNode] == sum_from_expr
        );
        auto varname = fmt::format("num_robots_in_node_{}", requirementNode);
        constraint_sum_from_robots.setName(varname.c_str());
        model.add(constraint_sum_from_robots);

        PRINT_DEF_DEBUG("constraint_sum_from_robots:");
        PRINT_DEF_DEBUG(constraint_sum_from_robots);
        //INFO("unvisited_nodes[requirementNode].robot_requirement "<<unvisited_nodes[requirementNode].robot_requirement)
    }
}

void IlpCtopDefinition::constraint_z_and_x() {
    for (int fromNode = 0; fromNode < unvisited_nodes.size() - 1; ++fromNode) {
        for (int toNode = 1; toNode < unvisited_nodes.size(); ++toNode) {
            if (toNode != fromNode) {
                IloExpr sum_x_expr(env);
                for (int robot_id = 0; robot_id < num_robots; ++robot_id) {
                    sum_x_expr += x[fromNode][toNode - 1][robot_id];
                }
                IloConstraint constraint_z(
                    sum_x_expr <= num_robots * z[fromNode][toNode - 1]
                );
                auto varname = fmt::format(VARIABLE_FROM_TO_Z_CONSTR_START, fromNode, toNode);
                constraint_z.setName(varname.c_str());
                model.add(constraint_z);
            }
        }
    }
}

void IlpCtopDefinition::constraint_start_times() {
    auto max_M_constant = static_cast<int>(budget + max_distance + max_node_duration);

    auto sz = unvisited_nodes.size();
    auto sz1 = sz - 1;
    for (int fromNode = 0; fromNode < sz1; ++fromNode) {
        for (int toNode = 1; toNode < sz; ++toNode) {
            if(fromNode == toNode) continue;

            IloConstraint constraint_start(
                    s[fromNode] + distances[fromNode][toNode] + unvisited_nodes[fromNode].node_duration - s[toNode]
                    <=
                    max_M_constant * (1 - z[fromNode][toNode - 1])
            );

            auto varname = fmt::format(VARIABLE_FROM_TO_Z_CONSTR_START, fromNode, toNode);
            constraint_start.setName(varname.c_str());
            model.add(constraint_start);

            //PRINT_DEF("fromNode " << fromNode << " toNode " << toNode << " distance " << distances[fromNode][toNode] << " from duration " << unvisited_nodes[fromNode].node_duration)
            //INFO("(budget + max_distance + max_node_duration) "<<(budget + max_distance + max_node_duration))
            PRINT_DEF_DEBUG("constraint_start:")
            PRINT_DEF_DEBUG(constraint_start)
        }
    }
}

void IlpCtopDefinition::constraint_visit_precedence() {
    for (auto&& rule : precendence_rules) {
        //int before = rule.before + (num_robots - 1);
        //int after = rule.after + (num_robots - 1);
        assert(brick_ids__unvisited_idx.find(rule.before) != brick_ids__unvisited_idx.end());
        assert(brick_ids__unvisited_idx.find(rule.after) != brick_ids__unvisited_idx.end());

        int before_idx = brick_ids__unvisited_idx[rule.before];
        int after_idx = brick_ids__unvisited_idx[rule.after];

        {
            IloConstraint constraint_visit(
                    y[before_idx] >= y[after_idx]
            );
            auto varname = fmt::format(VARIABLE_FROM_TO_Y_PRECEDENCE, before_idx, after_idx);
            constraint_visit.setName(varname.c_str());
            model.add(constraint_visit);

            PRINT_DEF_DEBUG("constraint_visit:")
            PRINT_DEF_DEBUG(constraint_visit)
        }

        {
            IloConstraint constraint_starts(
                    s[after_idx] - s[before_idx] >= y[before_idx] * unvisited_nodes[before_idx].node_duration
            );
            auto varname = fmt::format(VARIABLE_FROM_TO_S_PRECEDENCE, before_idx, after_idx);
            constraint_starts.setName(varname.c_str());
            model.add(constraint_starts);
            PRINT_DEF_DEBUG("constraint_precedence:")
            PRINT_DEF_DEBUG(constraint_starts)
        }
    }
}


void IlpCtopDefinition::constraint_side_by_side() {
    /* constaint not placing side by side */
    // or constraint between x1 x2 is : y <= x1+x2, y>=x1, y>=x2
    //s[left] + a[left] >= s[right]                       nebo                s[right] + a[right] >= s[left]
    //s[left] + y[left] * a[left] >= s[right]            nebo                s[right] + y[right] * a[right] >= s[left]
    //s[right] - s[left] - y[left] * a[left] >= 0    //right is after
    //s[left] - s[right] - y[right] * a[right] >= 0  //left is after
    //s[right] - s[left] - y[left] * a[left] >=  M * (x_1 - 1)   //x_1 == right is after
    //s[left] - s[right] - y[right] * a[right] >= M * (x_2 - 1)  //x_2 == left is after
    //y <= x_1 + x_2, y >= x_1, y >= x_2  							 // x_1 || x_2
    //std::vector<PrecedenceRule> bricks_next_to_each_other;
    //bricks_next_to_each_other.push_back(PrecedenceRule(4, 3));
    //bricks_next_to_each_other.push_back(PrecedenceRule(3, 5));
    //bricks_next_to_each_other.push_back(PrecedenceRule(5, 6));
    //bricks_next_to_each_other.push_back(PrecedenceRule(6, 7));	//plus 2 to ids
    for (auto & rule : concurrence_exclusion_rules) {
        //int left = rule.before + (num_robots - 1);
        //int right = rule.after + (num_robots - 1);
        assert(brick_ids__unvisited_idx.find(rule.before) != brick_ids__unvisited_idx.end());
        assert(brick_ids__unvisited_idx.find(rule.after) != brick_ids__unvisited_idx.end());
        int left_idx = brick_ids__unvisited_idx[rule.before];
        int right_idx = brick_ids__unvisited_idx[rule.after];

        IloConstraint constraint_left_after(
            s[left_idx] - s[right_idx] - y[right_idx] * unvisited_nodes[right_idx].node_duration >= 0
        );
        IloConstraint constraint_right_after(
            s[right_idx] - s[left_idx] - y[left_idx] * unvisited_nodes[left_idx].node_duration >= 0
        );
        IloOr or_exp(env);
        or_exp.add(constraint_left_after);
        or_exp.add(constraint_right_after);
        model.add(or_exp);

        PRINT_DEF_DEBUG("constraint_concurrence:");
        PRINT_DEF_DEBUG(constraint_left_after);
        PRINT_DEF_DEBUG(constraint_right_after);
    }
}

IlpCtopDefinition::~IlpCtopDefinition() = default;

void IlpCtopDefinition::printAllSubsets(std::list<std::list<int>> &list_to_fill) {
	for (auto&& l1 : list_to_fill) {
		std::stringstream ss;
		for (auto&& val : l1) {
			ss << val << " ";
		}
		PRINT_DEF(ss.str());
	}
}

/*
 For n=1, the set of subsets is {{}, {1}}
 For n>1, find the set of subsets of 1,...,n-1 and make two copies of it. For one of them, add n to each subset. Then take the union of the two copies.
 */
void IlpCtopDefinition::getAllSubsets(int n, std::list<std::list<int>>& list_to_fill) {
	PRINT_DEF("getAllSubsets " << n);
	if (n > 1) {
		getAllSubsets(n - 1, list_to_fill);
		auto list_to_fill_copy{list_to_fill};
		for (auto&& l: list_to_fill_copy) {
			l.push_back(n);
		}
		list_to_fill.insert(list_to_fill.end(), list_to_fill_copy.begin(), list_to_fill_copy.end());
		if (n < 6) {
			printAllSubsets(list_to_fill);
		}
	} else {
		if (!list_to_fill.empty()) {
			list_to_fill.clear();
		}
		std::list<int> without;
		std::list<int> with;
		with.push_back(1);
		list_to_fill.push_back(without);
		list_to_fill.push_back(with);
		//printAllSubsets(list_to_fill);
	}
}

std::list<IndexPairSOP> orderSolution(std::list<IndexPairSOP> solutionListUnordered) {
	std::list<IndexPairSOP> orderedSolution;
    //add first edge to solution
	orderedSolution.push_back(solutionListUnordered.front());
    solutionListUnordered.pop_front();

	while (!solutionListUnordered.empty()) {
		for (auto it = solutionListUnordered.begin(); it != solutionListUnordered.end(); ++it) {
			auto pair = *it;
			if (pair.cluster_to == orderedSolution.front().cluster_from) {
				//fond edge before actual one
				orderedSolution.push_front(pair);
                solutionListUnordered.erase(it);
				break;
			}
			if (pair.cluster_from == orderedSolution.back().cluster_to) {
				orderedSolution.push_back(pair);
                solutionListUnordered.erase(it);
				break;
			}
		}
	}
	return orderedSolution;
}

int IlpCtopDefinition::getLastImprovementIter() {
	return lastImprovementIter;
}

void IlpCtopDefinition::setLastImprovementIter(int lastImprovementIter_) {
	lastImprovementIter = lastImprovementIter_;
}

long IlpCtopDefinition::getLastImprovementTime() {
	return lastImprovementTime;
}

void IlpCtopDefinition::setLastImprovementTime(long lastImprovementTime_) {
	lastImprovementTime = lastImprovementTime_;
}

std::vector<ImprovementLogRecord> IlpCtopDefinition::getImprovementLog() {
	return improvementLog;
}
void IlpCtopDefinition::setImprovementLog(std::vector<ImprovementLogRecord> improvementLog_) {
	improvementLog = std::move(improvementLog_);
}

std::vector<SingleSolutionCTOP> IlpCtopDefinition::parseSolutionVector(IloCplex &cplex) {
	PRINT_DEF("parsing solution vector");
	std::vector<SingleSolutionCTOP> solution;

	IloNumArray vals_lok_s(env);
	cplex.getValues(vals_lok_s, s);
	for (int var = 0; var < unvisited_nodes.size(); ++var) {
		try {
			int y_var = static_cast<int>(std::round(cplex.getValue(y[var])));
			if (y_var == 1) {
				PRINT_DEF(
                    var << " brick "<<unvisited_idx__brick_ids[var]
                    << " visited at time " << vals_lok_s[var]
                    << " for duration " << unvisited_nodes[var].node_duration
                    << " by " << unvisited_nodes[var].robot_requirement << " robots"
                )
			}
		} catch (IloException &e) {
            ERROR("Error " << e)
		} catch (...) {
            ERROR("Unknown exception caught")
		}
	}

	for (int robot_id = 0; robot_id < num_robots; ++robot_id) {
		for (int fromNode = 0; fromNode < unvisited_nodes.size() - 1; ++fromNode) {
			/*
                IloNumArray vals_lok_z(env);
                IloNumArray vals_lok_x(env);
                cplex.getValues(vals_lok_z, z[fromNode]);
                cplex.getValues(vals_lok_x, x[fromNode]);
			*/
			// INFO("vals retrieved");
			for (int toNode = 1; toNode < unvisited_nodes.size(); ++toNode) {
				if (fromNode != toNode) {


					int z_var = static_cast<int>(std::round(cplex.getValue(z[fromNode][toNode - 1])));	// vals_lok_z[toNode - 1]);
					int x_var = static_cast<int>(std::round(cplex.getValue(x[fromNode][toNode - 1][robot_id])));	//vals_lok_x[toNode - 1]);
					if (x_var > 0) {

					    assert(unvisited_idx__brick_ids.find(fromNode) != unvisited_idx__brick_ids.end());
                        assert(unvisited_idx__brick_ids.find(toNode) != unvisited_idx__brick_ids.end());

                        auto fromNode_brick_id = unvisited_idx__brick_ids[fromNode];
                        auto toNode_brick_id = unvisited_idx__brick_ids[toNode];

						PRINT_DEF(
						    "path " << fromNode <<
						    "-" << toNode <<
						    " bricks " << fromNode_brick_id <<
						    "-" << toNode_brick_id <<
						    " using robot " << robot_id
						);
					}
                    /*
                    else {
                        if (z_var > 0) {
                            INFO("z_var should not be "<<z_var<<" for path "<<fromNode<<"-"<<toNode);
                        }
                    }
                    */
				}
			}
		}
	}

    //std::vector<std::pair<int, int>> actual_nodes;
    //pair<int, int> start(startIndex, num_robots);
    //actual_nodes.push_back(start);
	PRINT_DEF("parse solution:")

    //s
    // eparate starts for robots
	for (int robot_id = 0; robot_id < num_robots; ++robot_id) {
		SingleSolutionCTOP single_solution;

		auto& start_index = start_indexes[robot_id];
		auto& start_node = nodes_all[start_index];

        PRINT_DEF("add start index " << start_index << " for robot " << robot_id);

        //get visit time from cplex
        int start_index_unvisited = all_node_ids__unvisited_idx[start_index];
        auto visit_start_time = cplex.getValue(s[start_index_unvisited]);

        {
            NodeVisit start_node_visit{start_node};
            start_node_visit.visit_start_time = visit_start_time;
            single_solution.node_sequence.push_back(start_node_visit);
        }

		solution.push_back(single_solution);
	}

	for (int robot_id = 0; robot_id < num_robots; ++robot_id) {

	    auto& one_robot_solution = solution[robot_id];

		while (true) {
		    //get last node of the solution for one robot
			NodeVisit fromNode = one_robot_solution.node_sequence[solution[robot_id].node_sequence.size() - 1];
			if (fromNode.node_id != goalIndex) {
				int from_node_idx_unvisited = all_node_ids__unvisited_idx[fromNode.node_id];
				PRINT_DEF("test fromNode " << fromNode.node_id << " (unvisited id " << from_node_idx_unvisited << ") robot " << robot_id);
				for (int toNode = num_robots; toNode < unvisited_nodes.size(); ++toNode) {
					if (from_node_idx_unvisited != toNode) {
						int z_var = static_cast<int>(std::round(cplex.getValue(z[from_node_idx_unvisited][toNode - 1])));
						int x_var = static_cast<int>(std::round(cplex.getValue(x[from_node_idx_unvisited][toNode - 1][robot_id])));
						if (x_var > 0) {
							double s_var = cplex.getValue(s[toNode]);
							PRINT_DEF("path " << fromNode.node_id << "-" << unvisited_nodes[toNode].id << " with robot " << robot_id << " to arrive at " << s_var)
							NodeVisit node_visit{};
							node_visit.node_id = unvisited_nodes[toNode].id;
							node_visit.visit_start_time = s_var;
							node_visit.brick_id = unvisited_nodes[toNode].brick_id;
							node_visit.duration = unvisited_nodes[toNode].node_duration;
							solution[robot_id].node_sequence.push_back(node_visit);
						}
					}
				}
			} else {
			    break;
			}
		}
	}

    //PRINT_DEF("after");
    /*
    bool all_in_end = false;
    std::map<std::pair<int, int>, int> already_added_arcs;

    while (!all_in_end) {
        all_in_end = true;
        for (int solution_idx = 0; solution_idx < solution.size(); ++solution_idx) {
            NodeVisit fromNode = solution[solution_idx].node_sequence[solution[solution_idx].node_sequence.size() - 1];
            if (fromNode.node_id != goalIndex) {
                all_in_end = false;
                for (int robot_id = 0; robot_id < num_robots; ++robot_id) {
                    for (int toNode = num_robots; toNode < unvisited_nodes.size(); ++toNode) {
                        if (fromNode.node_id != toNode) {
                            //PRINT_DEF("fromNode.node_id "<<fromNode.node_id);
                            int z_var = std::round(cplex.getValue(z[fromNode.node_id][toNode - 1]));	// vals_lok_z[toNode - 1]);
                            int x_var = std::round(cplex.getValue(x[fromNode.node_id][toNode - 1][robot_id]));	//vals_lok_x[toNode - 1]);
                            if (x_var > 0) {
                                double s_var = cplex.getValue(s[toNode]);
                                PRINT_DEF("path "<<fromNode.node_id<<"-"<<toNode <<" using "<<x_var<<" robots "<<" to arrive at "<<s_var);
                                NodeVisit node_visit;
                                node_visit.node_id = toNode;
                                node_visit.visit_start_time = s_var;
                                node_visit.brick_id = unvisited_nodes[toNode].brick_id;
                                std::pair<int, int> arc_pair(fromNode.node_id, toNode);
                                bool add_new_arc = false;
                                if (already_added_arcs.count(arc_pair) == 1) {
                                    //already visited
                                    int &num_visits = already_added_arcs[arc_pair];
                                    if (num_visits < x_var) {
                                        add_new_arc = true;
                                    }
                                } else {
                                    //not visited
                                    already_added_arcs[arc_pair] = 0;
                                    add_new_arc = true;
                                }

                                if (add_new_arc) {
                                    solution[solution_idx].node_sequence.push_back(node_visit);
                                    already_added_arcs[arc_pair] += 1;
                                    PRINT_DEF("add arc from " << fromNode.node_id << " to " << toNode << " to solution " << solution_idx);
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    */
	return solution;
}

