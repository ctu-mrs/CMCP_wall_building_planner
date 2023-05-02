/*
 * DatasetLoaderWall.cpp
 *
 *  Created on: 4. 9. 2019
 *      Author: Robert Penicka
 */

#include "dataset_loader_wall.h"

#define PRINT_DEF(str) std::cout << str << std::endl;
#define PRINT_ERROR_DEF(str) std::cerr << str << std::endl;

#define COMMENT_CHAR '#'

using namespace ctop;

DatasetLoaderWall::DatasetLoaderWall() = default;
DatasetLoaderWall::~DatasetLoaderWall() = default;

DatasetSOP DatasetLoaderWall::loadDataset(const std::string& filename) {
	PRINT_DEF("DatasetLoaderGOP::loadDataset");
	DatasetSOP loadedDataset;

	int depot_id = 0;
	std::vector<GraphNode> nodes;
	//std::vector<std::vector<double>> weight_matrix;

	std::ifstream in(filename.c_str(), std::ifstream::in);
	PRINT_DEF("reading problem file from " << filename);
	int loaded = 0;
	if (!in) {
		std::cerr << "Cannot open " << filename << std::endl;
		exit(1);
	} else {
		std::string line;
		unsigned int lineNumber = 0;
		unsigned int actualGNID = 0;

		std::string delimiter(" ");
		std::string trimmed_line;

		GraphNode start;
		start.id = 0;
		start.type = 0;
		start.x = 0;
		start.y = 0;
		start.z = 0;
		start.reward = 0;
		start.node_duration = 0;
		start.robot_requirement = 0;
		nodes.push_back(start);

		while (getline(in, line)) {
			lineNumber++;

			trim(line);
            trimmed_line = line;

            if (trimmed_line[0] != COMMENT_CHAR) {

				std::istringstream s(line);
				GraphNode newGN;
				s >> newGN.id;
				newGN.brick_id = newGN.id;
				//newGN.id--; //in cop lib the nodes are numbered from 1
				s >> newGN.type;
				s >> newGN.x;
				s >> newGN.y;
				s >> newGN.z;
				s >> newGN.yaw_rotated;
				newGN.reward = 0; //BRICK_REWARDS[newGN.type];
				newGN.node_duration = 0; //BRICK_DURATIONS_S[newGN.type];
				newGN.robot_requirement = 0; //BRICK_NUM_ROBOT_REQUIREMENTS[newGN.type];
				PRINT_DEF(nodes.size()<<" brick type "<<newGN.type)
				nodes.push_back(newGN);
				loaded++;
			}
		}
	}

	GraphNode goal;
	goal.id = nodes.size();
	goal.type = 0;
	goal.x = 0;
	goal.y = 0;
	goal.z = 0;
	goal.reward = 0;
	goal.node_duration = 0;
	goal.robot_requirement = 0;
	nodes.push_back(goal);

	loadedDataset.startID = 0;
	loadedDataset.goalID = nodes.size() - 1;
	loadedDataset.is_with_nodes = true;
	loadedDataset.nodesAll = nodes;
	//loadedDataset.distance_matrix = weight_matrix;
	loadedDataset.edge_weight_type = TSP_EDGE_WEIGHT_TYPE::EUC_2D;
	loadedDataset.dimension = loaded + 2;

	//set precendece rules with separate layers
	for (int var_bef = 1; var_bef < nodes.size() - 1; ++var_bef) {
		for (int var_aft = 1; var_aft < nodes.size() - 1; ++var_aft) {
			if (nodes[var_bef].z < nodes[var_aft].z) {
				PrecedenceRule rule;
				rule.before = var_bef;
				rule.after = var_aft;
				loadedDataset.precendence_rules.push_back(rule);
				//PRINT_DEF("add precedence "<<rule.before<<" bef "<<rule.after);
			}
		}
	}

	//getSOPFromNodes(loadedDataset, loadedDataset.nodesAll, loadedDataset.startID,loadedDataset.goalID, loadedDataset.edge_weight_type);

//printSets(loadedDataset);
	PRINT_DEF("return loaded dataset")
	return loadedDataset;
}

std::vector<PrecedenceRule> DatasetLoaderWall::getPrecedenceRulesAccurate(
        const DatasetSOP& loadedDataset,
		std::map<std::string, std::vector<int>> &brick_size_length_depth_height_cm_map,
		std::map<int, std::string> &brick_ids_map_reversed
) {
	PRINT_DEF("getPrecedenceRulesAccurate begin");
	std::vector<PrecedenceRule> precendence_rules;
	const auto& nodes = loadedDataset.nodesAll;
	PRINT_DEF("nodes.size() "<<nodes.size())
	PRINT_DEF("brick_ids_map_reversed.size() "<< brick_ids_map_reversed.size())
	//set precendece rules with separate layers

	for (int var_bef = 1; var_bef < nodes.size() - 1; ++var_bef) {
		for (int var_aft = 1; var_aft < nodes.size() - 1; ++var_aft) {
			PRINT_DEF("var_bef "<<var_bef<<" var_aft "<<var_aft)
			const auto& aft_node = nodes[var_aft];
            const auto& bef_node = nodes[var_bef];
			if (bef_node.z < aft_node.z) { //only apply if aft_node is above
				PRINT_DEF("bef cornets")
				std::vector<Point3D> aft_corners = getBrickCorners(aft_node, brick_size_length_depth_height_cm_map, brick_ids_map_reversed);
				std::vector<Point3D> bef_corners = getBrickCorners(bef_node, brick_size_length_depth_height_cm_map, brick_ids_map_reversed);
				PRINT_DEF("has corners")
				//check the corner values
				//add from mrs_lib is point inside polygon

				bool bellow_cor_in_upper = false;
				for (int cor_bef_idx = 0; cor_bef_idx < bef_corners.size(); ++cor_bef_idx) {
					Point3D &cor_bef = bef_corners[cor_bef_idx];
					bellow_cor_in_upper = isPointInside(cor_bef.x, cor_bef.y, aft_corners);
					if (bellow_cor_in_upper) {
						break;
					}
				}
				PRINT_DEF("tested bellow_cor_in_upper")
				bool upper_cor_in_bellow = false;
				for (int cor_aft_idx = 0; cor_aft_idx < aft_corners.size(); ++cor_aft_idx) {
					Point3D &cor_aft = aft_corners[cor_aft_idx];
					upper_cor_in_bellow = isPointInside(cor_aft.x, cor_aft.y, bef_corners);
					if (upper_cor_in_bellow) {
						break;
					}
				}
				PRINT_DEF("tested upper_cor_in_bellow")
				if (bellow_cor_in_upper || upper_cor_in_bellow) {
					PrecedenceRule rule;
					rule.before = var_bef;
					rule.after = var_aft;
					precendence_rules.push_back(rule);
					PRINT_DEF("add precedence "<<rule.before<<" bef "<<rule.after);
				}
			}
		}
	}
	PRINT_DEF("getPrecedenceRulesAccurate end");
	return precendence_rules;
}

bool DatasetLoaderWall::isPointInside(double px, double py, const std::vector<Point3D>& corners) {
	int count = 0;
	// Cast a horizontal ray and see how many times it intersects all the edges
	auto sz = corners.size();
	for (int i = 0; i < sz; ++i) {

	    const auto& corner1 = corners[i];
        const auto& corner2 = corners[(i + 1) % sz];

		double v1x = corner1.x;
		double v1y = corner1.y;
		double v2x = corner2.x;
		double v2y = corner2.y;

		if (v1y > py && v2y > py) continue;
		if (v1y <= py && v2y <= py) continue;

		double intersect_x = (v1y == v2y) ? v1x : (py - v1y) * (v2x - v1x) / (v2y - v1y) + v1x;
		bool does_intersect = intersect_x > px;
		if (does_intersect) {
            ++count;
		}
	}

	return count % 2;
}

std::vector<Point3D> DatasetLoaderWall::getBrickCorners(
        const GraphNode &brick_node,
		std::map<std::string,std::vector<int>>& brick_size_length_depth_height_cm_map,
		std::map<int, std::string>& brick_ids_map_reversed) {
	std::vector<Point3D> corners;
	PRINT_DEF("brick_node.type " << brick_node.type)
	const auto& brick_type = brick_ids_map_reversed[brick_node.type];

	PRINT_DEF("brick_type " << brick_type)
	const auto& brick_l_h_d_cm = brick_size_length_depth_height_cm_map[brick_type];

	PRINT_DEF("dimensions readed " << brick_l_h_d_cm.size())
	int x_index = 0;
	int y_index = 1;
	int z_index = 2;

	if (brick_node.yaw_rotated) {
		x_index = 1;
		y_index = 0;
	}

	//PRINT_DEF("center of "<<brick_node.id<<" is "<<brick_node.x<<" "<<brick_node.y);
	//PRINT_DEF("yaw_rotated "<<brick_node.yaw_rotated<<" x_index "<<x_index<<" y_index "<<y_index);
	//PRINT_DEF("brick_l_h_d_cm[x_index] "<<brick_l_h_d_cm[x_index]<< " brick_l_h_d_cm[y_index] "<<brick_l_h_d_cm[y_index])

	const auto& width = brick_l_h_d_cm[x_index];
    const auto& height = brick_l_h_d_cm[y_index];

	auto width_half_m = width / 200.0; // is /2 /100
    auto height_half_m = height / 200.0;

    // corner map
    // x-y plane
    //    --x-->
    //   |  2 --------- 1  ---
    //   |  |           |   |
    //   y  |   (mid)   | height
    //   |  |           |   |
    //      3 --------- 4  ---
    //      |---width---|

	Point3D corner1{};
	corner1.x = brick_node.x + width_half_m; // is /2 /100
	corner1.y = brick_node.y + height_half_m;
	corner1.z = brick_node.z;
	corners.push_back(corner1);
    //PRINT_DEF("corner1 " << corner1.x << " " << corner1.y);

	Point3D corner2{};
	corner2.x = brick_node.x - width_half_m;
	corner2.y = brick_node.y + height_half_m;
	corner2.z = brick_node.z;
	corners.push_back(corner2);
    //PRINT_DEF("corner2 " << corner2.x << " " << corner2.y);

	Point3D corner3{};
	corner3.x = brick_node.x - width_half_m;
	corner3.y = brick_node.y - height_half_m;
	corner3.z = brick_node.z;
	corners.push_back(corner3);
    //PRINT_DEF("corner3 " << corner3.x << " " << corner3.y);


	Point3D corner4{};
	corner4.x = brick_node.x + width_half_m;
	corner4.y = brick_node.y - height_half_m;
	corner4.z = brick_node.z;
	corners.push_back(corner4);
	//PRINT_DEF("corner4 " << corner4.x << " " << corner4.y);

	return corners;
}

std::vector<PrecedenceRule> DatasetLoaderWall::getNextToEachOther(const DatasetSOP& problem, int min_distance_concurrent_brick_placing_cm) {
	PRINT_DEF("getNextToEachOther begin")
	std::vector<PrecedenceRule> precendence_rules;
	const auto& nodes = problem.nodesAll;
	for (int var1 = 1; var1 < nodes.size() - 1; ++var1) {
		for (int var2 = var1 + 1; var2 < nodes.size() - 1; ++var2) {
			const auto& node1 = nodes[var1];
            const auto& node2 = nodes[var2];
			const double diff_x = node1.x - node2.x;
			const double diff_y = node1.y - node2.y;
			double dist_centers_cm = std::sqrt(diff_x * diff_x + diff_y * diff_y) * 100;

			if (dist_centers_cm < min_distance_concurrent_brick_placing_cm) {
				precendence_rules.push_back(PrecedenceRule(var1, var2));
				//PRINT_DEF(var1<<" next to "<<var2<<" with dist cm "<<dist_centers_cm);
			}

		}
	}
	PRINT_DEF("getNextToEachOther end")
	return precendence_rules;
}

bool DatasetLoaderWall::areCurentBricksExcluded(const std::vector<PrecedenceRule>& concurrence_exclusion_rules, const std::vector<short int>& current_brick_ids){
	for (const auto& concurrence_exclusion_rule : concurrence_exclusion_rules) {
		bool left_there = false;
		bool right_there = false;
		for (const auto& current_brick_id : current_brick_ids) {
			if (concurrence_exclusion_rule.before == current_brick_id) {
				left_there = true;
			}
			if (concurrence_exclusion_rule.after == current_brick_id) {
				right_there = true;
			}
			if(left_there && right_there) break;
		}
		if (left_there && right_there) {
			PRINT_DEF(
					"concurrence exclusion rule is present in current bricks "<<concurrence_exclusion_rule.before<<" and "<<concurrence_exclusion_rule.after);
			return true;
		}
	}
	return false;
}
