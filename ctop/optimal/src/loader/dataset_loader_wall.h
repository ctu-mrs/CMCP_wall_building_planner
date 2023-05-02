/*
 * DatasetLoaderWall.h
 *
 *  Created on: 4. 9. 2019
 *      Author: Robert Penicka
 */

#ifndef SRC_DATASETLOADER_WALL_H_
#define SRC_DATASETLOADER_WALL_H_


#include <iostream>     // std::cout
#include <fstream>      // std::ifstream
#include <cstdio>
#include <string>
#include <vector>
#include <cstring>
#include <algorithm>
#include <iostream>

#include "../heuristic_types.h"

namespace ctop {

struct DatasetLoaderWall {
	DatasetLoaderWall();
	virtual ~DatasetLoaderWall();

	static DatasetSOP loadDataset(const std::string& filename);

	static std::vector<PrecedenceRule> getPrecedenceRulesAccurate(
            const DatasetSOP& loadedDataset,
			std::map<std::string, std::vector<int>> &brick_size_length_depth_height_cm_map,
			std::map<int, std::string> &brick_ids_map_reversed);

	static std::vector<Point3D> getBrickCorners(
            const GraphNode& brick_node,
	        std::map<std::string, std::vector<int>> &brick_size_length_depth_height_cm_map,
			std::map<int, std::string> &brick_ids_map_reversed);

	static bool isPointInside(double px, double py, const std::vector<Point3D>& corners);
	static std::vector<PrecedenceRule> getNextToEachOther(const DatasetSOP& problem, int min_distance_concurrent_brick_placing_cm);
	static bool areCurentBricksExcluded(const std::vector<PrecedenceRule>& concurrence_exclusion_rules, const std::vector<short int>& current_brick_ids);
};

}

#endif /* SRC_DATASETLOADER_WALL_H_ */
