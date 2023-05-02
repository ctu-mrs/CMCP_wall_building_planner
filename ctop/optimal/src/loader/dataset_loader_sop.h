/*
 * DatasetLoaderGOP.h
 *
 *  Created on: 15. 3. 2016
 *      Author: Robert Penicka
 */

#ifndef SRC_DATASETLOADER_SOP_H_
#define SRC_DATASETLOADER_SOP_H_

#include <iostream>
#include <fstream>
#include <cstdio>
#include <string>
#include <vector>
#include <cstring>
#include <algorithm>
#include <iostream>

#include "../heuristic_types.h"

namespace ctop {

struct DatasetLoaderSOP {
	DatasetLoaderSOP();
	virtual ~DatasetLoaderSOP();
	static DatasetSOP loadDataset(std::string filename);
	static std::vector<GraphNode> parseInitialPositions(std::string string);
	static SolutionGTSP readSolution(std::string filename);
};

}

#endif /* SRC_DATASETLOADER_SOP_H_ */
