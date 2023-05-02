/*
 * DatasetLoader.cpp
 *
 *  Created on: 15. 3. 2016
 *      Author: Robert Penicka
 */

#include <ctop/util/split.h>
#include "dataset_loader_sop.h"

#define NAME "NAME"
#define TYPE "TYPE"
#define DIMENSION "DIMENSION"
#define NODE_COORD_SECTION "NODE_COORD_SECTION"
#define CONSTRAINING_SET_SECTION "GTSP_SET_SECTION"
#define GTSP_SET_CENTER_COORD_SECTION "GTSP_SET_CENTER_COORD_SECTION"

#define EDGE_WEIGHT_SECTION "EDGE_WEIGHT_SECTION"

#define COMMENT "COMMENT"

#define TMAX "TMAX"
#define NUM_ROBOTS "NUM_ROBOTS"

#define NODE_COORD_SECTION_END "EOF"
#define EDGE_WEIGHT_TYPE "EDGE_WEIGHT_TYPE"

#define PRINT_DEF(str) std::cout << str << std::endl;
#define PRINT_ERROR_DEF(str) std::cerr << str << std::endl;

using namespace ctop;

DatasetLoaderSOP::DatasetLoaderSOP() = default;

DatasetLoaderSOP::~DatasetLoaderSOP() = default;

DatasetSOP DatasetLoaderSOP::loadDataset(std::string filename) {
	PRINT_DEF("DatasetLoaderGOP::loadDataset");
	DatasetSOP loadedDataset;

	int depot_id = 0;
	std::vector<GraphNode> nodes;
	std::vector<std::vector<double>> weight_matrix;
	bool use_nodes = true;
	//exit(1);
	std::ifstream in(filename.c_str(), std::ifstream::in);
	PRINT_DEF("reading problem file from "<<filename);
	if (!in) {
		std::cerr << "Cannot open " << filename << std::endl;
	} else {
		std::string line;
		unsigned int lineNumber = 0;
		unsigned int actualGNID = 0;
		bool dataSection, setSection, weightMatrixSection = false;
		std::string name;
		std::string type;
		std::string dimension_str;
		std::string comment;
		std::string tmax_str;
		std::string edge_type_str;
		std::string sets_str;
		std::string num_robots_str;
		std::string delimiter(" ");
		TSP_EDGE_WEIGHT_TYPE edge_type;
		int dimension;
		int loaded = 0;

		while (getline(in, line)) {
			lineNumber++;
			int delimiterPos = line.find(":");
			//PRINT_DEF("line:"<<line);
			if (delimiterPos != std::string::npos) {
				std::string bef = line.substr(0, delimiterPos);
				std::string aftr("");
				//PRINT_DEF_VAR(line.size());
				//PRINT_DEF(delimiterPos + 1);
				if (line.size() > delimiterPos + 1) {
					//PRINT_DEF("subs");
					aftr = line.substr(delimiterPos + 1);
				}
				trim(aftr);
				trim(bef);

				//PRINT_DEF("NAME "<<NAME);
				//PRINT_DEF("TYPE "<<TYPE);
				//PRINT_DEF("DIMENSION"<<DIMENSION);

				//PRINT_DEF(bef<<" "<<std::strcmp(bef.c_str(), CONSTRAINING_SET_SECTION));

				if (!std::strcmp(bef.c_str(), NAME)) {
					name = aftr;
					loadedDataset.name = name;
				} else if (!std::strcmp(bef.c_str(), TYPE)) {
					type = aftr;
					loadedDataset.type = name;
				} else if (!std::strcmp(bef.c_str(), DIMENSION)) {
					dimension_str = aftr;
					dimension = std::stoi(dimension_str);
					loadedDataset.dimension = dimension;
				} else if (!std::strcmp(bef.c_str(), COMMENT)) {
					comment = aftr;
					loadedDataset.comment = comment;
				} else if (!std::strcmp(bef.c_str(), EDGE_WEIGHT_TYPE)) {
					edge_type_str = aftr;
					loadedDataset.edge_weight_type = parse_tsp_edge_type(edge_type_str);
				} else if (!std::strcmp(bef.c_str(), CONSTRAINING_SET_SECTION)) {
					//PRINT_DEF("switch to "<<CONSTRAINING_SET_SECTION);
					dataSection = false;
					weightMatrixSection = false;
					setSection = true;
				} else if (!std::strcmp(bef.c_str(), GTSP_SET_CENTER_COORD_SECTION)) {
					dataSection = false;
					weightMatrixSection = false;
					setSection = false;
				} else if (!std::strcmp(bef.c_str(), EDGE_WEIGHT_SECTION)) {
					dataSection = false;
					setSection = false;
					weightMatrixSection = true;
				}
			} else {
				//PRINT_DEF("else");
				trim(line);

				//PRINT_DEF(line<<" "<<std::strcmp(line.c_str(), NODE_COORD_SECTION));
				if (!std::strcmp(line.c_str(), NODE_COORD_SECTION)) {
					//PRINT_DEF("switch to NODE_COORD_SECTION start");
					dataSection = true;
					setSection = false;
					weightMatrixSection = false;
				} else if (!std::strcmp(line.c_str(), NODE_COORD_SECTION_END)) {
					//PRINT_DEF("switch to "<<NODE_COORD_SECTION_END);
					dataSection = false;
					setSection = false;
					weightMatrixSection = false;
				} else if (!std::strcmp(line.c_str(), CONSTRAINING_SET_SECTION)) {
					//PRINT_DEF("switch to "<<CONSTRAINING_SET_SECTION);
					dataSection = false;
					setSection = true;
					weightMatrixSection = false;
				} else if (!std::strcmp(line.c_str(), EDGE_WEIGHT_SECTION)) {
					dataSection = false;
					setSection = false;
					weightMatrixSection = true;
				} else if (dataSection) {
					//PRINT_DEF("data section");
					std::istringstream s(line);
					GraphNode newGN;
					s >> newGN.id;
					newGN.id--; //in cop lib the nodes are numbered from 1
					s >> newGN.x;
					s >> newGN.y;
					s >> newGN.reward;
					s >> newGN.node_duration;
					s >> newGN.robot_requirement;
					PRINT_DEF("newGN.robot_requirement "<<newGN.robot_requirement)
					if (newGN.id != loaded) {
						PRINT_ERROR_DEF("wrong id assignment to dataset nodes");
						PRINT_ERROR_DEF(newGN.id<<" "<<newGN.x<<" "<<newGN.y)
					}
					newGN.id = loaded; //owerwrite id of point
					nodes.push_back(newGN);
					//PRINT_DEF(newGN.id<<" "<<newGN.x<<" "<<newGN.y)
					loaded++;
					if (nodes.size() != loaded) {
						PRINT_ERROR_DEF("wrong id assignment to dataset nodes");
						PRINT_ERROR_DEF(newGN.id<<" "<<newGN.x<<" "<<newGN.y)
						exit(1);
					}
				} else if (weightMatrixSection) {
					//PRINT_DEF("loading weight matrix "<<line);
					use_nodes = false;
					std::vector<std::string> tokens = ctop::split(line, delimiter);
					//PRINT_DEF("tokens.size "<<tokens.size())

					for (int var = 0; var < tokens.size(); ++var) {
						if (tokens[var].empty()) {
							continue;								//ommit empty char tokens
						}
						if (weight_matrix.size() == 0 || weight_matrix.back().size() >= dimension) {
							if (weight_matrix.size() != 0) {
								//PRINT_DEF("add new row, previous has "<<weight_matrix.back().size()<<" cols");
							}
							weight_matrix.push_back(std::vector<double>());
							//PRINT_DEF("added weight_matrix row "<<weight_matrix.size());
						}
						std::istringstream tikeniss(tokens[var]);
						double distance = 0;
						tikeniss >> distance;
						weight_matrix[weight_matrix.size() - 1].push_back(distance);
					}
				}
			}
		}
		PRINT_DEF("loaded "<<loaded <<" out of dimension "<<dimension);
	}

	if (!use_nodes) {
		//check distance matrix dimensions
		if (weight_matrix.size() != loadedDataset.dimension) {
			PRINT_ERROR_DEF("edge weight matrix rows not equal to dimension "<<weight_matrix.size()<<"!="<<loadedDataset.dimension);
			exit(1);
		}
		for (int var = 0; var < weight_matrix.size(); ++var) {
			if (weight_matrix[var].size() != loadedDataset.dimension) {
				PRINT_ERROR_DEF("edge weight matrix row "<<var<<" has not equal to dimension "<<weight_matrix[var].size()<<"!="<<loadedDataset.dimension);
				exit(1);
			}
		}
	}
	loadedDataset.startID = 0;
	loadedDataset.goalID = nodes.size() - 1;
	loadedDataset.is_with_nodes = use_nodes;
	loadedDataset.nodesAll = nodes;
	//loadedDataset.distance_matrix = weight_matrix;

	//printSets(loadedDataset);
	PRINT_DEF("return loaded dataset")
	return loadedDataset;
}

std::vector<GraphNode> DatasetLoaderSOP::parseInitialPositions(std::string string) {
	std::string delimiterPositions = "|";
	std::string delimiterXY = ";";
	std::vector<GraphNode> positions;
	std::vector<std::string> positionsArr;
	size_t pos = 0;
	std::string token;
	while ((pos = string.find(delimiterPositions)) != std::string::npos) {
		token = string.substr(0, pos);
		string.erase(0, pos + delimiterPositions.length());
		if (token.length() > 0) {
			positionsArr.push_back(token);
		}
	}
	if (string.length() > 0) {
		positionsArr.push_back(string);
	}
	for (int var = 0; var < positionsArr.size(); ++var) {
		std::string singlePosition = positionsArr[var];
		std::vector<std::string> xyPosition;
		PRINT_DEF(singlePosition);
		size_t posIN = 0;
		std::string token;
		while ((posIN = singlePosition.find(delimiterXY)) != std::string::npos) {
			token = singlePosition.substr(0, posIN);
			singlePosition.erase(0, posIN + delimiterXY.length());

			if (token.length() > 0) {
				xyPosition.push_back(token);
			}
		}
		if (singlePosition.length() > 0) {
			xyPosition.push_back(singlePosition);
		}

		if (xyPosition.size() == 3) {
			float x;
			float y;
			float z;
			std::sscanf(xyPosition[0].c_str(), "x=%f", &x);
			std::sscanf(xyPosition[1].c_str(), "y=%f", &y);
			std::sscanf(xyPosition[2].c_str(), "y=%f", &y);
			PRINT_DEF(x <<" "<<y<<" "<<z);
			positions.push_back(GraphNode(x, y, z, 0, 0, 0, 0, 0));
		} else {
			PRINT_ERROR_DEF("bad initial position specification")
			exit(1);
		}
		for (int var2 = 0; var2 < xyPosition.size(); ++var2) {
			PRINT_DEF(xyPosition[var2]);
		}
	}
	return positions;
}

SolutionGTSP DatasetLoaderSOP::readSolution(std::string filename) {
	SolutionGTSP solution;
	std::vector<GraphNode> nodes;
	std::ifstream in(filename.c_str(), std::ifstream::in);
	PRINT_DEF("reading solution file from filename");
	if (!in) {
		std::cerr << "Cannot open " << filename << std::endl;
	} else {
		std::string line;
		int num_clusters = 0;
		int lineNumber = 0;
		while (getline(in, line)) {
			std::istringstream s(line);
			if (lineNumber == 0) {
				s >> solution.num_nodes;
			} else if (lineNumber == 1) {
				s >> solution.length;
			} else {
				int node_id = 0;
				s >> node_id;
				//substract to have id numbering from 0
				node_id--;
				solution.node_ids.push_back(node_id);
			}
			lineNumber++;
		}
	}
	PRINT_DEF("readed solution with "<<solution.num_nodes<<" nodes and "<<solution.length<<" length");
	std::stringstream ss;
	for (int var = 0; var < solution.node_ids.size(); ++var) {
		ss << solution.node_ids[var] << " ";
	}
	PRINT_DEF("with node ids "<<ss.str());
	return solution;
}
