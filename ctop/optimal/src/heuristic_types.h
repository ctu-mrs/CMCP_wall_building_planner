/*
 * HeuristicTypes.h
 *
 *  Created on: Feb 22, 2016
 *      Author: Robert Penicka
 */

#ifndef SRC_FOURPHASEHEURISTIC_HEURISTICTYPES_H_
#define SRC_FOURPHASEHEURISTIC_HEURISTICTYPES_H_

#include <vector>
#include <cmath>
#include <iostream>
#include <limits>
#include <iomanip>
#include <map>

#include <ctop/geometry/Point.h>
#include <ctop/geometry/Point3D.h>

#include "heuristic/GraphNode.h"
#include "heuristic/SolutionCTOP.h"
#include "heuristic/SingleSolutionCTOP.h"
#include "heuristic/NodeVisit.h"
#include "heuristic/ImprovementLogRecord.h"
#include "heuristic/Tour.h"
#include "heuristic/DatasetOP.h"
#include "heuristic/DatasetSOP.h"
#include "heuristic/ClusterSOP.h"
#include "heuristic/SolutionGTSP.h"
#include <ctop/heuristic/PrecedenceRule.h>
#include "heuristic/ClusterNodeDist.h"
#include "heuristic/HeuristicTypes.h"
#include "heuristic/TimedImprovement.h"
#include "heuristic/StartGoalNodes.h"
#include "heuristic/InsertionHistoryRecord.h"

#include "heuristic/EdgeTypes.h"

#include <ctop/util/replace.h>
#include <ctop/util/trim.h>

#include "defines.h"

//#define DEBUG_ILP_SOP_FORMULAS

#define INFO(str) std::cout << str << std::endl;
#define ERROR(str)  std::cerr << str << std::endl;

#define PRINT_DEF(str) //std::cout << str << std::endl;
#define PRINT_DEF_COND( cond , x ) if(cond) { PRINT_DEF( x ) }

#ifdef DEBUG_ILP_SOP_FORMULAS
#define PRINT_DEF_DEBUG( x ) PRINT_DEF( x )
#else
#define PRINT_DEF_DEBUG( x )
#endif

#endif /* SRC_FOURPHASEHEURISTIC_HEURISTICTYPES_H_ */
