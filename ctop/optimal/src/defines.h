/*
 * my_defines.h
 *
 *  Created on: Jun 28, 2017
 *      Author: penicrob
 */

#ifndef LP_GOP_MY_DEFINES_H_
#define LP_GOP_MY_DEFINES_H_

#include <ilcplex/ilocplex.h>

#include <string>
#include <iostream>

#define OUTPUT_DEFAULT "\033[0m"
#define OUTPUT_BLACK "\033[30m"
#define OUTPUT_RED "\033[31m"
#define OUTPUT_GREEN "\033[32m"
#define OUTPUT_YELLOW "\033[33m"
#define OUTPUT_BLUE "\033[34m"
#define OUTPUT_MAGENTA "\033[35m"
#define OUTPUT_CYAN "\033[36m"
#define OUTPUT_WHITE "\033[37m"

#define INFO_RED(x) INFO( OUTPUT_RED << x << OUTPUT_DEFAULT )
#define INFO_YELLOW(x) INFO( OUTPUT_YELLOW << x << OUTPUT_DEFAULT )
#define INFO_MAGENTA(x) INFO( OUTPUT_MAGENTA <<  x << OUTPUT_DEFAULT )
#define INFO_CYAN(x) INFO( OUTPUT_CYAN <<  x << OUTPUT_DEFAULT )
#define INFO_GREEN(x) INFO( OUTPUT_GREEN <<  x << OUTPUT_DEFAULT )
#define INFO_WHITE(x) INFO( OUTPUT_WHITE <<  x << OUTPUT_DEFAULT )
#define INFO_BLUE(x) INFO( OUTPUT_BLUE <<  x << OUTPUT_DEFAULT )
#define INFO_BLACK(x) INFO( OUTPUT_BLACK <<  x << OUTPUT_DEFAULT )
#define INFO_COND( cond , x ) if(cond){ INFO( x ); }

#define VARIABLE_STR(s) #s
#define STR(s) VARIABLE_STR(s)
#define ROVNASE1(X) X =
#define ROVNASE(X)  ROVNASE1(X)
#define INFO_VAR(x) INFO( STR(x) << " = " <<  x )


#define VARIABLE_FROM_TO_ROBOT "x_{}_{}_r{}"
#define VARIABLE_FROM_TO_Z "z_{}_{}"
#define VARIABLE_FROM_TO_Z_CONSTR_START "constr_start_" VARIABLE_FROM_TO_Z
#define VARIABLE_FROM_TO_Y_PRECEDENCE "precedence_y_{}_{}"
#define VARIABLE_FROM_TO_S_PRECEDENCE "precedence_s_{}_{}"

#define VARIABLE_FROM_TO_U(fromCluster,fromClusterNode,toCluster,toClusterNode)  "u_" << fromCluster<<"_"<<fromClusterNode<<"__"<<toCluster<<"_"<<toClusterNode
#define VARIABLE_AUX_FROM_TO(fromCluster,toCluster) "y_" << fromCluster << "__" << toCluster

#define INDEX_1D( from , to , width ) ( (width) * (from) + (to) )
#define INDEX_2D_TO( id , width ) ( (id) % (width) )
#define INDEX_2D_FROM( id , width ) ( ((id) - ( (id)  % (width) )) / (width) )

namespace ctop {

using IloNumVarMatrix1D = IloNumVarArray ;
using IloNumVarMatrix2D = IloArray<IloNumVarArray>;
using IloNumVarMatrix3D = IloArray<IloArray<IloNumVarArray>> ;
using IloNumVarMatrix4D = IloArray<IloArray<IloArray<IloNumVarArray>>> ;

struct VarValue {
	std::string varName;
	int varValue;
};

struct IndexPairSOP {
	int cluster_from_node;
	int cluster_to_node;
	int cluster_from;
	int cluster_to;
};

struct IndexSOP {
    int nodeIndex;

	IndexSOP(int nodeIndex = -1)
    : nodeIndex(nodeIndex)
    {}
};

}

#endif /* LP_GOP_MY_DEFINES_H_ */
