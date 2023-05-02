
# Date:      2011/07/11 17:55
# Author:    Jan Faigl
#

CPLEX_ROOT_DIR=/opt/cplex-12.6.1

include Mk/libs.mk
include Mk/flags.mk

SRC_DIR=src
LOCAL_CFLAGS+= -I./include

OBJS=math_common.o heuristic_types.o dataset_loader_sop.o dataset_loader_wall.o ilp_ctop_definition.o ilp_ctop_solver.o ilp_ctop_alg_solver.o 

#

TARGET=ctop_planner_standalone
OBJ_DIR=obj

include Mk/comrob.mk
