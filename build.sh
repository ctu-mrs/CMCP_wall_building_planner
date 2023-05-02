#!/bin/bash
set -e
# commandline script for building main targets
# all build files will be in separate build directory

# install correct include directories for building comrob lib
./build_crl_symlinks.sh

# initialize cmake environment
./build_cmake_init.sh

# grasp is compatible with macOS, Windows, Linux
./build_cmake_target.sh planner_standalone_grasp
echo "GRASP planner compiled"

if [ "$(expr substr $(uname -s) 1 5)" == "Linux" ]; then
    echo "trying to compile CPLEX MILP planner"
    # requires cplex library available only for linux
    ./build_cmake_target.sh planner_standalone_optimal;
    echo "CPLEX planner compiled"
fi

