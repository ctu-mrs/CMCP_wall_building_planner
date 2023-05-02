#!/bin/bash
set -e
# running build scripts directly from commandline
cmake \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCATKIN_DEVEL_PREFIX:PATH=/home/parallels/mrs_workspace/devel \
    -DPLANNER_STANDALONE=TRUE \
    -G "CodeBlocks - Unix Makefiles" \
    -B cmake-build-debug \
    ./