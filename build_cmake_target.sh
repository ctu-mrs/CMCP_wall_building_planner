#!/bin/bash
set -e
# $1 - cmake target name, to list all targets use 'help' target

if [ -z "$1" ]; then
    echo "no cmake target specified"
    exit;
fi

cmake \
    --build ./cmake-build-debug \
    --target $1 \
    -- -j 6