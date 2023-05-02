#!/bin/bash
set -e
CURRENT_DIR="$(pwd)"
COMROB_LIB_PATH="$CURRENT_DIR/lib/comrob/crl"

create_symlink () {
   SYMLINK_PATH="$COMROB_LIB_PATH/$1/crl"
   echo "checking directory $SYMLINK_PATH"
   if [ -e ${SYMLINK_PATH} ]
   then
       echo "symlink $1 already exits"
   else
       echo "symlink $1 created"
       ln -s $COMROB_LIB_PATH/$1/src $COMROB_LIB_PATH/$1/crl;
   fi
}

create_symlink "crl";
create_symlink "crl.algorithm";
create_symlink "crl.gui";
create_symlink "crl.loader";
create_symlink "crl.tsplib";
