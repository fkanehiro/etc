#!/bin/bash

export DEVEL_DIR=$HOME/src/laas
source $DEVEL_DIR/config.sh
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$DEVEL_DIR
