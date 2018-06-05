#!/bin/bash -e

export DEVEL_DIR=$HOME/src/laas

mkdir -p $DEVEL_DIR
mkdir -p $DEVEL_DIR/src
mkdir -p $DEVEL_DIR/install

sudo apt-get install autoconf g++ cmake libboost-dev liburdfdom-dev libassimp-dev ros-indigo-xacro ros-indigo-kdl-parser ros-indigo-common-msgs ros-indigo-tf ros-indigo-tf-conversions ros-indigo-libccd ros-indigo-octomap ros-indigo-resource-retriever ros-indigo-srdfdom ros-indigo-pr2-robot flex bison asciidoc source-highlight git libomniorb4-dev omniorb-nameserver omniidl omniidl-python libltdl-dev python-matplotlib libtinyxml2-dev liblog4cxx10-dev libltdl-dev qt4-dev-tools libqt4-opengl-dev libqtgui4 oxygen-icon-theme libcdd-dev

sudo apt-get build-dep openscenegraph

wget -O $DEVEL_DIR/config.sh https://raw.githubusercontent.com/humanoid-path-planner/hpp-doc/rbprm/doc/config.sh
wget -O $DEVEL_DIR/src/Makefile https://raw.githubusercontent.com/humanoid-path-planner/hpp-doc/rbprm/doc/Makefile

cd $DEVEL_DIR
source config.sh

cd src
make robot_state_chain_publisher.install
make hpp-model-urdf.install
make hpp-gepetto-viewer.install
make hpp-rbprm-corba.install

git clone https://github.com/iit-DLSLab/hyq-description.git
git clone https://github.com/iit-DLSLab/sensors-description.git

export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$DEVEL_DIR
rosrun xacro xacro.py `rospack find hyq_description`/robots/hyq_model.urdf.xacro -o $DEVEL_DIR/install/share/hpp-rbprm-corba/hyq.urdf

