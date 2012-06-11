#!/bin/sh

export OPENHRPHOME=/home/kanehiro/openhrp3.0
export NS_OPT='-ORBInitRef NameService=corbaloc:iiop:localhost:2809/NameService'
#export MODEL_URL=file://$OPENHRPHOME/Controller/IOserver/robot/HRP2/model/HRP2main.wrl
export MODEL_URL=file:///home/kanehiro/Dropbox/manuscript/IROS12/HRP2skeleton.wrl
export OBSTACLE_URL=file://$PWD/table.wrl
export OBSTACLE_POS='0.8 0.0 0.0 0 0 0'   
export GOAL_URL=file://$PWD/cylinder.wrl
#export GOAL_POS='0.85 -0.25 0.9 0 0 0' 
export GOAL_POS='0.85 0.0 0.93 0 0 0' 
#
export LD_LIBRARY_PATH=$OPENHRPHOME/client/gui/corba:$LD_LIBRARY_PATH
export DYLD_LIBRARY_PATH=$LD_LIBRARY_PATH

echo ./findgoal2 $NS_OPT -robot $MODEL_URL -obstacle $OBSTACLE_URL $OBSTACLE_POS -goal $GOAL_URL -goalpos $GOAL_POS $*
./findgoal2 $NS_OPT -robot $MODEL_URL -obstacle $OBSTACLE_URL $OBSTACLE_POS -goal $GOAL_URL -goalpos $GOAL_POS $*
