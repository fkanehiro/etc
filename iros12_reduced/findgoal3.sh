#!/bin/sh

export OPENHRPHOME=/home/kanehiro/openhrp3.0
export NS_OPT='-ORBInitRef NameService=corbaloc:iiop:localhost:2809/NameService'
#export MODEL_URL=file://$OPENHRPHOME/Controller/IOserver/robot/HRP2/model/HRP2main.wrl
export MODEL_URL=file://$OPENHRPHOME/Controller/IOserver/robot/HRP2JRL/model/HRP2JRLmain.wrl
#export OBSTACLE_URL=file://$OPENHRPHOME/etc/house/table.main.wrl
#export OBSTACLE_POS='-1.0 1.0 0.0 0 0 0'   
export OBSTACLE_URL=file://$PWD/EnvPlant.wrl
export OBSTACLE_POS='0.0 0.0 0 0 0 0'
export GOAL_URL=file://$PWD/arrow.wrl
export GOAL_POS='0.71 0.03 0.9 0 -1.5708 0' 
#export GOAL_POS='0.75 0.03 1.0 0 -1.5708 0' 
#
export LD_LIBRARY_PATH=$OPENHRPHOME/client/gui/corba:$LD_LIBRARY_PATH
export DYLD_LIBRARY_PATH=$LD_LIBRARY_PATH

echo ./findgoal3 $NS_OPT -robot $MODEL_URL -obstacle $OBSTACLE_URL $OBSTACLE_POS -goal $GOAL_URL -goalpos $GOAL_POS $*
./findgoal3 $NS_OPT -robot $MODEL_URL -obstacle $OBSTACLE_URL $OBSTACLE_POS -goal $GOAL_URL -goalpos $GOAL_POS $*
