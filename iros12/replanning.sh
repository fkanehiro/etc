#!/bin/sh

export NS_OPT='-ORBInitRef NameService=corbaloc:iiop:localhost:2809/NameService'
export MODEL_URL=file://$HOME/openrtp/share/OpenHRP-3.1/robot/HRP2/model/HRP2main.wrl
#export OBSTACLE_URL=file://$OPENHRPHOME/etc/house/table.main.wrl
#export OBSTACLE_POS='-1.0 1.0 0.0 0 0 0'   
export OBSTACLE_URL=file://$PWD/EnvPlant.wrl
export OBSTACLE_POS='0.0 0.0 0 0 0 0'
export GOAL_URL=file://$PWD/arrow.wrl
export GOAL_POS='0.71 0.03 0.9 0 -1.5708 0' 
#export GOAL_POS='0.75 0.03 1.0 0 -1.5708 0' 
export OBSTACLE2_URL=file://$PWD/cube.wrl
#export OBSTACLE2_POS='0 0 -1 0 0 0'
#export OBSTACLE2_POS='0.3 0.45 0.65 0 0 0'
export OBSTACLE2_POS='0.5 0.15 0.85 0 0 0'
#
export DYLD_LIBRARY_PATH=$LD_LIBRARY_PATH

echo ./replanning $NS_OPT -robot $MODEL_URL -obstacle $OBSTACLE_URL $OBSTACLE_POS -goal $GOAL_URL -goalpos $GOAL_POS -obstacle $OBSTACLE2_URL $OBSTACLE2_POS $*
./replanning $NS_OPT -robot $MODEL_URL -obstacle $OBSTACLE_URL $OBSTACLE_POS -goal $GOAL_URL -goalpos $GOAL_POS -obstacle $OBSTACLE2_URL $OBSTACLE2_POS $*
