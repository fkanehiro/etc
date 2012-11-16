#!/bin/sh

export NS_OPT='-ORBInitRef NameService=corbaloc:iiop:localhost:2809/NameService'
export MODEL_URL=file:///home/kanehiro/openrtp/share/OpenHRP-3.1/robot/HRP4Cg/model/HRP4Cg_main.wrl
#export MODEL_URL=file:///home/kanehiro/openrtp/share/OpenHRP-3.1/robot/HRP2SH/model/HRP2SHmain.wrl
#export MODEL_URL=file:///home/kanehiro/openrtp/share/OpenHRP-3.1/robot/HRP2/model/HRP2main.wrl
export GOAL_URL=file://$PWD/arrow.wrl
export GOAL_POS='0.04 -0.35 0.7 -0.5 0 0' 
#export GOAL_POS='0.71 0.03 0.9 0 -1.5708 0' #unreachable 
export DYLD_LIBRARY_PATH=$LD_LIBRARY_PATH

echo ./iktest $NS_OPT -robot $MODEL_URL -goal $GOAL_URL -goalpos $GOAL_POS $*
./iktest $NS_OPT -robot $MODEL_URL  -goal $GOAL_URL -goalpos $GOAL_POS $*
