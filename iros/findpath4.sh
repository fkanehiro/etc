#!/bin/sh

export OPENHRPHOME=/home/kanehiro/openhrp3.0
export NS_OPT='-ORBInitRef NameService=corbaloc:iiop:localhost:2809/NameService'
export MODEL_URL=file://$OPENHRPHOME/Controller/IOserver/robot/HRP2/model/HRP2main.wrl
export OBSTACLE_URL=file://$PWD/table.wrl
export OBSTACLE_POS='0.8 0.0 0.0 0 0 0'   
#
export LD_LIBRARY_PATH=$OPENHRPHOME/client/gui/corba:$LD_LIBRARY_PATH
export DYLD_LIBRARY_PATH=$LD_LIBRARY_PATH

echo ./findpath4 $NS_OPT -robot $MODEL_URL -obstacle $OBSTACLE_URL $OBSTACLE_POS  $*
./findpath4 $NS_OPT -robot $MODEL_URL -obstacle $OBSTACLE_URL $OBSTACLE_POS $*
