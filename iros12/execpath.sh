#!/bin/sh

export NS_OPT='-ORBInitRef NameService=corbaloc:iiop:localhost:2809/NameService'
export MODEL_URL=file://$HOME/openrtp/share/OpenHRP-3.1/robot/HRP2/model/HRP2main.wrl
#export OBSTACLE_URL=file://$OPENHRPHOME/etc/house/table.main.wrl
#export OBSTACLE_POS='-1.0 1.0 0.0 0 0 0'   
export OBSTACLE_URL=file://$PWD/EnvPlant.wrl
export OBSTACLE_POS='0 0.0 0 0 0 0'
#
export DYLD_LIBRARY_PATH=$LD_LIBRARY_PATH

echo ./execpath $NS_OPT -robot $MODEL_URL -obstacle $OBSTACLE_URL $OBSTACLE_POS  -point-cloud plant.pc $*
./execpath $NS_OPT -robot $MODEL_URL -obstacle $OBSTACLE_URL $OBSTACLE_POS -point-cloud plant.pc $*
