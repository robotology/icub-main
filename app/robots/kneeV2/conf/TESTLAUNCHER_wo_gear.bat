start yarpserver
start yarp run --server /icubsrv
start IcubInterface --config iCubMotor_wo_gear.ini
timeout 3
start robotMotorGui --from robotMotorGui.ini
start simpleClient --robot icub --part right_leg