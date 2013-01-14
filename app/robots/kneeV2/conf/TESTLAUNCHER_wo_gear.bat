start yarpserver
start yarp run --server /icubsrv
start IcubInterface --config iCubMotor_wo_gear.ini
timeout 1
start robotMotorGui --from robotMotorGui.ini
start simpleClient --robot icub --part right_leg