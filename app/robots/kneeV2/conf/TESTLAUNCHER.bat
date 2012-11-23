start yarpserver
start yarp run --server /icubsrv
start IcubInterface --config iCubKnee.ini
timeout 10
start robotMotorGui --from robotMotorGui.ini