start yarpserver
start yarp run --server /icubsrv
start IcubInterface --config iCubKnee.ini
timeout 3
start robotMotorGui --from robotMotorGui.ini