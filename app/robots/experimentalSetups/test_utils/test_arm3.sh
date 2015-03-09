controlBoardDumper --name /ra_dumper --robot icub --part right_arm --joints "(0 1 2 3)" --rate 10 --dataToDump "(getEncoders getEncoderSpeeds getMotorEncoders getMotorEncoderSpeeds)" &
controlBoardDumper --name /la_dumper --robot icub --part left_arm  --joints "(0 1 2 3)" --rate 10 --dataToDump "(getEncoders getEncoderSpeeds getMotorEncoders getMotorEncoderSpeeds)" &
velocityObserver --name /la_encs &
velocityObserver --name /la_motencs &
velocityObserver --name /ra_encs &
velocityObserver --name /ra_motencs &
read -n1 -r -p "press key to continue" key
yarp connect /ra_dumper/right_arm/getMotorEncoders /ra_motencs/pos:i
yarp connect /ra_dumper/right_arm/getEncoders      /ra_encs/pos:i
yarp connect /la_dumper/left_arm/getMotorEncoders  /la_motencs/pos:i
yarp connect /la_dumper/left_arm/getEncoders       /la_encs/pos:i
read -n1 -r -p "press key to continue" key
yarpscope --xml yarpscope_speeds_la.xml &
yarpscope --xml yarpscope_speeds_ra.xml &
read -n1 -r -p "press key to terminate" key
killall -9 controlBoardDumper
killall -9 velocityObserver
killall -9 yarpscope

