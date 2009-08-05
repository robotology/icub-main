source ./config.sh

killall egoSphere
sleep .1
killall egoSphere
sleep .1
killall egoSphere
sleep .1
killall egoSphere

$ICUB_ROOT/src/egoSphere/egoSphere --name $ROBOT/egoSphere --file $ICUB_EYES_INI --group EGO_SPHERE>& egoSphere.output &
yarp wait $ROBOT/egoSphere/conf

yarp connect $ROBOT/sift/positions $ROBOT/egoSphere/mapObject/bottle_in

yarp connect $ROBOT/egoSphere/map_out $ROBOT/attentionSelection/i:map

yarp connect $ROBOT/egoSphere/map_out $VIEW_EGO
