source ./config.sh

killall attentionSelection

$ICUB_ROOT/src/attentionSelection/attentionSelection --name $ROBOT/attentionSelection --file $ATTENTION_SELECTION_INI >& attentionSelection.output &
yarp wait $ROBOT/attentionSelection/conf

yarp connect $ROBOT/egoSphere/map_out $ROBOT/attentionSelection/i:map
yarp connect $ROBOT/attentionSelection/o:position $ROBOT/controlGaze2/pos
yarp connect $ROBOT/controlGaze2/status:o $ROBOT/attentionSelection/i:gaze
