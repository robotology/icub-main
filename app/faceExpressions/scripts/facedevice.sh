CONFIGPATH=${1:-../conf}

yarpdev --name /icub/face/raw --device serial --subdevice serialport --file $CONFIGPATH/serialport.ini