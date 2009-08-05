# Stop all supporting processes
./viewers.sh stop
./facedetector.sh stop
#./cameras.sh stop
./portaudio.sh stop
./soundserver.sh stop
./dynamics.sh stop
./portaudio.sh kill
#yarp clean
