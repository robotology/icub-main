# Stop all supporting processes
./viewers_wo_gaze.sh stop
./facedetector.sh stop
#./simcameras_wo_gaze.sh stop
./portaudio.sh stop
./soundserver.sh stop
./portaudio.sh kill
#yarp clean
