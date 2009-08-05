This document describes how to run AudioAnalyser Module with drumming_EPFL application
It sends bottles of 16 integers to port "/right_arm/score/in"
the number of beats as 1's and the rest are 0's. So if the human partner played 3 beats
then a typical bottle is like this:
0 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0
The first 0 is necessary for the initialization of drumming_EPFL module.
Another bottle filled with frequency (this case 16 doubles, e.g. 0.5 ) is sent to port
"/interactive/in" of Drumming_EPFL application.

you can use the audioAnalyser.sh script file to run the module.
you can change the name of the ports if you like from audioAnalyserConfig.ini file except the ones mentioned above which are related to drumming_EPFL module.
you should also use audioAnalyserMicrophone.ini file for initializing the audio source

 
