This document describes how to run AudioAnalyser Module
You should have two ports in your drumming module

ack_out  	/uh/Drum/ack:o
pattern_in     /uh/Drum/pattern:i

with ack_out you would send 0 to /uh/Audio/ack:i when you finish drumming
from pattern_in you will take the number of beats(int) and duration of each beat(int)
e.g. if you receive 2 100 
this means there are two beats and duration between them is 100 ms

you can use the demo.sh and demo1.sh script files to run the programs
you can change the name of the ports if you like from audioAnalyserConfig.ini file
you should also use audioAnalyserMicrophone.ini file for initializing the audio source

the ack ports are connected in the demo1.sh and pattern ports are connected inside audioanalyser  

The drummer module makes "happy face" when it plays "drum bouts" and when the robot can not "hear"
--audioAnalyser sends beat no=0 then it makes "sad face".

The drummer module runs for 2 minutes and makes "sad face" at the end and waves left hand "good bye".
