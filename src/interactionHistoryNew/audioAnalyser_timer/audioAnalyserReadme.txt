-----AudioAnalyser module----------
----written by Hatice Kose-Bagci---

This document describes how to run AudioAnalyser Module.
This module takes sound from an external device and sends the patterns(number of beats,
and duration between these beats) to a port. Every time the device is silent for 1 sec,
the beats are sent to the port. The first number is the number of beats and the rest are 
the duration between the beats in msec. You should have one port in your receiver module 
to grab the pattern send in bottle:

pattern_in     /uh/Drum/pattern:i

from pattern_in you will take the number of beats(int) and duration of each beat(int)
e.g. if you receive 2 100 
this means there are two beats and duration between them is 100 ms

you can use the sample receiver file simple_receiver, and the sample script file demo.sh to run the program
you can change the name of the ports, and duration of the program (it runs for 1 min currently)
if you like from audioAnalyserConfig.ini file
you should also use audioAnalyserMicrophone.ini file for initializing the audio source

