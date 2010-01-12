#include <iCub/iha/AudioAnalyserModule.h>
#include <iCub/iha/iha_utils.h>

/**
 * @addtogroup icub_iha2_AudioAnalyser
 *
\section intro_sec Description
Audio Analyser for IHA 2.0

This module connects to a yarp sound server device and analyses
it for drumbeat-like periods of noise. The rate at which messages
are output (in ms) is set from the config file or the command
line. The output is a bottle containing a list of the number of beats  ihaNewAudioAnalyzer --name /iha/beat --file /usr/local/src/robot/iCub/app/ihaNew/conf/ihaAudioAnalyser.ini 


detected since the last message was sent and the duration of 
time between those beats.

\section lib_sec Libraries
- YARP libraries.
- IHA Debug Library

\section parameters_sec Parameters
\verbatim
--dbg <INT>                      : debug printing level
--name <STR>                     : process name for ports
--file <STR>                     : configuration from given file
--connect_to_soundgrabber [STR]  : autoconnect to specified port for sound
--soundsensorrate [INT]          : sensor data rate (in ms), 
                                   default 100 gives 10 frames/s
--soundgain [FLT]                : multiplier for sound level 
\endverbatim

\section portsa_sec Ports Accessed

 - expects input from a yarp sound grabber device

\section portsc_sec Ports Created

 - sndsensor:out - audio message output port
 - sndsensor:in  - yarp sound input port
 - quit          - module quit port
 
\section conf_file_sec Configuration Files
conf/ihaAudioAnalyser.ini

Sample INI file:
\verbatim
\endverbatim

\section tested_os_sec Tested OS
Linux

\section example_sec Example Instantiation of the Module
ihaAudioAnalyser --file conf/ihaAudioAnalyser.ini

\see iCub::contrib::AudioAnalyserModule

\author Frank Broz and Hatice Kose-Bagci

Copyright (C) 2009 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at \in src/interactionHistoryNew/audio_analyser/src/AudioAnalyserModule.cpp.
*/

AudioAnalyserModule::AudioAnalyserModule(){
}

AudioAnalyserModule::~AudioAnalyserModule(){ 
}


bool AudioAnalyserModule::open(Searchable& config){
   
	if (config.check("dbg")) { IhaDebug::setLevel(config.find("dbg").asInt()); }
 	ACE_OS::fprintf(stderr, "Debug level : %d\n",IhaDebug::getLevel());
    
    if (config.check("help","if present, display usage message")) {
		cerr << "Usage : " << "\n"
             << "----------------------------------------" << "\n"
             << "  --dbg [INT]   : debug printing level" << "\n"
             << "  --name [STR]  : process name for ports" << "\n"
             << "  --file [STR]  : config file" << "\n"
             << "----------------------------------------------------------------------------" << "\n"
             << "  --connect_to_soundgrabber [STR]  : autoconnect to specified port for sound" << "\n"
             << "----------------------------------------------------------------------------" << "\n"
             << "  --soundsensorrate [INT]  : sensor data rate (in ms), default 100" << "\n"
             << "                             gives a 10 frames/s" << "\n"
             << "  --soundgain [FLT]        : multiplier for sound level" << "\n"
             << "----------------------------------------------------------------------------" << "\n"
             << "\n";
        return false;
    }
    
    bool ok = true;
    
	//------------------------------------------------------
	// For Sensor Output
	// get the sensor data rate (in ms), default 100 gives a 10 frames/s
	//sndSensorRate = config.check("soundsensorrate",Value(100)).asInt();
	//IhaDebug::pmesg(DBGL_INFO,"soundsensorrate:%d\n",sndSensorRate);
	//sndGain = config.check("soundgain",Value(2.5)).asDouble();
	//IhaDebug::pmesg(DBGL_INFO,"soundgain:%f\n",sndGain);
	
	//------------------------------------------------------
	// open sound sensor output port
	sndSensorPort.open(getName("sndsensor:out"));

	//------------------------------------------------------
	// create input sound port port name
    ConstString soundPortName = getName("sndsensor:in");
	// open the sound input port - don't drop any samples
	soundPort.setStrict();
	soundPort.open(soundPortName.c_str());

	// If required connect to the sound grabber
	if (config.check("connect_to_soundgrabber")) {
		if (connectToParam(config,"connect_to_soundgrabber",soundPortName.c_str(), 0.25, this)) {
			IhaDebug::pmesg(DBGL_INFO,"Connected to sound grabber\n");
		} else {
            ok=false;
        }  
    }

    //initialize a few variables...
    hmask[0]= -1;
    hmask[1]= -1;
    hmask[2]=  1;
    hmask[3]=  1;

    lmask[0]=  1;
    lmask[1]=  1;
    lmask[2]= -1;
    lmask[3]= -1;

    f=0;
    z=0;
    count=0;
    count1=0;
    count2=0;
    count3=0;
    step=0;
        high=-AUDIO_MAX_VAL;
    low=AUDIO_MAX_VAL;
    avg=0;
    vhigh=0;
    vlow=0;
    H=0;
    L=0;
    max1=-AUDIO_MAX_VAL;
    max2=-AUDIO_MAX_VAL;
    max3=-AUDIO_MAX_VAL;
    maxid1=0;
    maxid2=0;
    maxid3=0;
    th1=10000;
    th=3000;
    beatNo=0;
    top = 20;
    duration=0;
    dur=0;
    for(int i = 0; i< 100; i++)
        durArr[i]=0;
    for(int i = 0; i< 20; i++)
        durArray[i]=0;
    mode=0;//1 play 0 listen
    stop=0;
    Threshold1=0.6;//0.4;
    Threshold2=1.0;//sends beats if human stays silent from 1 sec.
    
    //maybe these should be set somewhere outside of the 
    //update (not inside a loop in orginal code
    time(&stime);
    time(&ftime);  
    ftime1=ftime;
    //st1=clock();
    timeval tim;
    gettimeofday(&tim, NULL);
    st1=tim.tv_sec+(tim.tv_usec/1000000.0);
    //st1 = yarp::os::Time::now();
    ss=st1;

    ok &= quitPort.open(getName("quit"));
    attach(quitPort, true);
    return ok;
}
 
   
bool AudioAnalyserModule::close(){
    sndSensorPort.close();
    soundPort.close();
    return true;
}

bool AudioAnalyserModule::interruptModule(){
    sndSensorPort.interrupt();
    soundPort.interrupt();
    return true;
}

bool AudioAnalyserModule::updateModule(){


    //printf("in updateModule \n");
    
    Sound *sound = soundPort.read();
    if (isStopping() || sound==NULL){
        printf("returning false \n");
        return false;
    }
    
    //ftime1=ftime;
    for (int i=0;i<sound->getSamples();i++) {
        for(int j = 0; j < sound->getChannels(); j++) {
            val=(double)sound->get(i,j)/32768;
            window[z]=val;
            // a filter based algorithm is used to find the peaks in the sound wave
            // online, and filters the noise to find the beats.,
            if (z>=AUDIO_WINDOW_SIZE-1) {
                f=0;
                for (int l=0;l<AUDIO_WINDOW_SIZE;l++)
                    f= f + window[l]*hmask[l];
                if (f>Threshold1) {
                    count++;
                    if ((high<=f) && (H==0) && (count1-count3>th1)) {
                        high=f;
                        avg=window[1]+window[2];
                        low=AUDIO_MAX_VAL;
                        H=1;
                        L=0;
                        vhigh=window[1];
                        count2=2;
                        count3=count1;
                        time(&stime1);
                        //ft=clock();
                        timeval tim;
                        gettimeofday(&tim, NULL);
                        ft=tim.tv_sec+(tim.tv_usec/1000000.0);
                        //ft = yarp::os::Time::now();
                        if (beatNo!=0) {
                            //dur = (double)(ft - st) / CLOCKS_PER_SEC;
                            dur = ft - st;
                            durArr[beatNo]=dur;
                            //  double dur1 = ft - st;
                            //  printf("dur1=%lf ft=%lf st11=%lf st111=%lf ",dur1, ft,  st, durArr[beatNo]);
                            // durArr[beatNo]=ft - durArr[beatNo];//HKB DLETE
                            //printf("%d dur=%lf diff=%lf\n",beatNo, dur, durArr[beatNo]);
                        }
                    }
                }
                if (H==1) {
                    avg+=val;
                    count2++;
                }
                f=0;
                for (int l=0;l<AUDIO_WINDOW_SIZE;l++)
                    f= f + window[l]*lmask[l];
                if (f>Threshold1) {
                    count++;
                    if (low>=f && L==0 && H==1) {
                        low=f;
                        avg=avg-window[2]-window[3];
                        count2=count2-2;//window[2], window[3]
                        avg=avg/count2;
                        vlow=window[1];
                        printf("f=%lf avg=%lf ", f, avg);//HKB DELETE
                        //time(&ftime1);
                        //st=clock();
                        timeval tim;
                        gettimeofday(&tim, NULL);
                        double st=tim.tv_sec+(tim.tv_usec/1000000.0);
                        //st = yarp::os::Time::now();
                        //printf("st1=%lf  ", st);//HKB DELETE
                        beatNo++;
                        //durArr[beatNo]=st;//HKB
                        high=-AUDIO_MAX_VAL;
                        low=AUDIO_MAX_VAL;
                        avg=0;
                        H=0;
                        L=0;
                        
                    }
                }
                window[0]= window[1];
                window[1]= window[2];
                window[2]= window[3];
            } else //(z>=AUDIO_WINDOW_SIZE-1) 
                z++;
            count1++;
        } //channels loop
    } // samples loop
    
    //time(&ftime);
    //fs=clock();
    timeval tim;
    gettimeofday(&tim, NULL);
    double fs=tim.tv_sec+(tim.tv_usec/1000000.0);
    //fs = yarp::os::Time::now();
    //double dur = double(fs -ss);
    //duration_double=(double)(fs-ss)/CLOCKS_PER_SEC;
    duration_double = fs - ss;
    //if enough time has elapsed, send the output
    //printf("duration: %lf %lf\n", duration_double,Threshold2);
    if ( duration_double > Threshold2) { 
        //printf("TH1 %lf", Threshold2);
        // printf("%lf TH %lf ", duration_double, Threshold2);
        // printf("%lf fs=%4.2lf ss=%4.2lf", duration_double, fs, ss);//HKB DELETE
        stop=1;
        ss=fs;
        
        Bottle bot;
        bot.clear();
        printf("beats: %d \n",beatNo);
        bot.addInt(beatNo);
        for (int i=1; i<beatNo; i++) {
            duration = (int)(durArr[i]*AUDIO_MAX_VAL);
            durArr[i]=0;//HKB
            //   printf("%d %lf ",i, durArr[i] );//HKB DELETE
            bot.addInt(duration);//duration
        }
        //reset beats to zero
        beatNo = 0; 
        printf("send message: %s \n", bot.toString().c_str());
        // send the message
        sndSensorPort.write(bot);

    }
    return true;
}

bool AudioAnalyserModule::respond(const Bottle &command,Bottle &reply){
        
    return false;
} 	
