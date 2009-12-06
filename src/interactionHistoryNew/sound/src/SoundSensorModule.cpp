// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// vim:expandtab:tabstop=4:shiftwidth=4:softtabstop=4:

/*
 * Copyright (C) 2008 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author:  Assif Mirza
 * email:   assif.mirza@robotcub.org
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */


#include <iCub/iha/SoundSensorModule.h>
#include <iCub/iha/iha_utils.h>

using namespace iCub::iha;

/**
 * @addtogroup icub_iha_SoundSensor
 *
 * This module takes sound samples at intervals from sound_grabber 
 * interface (subdevice e.g. portaudio) and converts it into a 
 * single sensor number for that sound interval.
 * In this case it calculates only the "volume" of the sound by 
 * summing the amplitude of the samples.
 * In the future it should use FFA or similar to give a better set of
 * sensor values.
 * 
\section lib_sec Libraries
- YARP libraries.
- IHA Debug Library

\section parameters_sec Parameters
\verbatim
--dbg [INT]   : debug printing level
--name [STR]  : process name for ports
--file [STR]  : config file

--connect_to_soundgrabber [STR]  : autoconnect to specified port for sound

--soundsensorrate [INT]  : sensor data rate (in ms), default 100 gives a 10 frames/s
--soundgain [FLT]        : multiplier for sound level
\endverbatim

\section portsa_sec Ports Accessed
- /iha/sound_grabber

\section portsc_sec Ports Created
- /iha/sound/sndsensor:in
- /iha/sound/sndsensor:out
 
- /iha/sound/quit  - module quit port

\section conf_file_sec Configuration Files
conf/ihaSoundSensor.ini

Sample INI file:
\verbatim
# rate of production of samples
soundsensorrate 10

# Multiplying factor
soundgain 100
\endverbatim

\section tested_os_sec Tested OS
Linux

\section example_sec Example Instantiation of the Module
ihaSoundSensor --name /iha/sound --file conf/ihaSoundSensor.ini

See also the script $ICUB_ROOT/app/iha_manual/soundserver.sh

\see iCub::contrib::SoundSensorModule
\see \ref icub_iha_Dynamics

\author Assif Mirza

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at \in src/interactionHistory/sound_sensor/src/SoundSensor.cpp.
*/


SoundSensorModule::SoundSensorModule(){
}

SoundSensorModule::~SoundSensorModule(){ 
}


bool SoundSensorModule::open(Searchable& config){
   
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
	sndSensorRate = config.check("soundsensorrate",Value(100)).asInt();
	IhaDebug::pmesg(DBGL_INFO,"soundsensorrate:%d\n",sndSensorRate);
	sndGain = config.check("soundgain",Value(2.5)).asDouble();
	IhaDebug::pmesg(DBGL_INFO,"soundgain:%f\n",sndGain);
	
	//------------------------------------------------------
	// create output sound sensor port port name
	//char sndsensorportname[30];
	//ACE_OS::sprintf(sndsensorportname,"/%s/sndsensor:out",getName().c_str());

	// open sound sensor output port
	sndSensorPort.open(getName("sndsensor:out"));

	//------------------------------------------------------
	// create input sound port port name
	//char soundportname[30];
	//ACE_OS::sprintf(soundportname,"/%s/sndsensor:in",name.c_str());

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
	

    ok &= quitPort.open(getName("quit"));
    attach(quitPort, true);
    return ok;
}

bool SoundSensorModule::close(){
    sndSensorPort.close();
    soundPort.close();
    return true;
}

bool SoundSensorModule::interruptModule(){
    sndSensorPort.interrupt();
    soundPort.interrupt();
    return true;
}

bool SoundSensorModule::updateModule(){
    Sound *input = soundPort.read();
    if (isStopping() || input==NULL)
    {
        return false;
    }
    int total = 0;
    //float total = 0.0;
    int numsamples = input->getSamples();
    //frank: I don't know that this returned value is true
    //int numbytes = input->getBytesPerSample();
    int freq = input->getFrequency();
    int sampsize = input->getRawDataSize();
    int chan = input->getChannels();
    int numbytes = sampsize/(chan*numsamples);

    // add up the sound values (both channels)
    //int* rdp = (NetInt32*) input->getRawData();
    //for (int p=0;p<numsamples;p++){
    //    total+=abs(*rdp++);
    //}
    for(int c=0; c<chan;c++) {
        for(int p=0; p<numsamples;p++) {
            total += abs(input->get(p,c));
            //total += <unsigned int>input->get(p,c);
        }
    }
    //if (total != 0) {
    //    for(int c=0; c<chan;c++) {
    //        for(int p=0; p<numsamples;p++) {
    //            fprintf(stderr,"%d ", input->get(p,c));
    //        }
    //        fprintf(stderr,"\n ");
    //    }
    //    fprintf(stderr," total %8d samples %d bytes %d len %d freq %d chan %d gain %.2f \n",total,numsamples,numbytes,sampsize,freq,chan, sndGain);
    //}

    //using value for short int ("default" portaudio format for yarp)
    const int MAXVAL=32767;
    //double sval = (double)total * sndGain / (32768*numsamples);
    double sval = (double)total * sndGain / (MAXVAL*numsamples*chan);
    if (sval>1.0) sval=1.0;

    const int linelen = 60;
    if (IhaDebug::getLevel()==DBGL_STATUS1){
        int mark=(int)(sval*linelen);
        for (int i=0;i<mark;i++){
            ACE_OS::fprintf(stderr,"*");
        }
        for (int i=mark;i<linelen;i++){
                ACE_OS::fprintf(stderr," ");
        }
        //ACE_OS::fprintf(stderr,"|\r");
    }
    
    //ACE_OS::fprintf(stderr,"\n");
    //if (IhaDebug::getLevel()==DBGL_STATUS2)
    fprintf(stderr,"%8d\t %.4f  \r",total,sval);
    //if (IhaDebug::getLevel()>DBGL_STATUS2)fprintf(stderr,"%8d\t %.4f  \n",total,sval);
    // IhaDebug::pmesg(DBGL_STATUS2,"%8d\t %.4f  \n",total,sval);
    //fprintf(stderr," total %8d\t  sval %.4f  \n",total,sval);
    //}

    Bottle bot;
    bot.addDouble(sval);
    return sndSensorPort.write(bot);
}

//bool SoundSensorModule::respond(const Bottle &command,Bottle &reply){
//        
//    return false;
//} 	
