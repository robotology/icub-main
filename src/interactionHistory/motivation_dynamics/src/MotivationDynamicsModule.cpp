// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
// vim:expandtab:tabstop=4:shiftwidth=4:softtabstop=4:

/*
 * Copyright (C) 2008 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Assif Mirza
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


#include <iCub/iha/MotivationDynamicsModule.h>
#include <iCub/iha/iha_utils.h>
using namespace iCub::iha;

 /**
  * @addtogroup icub_iha_Dynamics

\section iha_motiv_reward Reward

Motivation feedback (reward) is provided through two mechanisms: observation of a face, and audio feedback.
\subsection iha_motiv_face Face

A face can be detected in the robot's camera image using OpenCV HAAR Cascades, and this provides direct positive reward. Habituation causes this reward to drop-off over time.

The reward for face detection, \f$R_{f}\f$, constrained to be in the range [0,1], is a function of the number of consecutive timesteps a face is seen. 
First the reward rises linearly, then holds at 1 for a period before decaying towards 0. \f$R_{f}\f$ is calculated incrementally

\subsection iha_motiv_sound Sound

Sound is captured from a microphone using \b Portaudio \b v1.9, and used both as an additional sensory signal as well as providing further environmental reward. 
The "energy" of the sound over the period of a timestep, \f$\varepsilon_{sound}\f$, provides a sensory input to the robot. 
It is calculated as the sum of the amplitude of the sound signal for every sound sample in a period of a timestep, and is normalized to take values in the range [0,1]. 

In converting \f$\varepsilon_{sound}\f$ to a reward signal \f$R_{s}\f$, low level background noise is attenuated by taking the square of the sound sensor variable for all values below a threshold \f$T_{sound}\f$, above which the reward value is set to 1. 
Taking the square of the sound signal results in a greater attenuation of smaller values of the variable than larger ones thus effectively reducing background noise and emphasizing the reward when the sound is above the threshold.


\subsection iha_motiv_result Resulting Reward Signal

The final reward signal is a combination of the sound and face reward signals, as follows:

\f[ R = \max( 1, \alpha ( R_{f} + R_{s} ) ) \f]

where \f$\alpha\f$, in the range [0,1] attenuates the reward signal. With \f$\alpha\f$=0.5, R is the average of the reward signals, and with \f$\alpha\f$=1, either of the reward signals can result in a maximum resulting reward. A reasonable setting is, \f$\alpha\f$=0.75, meaning that neither reward signal on its own can result in a maximum \f$R\f$, but requires support from the other reward signal. 

\section lib_sec Libraries
- YARP libraries.
- IHA Debug Library

\section parameters_sec Parameters
\verbatim
--dbg [INT]   : debug printing level
--name [STR]  : process name for ports
--file [STR]  : configuration file
--connect_to_coords [STR]       : autoconnect to specified port for face
--connect_to_soundsensor [STR]  : autoconnect to specified port for sound
--face_response_attack [INT]  : face response, increase period
--face_response_level [INT]   : face response, level off period
--face_response_decay [INT]   : face response, decay period
--sound_catch_threshold [FLT] : level at which sound is heard
--face_lost_count [INT]       : time after which face is considered lost
--reward_contrib_face [FLT]   : contribution of face to reward
--reward_contrib_sound [FLT]  : contribution of sound to reward
\endverbatim

\section portsa_sec Ports Accessed

\section portsc_sec Ports Created
 
\section conf_file_sec Configuration Files
conf/ihaMotivationDynamics.ini

Sample INI file:
\verbatim
dbg 40
name iha

# Controls face response envelope
face_response_attack 4
face_response_level 3
face_response_decay 20

# level at which sound is heard
sound_catch_threshold 0.67

# time after which face is considered lost
face_lost_count 3

# relative contributions to reward
reward_contrib_face 0.75
reward_contrib_sound 0.75
\endverbatim

\section tested_os_sec Tested OS
Linux

\section example_sec Example Instantiation of the Module
ihaMotivationDynamics --name /iha/dynamics --file conf/ihaMotivationDynamics.ini

See also the script $ICUB_ROOT/app/iha_manual/dynamics.sh

\see \ref icub_iha_IhaFaceDetect
\see iCub::contrib::IhaFaceDetectModule
\see \ref icub_iha_SoundSensor
\see iCub::contrib::SoundSensorModule

\see iCub::contrib::MotivationDynamicsModule

\author Assif Mirza

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at \in src/interactionHistory/motivation_dynamics/src/MotivationDynamicsModule.cpp.
\author Assif Mirza
 *
 */
MotivationDynamicsModule::MotivationDynamicsModule(){
    resetcount=0;
    acc=0;
    faceReward=0.0;
}

MotivationDynamicsModule::~MotivationDynamicsModule(){ 
}


bool MotivationDynamicsModule::open(Searchable& config){
   
	if (config.check("dbg")) { IhaDebug::setLevel(config.find("dbg").asInt()); }
 	ACE_OS::fprintf(stderr, "Debug level : %d\n",IhaDebug::getLevel());

    if (config.check("help","if present, display usage message")) {
		cerr << "usage : " << "\n"
		<< "----------------------------------------" << "\n"
		<< "  --dbg [INT]   : debug printing level" << "\n"
		<< "  --name [STR]  : process name for ports" << "\n"
		<< "  --file [STR]  : configuration file" << "\n"
		<< "---------------------------------------------------------------------------" << "\n"
        << "  --connect_to_coords [STR]       : autoconnect to specified port for face" << "\n"
        << "  --connect_to_soundsensor [STR]  : autoconnect to specified port for sound" << "\n"
		<< "---------------------------------------------------------------------------" << "\n"
        << "  --face_response_attack [INT]  : face response, increase period" << "\n"
        << "  --face_response_level [INT]   : face response, level off period" << "\n"
        << "  --face_response_decay [INT]   : face response, decay period" << "\n"
        << "  --sound_catch_threshold [FLT] : level at which sound is heard" << "\n"
        << "  --face_lost_count [INT]       : time after which face is considered lost" << "\n"
        << "  --reward_contrib_face [FLT]   : contribution of face to reward" << "\n"
        << "  --reward_contrib_sound [FLT]  : contribution of sound to reward" << "\n"
		<< "---------------------------------------------------------------------------" << "\n"
		<< "\n";
        return false;
    }

	face_response_attack = config.check("face_response_attack",Value(4)).asInt();
	IhaDebug::pmesg(DBGL_INFO,"face_response_attack:%d\n",face_response_attack);
	face_response_level = config.check("face_response_level",Value(2)).asInt();
	IhaDebug::pmesg(DBGL_INFO,"face_response_level:%d\n",face_response_level);
	face_response_decay = config.check("face_response_decay",Value(20)).asInt();
	IhaDebug::pmesg(DBGL_INFO,"face_response_decay:%d\n",face_response_decay);
	sound_catch_threshold = config.check("sound_catch_threshold",Value(0.67)).asDouble();
	IhaDebug::pmesg(DBGL_INFO,"sound_catch_threshold:%f\n",sound_catch_threshold);
	face_lost_count = config.check("face_lost_count",Value(3)).asInt();
	IhaDebug::pmesg(DBGL_INFO,"face_lost_count:%d\n",face_lost_count);
	reward_contrib_face = config.check("reward_contrib_face",Value(0.75)).asDouble();
	IhaDebug::pmesg(DBGL_INFO,"reward_contrib_face:%f\n",reward_contrib_face);
	reward_contrib_sound = config.check("reward_contrib_sound",Value(0.75)).asDouble();
	IhaDebug::pmesg(DBGL_INFO,"reward_contrib_sound:%f\n",reward_contrib_sound);
	
    bool ok = true;

	// create names of ports
    ConstString coordsPortName = getName("coords:in");
    ConstString soundSensorPortName = getName("soundsensor:in");
    ConstString outputPortName = getName("reward:out");
	
	// open the coordinates reading port
	coordsPort.open(coordsPortName.c_str());

	// if required we can connect to the facedetector coordinates port
	if (config.check("connect_to_coords")) {
		if (connectToParam(config,"connect_to_coords",coordsPortName.c_str(), 0.25, this)) {
            IhaDebug::pmesg(DBGL_INFO,"Connected to Face Coords\n");
        } else {
            ok = false;
        }
	}

	// open the sound sensor reading port
	soundSensorPort.open(soundSensorPortName.c_str());
	soundSensorPort.setStrict(false);

	// if required we can connect to the sound sensor port
	if (config.check("connect_to_soundsensor")) {
		if (connectToParam(config,"connect_to_soundsensor",soundSensorPortName.c_str(), 0.25, this)) {
            IhaDebug::pmesg(DBGL_INFO,"Connected to Sound Sensor\n");
        } else {
            ok = false;
        }
	}

	// open the output port
	outPort.open(outputPortName.c_str());

    ok &= quitPort.open(getName("quit"));
    attach(quitPort, true);
    return ok;
}

bool MotivationDynamicsModule::close(){
    coordsPort.close();
    soundSensorPort.close();
    outPort.close();
    return true;
}

bool MotivationDynamicsModule::interruptModule(){
    coordsPort.close();
    soundSensorPort.close();
    outPort.close();
    return true;
}

bool MotivationDynamicsModule::updateModule(){

    double face_attack_rate = 1.0 / (double)(face_response_attack);
    double face_decay_rate = 1.0 / (double)(face_response_decay-face_response_level);

    // find out if a face has been detected
    Vector* pc = coordsPort.read(true);
    if (pc!=NULL) {
        // accumulate the face sensor, resetting when we dont see it
        if ((*pc)[0]==0 ) {
           if (resetcount >= face_lost_count) {
                acc=0;
                resetcount=0;
           } else {
                resetcount++;
           }
        } else {
            acc++;
            resetcount=0;
        }
        // calc reward due to face:
        // final value will be between 0 and 1
        // When a face is seen, the reward increases linearly for a time face_response_attack 
        // until it reaches its maximum.
        // It stays at maximum for a further face_response_level  timesteps.
        // Then it starts to decay back to 0 over time face_response_decay (geometric)
        //double faceReward = 0;
        if (acc==0)  //  Zero at all times there is no face 
        {
            faceReward = faceReward - face_decay_rate;  // changed to decay
        } 
        else if (acc<face_response_attack) // Increase linearly for Attack phase
        {
            faceReward = faceReward + face_attack_rate;
        } 
        else if ( acc < face_response_level ) // stay level
        {
            faceReward = 1.0;
        }
        else // Linear decrease
        {
            faceReward = faceReward - face_decay_rate;
        }

        if (faceReward<0) faceReward=0.0;
        if (faceReward>1) faceReward=1.0;

        // Get the sound if we have it
        Bottle* bot = soundSensorPort.read(false);
        double sndval=0.0;
        if (bot!=NULL) {
            sndval = bot->get(0).asDouble();
        }


        // reward for sound is additive with face, maximum 1
        // the sound value is squared in order to attenuate low values
        double soundReward = 0;
        if (sndval > sound_catch_threshold) 
        {
            soundReward = 1.0;
        }
        else
        {
            soundReward = sndval*sndval;
        }

        double totalReward = faceReward*reward_contrib_face + soundReward*reward_contrib_sound;
        if (totalReward>1.0) totalReward=1.0;

        string astr="";
        if (totalReward==1) {
            astr.append("\E[31m**********\E[39m");
        } else {
            astr.append("===========",(int)(totalReward*10));
            astr.append("           ",10-(int)(totalReward*10));
        }

        IhaDebug::pmesg(DBGL_DEBUG1,"Face acc/rwd: %04d %06.4f Sound val/rwd: %06.4f %06.4f TotalRwd: %06.4f :%s:\n",acc,faceReward,sndval,soundReward,totalReward,astr.c_str());

        // write the reward to the port
        Bottle out;
        out.addDouble(totalReward);
        outPort.write(out);
    }

    return true;
}

bool MotivationDynamicsModule::respond(const Bottle &command,Bottle &reply){
        
    return false;
} 	
