#include <cmath>
#include <iCub/iha/MotivationDynamicsModule.h>
#include <iCub/iha/iha_utils.h>
using namespace iCub::iha;

 /**
  * @addtogroup icub_iha2_Dynamics

\section iha_motiv_reward Reward

Motivation feedback (reward) is provided through multiple mechanisms: 
 - observation of a face 
 - visual attention through gaze direction
 - presence of sound
 - reward from short term memory (calculated in \ref icub_iha2_Memory)

The contribution of each reward source may be set at the command line. The maximum
combined reward is capped to fall between 0 and 1.

\subsection iha2_motiv_face Face

A face can be detected in the robot's camera image using OpenCV HAAR Cascades, and this provides direct positive reward. Habituation causes this reward to drop-off over time.

The reward for face detection, \f$R_{f}\f$, constrained to be in the range [0,1], is a function of the number of consecutive timesteps a face is seen. 
First the reward rises linearly, then holds at 1 for a period before decaying towards 0. \f$R_{f}\f$ is calculated incrementally

\subsection iha2_motiv_gaze Visual Attention

This reward is based on whether or not the interacting human is looking at the robot at
this timestep. Without scaling, the value is either 0 or 1.

\subsection iha2_motiv_sound Sound

Sound is captured from a microphone using \b Portaudio \b v1.9, and used both as an additional sensory signal as well as providing further environmental reward. 
The "energy" of the sound over the period of a timestep, \f$\varepsilon_{sound}\f$, provides a sensory input to the robot. 
It is calculated as the sum of the amplitude of the sound signal for every sound sample in a period of a timestep, and is normalized to take values in the range [0,1]. 

In converting \f$\varepsilon_{sound}\f$ to a reward signal \f$R_{s}\f$, low level background noise is attenuated by taking the square of the sound sensor variable for all values below a threshold \f$T_{sound}\f$, above which the reward value is set to 1. 
Taking the square of the sound signal results in a greater attenuation of smaller values of the variable than larger ones thus effectively reducing background noise and emphasizing the reward when the sound is above the threshold.

\subsection iha2_motive_turntake Drumming and Hiding

Drumming and hiding are task-based scores that are receieved from the \ref icub_iha2_Memory. See that module for
details about their meaning and how they are calculated. 

\subsection iha_motiv_result Resulting Reward Signal

The final reward signal is a combination of the selected reward signals, as follows:

\f[ R = \max( 1,( R_{1} + R_{2} + ... ) ) \f]


\section lib_sec Libraries
- YARP libraries.
- IHA Debug Library

\section parameters_sec Parameters
\verbatim
--dbg [INT]   : debug printing level
--name [STR]  : process name for ports
--file [STR]  : configuration file

--connect_to_data [STR]       : autoconnect to specified port for sensor data
--connect_to_mem [STR]       : autoconnect to specified port for short term memory
--connect_to_expression [STR]  : connect to port for sending emotion actions

--face_response_attack [INT]  : face response, increase period
--face_response_level [INT]   : face response, level off period
--face_response_decay [INT]   : face response, decay period
--face_lost_count [INT]       : time after which face is considered lost
--sound_catch_threshold [FLT] : level at which sound is heard

--reward_contrib_face [FLT]   : contribution of face to reward
--reward_contrib_sound [FLT]  : contribution of sound to reward
--reward_contrib_gaze [FLT]  : contribution of gaze to reward
--reward_contrib_drum [FLT]  : contribution of drumming to reward
--reward_contrib_hide [FLT]  : contribution of hiding to reward

--th_ehi [FLT]              : threshold for high reward
--th_elo [FLT]              : threshold for low reward
\endverbatim

\section portsa_sec Ports Accessed

 - /icub/face/raw/in Raw expression port for the robot

\section portsc_sec Ports Created
 
 - data:in reads sensor data from sensor motor interface
 - mem:in reads task-based scores for drumming and hiding from short term memory

 - expression:out sends raw expression commands as feedback for current reward value
 - reward:out sends combined reward value

 
\section conf_file_sec Configuration Files
conf/ihaMotivationDynamics.ini

Sample INI file:
\verbatim
name iha
dbg 40


#Make the sensor list and bin values accessible 
[include "ihaSensorMotorInterface.ini"]

# Controls face response envelope
#4
face_response_attack 10
face_response_level 20
face_response_decay 50

# level at which sound is heard
sound_catch_threshold 0.7

# time after which face is considered lost
face_lost_count 4

# relative contributions to reward
reward_contrib_face 1.0
reward_contrib_sound 0.60

# Physical Display ON/OFF
reward_display TRUE

###########################################################
# emotion actions
#
# Hi/Lo/Mid Actions
# Thresholds
th_ehi 0.7
th_elo 0.3
#
###########################################################
\endverbatim

\section tested_os_sec Tested OS
Linux

\section example_sec Example Instantiation of the Module
ihaMotivationDynamics --name /iha/dynamics --file conf/ihaMotivationDynamics.ini

See also the script $ICUB_ROOT/app/ihaNew/dynamics.sh

\see \ref icub_iha2_IhaFaceDetect
\see \ref icub_iha2_SoundSensor

\author Frank Broz and Assif Mirza

Copyright (C) 2009 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at \in src/interactionHistory/motivation_dynamics/src/MotivationDynamicsModule.cpp.
 *
 */
MotivationDynamicsModule::MotivationDynamicsModule(){
    resetcount=0;
    acc=0;
    faceReward=0.0;
    gazeReward=0.0;
    gazeAlpha=0.0;
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
        << "  --connect_to_data [STR]       : autoconnect to specified port for sensor data" << "\n"
        << "  --connect_to_mem [STR]       : autoconnect to specified port for short term memory" << "\n"
        << "  --connect_to_expression [STR]  : connect to port for sending emotion actions" << "\n"
		<< "---------------------------------------------------------------------------" << "\n"
        << "  --face_response_attack [INT]  : face response, increase period" << "\n"
        << "  --face_response_level [INT]   : face response, level off period" << "\n"
        << "  --face_response_decay [INT]   : face response, decay period" << "\n"
        << "  --sound_catch_threshold [FLT] : level at which sound is heard" << "\n"
        << "  --face_lost_count [INT]       : time after which face is considered lost" << "\n"
        << "  --reward_contrib_face [FLT]   : contribution of face to reward" << "\n"
        << "  --reward_contrib_sound [FLT]  : contribution of sound to reward" << "\n"
        << "  --reward_contrib_gaze [FLT]  : contribution of gaze to reward" << "\n"
        << "  --reward_contrib_drum [FLT]  : contribution of drumming to reward" << "\n"
        << "  --reward_contrib_hide [FLT]  : contribution of hiding to reward" << "\n"
        << "  --th_ehi [FLT]              : threshold for high reward" << "\n"
        << "  --th_elo [FLT]              : threshold for low reward" << "\n"
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
	reward_contrib_gaze = config.check("reward_contrib_gaze",Value(0.75)).asDouble();
	IhaDebug::pmesg(DBGL_INFO,"reward_contrib_gaze:%f\n",reward_contrib_gaze);
	reward_contrib_drum = config.check("reward_contrib_drum",Value(0.75)).asDouble();
	IhaDebug::pmesg(DBGL_INFO,"reward_contrib_drum:%f\n",reward_contrib_drum);
	reward_contrib_hide = config.check("reward_contrib_hide",Value(0.75)).asDouble();
	IhaDebug::pmesg(DBGL_INFO,"reward_contrib_hide:%f\n",reward_contrib_hide);
	

    //indices used to access the sensor data
    //these are defined in the SMI config file and included for motivation dynamics
    ts_offset = config.check("ts_offset",Value(1)).asInt();
    num_encoders = config.check("num_encoders",Value(38)).asInt();
    Value *v;
    config.check("face_offset",v);
    face_index = v->asInt() + ts_offset;
    config.check("sound_offset",v);
    sound_index = v->asInt() + ts_offset;
    config.check("gaze_offset",v);
    gaze_index = v->asInt() + ts_offset;
    config.check("beat_offset",v);
    beat_index = v->asInt() + ts_offset;
    config.check("action_offset",v);
    action_index = v->asInt() + ts_offset;
    config.check("reward_offset",v);
    reward_index = v->asInt() + ts_offset;
    config.check("insert_offset",v);
    insert_index = v->asInt() + ts_offset;

    last_enc= new double[num_encoders];

	// emotive actions. The actions executed in response
	// to reward values, and the thresholds
	reward_display = boolStringTest(config.check("reward_display",Value("TRUE")).asString());
	IhaDebug::pmesg(DBGL_INFO,"reward_display %s\n",reward_display?"TRUE":"FALSE");

    //note: these are no longer action indicies, they should
    //just have different values
    action_ehi = config.check("action_ehi",Value(1)).asInt();
    action_elo = config.check("action_elo",Value(2)).asInt();
    action_emid = config.check("action_emid",Value(3)).asInt();
    th_ehi = config.check("th_ehi",Value(0.8)).asDouble();
    th_elo = config.check("th_elo",Value(0.3)).asDouble();
    
    current_eout=action_emid;
    IhaDebug::pmesg(DBGL_INFO,"Emote actions: (%d) < %f (%d) < %f (%d)\n",action_elo,th_elo,action_emid,th_ehi,action_ehi);

	// open the sensor data input port
    ConstString dataPortName = getName("data:in");
	dataPort.open(dataPortName.c_str());

    bool ok = true;
	// if required we can connect to the data port
	if (config.check("connect_to_data")) {
		if (connectToParam(config,"connect_to_data",dataPortName.c_str(), 0.25, this)) {
            IhaDebug::pmesg(DBGL_INFO,"Connected to SMI Data\n");
        } else {
            ok = false;
        }
	}

	// open the short term memory port
    ConstString memPortName = getName("mem:in");
	memPort.open(memPortName.c_str());

    ok = true;
	// if required we can connect to the data port
	if (config.check("connect_to_mem")) {
		if (connectToParam(config,"connect_to_mem",memPortName.c_str(), 0.25, this)) {
            IhaDebug::pmesg(DBGL_INFO,"Connected to short term memory\n");
        } else {
            ok = false;
        }
	}

	//------------------------------------------------------
	// open the output port where we write expressions
    // this should be connected to the raw port
    ConstString expressionRawPortName = getName("expression:out");
	IhaDebug::pmesg(DBGL_INFO,"Writing expressions to port %s\n",expressionRawPortName.c_str());
	expressionRawPort.open(expressionRawPortName.c_str());
	//------------------------------------------------------


	if (config.check("connect_to_expression")) {
		// reverse connection
		if (connectToParamReverse(config,"connect_to_expression",expressionRawPortName.c_str(), 0.25, this)) {
            IhaDebug::pmesg(DBGL_INFO,"Connected to Expression Raw\n");
        } else {
            ok = false;
        }
	}


	// open the data (with reward added) output port
    ConstString outputPortName = getName("reward:out");
	outPort.open(outputPortName.c_str());

    ok &= quitPort.open(getName("quit"));
    attach(quitPort, true);
    return ok;
}

bool MotivationDynamicsModule::close(){
    IhaDebug::pmesg(DBGL_DEBUG1,"In closeModule\n");
    dataPort.close();
    memPort.close();
    outPort.close();

    delete [] last_enc;
    last_enc = NULL;

    return true;
}

bool MotivationDynamicsModule::interruptModule(){
    IhaDebug::pmesg(DBGL_DEBUG1,"In interruptModule\n");
    dataPort.close();
    memPort.close();
    outPort.close();
    return true;
}



void MotivationDynamicsModule::sendExpression(int expr) {
    Bottle out;
    // hard coded for now
    if (expr==action_ehi) {
        out.add("L04");
        expressionRawPort.write(out);
        out.clear();
        out.add("R04");
        expressionRawPort.write(out);
        out.clear();
        out.add("M0B");
        expressionRawPort.write(out);
        out.clear();
        out.add("S7F");
        expressionRawPort.write(out);
        out.clear();
    }
    if (expr==action_emid) {
        out.add("L02");
        expressionRawPort.write(out);
        out.clear();
        out.add("R02");
        expressionRawPort.write(out);
        out.clear();
        out.add("M08");
        expressionRawPort.write(out);
        out.clear();
        out.add("S7F");
        expressionRawPort.write(out);
        out.clear();
    }
    if (expr==action_elo) {
        out.add("L04");
        expressionRawPort.write(out);
        out.clear();
        out.add("R04");
        expressionRawPort.write(out);
        out.clear();
        //out.add("M0B");
        out.add("M38");
        expressionRawPort.write(out);
        out.clear();
        out.add("S5B");
        expressionRawPort.write(out);
        out.clear();
    }
    
}


bool MotivationDynamicsModule::updateModule(){

    vector<string> reward_names;
    vector<double> rewards;
    vector<double> reward_contribs;
    int num_rewards = 0;

    double face_attack_rate = 1.0 / (double)(face_response_attack);
    double face_decay_rate = 1.0 / (double)(face_response_decay-face_response_level);

    
    //get the turn-taking reward 
    IhaDebug::pmesg(DBGL_DEBUG1,"Reading from mem port \n");
    Bottle* mb = memPort.read(true);
    IhaDebug::pmesg(DBGL_DEBUG1,"Read from mem port \n");

    reward_names.push_back("drum");
    double drumscore = mb->get(1).asDouble();
    rewards.push_back(drumscore);
    reward_contribs.push_back(reward_contrib_drum);
    num_rewards++;
    reward_names.push_back("hide");
    double hidescore = mb->get(3).asDouble();
    rewards.push_back(hidescore);
    reward_contribs.push_back(reward_contrib_hide);
    num_rewards++;


    IhaDebug::pmesg(DBGL_DEBUG1,"Reading from data port \n");
    Bottle* db = dataPort.read(true);
    IhaDebug::pmesg(DBGL_DEBUG1,"Read from data port \n");
    if (db!=NULL) {

      double action = db->get(action_index).asDouble();
      
      //accumulate the face sensor, resetting when we dont see it
      IhaDebug::pmesg(DBGL_DEBUG1,"Face %lf %d \n",db->get(face_index).asDouble(),(int)db->get(face_index).asDouble());
      if (((int) db->get(face_index).asDouble()) == 0 ) {
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
	  IhaDebug::pmesg(DBGL_DEBUG1,"Face acc/rwd: %04d %06.4f decaying 1 \n",acc,faceReward);
        } 
      else if (acc<face_response_attack) // Increase linearly for Attack phase
        {
	  faceReward = faceReward + face_attack_rate;
	  IhaDebug::pmesg(DBGL_DEBUG1,"Face acc/rwd: %04d %06.4f increasing %d %lf \n",acc,faceReward, face_response_attack, face_attack_rate);
        } 
      else if ( acc < face_response_level ) // stay level
        {
	  IhaDebug::pmesg(DBGL_DEBUG1,"Face acc/rwd: %04d %06.4f level %d \n",acc,faceReward, face_response_level);
	  faceReward = 1.0;
        }
      else // Linear decrease
        {
	  faceReward = faceReward - face_decay_rate;
	  IhaDebug::pmesg(DBGL_DEBUG1,"Face acc/rwd: %04d %06.4f decaying 2 \n",acc,faceReward);
        }
      
      if (faceReward<0.0) faceReward=0.0;
      if (faceReward>1.0) faceReward=1.0;
      
      reward_names.push_back("face");
      rewards.push_back(faceReward);
      reward_contribs.push_back(reward_contrib_face);
      num_rewards++;
      
      double sndval = db->get(sound_index).asDouble();
      
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
      
      reward_names.push_back("sound");
      rewards.push_back(soundReward);
      reward_contribs.push_back(reward_contrib_sound);
      num_rewards++;
      
      //need to save and compare the last set of encoder values to
      //compute both the motion and the turn-taking rewards
      //how do I find out how many values are for the encoders at this point?
      //need a way to label the data
      
      //this assumes that the encoders are always the first values in the sensor array
      double enc_diff = 0.0;
      for(int i = 0; i < num_encoders; i++) {
	double e = db->get(i + ts_offset).asDouble();
	enc_diff += fabs(e - last_enc[i]);
	last_enc[i] = e;
      }
      IhaDebug::pmesg(DBGL_DEBUG1,"Encoder diff %06.4f \n",enc_diff);
      
      //Frank: new sources of reward
      //motion
      //have robot get reward for moving? How to do habituation correctly?
      //Aim for giving max reward when there is some movement, but not "too much"
      
      
      //mutual gaze
      //reward visual attention 
      double gaze = db->get(gaze_index).asDouble();
      gazeReward = gazeAlpha*gaze + (1 - gazeAlpha)*gazeReward;
      reward_names.push_back("gaze");
      rewards.push_back(gazeReward);
      reward_contribs.push_back(reward_contrib_gaze);
      num_rewards++;
        

      //compute reward from a vector of arbitrary length
      double totalReward = 0.0;
      for(int i=0; i < num_rewards; i++) {
	totalReward += rewards[i]*reward_contribs[i];
      }
      //double totalReward = faceReward*reward_contrib_face + soundReward*reward_contrib_sound;
      if (totalReward>1.0) totalReward=1.0;
      if (totalReward<0.0) totalReward=0.0;
      
      //string astr="";
      //if (totalReward==1) {
      //     astr.append("\E[31m**********\E[39m");
      //} else {
      //    astr.append("===========",(int)(totalReward*10));
      //    astr.append("           ",10-(int)(totalReward*10));
      //}
      
      for(int i=0; i < num_rewards; i++) {
	IhaDebug::pmesg(DBGL_DEBUG1,"%s rwd: %06.4f contrib: %06.4f \n",reward_names[i].c_str(),rewards[i],reward_contribs[i]);
      }
      IhaDebug::pmesg(DBGL_DEBUG1,"TotalRwd: %06.4f : %d \n",totalReward, num_rewards);
      
      // add the reward to the sensor output in the right place
      //indicies from ExMetSpace.ini 
      Bottle out;
      out.copy(*db,0,insert_index);
      out.addDouble(drumscore);
      out.addDouble(hidescore);
      out.addDouble(action);
      out.addDouble(totalReward);
      Bottle tmp;
      tmp.copy(*db,insert_index+1,db->size()-insert_index-1); //skip action since 
                                                              //inserted above 
      out.append(tmp);
      outPort.write(out);
      IhaDebug::pmesg(DBGL_DEBUG1,"Wrote output to port\n");
      
      if (reward_display) {
	if (totalReward >= th_ehi) {
	  new_eout=action_ehi;
	} else if (totalReward < th_elo) {
	  new_eout=action_elo;
	} else {
	  new_eout=action_emid;
	}
	
	if (current_eout != new_eout) {
	  IhaDebug::pmesg(DBGL_STATUS1,"============ Emote action %d\n",new_eout);
	  sendExpression(new_eout);
	  IhaDebug::pmesg(DBGL_DEBUG1,"============ Emote action %d done\n",new_eout);
	}
	current_eout = new_eout;
      }
      
    }

    return true;
}

bool MotivationDynamicsModule::respond(const Bottle &command,Bottle &reply){
        
    return false;
} 	
