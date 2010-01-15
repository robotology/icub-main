#include <iCub/iha/ShortTermMemoryModule.h>

#include <iCub/iha/iha_utils.h>
#include <iCub/iha/Actions.h>

using namespace iCub::iha;
using namespace std;

/**
 * @addtogroup icub_iha2_Memory
 *

This module receives data for a few specified variables from \ref icub_iha_SensorMotorInterface. The data
reported is:
 - iCub's current action
 - whether the human's face is detected (estimate of hiding behavior)
 - number of current drumbeats (estimate of drumming behavior)

This data is stored for a short window of time of a number of seconds specified by the user. How
frequently the data is stored into this memory is also user specified. The relationship
between the robot's action and the human's behavior is used to produce task-specific scores for 
peek-a-boo and drumming based on the current contents of the short term memory. These
scores are sent to \ref icub_iha2_Dynamics to be used in computing the reward.

\section lib_sec Libraries
- YARP libraries.
- IHA Debug Library

\section parameters_sec Parameters
\verbatim
--dbg <INT>                : debug printing level
--name <STR>               : process name for ports
--file <STR>               : configuration from given file

--resolution [INT]  : number of times per second to record data 
--mem_length [INT]  : number of seconds of memory to store

--connect_to_data [STR]       : connect to sensor data input
\endverbatim

\section portsa_sec Ports Accessed
 - memsensor:out - data output from \ref icub_iha2_SensorMotorInterface


\section portsc_sec Ports Created
 - data:in - get data input
 - score:out - send scores for drumming and peek-a-boo

 - /iha/sm/quit  - module quit port
 
\section conf_file_sec Configuration Files
conf/ihaShortTermMemory.ini

Sample INI file:
\verbatim
dbg 40
name iha

action_defs /usr/local/src/robot/iCub/app/ihaNew/conf/iha_actiondefs.ini

#data stored per second
resolution 20
#length of memory (in seconds)
mem_length 4
\endverbatim

\section tested_os_sec Tested OS
Linux

\section example_sec Example Instantiation of the Module
ihaNewShortTermMemory --file conf/ihaShortTermMemory.ini

\author Frank Broz

Copyright (C) 2009 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at \in src/interactionHistoryNew/short_term_memory/src/ShortTermMemoryModule.cpp.
*/

ShortTermMemoryModule::ShortTermMemoryModule(){
}

ShortTermMemoryModule::~ShortTermMemoryModule(){ 
}


bool ShortTermMemoryModule::open(Searchable& config){
    
	if (config.check("dbg")) { IhaDebug::setLevel(config.find("dbg").asInt()); }
 	ACE_OS::fprintf(stderr, "Debug level : %d\n",IhaDebug::getLevel());
    
    if (config.check("help","if present, display usage message")) {
		cerr << "Usage : " << "\n"
             << "------------------------------------------" << "\n"
             << "  --dbg [INT]   : debug printing level" << "\n"
             << "  --name [STR]  : process name for ports" << "\n"
             << "  --file [STR]  : config file" << "\n"
             << "---------------------------------------------------------------------------" << "\n"
             << "  --resolution [INT]  : number of times per second to record data " << "\n"
             << "  --mem_length [INT]  : number of seconds of memory to store" << "\n"
             << "---------------------------------------------------------------------------" << "\n"
             << "  --connect_to_data [STR]       : connect to sensor data input" << "\n"
             << "---------------------------------------------------------------------------" << "\n"
             << "\n";
        return false;
    }
    
    bool ok = true;
    
    // Read parameters
    //cycles/second
	resolution = config.check("resolution",Value(10)).asInt();
	IhaDebug::pmesg(DBGL_INFO,"resolution:%d\n",resolution);
    //seconds
	mem_length = config.check("mem_length",Value(4)).asInt();
	IhaDebug::pmesg(DBGL_INFO,"mem_length:%d\n",mem_length);
	sound_thresh = config.check("sound_thresh",Value(0.5)).asDouble();
	IhaDebug::pmesg(DBGL_INFO,"sound_thresh:%d\n",sound_thresh);
    
	// create names of ports
    ConstString coordsPortName = getName("data:in");
    ConstString outPortName = getName("score:out");
	
	// open the coordinates reading port
	coordsPort.open(coordsPortName.c_str());
	outPort.open(outPortName.c_str());
    
	// if required we can connect to the coordinates port
	if (config.check("connect_to_data")) {
		if (connectToParam(config,"connect_to_data",coordsPortName.c_str(), 0.25, this)) {
            IhaDebug::pmesg(DBGL_INFO,"Connected to Input Data\n");
        } else {
            ok = false;
        }
	}
    
    waitTime = 1.0/resolution;
    lastTime = 0;
    
    //We need to also open the iCub's action definitions,
    //so we can refer to the actions by name 
    //rather than a number that might change
    
    //------------------------------------------------------
	// get all the configured actions
	ConstString action_defs_file = config.check("action_defs",Value("conf/action_defs.ini")).asString();
	//ConstString sequence_directory = config.check("sequence_dir",Value(".")).asString();
    
	// create the action defs object and read the actions
	// from the config file
	Property actiondefs_props;
    Actions iCubActions;
	actiondefs_props.fromConfigFile(action_defs_file.c_str()); 
    
	if (!iCubActions.open(actiondefs_props)) {
        IhaDebug::pmesg(DBGL_INFO,"Error in action definitions\n");
		exit(-1);
	}
    
    //note: must match the name in action_defs.ini
    hide_act = iCubActions.getActionIndex("Hide-face");
    drum_act = iCubActions.getActionIndex("RArm-Drum");
    
    //store ways that we are going to process the data
    //in the short-term memory in order to determine a score
    vector<int> v;
    memory_process.insert(make_pair("robot_drum",v));
    memory_process.insert(make_pair("human_drum",v));
    memory_process.insert(make_pair("both_drum",v));
    memory_process.insert(make_pair("robot_hide",v));
    memory_process.insert(make_pair("human_hide",v));
    memory_process.insert(make_pair("human_only_hide",v));

    memory_sum.insert(make_pair("robot_drum", 0));
    memory_sum.insert(make_pair("human_drum", 0));
    memory_sum.insert(make_pair("both_drum", 0));
    memory_sum.insert(make_pair("robot_hide", 0));
    memory_sum.insert(make_pair("human_hide", 0));
    memory_sum.insert(make_pair("human_only_hide", 0));
    

    ok &= quitPort.open(getName("quit"));
    attach(quitPort, true);
    return ok;

}

bool ShortTermMemoryModule::close(){
    coordsPort.close();
    outPort.close();
    return true;
}

bool ShortTermMemoryModule::interruptModule(){
    coordsPort.interrupt();
    outPort.interrupt();
    return true;
}

bool ShortTermMemoryModule::updateModule(){
    
    //get the latest sensor reading from SMI
    Bottle* smUpdate = coordsPort.read();
    double currTime = Time::now();
    Bottle outMem;
    
    //if enough time has passed, store it
    if((currTime - lastTime) > waitTime) {
        lastTime = currTime;
        
        
        if(memory["action"].size() >= (resolution*mem_length)) {
            //discard the old memory before adding the new one
            map<string,std::vector<double> >::iterator iter;   
            for( iter = memory.begin(); iter != memory.end(); iter++ ) {
                iter->second.pop_back();
            }
        }
        //store the new data (notice that values are arranged new to old)
        for(int i = 0; i < smUpdate->size(); i=i+2) {
            IhaDebug::pmesg(DBGL_DEBUG1,"%s : %f \n",smUpdate->get(i).asString().c_str(),smUpdate->get(i+1).asDouble());
            std::string s =  string(smUpdate->get(i).asString().c_str()); 
            vector<double>::iterator iter = memory[s].begin();   
            memory[s].insert(iter,smUpdate->get(i+1).asDouble());
        }
        
        int curr_act = int(memory["action"].front());
        //int human_drum = int(memory["sound"].front() > sound_thresh);
        int human_drum = int(memory["beat"].front() > 0.0);
        int robot_drum = int(curr_act == drum_act);
        int both_drum = int(human_drum && robot_drum);
        int robot_hide = int(curr_act == hide_act);
        int human_hide =int(memory["face"].front() == 0.0);
        int human_only_hide = int(human_hide && !robot_hide);
        
        IhaDebug::pmesg(DBGL_DEBUG1,"Current action %d Drum %d Hide %d\n",curr_act, drum_act,hide_act);
        IhaDebug::pmesg(DBGL_DEBUG1,"Drumming: human %d robot %d both %d\n",human_drum,robot_drum,both_drum);
        IhaDebug::pmesg(DBGL_DEBUG1,"Hiding: human %d robot %d human_only %d\n",human_hide,robot_hide,human_only_hide);

        
        if(memory_process["robot_drum"].size() >= (resolution*mem_length)) {
            //discard the old memory before adding the new one
            map<string,std::vector<int> >::iterator iter;   
            for( iter = memory_process.begin(); iter != memory_process.end(); iter++ ) {
                int tmp = iter->second.back();
                iter->second.pop_back();
                memory_sum[iter->first] = memory_sum[iter->first] - tmp;
            }
        }
        //store the new data (notice that values are arranged new to old)
        vector<int>::iterator iter = memory_process["robot_drum"].begin();   
        memory_process["robot_drum"].insert(iter,robot_drum);
        memory_sum["robot_drum"] = memory_sum["robot_drum"] + robot_drum;
        iter = memory_process["human_drum"].begin();   
        memory_process["human_drum"].insert(iter,human_drum);
        memory_sum["human_drum"] = memory_sum["human_drum"] + human_drum;
        iter = memory_process["both_drum"].begin();   
        memory_process["both_drum"].insert(iter,both_drum);
        memory_sum["both_drum"] = memory_sum["both_drum"] + both_drum;
        iter = memory_process["robot_hide"].begin();   
        memory_process["robot_hide"].insert(iter,robot_hide);
        memory_sum["robot_hide"] = memory_sum["robot_hide"] + robot_hide;
        iter = memory_process["human_hide"].begin();   
        memory_process["human_hide"].insert(iter,human_hide);
        memory_sum["human_hide"] = memory_sum["human_hide"] + human_hide;
        iter = memory_process["human_only_hide"].begin();   
        memory_process["human_only_hide"].insert(iter,human_only_hide);
        memory_sum["human_only_hide"] = memory_sum["human_only_hide"] + human_only_hide;
        
    }
    
    IhaDebug::pmesg(DBGL_DEBUG1,"MS Drumming: human %d robot %d both %d\n",memory_sum["human_drum"],memory_sum["robot_drum"],memory_sum["both_drum"]);
    IhaDebug::pmesg(DBGL_DEBUG1,"MS Hiding: human %d robot %d human_only %d\n",memory_sum["human_hide"],memory_sum["robot_hide"],memory_sum["human_only_hide"]);

    //calculate a score for the turn-taking synchronization between the
    //human and the robot for each form of turn-taking
    //this doesn't penalize the robot enough for drumming at the 
    //same time that the human does
    //need a model of duration and start and end of 
    //uninterrupted drumming
    double drum_score = (0.5* (memory_sum["robot_drum"] + memory_sum["human_drum"]) \
                         - memory_sum["both_drum"])/(resolution*mem_length);
    //for peekaboo, if the person is hiding for too long
    //(over a couple of seconds), they are probably not being tracked
    // by the face-tracker, not hiding
    double hide_score = 0;
    //for peekaboo, if the person is hiding for too long
    //(over a couple of seconds), they are probably not being tracked
    // by the face-tracker, not hiding
    //similarly, if too short, might be a tracking err
    //if (memory_sum["human_only_hide"] < (2.5*resolution))
    //if person was hiding (not lost), give robot more reward
    if((memory_sum["human_only_hide"] < (resolution*2.5)) && (memory_sum["human_only_hide"] > (resolution*0.5))){
      hide_score += memory_sum["human_only_hide"];
      hide_score += memory_sum["robot_hide"];
    } else {
        //no reward
        //hide_score += 0.25*memory_sum["robot_hide"];
    }
    hide_score = hide_score/(resolution*mem_length);
    
    //send the memory-based reward to motivation dynamics
    //not the cleanest way to implement, but avoids having to pass
    //whole memory to motivation dynamics
    outMem.addString("drum");    
    outMem.addDouble(drum_score);    
    outMem.addString("hide");    
    outMem.addDouble(hide_score);    
    
    outPort.write(outMem);
    
    return true;
}

bool ShortTermMemoryModule::respond(const Bottle &command,Bottle &reply){
        
    return false;
} 	
