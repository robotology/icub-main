
// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2008 Sarah Degallier Ludovic Righetti BIRG EPFL Lausanne
 * RobotCub Consortium, European Commission FP6 Project IST-004370
 * email:   sarah.degallier@robotcub.org
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


/**
 * @ingroup icub_module
 *
 * \defgroup icub_DrumManager DrumManager
 *
 *This module is part of the application \ref icub_drummingEPFL "drummingEPFL"
 *
 *\section intro_sec Description 
 *
 * This module transforms a score into the appropriate parameters for the dynamical systems generating the trajectories (\in \ref icub_DrumGenerator "DrumGenerator"). It sends the parameters at timing corresponding to the beat of each of the part.If you want to  use the whole drumming application, please refer to \ref icub_drummingEPFL "drummingEPFL". 
 *
 *\section lib_sec Libraries
 *
 *No external libraries
 *
 *
 *\section parameters_sec Parameters
 *
 * --config-path $ICUB_ROOT/app/DrummingEpfl/conf 
 *
 *\section portssa_sec Ports Accessed 
 *
 * Input ports\n
 * <ul>
 * <li> for each active \a part (i.e left_arm, right_arm, left_leg, right_leg, head) of the robot: /part/parameters/in (created by the \ref icub_DrumGenerator "DrumGenerator" module)
 *<li> for each active \a part of the robot: /part/sound/in (created by the \ref icub_DrumGenerator "DrumGenerator" module)
 *</ul>
 *
 * Output ports\n
 * <ul>
 * <li> for each active \a part of the robot: /part/check_motion/out (created by the \ref icub_DrumGenerator "DrumGenerator" module)
 *<li> a port /midiDrum/server/out (created by the \ref icub_midiDrum "midiDrum" module)
 * <li> for each active \a part of the robot: /part/score/out (created by the \ref icub_guiDemo "guiDemo" module)
* <li> for each active \a part of the robot: /part/phase/out (created by the \ref icub_guiDemo "guiDemo" module)
 * <li> a port /interactive/out (created by the \ref icub_guiDemo "guiDemo" module)
 * </ul>
 *  
 *
 *\section portsc_sec Ports Created
 *
 * Input ports\n
 * <ul>
 *<li> For each active \a part of the robot, a corresponding port /part/check_motion/in receives two integers:
 * <ul>
 * <li> the current beat of the \a part 
 * <li> a value 0 or 1 to check if the motion is finished (not used yet);
 * </ul>
 * <li> For each \a part of the robot, a corresponding port /part/score/in receives a vector of integers corresponding to the target positions at the different time steps;
* <li> For each \a part of the robot, a corresponding port /part/phase/in receives a vector of doubles corresponding to the phase shift of each part at the different time steps;
 * <li> A port /interactive/in receives a vector corresponding to the frequency at the different time steps. \n
 *<li> A port /midiDrum/in receives information on the sound feedback, the third  value corresponds to the instrument and the fourth one to the velocity of the impact. 
 * </ul>
 * Output ports\n
 * <ul>
 * <li> For each \a part of the robot, a corresponding port /part/parameters/out sends 2*ndofs+1 doubles (where ndofs is the number of dofs of \a part)
 * <ul> 
 * <li> the amplitude of the movement (1 if drumming; -5 if idle) and the target position for each dof
 * <li> the phase shift of the \a part (for the the legs the phase shift is applied to the right one, the left one always stays synchronized with the clock)
 * </ul>
 *<li> For each \a part of the robot, a corresponding port /part/sound/out sends one integer (1 if feedback should be enabled, 0 otherwise)
 *</ul>
 *
 *\section conf_file_sec Configuration Files
 *
 * For each active part, this module requires:
 *<ul>
 *<li> partConfig.ini
 *<li> partTargets.ini
 *</ul>
 *
 * Examples of such files can be found at \in src/drummingEPFL/config/left_armConfig.ini and at \in src/drummingEPFL/config/left_armTargets.ini.  
 *
 *
 *\section tested_os_sec Tested OS
 *
 * This module has been tested on Linux and Windows. 
 *
 *\section example_sec Example Instantiation of the Module
 *
 * ./DrumManager --config-path $ICUB_ROOT/app/DrummingEpfl/conf/
 *
 * This file can be edited at \in src/drummingEPFL/DrumManager/main.cpp 
 *
 *\authors Sarah Degallier Ludovic Righetti
 *
 **/

#include "drum.h"

//which dof oscillate.. 
double  mu_on[5][4] ={{1.0,-5.0,-5.0,1.0}, //left arm 
		      {1.0, -5.0, -5.0, 1.0}, //right arm
		      {1.0, -5.0, -5.0, 1.0}, //left leg
		      {1.0, -5.0, -5.0, 1.0}, //right leg
		      {1.0, -5.0, -5.0, -5.0}}; //head
   
drum *MyGMP;


//********GETTING nbDOFs FOR EACH PART IN CONFIG FILES ******************************

void getConfig(drum *MyGMP){

    for(int i=0; i<MyGMP->nbparts; i++)
        { 
            if(MyGMP->ok[i])
                {
                    bool confFile;
                    char partName[255];
                    sprintf(partName, "%s/%sConfig.ini", MyGMP->pathToConfig, MyGMP->parts[i].c_str());
                    Property partConf;
                    confFile=partConf.fromConfigFile(partName);
                    if(!confFile)
                        {
                            ACE_OS::printf("Config file \"%s\" not found for part %s\n", partName, MyGMP->parts[i].c_str());
                            Network::fini();
                            exit(-1);
                        }
                    MyGMP->controlled_dofs[i] = partConf.find("nbDOFs").asInt();
                    ACE_OS::printf("Controlling %d dofs for part %s\n", MyGMP->controlled_dofs[i], MyGMP->parts[i].c_str());
                }
        }
}

//********GETTING DRUM POSITION INFORMATION FOR EACH ACTIVE PART **************************

void getDrumInfo(drum *MyGMP){

    for(int i=0; i<MyGMP->nbparts; i++)
        { 
            if(MyGMP->ok[i])
                {
                    bool file;
                    char drumName[255], temp[255];
                    sprintf(temp, "%s/%sTargets.ini", MyGMP->pathToConfig, MyGMP->parts[i].c_str());
                    Property drum;
                    file=drum.fromConfigFile(temp);
                    if(!file)
                        {
                            ACE_OS::printf("Target positions file \"%s\" not found for part %s\n", temp, MyGMP->parts[i].c_str());
                            Network::fini();
                            exit(-1);
                        }
                    MyGMP->nbDrums[i] = drum.find("NbDrums").asInt();
                    ACE_OS::printf("nb drums %d\n",MyGMP->nbDrums[i]);
                    for(int j=1; j<(MyGMP->nbDrums[i]+1); j++)
                        {
			  sprintf(drumName, "Drum_%d", j);			  
			  Bottle& Target= drum.findGroup(drumName);
                            
			  for (int k=1; k<Target.size(); k++) 
			    {
			      MyGMP->G[i][j][k-1]=3.14/180.0*Target.get(k).asDouble();
			      ACE_OS::printf("%f ", MyGMP->G[i][j][k-1]);
			    }
			  ACE_OS::printf("\n");
			  Target.clear();
                        }
                    Bottle& Target2 = drum.findGroup("Idle");
                    for (int k=1; k<Target2.size(); k++) {
		      MyGMP->G[i][0][k-1]=3.14/180.0*Target2.get(k).asDouble();
		      ACE_OS::printf("%f ", MyGMP->G[i][0][k-1]);
                    }
                    printf("\n");
                    Target2.clear();
                    
		    ACE_OS::printf("Notes: ");
                    Bottle& Target3 = drum.findGroup("Notes");
                    for (int k=1; k<Target3.size(); k++) {
                        printf("%f ", Target3.get(k).asDouble());
                        MyGMP->notes[i][k-1]=Target3.get(k).asInt();
                    }
                    printf("\n");
                    Target3.clear();
                    
                    //printf("\n");
                    ACE_OS::printf("Target angles G for part %s:\n",MyGMP->parts[i].c_str());
                    for(int j=0; j<MyGMP->max_drums; j++){
                        for(int k=0; k<MyGMP->max_dofs; k++){
                            printf("%f ",MyGMP->G[i][j][k]);
                        }
                        printf("\n");
                    }
                }
        }
}

//***********OPENING PORTS AND CONNECTING********************

void doConnect(drum *MyGMP){

    for(int i=0; i<MyGMP->nbparts; i++)
        {
            MyGMP->openPort(i,MyGMP->param_port,"parameters",1,1);// OUT sends parameters to the generator
      
            if(MyGMP->ok[i])
                {
                    ACE_OS::printf("Using part %s\n", MyGMP->parts[i].c_str());
	  
                    //opening ports and connecting with the DrumGenerator modules
                    MyGMP->openPort(i,MyGMP->check_port, "check_motion",0,1);  // IN receives beat 
                    MyGMP->openPort(i,MyGMP->sound_port, "sound", 1,1); // OUT sends soundfeedback info to the generator 

                    //opening ports for the connection with the guiDemo
                    MyGMP->openPort(i,MyGMP->score_port, "score", 0,0); //IN receives score
                    MyGMP->openPort(i,MyGMP->phase_port, "phase", 0,0); //IN receives phase
                }
           
            else
                {
                    MyGMP->param_port[i].close();
                    ACE_OS::printf("Not using %s\n", MyGMP->parts[i].c_str());
                }
        }

    //connecting with the clock
    MyGMP->clock_port.open("/clock/parameters/out");
    bool okClock=Network::connect("/clock/parameters/out","/clock/parameters/in", "tcp");
    if(!okClock)ACE_OS::printf("troubles connecting with the clock\n");
    MyGMP->beat_clock_port.open("/clock/check_motion/in");
    bool okClock2=Network::connect("/clock/check_motion/out","/clock/check_motion/in", "tcp");
    if(!okClock2)ACE_OS::printf("troubles connecting with the clock\n");

    //opening port to get frequency and couplings from the DrumManager
    MyGMP->interactive_port.open("/interactive/in");
  
    //opening and connecting ports for the SOUND FEEDBACK
    //MyGMP->midi_port.open("/midiDrum/in");
    //MyGMP->midi_port.setStrict(true);
    //MyGMP->soundFeedback=Network::connect("/midiDrum/server/out","/midiDrum/in", "tcp");
    //if(!MyGMP->soundFeedback){ACE_OS::printf("No sound feedback\n");}
    //else{ACE_OS::printf("Sound feedback enabled\n");}
 
}

  

//********* DRUMMING TASK *****************************************************************
 

void Drumming(drum *MyGMP){
 
    int escape=1;//parameters to exit while
  
    int Score[MyGMP->nbparts][MyGMP->sizeScore];
    for(int i=0;i<MyGMP->nbparts;i++)
        {
            for(int j=0;j<MyGMP->sizeScore;j++)
                {
                    Score[i][j]=0;
                }
        }
    int init[MyGMP->nbparts];
    for(int i=0; i<MyGMP->nbparts;i++)
        {
            init[i]=0;
        }

    for(int i=0;i<MyGMP->sizeScore; i++)
        {
            MyGMP->rhythmParam[i]=0.0;
        }

    for(int i=0; i<MyGMP->sizeScore; i++)
        {for(int k=0; k<MyGMP->nbparts; k++)
            {MyGMP->phase_shift[k][i]=0.0;}}

    int beat_clock = -1;
    int drum_beat_clock;

    while(true)
        {
            //******Getting rhythm*********
            Bottle *Rhythm = MyGMP->interactive_port.read(false);
            if(Rhythm!=NULL) 
                {
                    if(beat_clock==-1)
                        {
                            Bottle *time_init=MyGMP->beat_clock_port.read();
                            beat_clock=0;
                            if(time_init!=NULL) drum_beat_clock = time_init->get(0).asInt();
                        }
                    else
                        {
                            Bottle *beat_c= MyGMP->beat_clock_port.read(false);
                            if(beat_c!=NULL) beat_clock = beat_c->get(0).asInt()-drum_beat_clock;	  
                        }
	    
                    escape=MyGMP->getRhythm(Rhythm, beat_clock);

                    if(escape==0)
                        {
                            ACE_OS::printf("Closing command received from the gui...\n");
                            break; //GETTING OUT OF THE WHILE LOOP: if negative frequency is sent by the gui, the manager closes.
                        }
                }
      

            //********Getting and sending sound feedback information (if enabled)***********
            if(MyGMP->soundFeedback)
                {
                    Bottle *Hit= MyGMP->midi_port.read(false);
                    if(Hit!=NULL) MyGMP->sendSoundFeedback(Hit,MyGMP->sound_port);
              
                    else
		      {
                        for(int i=0; i<MyGMP->nbparts; i++)
                            {
                                Bottle& soundfeed2= MyGMP->sound_port[i].prepare();
                                soundfeed2.clear();  
                                soundfeed2.addInt(-1);  
                                MyGMP->sound_port[i].write();
                            }
                    }
                }


            //******Getting scores*****************
            for(int i=0; i<MyGMP->nbparts; i++)
                {
                    if(MyGMP->ok[i])
                        {
                            Bottle *newScore = MyGMP->score_port[i].read(false); 
                            if(newScore!=NULL) 
			      {				
				if(MyGMP->beat[i]==-1)//getting beat of the generator
				  {
				    Bottle *time_init=MyGMP->check_port[i].read();
				    MyGMP->beat[i]=0;
				    if(time_init!=NULL) MyGMP->drum_beat[i] = time_init->get(0).asInt();
				  }
				ACE_OS::printf("Score for part %s: ",MyGMP->parts[i].c_str());
				for (int j=0; j<MyGMP->sizeScore; j++) 
				  {
				    int indiceScore = (j+MyGMP->beat[i])%MyGMP->sizeScore;
				    Score[i][indiceScore]= newScore->get(j).asInt();
				    ACE_OS::printf("%d ",Score[i][indiceScore], newScore->get(j).asInt());
				  }
			      	ACE_OS::printf("\n");
			      }
	     

			    if(MyGMP->beat[i]!=-1)
			      {
				Bottle *newPhase = MyGMP->phase_port[i].read(false); 
				if(newPhase!=NULL) 
				  {
				    for(int j=0; j<MyGMP->sizeScore; j++) 
				      {
					int indiceScore = (j+MyGMP->beat[i])%MyGMP->sizeScore;
					MyGMP->phase_shift[i][indiceScore]= newPhase->get(j).asDouble();
				      }
				    
				    ACE_OS::printf("Phase shifts for part %s: ",MyGMP->parts[i].c_str());
				    for (int j=0; j<MyGMP->sizeScore; j++) 
				      {
					ACE_OS::printf("%f ", MyGMP->phase_shift[i][j]);
				      }					
				    ACE_OS::printf("\n");
				  }
			      }
			}}                  
   
	    //SENDING SCORES
   
            for(int i=0; i<MyGMP->nbparts; i++)
                {	   
                    if(MyGMP->ok[i] && MyGMP->beat[i]!=-1)
                        {
                            Bottle *answer = MyGMP->check_port[i].read(false);
                            if(answer!=NULL)
                                {
                                    MyGMP->current_beat[i] = answer->get(0).asInt();
                                    MyGMP->beat[i]=(MyGMP->current_beat[i]-MyGMP->drum_beat[i])%MyGMP->sizeScore;
		      
                                    ACE_OS::printf("beat %d for part %s\n",MyGMP->beat[i], MyGMP->parts[i].c_str());	                                       ACE_OS::printf("Parameters sent to %s: ", MyGMP->parts[i].c_str());
                                    for(int k=0; k<MyGMP->controlled_dofs[i]; k++)
                                        {
                                            MyGMP->g[i][k]= MyGMP->G[i][Score[i][MyGMP->beat[i]]][k];
                                            ACE_OS::printf("%f ", MyGMP->g[i][k]); 
                                            if(Score[i][MyGMP->beat[i]]>0) //BEATING
                                                {
                                                    MyGMP->mu[i][k] = mu_on[i][k];
                                                    init[i] = 1;
                                                }
                                            else //HOLD ON; PART NOT ACTIVE
                                                {
                                                    MyGMP->mu[i][k] = MyGMP->m_off;
                                                    init[i]=0;
                                                }
                                        }
                                    ACE_OS::printf("\n");
                                    MyGMP->sendNewParameterSet(MyGMP->mu[i],MyGMP->g[i],MyGMP->phase_shift[i][MyGMP->beat[i]],MyGMP->rhythmParam[MyGMP->beat[i]], i,MyGMP->param_port);}
		  	  
                            //sending frequency to the clock	      		  
                            Bottle& HeadBot = MyGMP->clock_port.prepare();
                            HeadBot.clear();  
                            HeadBot.addDouble(MyGMP->rhythmParam[MyGMP->beat[i]]); //freq
                            MyGMP->clock_port.write();	       
                        }
                }

            if(beat_clock!=-1)
                {
                    Bottle *be = MyGMP->beat_clock_port.read(false);
                    if(be!=NULL)
                        {
                            int current_clock= be->get(0).asInt();
                            beat_clock=(current_clock-drum_beat_clock)%MyGMP->sizeScore;		      
                            ACE_OS::printf("beat %d for clock\n", beat_clock);
                        }
                }
            Time::delay(0.025);
	}
 
}

#undef main 

int main(int argc,char **argv) 
{
    Network yarp;
 
    MyGMP = new drum();

    Property prop;
    prop.fromCommand(argc, argv);

    if (!prop.check("config-path"))
        {
            fprintf(stderr, "Please specify --config-path path to config files\n");
            return -1;
        }
    
    sprintf(MyGMP->pathToConfig, "%s", prop.find("config-path").asString().c_str());
    fprintf(stderr, "Using config files from %s\n",MyGMP->pathToConfig);
   
    MyGMP->feedback_file=fopen("feedback_time.dat","w");
 	
    //setting ports
    doConnect(MyGMP);
    
    //get information from files
    getConfig(MyGMP);
    getDrumInfo(MyGMP);

    //run
    Drumming(MyGMP);

    //close     
    ACE_OS::printf("Closing...\n");
    delete MyGMP;

    return 1;
}

 
