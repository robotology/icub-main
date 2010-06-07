
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

#include <stdlib.h>
 
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
				printf("Config file \"%s\" not found for part %s\n", partName, MyGMP->parts[i].c_str());
				Network::fini();
				exit(-1);
			}
			MyGMP->controlled_dofs[i] = partConf.find("nbDOFs").asInt();
			
			printf("Part %s: %d dofs \n",
				MyGMP->parts[i].c_str(),
				MyGMP->controlled_dofs[i]);
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
                    printf("Using part %s\n", MyGMP->parts[i].c_str());
	  
                    //opening ports and connecting with the DrumGenerator modules
                    MyGMP->openPort(i,MyGMP->check_port, "check_motion",0,1);  // IN receives beat 

                    //opening ports for the connection with the guiDemo
                    MyGMP->openPort(i,MyGMP->score_port, "score", 0,0); //IN receives score
                    MyGMP->openPort(i,MyGMP->phase_port, "phase", 0,0); //IN receives phase
                }
           
            else
                {
                    MyGMP->param_port[i].close();
                    printf("Not using %s\n", MyGMP->parts[i].c_str());
                }
        }

    //connecting with the clock
    MyGMP->clock_port.open("/clock/parameters/out");
    bool okClock=Network::connect("/clock/parameters/out","/clock/parameters/in", "tcp");
    if(!okClock)printf("troubles connecting with the clock\n");
    MyGMP->beat_clock_port.open("/clock/check_motion/in");
    bool okClock2=Network::connect("/clock/check_motion/out","/clock/check_motion/in", "tcp");
    if(!okClock2)printf("troubles connecting with the clock\n");

    //opening port to get frequency and couplings from the DrumManager
    MyGMP->interactive_port.open("/interactive/in");
 
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
				printf("Closing command received from the gui...\n");
				break; //if negative frequency is sent by the gui, the manager closes.
			}
		}
      

        //******Getting scores*****************
        for(int i=0; i<MyGMP->nbparts; i++)
        {
        	if(MyGMP->ok[i]) //if part is active
           	{
            	Bottle *newScore = MyGMP->score_port[i].read(false);                             
                if(newScore!=NULL) 
			    {				
					if(MyGMP->beat[i]==-1) //first time: get beat from the generator
				  	{
				    	Bottle *time_init=MyGMP->check_port[i].read();
				    	MyGMP->beat[i]=0;
				    	if(time_init!=NULL) MyGMP->drum_beat[i] = time_init->get(0).asInt();
				  	}
					printf("Score for part %s: ",MyGMP->parts[i].c_str());
					
					for (int j=0; j<MyGMP->sizeScore; j++) //get score
				  	{
				    	int indiceScore = (j+MyGMP->beat[i])%MyGMP->sizeScore;
				    	Score[i][indiceScore]= newScore->get(j).asInt();
				    	printf("%d ",Score[i][indiceScore], newScore->get(j).asInt());
				  	}
			      	printf("\n");
			  	}
	     

			    if(MyGMP->beat[i]!=-1) //only if we have already received a score
			    {
					Bottle *newPhase = MyGMP->phase_port[i].read(false); 
					if(newPhase!=NULL) 
				  	{
				    	for(int j=0; j<MyGMP->sizeScore; j++) //get phase shifts
				      	{
							int indiceScore = (j+MyGMP->beat[i])%MyGMP->sizeScore;
							MyGMP->phase_shift[i][indiceScore]= newPhase->get(j).asDouble();
				      	}
				    
				    	printf("Phase shifts for part %s: ",MyGMP->parts[i].c_str());
				    	for (int j=0; j<MyGMP->sizeScore; j++) 
				     	{
							printf("%f ", MyGMP->phase_shift[i][j]);
				    	}					
				    	printf("\n");
				  	}
			      }
			}}                  
   
	  
	  //SENDING SCORES
   
        
        	for(int i=0; i<MyGMP->nbparts; i++)
            {	   
                if(MyGMP->ok[i] && MyGMP->beat[i]!=-1) //if part is active and at least one score received
                {
					Bottle *answer = MyGMP->check_port[i].read(false); 
					if(answer!=NULL)//we wait for new beat to send a command
					{
							MyGMP->current_beat[i] = answer->get(0).asInt(); //beat of the generator
							//beat of the generator in the manager time reference (modulo the Size Score):
							MyGMP->beat[i]=(MyGMP->current_beat[i]-MyGMP->drum_beat[i])%MyGMP->sizeScore;								  
							printf("beat %d for part %s\n",MyGMP->beat[i], MyGMP->parts[i].c_str());
							
							//we send the parameters
							MyGMP->sendNewParameterSet(
											Score[i][MyGMP->beat[i]],
											MyGMP->phase_shift[i][MyGMP->beat[i]],
											MyGMP->rhythmParam[MyGMP->beat[i]], i);
											
							printf("Parameters sent %d, %4.2f, %4.2f\n",
											Score[i][MyGMP->beat[i]],
											MyGMP->phase_shift[i][MyGMP->beat[i]],
											MyGMP->rhythmParam[MyGMP->beat[i]]);							
					}
	  
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
                            printf("beat %d for clock\n", beat_clock);
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

    if(prop.check("file"))
	{
		sprintf(MyGMP->pathToConfig, "%s", prop.find("config-path").asString().c_str());
	}
    else
    {
    	const char *cubPath;
    	cubPath = getenv("ICUB_DIR");
    	if(cubPath == NULL) {
    		printf("generatorThread::init>> ERROR getting the environment variable ICUB_DIR, exiting\n");
    		return false;
    	}
    	string cubPathStr(cubPath);
    	sprintf(MyGMP->pathToConfig, "%s/app/drummingEpfl/conf", cubPathStr.c_str());
	}
    
    fprintf(stderr, "Using config files from %s\n",MyGMP->pathToConfig);

    //setting ports
    cout << "\n starting configuration... \n";
    doConnect(MyGMP);
    cout << "\n connected...\n";
    
    //get information from files
    getConfig(MyGMP);
    cout << "configured...\n";

    //run
    Drumming(MyGMP);
	
    //close     
    printf("Closing...\n");
    delete MyGMP;

    return 1;
}

 
