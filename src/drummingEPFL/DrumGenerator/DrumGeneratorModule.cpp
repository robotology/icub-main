// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "DrumGeneratorModule.h"
#include <ace/OS.h>

#define DEBUG 0


generatorThread::generatorThread(int period) : RateThread(period)
{
    this->period = ((double)period)/1000.0;
}

generatorThread::~generatorThread()
{}

void generatorThread::checkJointLimits()
{
    for(int i=0;i<nbDOFs;i++)
        {
            if(states[i] > joint_limit_up[i] - LIMIT_TOL)
            {
                ACE_OS::printf("warning exceeded pos %f to joint %d, cutting to %f\n",
                               states[i],i,joint_limit_up[i]-LIMIT_TOL);
                states[i] = joint_limit_up[i] - LIMIT_TOL;
            }
            else
            {
                if(states[i] < joint_limit_down[i] + LIMIT_TOL)
                    {
                        ACE_OS::printf("warning exceeded pos %f to joint %d, cutting to %f\n",
                                       states[i],i,joint_limit_down[i]+LIMIT_TOL);
                        states[i] = joint_limit_down[i] + LIMIT_TOL;
                    }
			}
        }
}

bool generatorThread::getEncoders()
{
    for(int i=0;i<nbDOFs;i++)
        {
            if(!PartEncoders->getEncoder(jointMapping[i],&(encoders[i])))
                return false;

            fprintf(encoder_file,"%f ",encoders[i]);
        }
    fprintf(encoder_file,"%f ", Time::now());
    fprintf(encoder_file,"\n");
    

    return true;
}


bool generatorThread::sendFastJointCommand()   
{
    checkJointLimits();
    Bottle& cmd = vcFastCommand_port.prepare();

    cmd.clear();

    for(int i=0;i<nbDOFs;i++)
	{
		cmd.addInt(jointMapping[i]);
		cmd.addDouble(states[i]);
		cmd.addInt(jointMapping[i] + VELOCITY_INDEX_OFFSET);
		cmd.addDouble(dstates[i]);
	}

    vcFastCommand_port.write(true);

    return true;
}

//read the parameters coming from the manager and update the cpg myManager

void generatorThread::getParameters()
{
    Bottle *command = parameters_port.read(false);
    if(command!=NULL)
        if(command->size() >=2*nbDOFs+2)
            {
                //ACE_OS::printf("Receiving parameters for part %s:\n ", partName.c_str());
                for (int i=0; i<2*nbDOFs; i++)
                {
                    myManager->next_parameters[i]= command->get(i).asDouble();
                    fprintf(parameters_file,"%f \t", myManager->next_parameters[i]);
                        //ACE_OS::printf("%f \t", myManager->next_parameters[i]);
                }
                //ACE_OS::printf("\n");
                
               if(myManager->parameters[1]!=myManager->next_parameters[1])
                {
                    y_cpgs[nbDOFs*4+2]=0.0; //reset go command
                   y_cpgs[2*(nbDOFs*4+3)-1]=0.0;//go command of the observer
                }

                double freq = command->get(2*nbDOFs).asDouble();

                if(freq < MAX_FREQUENCY)
                    myManager->next_nu = freq;
                else
                    ACE_OS::printf("trying to set a too high freq %f\n",freq);

                double phase = command->get(2*nbDOFs+1).asDouble();

                if(phase != myManager->next_theta)
                    lastBeat_time = lastBeat_time+(myManager->next_theta-phase)/(2*3.14*myManager->next_nu);

                myManager->next_theta=phase;       
                
                //ACE_OS::printf("freq and phase: %4.2f %4.2f\n", myManager->next_nu, phase);       

                fprintf(parameters_file,"%f %f ",myManager->next_nu,phase);
                fprintf(parameters_file,"%f \n",Time::now()/*-original_time*/);
            
            
            }
        else
        {
            ACE_OS::printf("warning, manager sending crappy values\n");
        }
}

void generatorThread::getHit()
{	
	//	if(myManager->drumHit==0)
	//	{
	//		if(partName=="left_arm" || partName=="right_arm")
	//		{
	//			double force_threshold;
	//			if(partName=="left_arm")
	//			{
	//				force_threshold=LEFT_THRESHOLD;
	//			}
	//			else
	//			{
	//				force_threshold=RIGHT_THRESHOLD;
	//			}
	//			Bottle *force = ForceSensor_port.read(false);
	//			if(force!=NULL)
	//			{
	//				double force_value = 0;
	//				if(moving_average.size()==MOV_AV)
	//				{
	//					moving_average.pop_front();
	//				}
	//				moving_average.push_back(force->get(4).asDouble());
	//				for(list<double>::iterator it=moving_average.begin(); it!=moving_average.end(); it++)
	//				{
	//					force_value += *it;
	//				}
					
	//				force_value=force_value/moving_average.size();
				
	//				if(force_value < force_threshold)
	//				{
	//					myManager->drumHit=1;
	//					//myManager->stuckCounter=0;                                          
	//					fprintf(feedback_file, "%f \n", Time::now()/*-original_time*/);
	//					ACE_OS::printf("FEEDBACK ENABLED FOR PART %s, sensor value %f\n", partName.c_str(), force_value); 
	//				}
	//			}
	//		}
	//	}
		
		if(partName=="left_leg" || partName=="right_leg")
	{
    Bottle *hit =sound_port.read(false);
    if(hit!=NULL)
	{
		int check=hit->get(3).asInt();
		int NoteID;

		if(check>0)
		{
			NoteID=hit->get(2).asInt();
			if (NoteID>0)
			{
				for(int i=0; i<3;i++)
				{
					if(NoteID == notes[i])
					{
						myManager->drumHit=1;
						myManager->stuckCounter=0;                                           
						fprintf(feedback_file, "%f \n", Time::now()/*-original_time*/);
						ACE_OS::printf("FEEDBACK ENABLED FOR PART %s\n", partName.c_str()); 
					}
				}
			}
		}          
		
	}
	}        
}

bool generatorThread::getExternalClock()
{

    Bottle *btl = clock_port.read(false);
    if(btl==NULL)
        {
            static int count1=0;
            count1++;
            if(count1%10==0)
                 ACE_OS::printf("cannot get clock\n");

            return false;
        }
    else
        {
            static int count2=0;
            count2++;

            // if(count2%10==0)
            //   fprintf(stderr, "Got clock\n");
        }

    double x = btl->get(0).asDouble();
    double y = btl->get(1).asDouble();

    y_cpgs[0] = x;
    y_cpgs[1] = y;

    y_cpgs[nbDOFs*4+3]=x;
    y_cpgs[nbDOFs*4+4]=y;	
  
	return true;
}

void generatorThread::run()
{
    //time0=time0+command_step;
    static double time_now=Time::now();

    if( Time::now() - time_now >0.1)
        fprintf(stderr,"Warning time too big\n");

    time_now = Time::now();

    double time_residue = time_now - original_time - theoretical_time;

    theoretical_time += period + time_residue;

#if !DEBUG

    ///we get encoders

    if(!getEncoders())
    {
            ACE_OS::printf("Error getting encoders positions\n");
            this->stop();
            return;
    }

#endif

    //we get potential new parameters

    getParameters();
    getHit();

    //we get the clock

    getExternalClock();

    if(myManager->nu<0)
    {
        ACE_OS::printf("Task is finished\n");
        this->stop();
        return;
    }

 
    //integrate the system

    int inner_steps = (int)((period+time_residue)/myManager->get_dt());

    for(int j=0; j<inner_steps; j++)
    {
        myManager->integrate_step(y_cpgs,states);
    }
 
    ////calculate the current beat

    if(getNewBeat())
    {

        //ACE_OS::printf("current beat: %d\n", beat);
        beat++;
        lastBeat_time = time_now;
        y_cpgs[nbDOFs*4+2]=0.0; //reset go command
        y_cpgs[2*(nbDOFs*4+3)-1]=0.0;//go command of the observer

        ///////send status to manager

        Bottle& output = check_motion_port.prepare();
        output.clear();
        output.addInt(beat);//beat=0 score not started yet
        check_motion_port.write();  
        
    }
        ///update parameters

        for (int i=0; i<2*nbDOFs; i++)
        {
            myManager->parameters[i] = myManager->next_parameters[i];
        }

        myManager->nu = myManager->next_nu;
        //fprintf(stderr, "%lf\n", myManager->nu);
      
        for(int i=0; i<nbDOFs; i++)
        {
            myManager->theta[i][0] = myManager->theta_init[i][0]+myManager->next_theta;
        }
        
    //}   


    ///we update of the previous states

    for(int i=0; i<nbDOFs; i++)
        {
			dstates[i] = (states[i] - previous_states[i]) / (period+time_residue);
            previous_states[i]=states[i];
            fprintf(target_file,"%f \t", states[i]);
        }
        
    fprintf(target_file,"%f %f \t", y_cpgs[0], y_cpgs[1]);

    ///save time stamp

    fprintf(target_file,"%f \n",Time::now()/*-original_time*/);


#if !DEBUG  

    ///////WE SEND THE COMMAND TO THE ROBOT//////////

    if(!sendFastJointCommand())
        {
            ACE_OS::printf("error in joint command, quitting...\n");
            this->stop();
            return;
        }
    /////////////////////////////////////////////////
#endif

  
    double timef=Time::now();
    double d=timef - time_now;

    //ACE_OS::printf("loop time %f\n",d);

}



bool generatorThread::getNewBeat()
{

    double tmp_clock[2];

    tmp_clock[0] = cos(myManager->theta[0][0])*y_cpgs[0] - sin(myManager->theta[0][0])*y_cpgs[1];
    tmp_clock[1] = sin(myManager->theta[0][0])*y_cpgs[0] + cos(myManager->theta[0][0])*y_cpgs[1];

    //check if init quadrant

    if((tmp_clock[0] >0.0) && (tmp_clock[1]>0.0))
    {
        //previous_quadrant[0] = true;
        previous_quadrant[1] = true;
    }


    //check if in previous quadrant y< && x>0

    if((tmp_clock[0] <0.0) && (tmp_clock[1]<0.0))
    //if((tmp_clock[0] <0.0) && (tmp_clock[1]>0.0))
    {
        if(previous_quadrant[1])

            {
                previous_quadrant[0] = true;
                previous_quadrant[1] = false;

            }
    }


    if((tmp_clock[0] >0.0) && (tmp_clock[1]<0.0))
    //if((tmp_clock[0]<0.0) && (tmp_clock[1]<0.0))
    {
        if(previous_quadrant[0])
        {
            previous_quadrant[0] = false;
            return true;
        }
    }
  

    return false;

}



bool generatorThread::threadInit()
{
    fprintf(stderr, "%s thread init\n", partName.c_str());
    // Bottle& cmd = vcControl_port.prepare();
    //  cmd.clear();
    //  cmd.addVocab(Vocab::encode("run"));
    //  vcControl_port.write(true);
    
    return true;
}



void generatorThread::threadRelease()
{
    fprintf(stderr, "%s thread releasing\n", partName.c_str());
    ///we stop the vcControl
    
    fclose(target_file);
    fclose(parameters_file);
    fclose(encoder_file);
    fclose(feedback_file);

#if !DEBUG

    //setting gains to 0

    for(int i=0;i<nbDOFs;i++)
        {

            Bottle& cmd = vcControl_port.prepare();
            cmd.clear();
            cmd.addVocab(Vocab::encode("gain"));
            cmd.addInt(jointMapping[i]);
            cmd.addDouble(0.0);

            vcControl_port.write(true);

            Time::delay(0.1);
        }
        
        
    vcControl_port.close();
    vcFastCommand_port.close();
    ForceSensor_port.close();

#endif

    parameters_port.close(); 
    check_motion_port.close(); 
    sound_port.close();
    clock_port.close();
    
    delete ddPart;

    delete[] y_cpgs;
    delete[] states;
    delete[] dstates;
    delete[] previous_states;
    delete[] encoders;

    delete[] joint_limit_up;
    delete[] joint_limit_down;
    delete[] jointMapping;
    delete[] initPos;

    delete myManager;

    fprintf(stderr, "%s thread released\n", partName.c_str());
}

bool generatorThread::init(Searchable &s)
{
    Property options(s.toString());
    Time::turboBoost();

    ///init period

    //period = 0.05; //in sec


    //////getting part to interface with

    if(options.check("part"))
        {
            partName = options.find("part").asString().c_str();
            ACE_OS::printf("module taking care of part %s\n",partName.c_str());
        }
    else
        {
            ACE_OS::printf("Please specify part to control (e.g. --part head)\n");
            return false;
        }

  
    char tmp1[255],tmp2[255];
    char targetPart[255], paramPart[255], encoderPart[255], feedPart[255];

    sprintf(targetPart, "%s_target_position.dat", partName.c_str());
    sprintf(paramPart, "%s_parameters.dat", partName.c_str()); 
    sprintf(encoderPart, "%s_encoders.dat", partName.c_str());
    sprintf(feedPart, "%s_feedback.dat", partName.c_str());

    target_file = fopen(targetPart, "w");
    parameters_file = fopen(paramPart, "w");  
    encoder_file = fopen(encoderPart, "w");
    feedback_file = fopen(feedPart, "w");


#if !DEBUG

    ////////////////////////////////////////////////////////////////////

    ////////Getting access to the Polydriver of the part////////////////

    ////////////////////////////////////////////////////////////////////

    Property ddOptions;

    ddOptions.put("robot","icub");
    ddOptions.put("device","remote_controlboard");

    sprintf(tmp1,"/%s/enc",partName.c_str());
    sprintf(tmp2,"/icub/%s",partName.c_str());

    ddOptions.put("local",tmp1);
    ddOptions.put("remote",tmp2);

    ddPart = new PolyDriver(ddOptions);

    if(!ddPart->isValid())
        {
            ACE_OS::printf("Device not available. Here are the known devices:\n");
            ACE_OS::printf("%s", Drivers::factory().toString().c_str());
            return false;
        }

    ///encoders interface

    if(!ddPart->view(PartEncoders))
        {
            ACE_OS::printf("Cannot view the encoders interface of %s\n",partName.c_str());
            return false;
        }

    ///////////////////////////////////////////////////////////////////////

    ////////Connection to the velocity control module//////////////////////

    ///////////////////////////////////////////////////////////////////////



    ///normal connection

    sprintf(tmp1,"/%s/vcControl",partName.c_str());
    if(!vcControl_port.open(tmp1))
        {
            ACE_OS::printf("Cannot open vcControl port of %s\n",partName.c_str());
            return false;
        }

    sprintf(tmp2,"/icub/vc/%s/input",partName.c_str());

    if(!Network::connect(tmp1,tmp2))
        {
            ACE_OS::printf("Cannot connect to vc/input port of %s\n",partName.c_str());
            return false;
        }


    ///connection to the thread

    sprintf(tmp1,"/%s/vcFastCommand",partName.c_str());

    if(!vcFastCommand_port.open(tmp1))
        {
            ACE_OS::printf("Cannot open vcFastCommand port of %s\n",partName.c_str());
            return false;
        }

    sprintf(tmp2,"/icub/vc/%s/fastCommand",partName.c_str());

  
    if(!Network::connect(tmp1,tmp2,"udp"))
    {
        ACE_OS::printf("Cannot connect to vc/fastCommand port of %s\n",partName.c_str());
        return false;
    }

//    if(partName=="left_arm")
//    {    
//		sprintf(tmp1,"/%s/forcesensor/in",partName.c_str());

//		if(!ForceSensor_port.open(tmp1))
//        {
//            ACE_OS::printf("Cannot open ForceSensor port of %s\n",partName.c_str());
//            return false;
//        }

//		if(!Network::connect("/icub/leftarm/analog:o",tmp1,"udp"))
//        {
//            ACE_OS::printf("Cannot connect to force sensor port of %s\n",partName.c_str());
//            return false;
//        }
//	}
	
//	if(partName=="right_arm")
//    {    
//		sprintf(tmp1,"/%s/forcesensor/in",partName.c_str());

//		if(!ForceSensor_port.open(tmp1))
//        {
//            ACE_OS::printf("Cannot open ForceSensor port of %s\n",partName.c_str());
//            return false;
//        }

//		if(!Network::connect("/icub/rightarm/analog:o",tmp1,"udp"))
//        {
//            ACE_OS::printf("Cannot connect to force sensor port of %s\n",partName.c_str());
//            return false;
//        }
//	}

#endif


    /////////////////////////////////////////////////////////////////////////
    //////////Opening of ports to receive commands from the manager//////////
    /////////////////////////////////////////////////////////////////////////

    //////////opening the parameter port to receive input

    bool ok;

    sprintf(tmp1,"/%s/parameters/in",partName.c_str());
    ok= parameters_port.open(tmp1); 

    if(!ok)
        {
            ACE_OS::printf("Failed to open port to get parameters of part %s \n",partName.c_str());
            return false;
        }

    ////////////opening check motion port to send status

    sprintf(tmp2,"/%s/check_motion/out",partName.c_str());
    ok= check_motion_port.open(tmp2); 

    if(!ok)
        {
            ACE_OS::printf("Failed to open port to check motion of part %s \n",partName.c_str());
            return false;
        }


    ////////////////
    /////////Opening ports to have sound feedback
    //////////////////
  
    sprintf(tmp2,"/%s/sound/in", partName.c_str());
    ok= sound_port.open(tmp2);
    sound_port.setStrict(true);
    bool feed_ok = Network::connect("/midiDrum/server/out", tmp2, "tcp");
    if(!feed_ok){ACE_OS::printf("Feedback OFF\n");}
    else{ACE_OS::printf("Feedback ON\n");}


    ////////////////////////////////////////////////////////////////
    //////////Opening port to get external clock command////////////
    ////////////////////////////////////////////////////////////////

    sprintf(tmp1,"/%s/clock/in",partName.c_str());
    sprintf(tmp2,"/clock/out");

    ok=clock_port.open(tmp1);
    ok&= Network::connect(tmp2,tmp1,"udp");

    if(!ok)
    {
        ACE_OS::printf("Warning cannot connect to external clock part %s\n",partName.c_str());
    }

    external_clock = ok;

    ////////////////////////////////////////////////////////////////
    ///////////////getting internal configuration params////////////
    ////////////////////////////////////////////////////////////////

  
    ////getting the nbDOFs

    if(options.check("nbDOFs"))
        nbDOFs = options.find("nbDOFs").asInt();

    else
        {
            ACE_OS::printf("Please specify the nbDOFs of part%s\n",partName.c_str());
            return false;
        }

    ///we create the manager
    myManager = new cpg_manager(nbDOFs);
    
    myManager->part_name = partName;


    if(!options.check("Notes"))
        {
            ACE_OS::printf("Drums ID not found for part %s\n", partName.c_str());
           Network::fini();
            exit(-1);
        }
    
    else
        {
            for(int i=0; i<3; i++)
                {
                    notes[i]=0;
                }
            
            ACE_OS::printf("Notes: ");
            Bottle& Target = options.findGroup("Notes");
            
            for (int k=1; k<Target.size(); k++) {
                printf("%d ", Target.get(k).asInt());
                notes[k-1]=Target.get(k).asInt();
            }
            printf("\n");
            Target.clear();
        }

    /// getting the initial position
    if(options.check("init_pos"))
        {

            Bottle& pos = options.findGroup("init_pos");

            if(pos.size()!=nbDOFs+1)

                {

                    ACE_OS::printf("Incorrect specification of the initial positions of part%s\n",partName.c_str());

                    return false;

                }

            initPos = new double[nbDOFs];

            for(int i=0; i<nbDOFs; i++)
                {
                    initPos[i]=pos.get(i+1).asDouble();
                    myManager->parameters[2*i+1]=initPos[i]/180.0*3.1415;
                    myManager->next_parameters[2*i+1]=initPos[i]/180.0*3.1415;
                    ACE_OS::printf("setting init pos %f joint %d\n",initPos[i],i);
                }
        }

    else
        ACE_OS::printf("Warning no initial positions found\n");


    ///getting the amplitude of oscillations (Warning these should never be close to 0!!
    //otherwise the integration of the ODEs will diverge!

    if(options.check("amplitudes"))

        {

            Bottle& amp=options.findGroup("amplitudes");

            if(amp.size()!=nbDOFs+1)

                {

                    ACE_OS::printf("Incorrect nb of amplitudes part %s\n",partName.c_str());

                    return false;

                }

            for(int i=0;i<nbDOFs;i++)

                {

                    double ampl = amp.get(i+1).asDouble();

                    if((fabs(ampl)<0.1) || (fabs(ampl) > 1.0))

                        {

                            ACE_OS::printf("warning ampl of joint %d exceeds limits\n",i);

                            if (ampl>0.0)

                                ampl=0.1;

                            else

                                ampl=-0.1;

                            myManager->ampl[i] =ampl;

                        }

                    else

                        myManager->ampl[i] = ampl;

                }

        }

    else
        {

            ACE_OS::printf("Warning amplitude vector not defined, setting to default\n");

            for(int i=0;i<nbDOFs;i++)

                myManager->ampl[i]=0.1;

        }
 

    ///getting the joint mapping

    if(options.check("joint_mapping"))

        {

            Bottle& jm = options.findGroup("joint_mapping");

            if(jm.size()!=nbDOFs+1)

                {

                    ACE_OS::printf("Incorrect nb of mapped joints of part%s\n",partName.c_str());

                    return false; 

                }

            jointMapping = new int[nbDOFs];

            for(int i=0;i<nbDOFs;i++)

                {

                    jointMapping[i] = jm.get(i+1).asInt();

                    ACE_OS::printf("mapping state %d with joint %d\n",i,jointMapping[i]);

                }

        }

    else

        {

            ACE_OS::printf("Please specify the joint mapping of part%s\n",partName.c_str());

            return false;

        }


#if !DEBUG

    //reading the max velocity in conf file

    if(options.check("maxVelocity"))

        {

            Bottle& mv = options.findGroup("maxVelocity");

            if(mv.size()!=nbDOFs+1)

                ACE_OS::printf("wrong number of max velocity\n");

            else

                {

                    for(int i=0;i<nbDOFs;i++)

                        {

                            double vel = mv.get(i+1).asDouble();

                            Bottle& cmd = vcControl_port.prepare();

                            cmd.clear();

                            cmd.addVocab(Vocab::encode("svel"));

                            cmd.addInt(jointMapping[i]);

                            cmd.addDouble(vel);

                            vcControl_port.write(true);

                            Time::delay(0.1);

                        }

                }

        }

    else

        ACE_OS::printf("no max velocity defined, using default\n");





    ///reading the Kp gains in the conf file

    if(options.check("controlGains"))

        {

            Bottle& botG = options.findGroup("controlGains");

            if(botG.size()!=nbDOFs+1)

                ACE_OS::printf("wrong number of gains\n");

            else

                {
                    ACE_OS::printf("\nSetting the gains: \t");

                    for(int i=0;i<nbDOFs;i++)

                        {

                            double gain = botG.get(i+1).asDouble();

                            ACE_OS::printf("joint %d, gain = %f \n", i, gain);
                            Bottle& cmd = vcControl_port.prepare();

                            cmd.clear();

                            cmd.addVocab(Vocab::encode("gain"));

                            cmd.addInt(jointMapping[i]);

                            cmd.addDouble(gain);

                            vcControl_port.write(true);

                            Time::delay(0.1);

                        }

                }

        }

    else

        ACE_OS::printf("no gains defined, using 0\n");

  

#endif

  

    ///getting the joint limit

    //up

    if(options.check("joint_limit_up"))

        {

            Bottle& jl = options.findGroup("joint_limit_up");

            if(jl.size()!=nbDOFs+1)

                {

                    ACE_OS::printf("Incorrect nb of joint limit up, part %s\n",partName.c_str());

                    return false;

                }

            joint_limit_up = new double[nbDOFs];

            for(int i=0;i<nbDOFs;i++)

                {

                    joint_limit_up[i] = jl.get(i+1).asDouble();

                    ACE_OS::printf("upper limit %f, joint %d\n",joint_limit_up[i],i);

                }

        }

    else

        {

            ACE_OS::printf("Please specify upper joint limit, part %s\n",partName.c_str());

            return false;

        }

    //down

    if(options.check("joint_limit_down"))

        {

            Bottle& jl = options.findGroup("joint_limit_down");

            if(jl.size()!=nbDOFs+1)

                {

                    ACE_OS::printf("Incorrect nb of joint limit down, part %s\n",partName.c_str());

                    return false;

                }

            joint_limit_down = new double[nbDOFs];

            for(int i=0;i<nbDOFs;i++)

                {

                    joint_limit_down[i] = jl.get(i+1).asDouble();

                    ACE_OS::printf("down limit %f, joint %d\n",joint_limit_down[i],i);

                }

        }

    else

        {

            ACE_OS::printf("Please specify lower joint limit, part %s\n",partName.c_str());
            return false;

        }



    ///we create the vectors

    y_cpgs = new double[2*(nbDOFs*4+3)];
    states = new double[nbDOFs];
    dstates = new double[nbDOFs];
    previous_states = new double[nbDOFs];
    encoders = new double[nbDOFs];



#if DEBUG

    for(int i=0; i< nbDOFs; i++)

        {

            states[i]=0.0;

            previous_states[i]=0.0;

            y_cpgs[4*i+2]=0.0/180.0*3.1415/myManager->ampl[i];

            y_cpgs[4*i+3]=0.0;

            y_cpgs[4*i+4]=0.0/180.0*3.1415/myManager->ampl[i];

            y_cpgs[4*i+5]=0.0;


            y_cpgs[4*i+2+nbDOFs*4+3]=0.0/180.0*3.1415/myManager->ampl[i];

            y_cpgs[4*i+3+nbDOFs*4+3]=0.0;

            y_cpgs[4*i+4+nbDOFs*4+3]=0.0/180.0*3.1415/myManager->ampl[i];

            y_cpgs[4*i+5+nbDOFs*4+3]=0.0;

        }

#else

    if(!getEncoders())

        {

            ACE_OS::printf("error getting encoders, part %s\n",partName.c_str());

            return false;

        }

  

    for(int i=0; i< nbDOFs; i++)

        {

            states[i]=encoders[i];

            previous_states[i]=encoders[i];

            y_cpgs[4*i+2+nbDOFs*4+3]=encoders[i]/180.0*3.1415/myManager->ampl[i];

            y_cpgs[4*i+3+nbDOFs*4+3]=0.0;

            y_cpgs[4*i+4+nbDOFs*4+3]=encoders[i]/180.0*3.1415/myManager->ampl[i];

            y_cpgs[4*i+5+nbDOFs*4+3]=0.0;

            y_cpgs[4*i+2]=encoders[i]/180.0*3.1415/myManager->ampl[i];

            y_cpgs[4*i+3]=0.0;

            y_cpgs[4*i+4]=encoders[i]/180.0*3.1415/myManager->ampl[i];

            y_cpgs[4*i+5]=0.0;


        }

#endif

    //the clock

    y_cpgs[0]=0.0;

    y_cpgs[1]=1.0;

    y_cpgs[nbDOFs*4+3]=0.0;

    y_cpgs[nbDOFs*4+4]=1.0;
    
    //the go command

    y_cpgs[nbDOFs*4+3-1]=0.0;

    y_cpgs[2*(nbDOFs*4+3)-1]=0.0;

 

    //we get the coupling parameters

    for(int i=0;i<nbDOFs;i++)
        {

            sprintf(tmp1,"Joint%d",i);

            Bottle& tmpBot = options.findGroup(tmp1);

            if(tmpBot.isNull())

                ACE_OS::printf("No coupling info for joint %d, part %s\n",i,partName.c_str());

            else

                {

                    ///coupling strength

                    Bottle& tmpBot2 = tmpBot.findGroup("strength");

                    if(tmpBot2.isNull() || tmpBot2.size()<nbDOFs+2)

                        ACE_OS::printf("No coupl. strength info for joint %d, part %s\n",i,partName.c_str());

                    else

                        for(int j=0;j<nbDOFs+1;j++)

                            myManager->epsilon[i][j] = tmpBot2.get(j+1).asDouble();



                    ///coupling phase diff

                    Bottle& tmpBot3 = tmpBot.findGroup("phase");

                    if(tmpBot3.isNull() || tmpBot3.size()<nbDOFs+2)

                        ACE_OS::printf("No coupl. phase info for joint %d, part %s\n",i,partName.c_str());

                    else

                        for(int j=0;j<nbDOFs+1;j++)
                            {
                                myManager->theta[i][j] = tmpBot3.get(j+1).asDouble();
                                myManager->theta_init[i][j] = myManager->theta[i][j];
                            }
    
                }

        }

    myManager->next_theta = myManager->theta[0][0];

    //we print the internal variables of the cpg

    myManager->printInternalVariables();

    ////

    beat = 0;




    ////////set original time, then the module will start

    theoretical_time = 0.0;

    lastBeat_time = Time::now();

    original_time = Time::now();

    return true;

}



////////////DRUM GENERATOR MODULE/////////////////



double DrumGeneratorModule::getPeriod()

{

    //    ACE_OS::printf("Drum Generator Module is running\n");

    return 1.0;

}



bool DrumGeneratorModule::updateModule()
{
    return true;

}



bool DrumGeneratorModule::close()
{
    fprintf(stderr, "%s module closing\n", partName.c_str());
    theThread->stop();
    delete theThread;
    fprintf(stderr, "%s module closed\n", partName.c_str());
    return true;
}


bool DrumGeneratorModule::open(yarp::os::Searchable &s)
{
    Property options(s.toString());
    int period = 50; // in ms

    if(options.check("period"))
        {
            period = options.find("period").asInt();
        }

    if(options.check("part"))
        {
            partName=options.find("part").asString().c_str();
            ACE_OS::printf("module taking care of part %s\n",partName.c_str());
        }

  
    theThread = new generatorThread(period);
    if(!theThread->init(s))
        return false;

    theThread->start();
    return true;

}

