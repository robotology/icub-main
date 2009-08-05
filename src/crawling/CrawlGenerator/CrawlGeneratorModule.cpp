// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CrawlGeneratorModule.h"
#include <ace/OS.h>

#define DEBUG 1


//#include <ppEventDebugger.h>

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
                if(states[i] < joint_limit_down[i] + LIMIT_TOL)
                    {
                        ACE_OS::printf("warning exceeded pos %f to joint %d, cutting to %f\n",
                                       states[i],i,joint_limit_down[i]+LIMIT_TOL);
                        states[i] = joint_limit_down[i] + LIMIT_TOL;
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
        }

    vcFastCommand_port.write(true);

    return true;
}

//read the parameters coming from the manager and update the cpg myCpg

void generatorThread::getParameters()
{
    Bottle *command = parameters_port.read(false);
    if(command!=NULL)
        if(command->size() >=2*nbDOFs+2)
            {
                for (int i=0; i<2*nbDOFs; i++)
                    {
                        myCpg->parameters[i]= command->get(i).asDouble();
                        fprintf(parameters_file,"%f \t", myCpg->parameters[i]);
                    }

                double freq = command->get(2*nbDOFs).asDouble();

                if(freq < MAX_FREQUENCY)
                    myCpg->om_stance = freq*2*3.1415;
                else
                    ACE_OS::printf("trying to set a too high st freq %f\n",freq);

                freq = command->get(2*nbDOFs+1).asDouble();
                if(freq < MAX_FREQUENCY)
                    myCpg->om_swing = freq*2*3.1415;
                else
                    ACE_OS::printf("trying to set a too high sw freq %f\n",freq);


                fprintf(parameters_file,"%f %f ",myCpg->om_stance,myCpg->om_swing);
                fprintf(parameters_file,"%f \n",Time::now()/*-original_time*/);
                fflush(parameters_file);
                current_action = true;
            }
        else
            ACE_OS::printf("warning, manager sending crappy values\n");
}


bool generatorThread::getOtherLimbStatus()
{
    for(int i=0;i<3;i++)
        if(other_part_connected[i])
            {
                Bottle *btl = other_part_port[i].read(false);
                if(btl!=NULL)
                    {
                        y_cpgs[nbDOFs*4+2*i] = btl->get(0).asDouble();
                        y_cpgs[nbDOFs*4+2*i+1] = btl->get(1).asDouble();
                    }
            }

    Bottle &bot = current_state_port.prepare();
    bot.clear();
    bot.addDouble(y_cpgs[2]);
    bot.addDouble(y_cpgs[3]);
    current_state_port.write();
	return true;
}

void generatorThread::connectToOtherLimbs()
{
    for(int i=0;i<3;i++)
        {
            if(!other_part_connected[i])
                {
                    char tmp1[255];
                    sprintf(tmp1,"/%s/cpg_status/out",other_part_name[i].c_str());
                    Contact query = Network::queryName(tmp1);
                    if(query.isValid())
                        {
                            char tmp2[255];
                            sprintf(tmp2,"/%s/cpg_status/%s/in",partName.c_str(),other_part_name[i].c_str());
                            bool ok = other_part_port[i].open(tmp2);
                            if(ok)
                                {
                                    ok = Network::connect(tmp1,tmp2);
                                    if(!ok)
                                        ACE_OS::printf("error in connecting %s\n",tmp2);
                                    else
                                        {
                                            other_part_connected[i] = true;
                                            myCpg->external_coupling[i] = myCpg->next_external_coupling[i];
                                            myCpg->printInternalVariables();
                                        }
                                }
                            else
                                {
                                    ACE_OS::printf("error in opening %s\n",tmp2);
                                }
                        }
                }
        }
    Time::delay(0.5);
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

    //we get the states of the other limbs and send our current status
    if(current_action)
        getOtherLimbStatus();
    else
        connectToOtherLimbs();

    if(myCpg->om_stance<0)
        {
            ACE_OS::printf("Task is finished\n");
            this->stop();
            return;
        }

 
    //integrate the system
    int inner_steps = (int)((period+time_residue)/myCpg->get_dt());

    for(int j=0; j<inner_steps; j++)
        myCpg->integrate_step(y_cpgs,states);

    if(partName=="left_arm" || partName=="right_arm")
        {
            states[3] = 10.0 + 50.0*exp(-4.0*(y_cpgs[3]+1.0)*(y_cpgs[3]+1.0));
        }


    ///we update of the previous states

    for(int i=0; i<nbDOFs; i++)
        {
            previous_states[i]=states[i];
            fprintf(target_file,"%f \t", states[i]);
        }

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

#endif

    delete ddPart;

    delete[] y_cpgs;
    delete[] states;
    delete[] previous_states;
    delete[] encoders;

    delete[] joint_limit_up;
    delete[] joint_limit_down;
    delete[] jointMapping;
    delete[] initPos;

    delete myCpg;

    fclose(target_file);
    fclose(parameters_file);
    fclose(encoder_file);
    fclose(feedback_file);

    fprintf(stderr, "%s thread released\n", partName.c_str());
}

bool generatorThread::init(Searchable &s)
{
    Property options(s.toString());
    Time::turboBoost();

    current_action = false;

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

    ////////////////////////////////////////////////////////////////
    //////////Opening port to send current CPG state ///////////////
    ////////////////////////////////////////////////////////////////

    sprintf(tmp1,"/%s/cpg_status/out",partName.c_str());
    ok=current_state_port.open(tmp1);
  
    if(!ok)
        {
            ACE_OS::printf("Warning cannot open current cpg status port, part %s\n",partName.c_str());
        }

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

    ///we create the CPG
    myCpg = new cpgs(nbDOFs);
    
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
                    myCpg->parameters[2*i]=initPos[i]/180.0*3.1415;
                    //myCpg->next_parameters[2*i]=initPos[i]/180.0*3.1415;
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

                            myCpg->ampl[i] =ampl;
                        }
                    else
                        myCpg->ampl[i] = ampl;
                }
        }
    else
        {
            ACE_OS::printf("Warning amplitude vector not defined, setting to default\n");
            for(int i=0;i<nbDOFs;i++)
                myCpg->ampl[i]=0.1;
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
                    for(int i=0;i<nbDOFs;i++)
                        {
                            double gain = botG.get(i+1).asDouble();

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

    y_cpgs = new double[nbDOFs*4+2*3+1];
    states = new double[nbDOFs];
    previous_states = new double[nbDOFs];
    encoders = new double[nbDOFs];

    for(int i=0;i<4*nbDOFs+7;i++)
        y_cpgs[i] = 0.0;

#if DEBUG
    for(int i=0; i< nbDOFs; i++)
        {
            states[i]=0.0;
            previous_states[i]=0.0;

            y_cpgs[4*i]=0.0/180.0*3.1415/myCpg->ampl[i];

            y_cpgs[4*i+1]=0.0;

            y_cpgs[4*i+2]=0.0/180.0*3.1415/myCpg->ampl[i];

            y_cpgs[4*i+3]=0.01;
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

            y_cpgs[4*i]=encoders[i]/180.0*3.1415/myCpg->ampl[i];

            y_cpgs[4*i+1]=0.0;

            y_cpgs[4*i+2]=encoders[i]/180.0*3.1415/myCpg->ampl[i];

            y_cpgs[4*i+3]=0.01;
        }

#endif

    //the go command
    y_cpgs[nbDOFs*4+6]=0.0;
    

    ////we get the frequencies
    if(options.check("omStance"))
        {
            Bottle& tmpBot = options.findGroup("omStance");
            if(tmpBot.size()==2)
                {
                    myCpg->om_stance = tmpBot.get(1).asDouble();
                }
            else
                {
                    ACE_OS::printf("Please specify omStance for part %s\n",partName.c_str());
                    return false;
                }
        }
    else
        {
            ACE_OS::printf("Please specify omStance for part %s\n",partName.c_str());
            return false;
        }
    if(options.check("omSwing"))
        {
            Bottle& tmpBot = options.findGroup("omSwing");
            if(tmpBot.size()==2)
                {
                    myCpg->om_swing = tmpBot.get(1).asDouble();
                }
            else
                {
                    ACE_OS::printf("Please specify omSwing for part %s\n",partName.c_str());
                    return false;
                }
        }
    else
        {
            ACE_OS::printf("Please specify omSwing for part %s\n",partName.c_str());
            return false;
        }

    //we get tje external couplings
    if(options.check("External_coupling"))
        {
            Bottle& tmpBot = options.findGroup("External_coupling");
            Bottle& tmpBot2 = tmpBot.findGroup("parts");
            Bottle& tmpBot3 = tmpBot.findGroup("coupling");
            if(tmpBot2.isNull() || tmpBot2.size()<4 || tmpBot3.isNull() || tmpBot3.size()<4)
                {
                    ACE_OS::printf("Please specify external coupling for part %s\n",partName.c_str());
            return false;
                }
            for(int i=0;i<3;i++)
                {
                    other_part_connected[i] = false;
                    other_part_name[i] = tmpBot2.get(i+1).asString();
                    myCpg->next_external_coupling[i] = tmpBot3.get(i+1).asDouble();
                    myCpg->external_coupling[i] = 0.0;
                }
        }
    else
        {
            ACE_OS::printf("Please specify external coupling for part %s\n",partName.c_str());
            return false;
        }

    //we get the internal coupling parameters

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
                        for(int j=0;j<nbDOFs;j++)
                            myCpg->epsilon[i][j] = tmpBot2.get(j+1).asDouble();



                    ///coupling phase diff

                    Bottle& tmpBot3 = tmpBot.findGroup("phase");

                    if(tmpBot3.isNull() || tmpBot3.size()<nbDOFs+2)
                        ACE_OS::printf("No coupl. phase info for joint %d, part %s\n",i,partName.c_str());
                    else
                        for(int j=0;j<nbDOFs;j++)
                            {
                                myCpg->theta[i][j] = tmpBot3.get(j+1).asDouble();
                                myCpg->theta_init[i][j] = myCpg->theta[i][j];
                            }
                }
        }

    //myCpg->next_theta = myCpg->theta[0][0];

    //we print the internal variables of the cpg
    myCpg->printInternalVariables();

    ////
    beat = 0;

    ////////set original time, then the module will start
    theoretical_time = 0.0;

    lastBeat_time = Time::now();

    original_time = Time::now();

    return true;
}



////////////DRUM GENERATOR MODULE/////////////////



double CrawlGeneratorModule::getPeriod()

{

    //    ACE_OS::printf("Crawl Generator Module is running\n");

    return 1.0;

}



bool CrawlGeneratorModule::updateModule()
{
    return true;

}



bool CrawlGeneratorModule::close()
{
    fprintf(stderr, "%s module closing\n", partName.c_str());
    theThread->stop();
    delete theThread;
    fprintf(stderr, "%s module closed\n", partName.c_str());
    return true;
}


bool CrawlGeneratorModule::open(yarp::os::Searchable &s)
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

