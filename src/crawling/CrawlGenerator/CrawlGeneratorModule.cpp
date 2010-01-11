
#include "CrawlGeneratorModule.h"

#include <ace/OS.h>

#define DEBUG 0

#define SGN(a) (a<0 ? -1 : 1)
#define ABS(a) (a<0 ? -a : a)

//#include <ppEventDebugger.h>

GeneratorThread::GeneratorThread(int period) : RateThread(period)
{
	this->period = ((double)period)/1000.0;
}

GeneratorThread::~GeneratorThread()
{   
}
///this function checks if the joint limits are respected and adapt the commands if needed
bool GeneratorThread::checkJointLimits()
{
	for(int i=0;i<nbDOFs;i++)
	{
		if(states[i]!=states[i])
			{
				ACE_OS::printf("ERROR value for joint %d undefined (NaN)", i);
				return false;
	    }
	    
		if(states[i] > joint_limit_up[i] - LIMIT_TOL)
		{
			//printf("warning exceeded pos %f to joint %d, cutting to %f\n",
			//               states[i],i,joint_limit_up[i]-LIMIT_TOL);
			states[i] = joint_limit_up[i] - LIMIT_TOL;
		}
		else
			if(states[i] < joint_limit_down[i] + LIMIT_TOL)
			{
				//printf("warning exceeded pos %f to joint %d, cutting to %f\n",
				//              states[i],i,joint_limit_down[i]+LIMIT_TOL);
				states[i] = joint_limit_down[i] + LIMIT_TOL;
			}
	}
	return true;
}

///this function sends the speed component of the rhythmic system to the manager
void GeneratorThread::sendStatusForManager()
{
	Bottle& cmd =check_status_port.prepare();

	cmd.clear();
	for(int i=0; i<nbDOFs;i++)
		cmd.addDouble(y_cpgs[4*i+3]);

	check_status_port.write(false);

}

///we get the encoders
bool GeneratorThread::getEncoders()
{
	int nj;
	PartEncoders->getAxes(&nj);
	double *tmp_enc = new double[nj];

	if(PartEncoders->getEncoders(tmp_enc))
	{
		fprintf(encoder_file,"%f ",Time::now());
		for(int i=0;i<nbDOFs;i++)
		{
			encoders[i] = tmp_enc[jointMapping[i]];
			fprintf(encoder_file,"%f ",encoders[i]);
		}
		fprintf(encoder_file,"\n");
		fflush(encoder_file);
	}
	else
	{
		cout << "there was an error getting the encoders" << endl;
		ACE_OS::printf("for part %s", partName.c_str());
		delete[] tmp_enc;
		return false;
	}
	delete[] tmp_enc;

	return true;
}

/// sends the command the velocity controllers
bool GeneratorThread::sendFastJointCommand()   
{
	if(!checkJointLimits())
	{
		return false;
	}
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

///read the parameters coming from the manager and update those of the cpgs class
void GeneratorThread::getParameters()
{
	//cout << "getting param ";
	Bottle *command = parameters_port.read(false);
	if(command!=NULL)
		if(command->size() >=2*nbDOFs+3)
		{
			vector<double> params;
			for (int i=0; i<2*nbDOFs; i++)
				params.push_back(command->get(i).asDouble());

			int discrete=0;
			for(int i=0; i<nbDOFs; i++)
				if(myCpg->parameters[2*i+1]!=params[2*i+1])
				{
					discrete++;
					printf("discrete movement %d\n", discrete);
				}

			for (int i=0; i<2*nbDOFs; i++)
				myCpg->parameters[i] = params[i];

			for (int i=0; i<2*nbDOFs; i++)
				fprintf(parameters_file,"%f \t", myCpg->parameters[i]);

		
			double freq = command->get(2*nbDOFs).asDouble();

			if(freq < MAX_FREQUENCY)
				myCpg->om_stance = freq;
			else
				printf("trying to set a too high st freq %f\n",freq);

			freq = command->get(2*nbDOFs+1).asDouble();
			if(freq < MAX_FREQUENCY)
				myCpg->om_swing = freq;
			else
				printf("trying to set a too high sw freq %f\n",freq);

			double angle = command->get(2*nbDOFs+2).asDouble();
			if(angle < MAX_TURN_ANGLE)
				if(angle > -MAX_TURN_ANGLE)
					myCpg->turnAngle=angle;
				else
					printf("turning angle %f too small\n", angle);
			else
				printf("turning angle %f too big\n", angle);
				
			myCpg->printInternalVariables();	


			myCpg->ampl[0]= myIK->getTurnParams(myCpg->turnAngle, amplit, side, limb);
			myIK->getTurnParams(myCpg->turnAngle, amplit, side, limb);

			fprintf(parameters_file,"%f %f %f",myCpg->om_stance,myCpg->om_swing, myCpg->turnAngle);
			fprintf(parameters_file,"%f \n",Time::now()/*-original_time*/);
			fflush(parameters_file);


			printf("RECEIVING COMMANDS FROM THE MANAGER FOR PART %s\n", partName.c_str());

			//reset the go command
			if(discrete>0) {
				y_cpgs[4*nbDOFs+2*3]=0.0;
			}

			current_action = true;
		}
		else {
			printf("warning, manager sending crappy values\n");
		}
	//cout << "finish" << endl;
}


/// get other limbs rhythmic states for the coupling with the other limbs
/// only for the legs and the arms
bool GeneratorThread::getOtherLimbStatus()
{
	for(int i=0;i<nbLIMBs;i++)
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
	bot.addDouble(y_cpgs[2]-y_cpgs[0]);
	bot.addDouble(y_cpgs[3]);
	current_state_port.write();

	return true;
}

///when the feedback is on, get the needed contact info 
/// not used for the head and torso
void GeneratorThread::getContactInformation()
{
	char tmp1[255], tmp2[255];

	sprintf(tmp1,"/feedback/%s/contact",partName.c_str());
	sprintf(tmp2,"/%s/contacts_in",partName.c_str());
	if(Network::isConnected(tmp1,tmp2))
	{
		myCpg->feedback_on = 1;
		Bottle *btl = contact_port.read(false);
		if(btl!=NULL)
		{
			myCpg->contact[0] = btl->get(0).asDouble();
			myCpg->contact[1] = btl->get(1).asDouble();
		}
		printf("Part %s receiving feedback: %f %f\n",
				partName.c_str(), myCpg->contact[0],myCpg->contact[1]);
		fprintf(feedback_file, "%f %f %f \n", Time::now()-original_time, myCpg->contact[0],myCpg->contact[1]);
	}
	else
	{
		myCpg->feedback_on = 0;
	}
}

/// continuously tries to connect to other limbs (arms and legs) until successful
/// this is used to transfer the status info for the external couplings
void GeneratorThread::connectToOtherLimbs()
{
	for(int i=0;i<nbLIMBs;i++)
	{
		if(!other_part_connected[i])
		{
			char tmp1[255];
			sprintf(tmp1,"/%s/cpg_status/out",other_part_name[i].c_str());
			Contact query = Network::queryName(tmp1);
			if(query.isValid())
			{
				cout << "port " << tmp1 << " found open" << endl;
				char tmp2[255];
				sprintf(tmp2,"/%s/cpg_status/%s/in",partName.c_str(),other_part_name[i].c_str());
				bool ok = other_part_port[i].open(tmp2);
				if(ok)
				{
					ok = Network::connect(tmp1,tmp2);
					if(!ok)
						printf("error in connecting %s\n",tmp2);
					else
					{
						other_part_connected[i] = true;
						myCpg->external_coupling[i] = myCpg->next_external_coupling[i];
						//myCpg->printInternalVariables();
					}
				}
				else
				{
					printf("error in opening %s\n",tmp2);
				}
			}
			Time::delay(0.1);
		}
	}
}

void GeneratorThread::run()
{

	static double time_now=Time::now();
	time_now = Time::now();
	double time_residue = time_now - original_time - theoretical_time;
	theoretical_time += period + time_residue;

#if !DEBUG

	//we get encoders
	if(!getEncoders())
	{
		printf("Error getting encoders positions\n");
	}
#endif

	//we get the states of the other limbs and send our current status
	if(current_action)
	{
		getOtherLimbStatus();

		//if not torso or head, we try to get the information
		//if(myCpg->feedbackable)
		//getContactInformation();

		//integrate the system
		int inner_steps = (int)((period+time_residue)/myCpg->get_dt());
		for(int j=0; j<inner_steps; j++) {
			myCpg->integrate_step(y_cpgs,states);
		}

		//we send the current status of the cpgs to the higher instance
		sendStatusForManager();
	}
	else 
	{//try to connect to other limbs
		connectToOtherLimbs();
	}

	//change parameters
	getParameters();

	//stop the module if the om_stance sent my the manager is negative
	if(myCpg->om_stance<0.0)
	{
		printf("Task is finished\n");
		this->threadRelease();
		this->stop();
		return;
	}

	//we update of the previous states
	//save time stamp and print
	fprintf(target_file,"%f ",time_now);


	//we calculate dstates and log the values
	for(int i=0; i<nbDOFs; i++)
	{
		dstates[i] = (states[i] - previous_states[i]) / (period+time_residue);
		previous_states[i]=states[i];
		fprintf(target_file,"%f \t", states[i]);
	}
	fprintf(target_file,"\n");
	fflush(target_file);

#if !DEBUG  

	///////WE SEND THE COMMAND TO THE ROBOT//////////

	if(!sendFastJointCommand())
	{
		printf("error in joint command, quitting...\n");
		this->stop();
		return;
	}
	/////////////////////////////////////////////////
#endif

	//we print the loop time
	double timef=Time::now();
	double d=timef - time_now;

	//printf("loop time %f\n",d);

}



bool GeneratorThread::threadInit()
{
	fprintf(stderr, "%s thread init\n", partName.c_str());
	return true;
}


void GeneratorThread::disconnectPorts()
{
	char tmp1[255],tmp2[255];

#if !DEBUG

	sprintf(tmp1,"/%s/vcControl",partName.c_str());
	sprintf(tmp2,"/icub/vc/%s/input",partName.c_str());
	if(Network::isConnected(tmp1,tmp2))
		Network::disconnect(tmp1,tmp2);
	vcControl_port.close();

	sprintf(tmp1,"/%s/vcFastCommand",partName.c_str());
	sprintf(tmp2,"/icub/vc/%s/fastCommand",partName.c_str());
	if(Network::isConnected(tmp1,tmp2))
		Network::disconnect(tmp1,tmp2);
	vcFastCommand_port.close();

#endif

	if(myCpg->feedbackable)
	{
		sprintf(tmp1,"/feedback/%s/contact",partName.c_str());
		sprintf(tmp2,"/%s/contacts_in",partName.c_str());
		if(Network::isConnected(tmp1,tmp2))
			Network::disconnect(tmp1,tmp2);
		contact_port.close();
	}

	parameters_port.close();
	current_state_port.close();

	for(int i=0;i<nbLIMBs;i++)
		if(other_part_connected[i])
		{
			sprintf(tmp1,"/%s/cpg_status/out",other_part_name[i].c_str());
			sprintf(tmp2,"/%s/cpg_status/%s/in",partName.c_str(),other_part_name[i].c_str());
			if(Network::isConnected(tmp1,tmp2))
				Network::disconnect(tmp1,tmp2);
			other_part_port[i].close();
		}
}

void GeneratorThread::threadRelease()
{
	fprintf(stderr, "%s thread releasing\n", partName.c_str());
	//we stop the vcControl

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

	disconnectPorts();

#if !DEBUG
	delete ddPart;
#endif

	delete[] y_cpgs;
	delete[] states;
	delete[] dstates;
	delete[] previous_states;
	delete[] encoders;

	delete[] joint_limit_up;
	delete[] joint_limit_down;
	delete[] jointMapping;
	delete[] initPos;

	delete myCpg;
	delete myIK;

	fclose(target_file);
	fclose(parameters_file);
	fclose(encoder_file);
	fclose(feedback_file);
    fclose(velocity_file);

	fprintf(stderr, "%s thread released\n", partName.c_str());
}

bool GeneratorThread::init(Searchable &s)
{
	Property arguments(s.toString());
	Time::turboBoost();

	current_action = false;
	previous_quadrant= 0;

	myIK = new IKManager;

	//getting part to interface with
	Property options;

	side = 0;
	limb = 0;

	if(arguments.check("part"))
	{
		partName = arguments.find("part").asString().c_str();

		printf("module taking care of part %s\n\n",partName.c_str());
		if(partName=="left_arm" || partName=="left_leg")
			side = LEFT;
		if(partName=="right_arm" || partName=="right_leg")
			side = RIGHT;
		if(partName=="left_arm" || partName=="right_arm")
			limb = ARM;
		if(partName=="left_leg" || partName=="right_leg")
			limb = LEG;
	}
	else
	{
		printf("Please specify part to control (e.g. --part head)\n");
		return false;
	}

	if(arguments.check("file"))
	{
		options.fromConfigFile(arguments.find("file").asString().c_str());
	}
	else
	{
		char *cubPath;
		cubPath = getenv("ICUB_DIR");
		if(cubPath == NULL) {
			printf("GeneratorThread::init>> ERROR getting the environment variable ICUB_DIR, exiting\n");
			return false;
		}
		yarp::String cubPathStr(cubPath);
		options.fromConfigFile((cubPathStr + "/app/Crawling/config/" + partName + "Config.ini").c_str());
	}

	if(options.check("robot"))
	{
		robot = options.find("robot").asString();
	}
	else
	{
		robot = "icub";
	}


	char tmp1[255],tmp2[255];
	char targetPart[255], paramPart[255], encoderPart[255];
	char velPart[255], feedPart[255];

	sprintf(targetPart, "%s_target_position.dat", partName.c_str());
	sprintf(paramPart, "%s_parameters.dat", partName.c_str());
	sprintf(encoderPart, "%s_encoders.dat", partName.c_str());
	sprintf(feedPart, "%s_feedback.dat", partName.c_str());
	sprintf(velPart, "%s_velocity.dat", partName.c_str());

	target_file = fopen(targetPart, "w");
	parameters_file = fopen(paramPart, "w");
	encoder_file = fopen(encoderPart, "w");
	feedback_file = fopen(feedPart, "w");
    velocity_file = fopen(velPart, "w");


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
	ddOptions.put("carrier","udp");

	ddPart = new PolyDriver(ddOptions);

	if(!ddPart->isValid())
	{
		printf("Device not available. Here are the known devices:\n");
		printf("%s", Drivers::factory().toString().c_str());
		return false;
	}

	///encoders interface
	if(!ddPart->view(PartEncoders))
	{
		printf("Cannot view the encoders interface of %s\n",partName.c_str());
		return false;
	}

	///////////////////////////////////////////////////////////////////////
	////////Connection to the velocity control module//////////////////////
	///////////////////////////////////////////////////////////////////////

	///normal connection
	sprintf(tmp1,"/%s/vcControl",partName.c_str());
	if(!vcControl_port.open(tmp1))
	{
		printf("Cannot open vcControl port of %s\n",partName.c_str());
		return false;
	}

	sprintf(tmp2,"/icub/vc/%s/input",partName.c_str());

	if(!Network::connect(tmp1,tmp2))
	{
		printf("Cannot connect to vc/input port of %s\n",partName.c_str());
		return false;
	}

	///connection to the thread

	sprintf(tmp1,"/%s/vcFastCommand",partName.c_str());

	if(!vcFastCommand_port.open(tmp1))
	{
		printf("Cannot open vcFastCommand port of %s\n",partName.c_str());
		return false;
	}

	sprintf(tmp2,"/icub/vc/%s/fastCommand",partName.c_str());

	if(!Network::connect(tmp1,tmp2,"udp"))
	{
		printf("Cannot connect to vc/fastCommand port of %s\n",partName.c_str());
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
		printf("Failed to open port to get parameters of part %s \n",partName.c_str());
		return false;
	}

	sprintf(tmp1,"/%s/status_for_manager/out",partName.c_str());
	ok=check_status_port.open(tmp1);

	if(!ok)
	{
		printf("Warning cannot open status port for the manager, part %s\n",partName.c_str());
	}

	////////////////////////////////////////////////////////////////
	//////////Opening port to send current CPG state ///////////////
	////////////////////////////////////////////////////////////////

	sprintf(tmp1,"/%s/cpg_status/out",partName.c_str());
	ok=current_state_port.open(tmp1);

	if(!ok)
	{
		printf("Warning cannot open current cpg status port, part %s\n",partName.c_str());
	}

	////////////////////////////////////////////////////////////////
	///////////////getting internal configuration params////////////
	////////////////////////////////////////////////////////////////

	////getting the nbDOFs

	if(options.check("nbDOFs"))
		nbDOFs = options.find("nbDOFs").asInt();
	else
	{
		printf("Please specify the nbDOFs of part%s\n",partName.c_str());
		return false;
	}

	if(options.check("nbLIMBs"))
		nbLIMBs = options.find("nbLIMBs").asInt();
	else
	{
		printf("Please specify the nbLIMBs coupled with part%s\n",partName.c_str());
		return false;
	}

	///we create the CPG
	printf("nb limbs is %d\n", nbLIMBs);
	fflush(stdout);
	myCpg = new Cpgs(nbDOFs, nbLIMBs);
	fflush(stdout);

	myCpg->partName = this->partName;

	//////////////opening feedback ports if applicable

	if(partName =="head" || partName =="torso")
	{
		myCpg->feedbackable=0; //no feedback
		printf("No feedback for part %s\n", partName.c_str());
	}
	else
	{
		myCpg->feedbackable=1; //feedback
		sprintf(tmp1,"/%s/contacts_in", partName.c_str());
		if(!contact_port.open(tmp1))
		{
			printf("Cannot open contact receiving port of %s\n",partName.c_str());
			return false;
		}
	}


	/// getting the initial position
	if(options.check("init_pos"))
	{
		Bottle& pos = options.findGroup("init_pos");

		if(pos.size()!=nbDOFs+1)

		{
			printf("Incorrect specification of the initial positions of part%s\n",partName.c_str());

			return false;
		}

		initPos = new double[nbDOFs];

		for(int i=0; i<nbDOFs; i++)
		{
			initPos[i]=pos.get(i+1).asDouble();
			myCpg->parameters[2*i+1]=initPos[i]/180.0*3.1415;
			//myCpg->next_parameters[2*i]=initPos[i]/180.0*3.1415;
			printf("setting init pos %f joint %d\n",initPos[i],i);
		}
	}

	else
		printf("Warning no initial positions found\n");


	///getting the amplitude of oscillations -> Warning these should never be close to 0!!
	//otherwise the integration of the ODEs will diverge!

	if(options.check("amplitudes"))
	{
		Bottle& amp=options.findGroup("amplitudes");

		if(amp.size()!=nbDOFs+1)
		{
			printf("Incorrect nb of amplitudes part %s\n",partName.c_str());
			return false;
		}

		for(int i=0;i<nbDOFs;i++)
		{
			double ampl = amp.get(i+1).asDouble();

			if((fabs(ampl)<0.1) || (fabs(ampl) > 1.0))
			{
				printf("warning ampl of joint %d exceeds limits\n",i);
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
		printf("Warning amplitude vector not defined, setting to default\n");
		for(int i=0;i<nbDOFs;i++)
			myCpg->ampl[i]=0.1;
	}

	amplit=myCpg->ampl[0];

	if(partName=="left_arm" || partName=="right_arm")
	{
		myCpg->ampl[0]=myIK->getArmAmplitude(initPos, myCpg->ampl[0]);
	}

	printf("amplitude is %f\n", myCpg->ampl[0]);
	///getting the joint mapping
	if(options.check("joint_mapping"))
	{
		Bottle& jm = options.findGroup("joint_mapping");

		if(jm.size()!=nbDOFs+1)
		{
			printf("Incorrect nb of mapped joints of part%s\n",partName.c_str());
			return false;
		}
		jointMapping = new int[nbDOFs];

		for(int i=0;i<nbDOFs;i++)
		{
			jointMapping[i] = jm.get(i+1).asInt();

			printf("mapping state %d with joint %d\n",i,jointMapping[i]);
		}
	}
	else
	{
		printf("Please specify the joint mapping of part%s\n",partName.c_str());
		return false;
	}

#if !DEBUG
	//reading the max velocity in conf file
	if(options.check("maxVelocity"))
	{
		Bottle& mv = options.findGroup("maxVelocity");

		if(mv.size()!=nbDOFs+1)
			printf("wrong number of max velocity\n");
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
		printf("no max velocity defined, using default\n");



	///reading the Kp gains in the conf file
	if(options.check("controlGains"))
	{
		Bottle& botG = options.findGroup("controlGains");

		if(botG.size()!=nbDOFs+1)
			printf("wrong number of gains\n");
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
		printf("no gains defined, using 0\n");

#endif



	///getting the joint limit
	//up
	if(options.check("joint_limit_up"))
	{
		Bottle& jl = options.findGroup("joint_limit_up");
		if(jl.size()!=nbDOFs+1)
		{
			printf("Incorrect nb of joint limit up, part %s\n",partName.c_str());
			return false;
		}

		joint_limit_up = new double[nbDOFs];

		for(int i=0;i<nbDOFs;i++)
		{
			joint_limit_up[i] = jl.get(i+1).asDouble();

			printf("upper limit %f, joint %d\n",joint_limit_up[i],i);
		}
	}
	else
	{
		printf("Please specify upper joint limit, part %s\n",partName.c_str());
		return false;
	}

	//down
	if(options.check("joint_limit_down"))
	{
		Bottle& jl = options.findGroup("joint_limit_down");

		if(jl.size()!=nbDOFs+1)
		{

			printf("Incorrect nb of joint limit down, part %s\n",partName.c_str());
			return false;
		}

		joint_limit_down = new double[nbDOFs];

		for(int i=0;i<nbDOFs;i++)
		{
			joint_limit_down[i] = jl.get(i+1).asDouble();

			printf("down limit %f, joint %d\n",joint_limit_down[i],i);
		}
	}
	else
	{
		printf("Please specify lower joint limit, part %s\n",partName.c_str());
		return false;
	}

	///we create the vectors
	fflush(stdout);
	y_cpgs = new double[nbDOFs*4+2*nbLIMBs+1]; //4 states per internal dof + 2 per coupled dof + 1 go command
	states = new double[nbDOFs];
	dstates = new double[nbDOFs];
	previous_states = new double[nbDOFs];
	encoders = new double[nbDOFs];

	for(int i=0;i<4*nbDOFs+2*nbLIMBs+1;i++)
		y_cpgs[i] = 0.0;

#if DEBUG
	for(int i=0; i< nbDOFs; i++)
	{
		states[i]=0.0;
		previous_states[i]=0.0;
		dstates[i] = 0.0;

		y_cpgs[4*i]=0.0/180.0*3.1415/myCpg->ampl[i];

		y_cpgs[4*i+1]=0.01;

		y_cpgs[4*i+2]=0.0/180.0*3.1415/myCpg->ampl[i];

		y_cpgs[4*i+3]=0.01;
	}
#else
	double watchdog = Time::now();
	double now = watchdog;
	while(!getEncoders())
	{
		now = Time::now();
		if(now-watchdog>1.0)
		{
			printf("error getting encoders, part %s\n",partName.c_str());

			return false;
		}
	}

	for(int i=0; i< nbDOFs; i++)
	{
		states[i]=encoders[i];

		previous_states[i]=encoders[i];
		dstates[i] = 0.0;

		y_cpgs[4*i]=encoders[i]/180.0*3.1415/myCpg->ampl[i];

		y_cpgs[4*i+1]=0.0;

		y_cpgs[4*i+2]=encoders[i]/180.0*3.1415/myCpg->ampl[i];

		y_cpgs[4*i+3]=0.0;
	}

#endif

	//the go command
	y_cpgs[nbDOFs*4+2*nbLIMBs]=0.0;


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
			printf("Please specify omStance for part %s\n",partName.c_str());
			return false;
		}
	}
	else
	{
		printf("Please specify omStance for part %s\n",partName.c_str());
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
			printf("Please specify omSwing for part %s\n",partName.c_str());
			return false;
		}
	}
	else
	{
		printf("Please specify omSwing for part %s\n",partName.c_str());
		return false;
	}

	//we get tje external couplings
	if(options.check("External_coupling"))
	{
		Bottle& tmpBot = options.findGroup("External_coupling");
		Bottle& tmpBot2 = tmpBot.findGroup("parts");
		Bottle& tmpBot3 = tmpBot.findGroup("coupling");
		if(tmpBot2.isNull() || tmpBot2.size()<nbLIMBs+1 || tmpBot3.isNull() || tmpBot3.size()<nbLIMBs+1)
		{
			printf("Please specify external coupling for part %s\n",partName.c_str());
			return false;
		}
		for(int i=0;i<nbLIMBs;i++)
		{
			other_part_connected[i] = false;
			other_part_name[i] = tmpBot2.get(i+1).asString();
			myCpg->next_external_coupling[i] = tmpBot3.get(i+1).asDouble();
			myCpg->external_coupling[i] = 0.0;
		}
	}
	else
	{
		printf("Please specify external coupling for part %s\n",partName.c_str());
		return false;
	}


	//we get the internal coupling parameters

	for(int i=0;i<nbDOFs;i++)
	{
		sprintf(tmp1,"Joint%d",i);

		Bottle& tmpBot = options.findGroup(tmp1);

		if(tmpBot.isNull())
			printf("No coupling info for joint %d, part %s\n",i,partName.c_str());

		else
		{
			///coupling strength
			Bottle& tmpBot2 = tmpBot.findGroup("strength");

			if(tmpBot2.isNull() || tmpBot2.size()<nbDOFs+1)
				printf("No coupl. strength info for joint %d, part %s\n",i,partName.c_str());
			else
				for(int j=0;j<nbDOFs;j++)
					myCpg->epsilon[i][j] = tmpBot2.get(j+1).asDouble();



			///coupling phase diff

			Bottle& tmpBot3 = tmpBot.findGroup("phase");

			if(tmpBot3.isNull() || tmpBot3.size()<nbDOFs+1)
				printf("No coupl. phase info for joint %d, part %s\n",i,partName.c_str());
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



////////////CRAWL GENERATOR MODULE/////////////////

CrawlGeneratorModule::CrawlGeneratorModule()
{
}

CrawlGeneratorModule::~CrawlGeneratorModule()
{
}

double CrawlGeneratorModule::getPeriod()

{

	//    printf("Crawl Generator Module is running\n");

	return 1.0;

}



bool CrawlGeneratorModule::updateModule()
{
	return true;

}


bool CrawlGeneratorModule::close()
{    
	theThread->stop();
	delete theThread;
	fprintf(stderr, "%s module closed\n", partName.c_str());
	return true;
}


bool CrawlGeneratorModule::open(yarp::os::Searchable &s)
{
	Property options(s.toString());
	int period = 25; // in ms

	if(options.check("period"))
	{
		period = options.find("period").asInt();
	}

	if(options.check("part"))
	{
		partName=options.find("part").asString().c_str();
		printf("module taking care of part %s\n",partName.c_str());
	}


	theThread = new GeneratorThread(period);
	if(!theThread->init(s))
	{
		printf("Failed to initialize the thread\n");
		fflush(stdout);
		return false;
	}

	theThread->start();

	return true;

}

