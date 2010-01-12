// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2007 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author Manuel Lopes, Jonas Ruesch, Alex Bernardino
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

// capitalization problem on linux
#include <iCub/control_gaze.h>

#include <iCub/iKinFwd.h>

using namespace std;
using namespace iKin;
using namespace yarp::math;



using namespace iCub::contrib;

double Control_GazeModule::getPeriod()
{
	return 0.02;
}

int Control_GazeModule::mycheckmotion( int axis, double target)
{
    double curr;

    //CAREFULL -- DEFINE A WAY TO PREVENT DEADLOCK.

    //double min, max;

	/*
	// check if target over the limit
	ilim->getLimits(axis, &min, &max);
	if( (target>max) || (target<min) )
    return 1;
	*/
	ienc->getEncoder( axis, &curr);
	if( ((curr-target)*(curr-target)) < 0.01 )
		return 1;
	else
		return 0;
}
// The use of characters embedded in doubles is a bit awkward
// to use, outside of c code.
// Accept simpler codes as well.
static char getLabel(const double& d) {
    char ch = (char)(d+0.5);  // allow an epsilon of drift in representation
    switch (ch) {
    case 0:
        ch = 'a';
        break;
    case 1:
        ch = 'r';
        break;
    case 2:
        ch = 'p';
        break;
    }
    return ch;
}


double saturatevalues(double aux, double limit)
{

    if(aux > limit)
        aux = 0;
    else if(aux < -limit)
        aux = 0;

    return aux;
}
Control_GazeModule::Control_GazeModule(){
    for (int i=0; i<8; i++) {
        headpos[i] = 0;
    }
    vergenceGain=0.0;
}

Control_GazeModule::~Control_GazeModule(){

}


bool Control_GazeModule::configure(yarp::os::ResourceFinder &rf){
   
	ConstString str = rf.check("name", Value("/controlGaze"), "module name (string)").asString();
	setName(str.c_str()); // modulePortName  
	attachTerminal();

	std::string strCamConfigFile = "";
	std::string strPredConfigFile = "";
	std::string strAppPath = "";	

	bool res = true;
    cout << "Opening Module ..." << endl;
	
	strAppPath = rf.check("appPath", "", "Absolute path to the application folder (string).").asString().c_str();
	strPredConfigFile = rf.check("configPredictors","","Name of the configuration file containing the predictors parameters (string)").asString().c_str();
	strCamConfigFile = rf.check("configCamera","","Name of the configuration file containing the camera parameters (string)").asString().c_str();

	if(!rf.check("appPath")){
		strPredConfigFile = (rf.findFile(strPredConfigFile.c_str())).c_str();
		strCamConfigFile = (rf.findFile(strCamConfigFile.c_str())).c_str();
	}
	else{
		strCamConfigFile = strAppPath + std::string("/conf/") + strCamConfigFile;
		strPredConfigFile = strAppPath + std::string("/conf/") + strPredConfigFile;
	}

	//fake_velocity_control is good for use with the simulator
	if(rf.check("fake_velocity_control"))
	{
		_fake_velocity_control = true;
	}
	else
	{
		_fake_velocity_control = false;
	}

	//this is only used if _fake_velocity_control is true.
	_fake_velocity_gain = rf.check("fake_velocity_gain", Value(5.0), "Gain for the fake velocity controller. Set with care: this may lead to unstability.").asDouble();

	//otherwise use this
	_velocity_gain = rf.check("velocity_gain", Value(1.0), "Gain for the velocity controller. Set with care: this may lead to unstability.").asDouble();
	
	// getting predictors properties
	bool use_pred=true;
	if(!rf.check("configPredictors")){
		use_pred = false;
	}
	if(use_pred){	
		Property propPredictors;
		propPredictors.setMonitor(rf.getMonitor());            
		res = propPredictors.fromConfigFile(strPredConfigFile.c_str());
		if(!res){
			cout << "Error reading predictors configuration from file: " << strPredConfigFile.c_str() << endl;
			return false;
		}
		// open camera instance
		#if USE_PREDICTIVE_CONTROL
		_pred.open(propPredictors);
		#endif
	}

	// getting camera properties
	Property propCamera;
	propCamera.setMonitor(rf.getMonitor());
	res = propCamera.fromConfigFile(strCamConfigFile.c_str());
	if(!res){
		cout << "Error reading camera configuration from file: " << strCamConfigFile.c_str() << endl;
		return false;
	}

	// open camera instance
    _cam.open(propCamera);

	Bottle &xtmp = rf.findGroup("imageSize", "Actual image size (not calibration size)");
	if(xtmp.size()==3){
		int w = xtmp.get(1).asInt();
		int h = xtmp.get(2).asInt();
		_cam.setImageSize(w,h);
		printf("Set actual image size to width = %d, heigth = %d\n", _cam.width(), _cam.height());
	}
  
    string strRemoteControlboard = rf.check("motorboard","","Portname of the remote (server) controlboard").asString().c_str();

	controlType = visON;
	int a = rf.check("pidON", 0, "use integral term in tracker?").asInt();

	controlType = controlType | (pidON * rf.check("pidON", 0, "use integral term in tracker?").asInt());
	double pidGAIN = rf.check("pidGAIN", 5.0, "integral gain").asDouble();

	head.setintgain( pidGAIN);

	_bVOR = (bool)rf.check("vorON", 0, "use inertial sensor (0/1)").asInt();
	controlType = controlType | (vorON * (int)_bVOR);
	
	_bLog = (bool)rf.check("log", Value(0), "write controller log information? (0/1)").asInt();
	string strLogFile = rf.check("logfilename","log","Name of the file to write the log information (string)").asString().c_str();
	char buf[20];
	sprintf(buf, "%09.3f", yarp::os::Time().now()); //This gives a pretty unique name
	string strLogFilePath = strAppPath + string("/conf/") + strLogFile + string(buf) + string(".txt");

	
	// added JR 070723 (to control look back to center hack)
    _limitResetTime = rf.check("limitResetTime",
                                   Value(4.0),
                                   "Time from start of saccade until look back to center if saccade reaches a joint limit (double).").asDouble();

	// added AB 080532 (to control the minimum duration of saccades and make more natural behavior)
	_headSaccadeDelay = rf.check("headSaccadeDelay",
                                     Value(0.0),
                                     "Minimum time between saccades (double).").asDouble();



    // used a reference here - otherwise check for null doesn't work
    // (cannot copy a null bottle)
	
	controlrate = rf.check("ControlRate",
                             Value(50.0),
                             "Rate of the control thread (cycles per second)").asDouble();
	
	framerate = rf.check("FrameRate",
                             Value(20.0),
                             "Expected rate of velocity commands").asDouble();
	

	vergenceGain = rf.check("vergenceGain", Value(0.02), "Gain for Vergence Controller").asDouble();
	//init head controller with saccade gains
	head.setGazeControllerGain(  controlrate );

	Bottle& K = rf.findGroup("K", "controller gain");

	if( !K.isNull() )
        {
            gsl_matrix *Kgsl = gsl_matrix_alloc( 2, 3);
            if( K.size() < 6 )
                {
                    printf("wrong dimension of controller gain\n");
                    return false;
                }
            gsl_matrix_set( Kgsl, 0, 0, K.get(1).asDouble()  );
            gsl_matrix_set( Kgsl, 0, 1, K.get(2).asDouble() );
            gsl_matrix_set( Kgsl, 0, 2, K.get(3).asDouble() );
            gsl_matrix_set( Kgsl, 1, 0, K.get(4).asDouble() );
            gsl_matrix_set( Kgsl, 1, 1, K.get(5).asDouble() );
            gsl_matrix_set( Kgsl, 1, 2, K.get(6).asDouble() );

            head.setGazeControllerGain( Kgsl  );
            gsl_matrix_free( Kgsl );
        }
	else
		printf("NO CONTROLLER GAIN READ\n");

	egosphereVisualUpdateThreshold =
		rf.check("egosphereVisualUpdateThreshold", 1, "threshold for ending saccade").asDouble();

	//	head.setGazeControllerGain(gsl_matrix *K)
	Property propBoard;

	propBoard.put("device", "remote_controlboard");
	propBoard.put("remote" , strRemoteControlboard.c_str());
	cout << strRemoteControlboard << endl;
	propBoard.put("local", getName("controlboard"));
	bool ddOk = dd.open(propBoard);
	if(!ddOk){
		cout << "Failed to open remote-controlboard with properties: " << std::string(propBoard.toString().c_str()) << std::endl;
		return false;
	}
	if(!dd.view(ipos)){
		cout << "No position control... " << endl;
		return false;
	}
	if(!dd.view(ivel)){
		cout << "No velocity control... " << endl;
		return false;
	}
	if(!dd.view(ienc)){
		cout << "No encoders.. " << endl;
		return false;
	}

	if(!dd.view(ilim)){
		cout << "No limits.. " << endl;
		return false;
	}
	if(!dd.view(iamp)){
		cout << "No amplifier.. " << endl;
		return false;
	}

	if(!dd.view(ipid)){
		cout << "No pid.. " << endl;
		return false;
	}

    ipos->getAxes(&_numAxes);
    if (_numAxes == 0){
        cout << "*** Controlboard provides no axes. Probably connection to server controlboard was not established properly. Check your motorboard configuration value." << endl;
        // return false;
    }
	ivel->getAxes(&_numAxes);
    if (_numAxes == 0){
        cout << "*** Controlboard provides no axes. Probably connection to server controlboard was not established properly. Check your motorboard configuration value." << endl;
        // return false;
    }

	//setting the limits for eye/head/torso chain. 
	double mi, ma;
	Vector maxL(8), minL(8);
	maxL(0) = 80; //torso 1
	maxL(1) = 80; //torso 2
	maxL(2) = 80; //torso 3
	minL(0) = -80; //torso 1
	minL(1) = -80; //torso 2
	minL(2) = -80; //torso 3
	ilim->getLimits(0, &mi, &ma); //neck tilt
	maxL(3) = ma; minL(3) = mi;
	ilim->getLimits(1, &mi, &ma); //neck swing
	maxL(4) = ma; minL(4) = mi;
	ilim->getLimits(2, &mi, &ma); //neck pan
	maxL(5) = ma; minL(5) = mi;
	ilim->getLimits(3, &mi, &ma); //eye tilt
	// limit eye's tilt due to eyelids
    if (mi<-28.0)
          mi=-28.0;
    if (ma>12.0)
          ma=12.0;
	maxL(6) = ma; minL(6) = mi;
	ilim->getLimits(4, &mi, &ma); //eye pan - assume identical for both eyes
	maxL(7) = ma; minL(7) = mi;
//	maxL(0)=maxL(1)=maxL(2)=maxL(3)=maxL(4)=maxL(5)=maxL(6)=maxL(7)=50;
//	minL(0)=minL(1)=minL(2)=minL(3)=minL(4)=minL(5)=minL(6)=minL(7)=-50;

	head.initEyeKin(maxL, minL);
	head.initInertialKin(maxL, minL);
	
	_smoothInput_port.open( getName("/vel"));
	_saccadeInput_port.open( getName("/pos"));
	_imageCoordPort.open( getName("/imgcoord"));
	_inertialInput_port.open( getName("/imu"));
	_torsoInput_port.open( getName("/torso"));
	_disparityInput_port.open( getName("/dis"));
	_trackersignalOutput_port.open( getName("/trackersignal/bot:o"));
	_posdirOutput_port.open( getName("/possibledirections/vec:o") );
    _status_port.open(getName("/status:o"));
    // open config port
    _configPort.open(getName("/conf"));
    attach(_configPort);

	//INITIAL COMMAND AND STATE
	_targ_azy = 0; 
	_targ_elev = 0;
	_abs_ref_az = 0;
	_abs_ref_el = 0;
//	_targ_verg = 0;

	desazy = 0;	
	deselev = 0;
//	desverg = 0;

//	desazy_oe = 0;	
//	deselev_oe = 0;
//	desverg_oe = 0;

	currenterror=0;
	saccadeid = 0;
	_cmd = NONE;
	_coord = RELATIVE_ANGLE;
    _behav = REST; //rest where it starts
	_command_x = 0.0;
	_command_y = 0.0;

	//targtype = 'a';
	//behavior = 's';
    /*
      neckvel = gsl_vector_calloc( 3 );
      eyevel = gsl_vector_calloc( 2 );

      wn = gsl_vector_calloc( 3 );
      wo = gsl_vector_calloc( 3 );
      J = gsl_matrix_calloc( 6, head.Rl->Getm_njoints());
      J45inv = gsl_matrix_calloc( 2, 3);
      J13inv = gsl_matrix_calloc( 3, 3);
      J13 = gsl_matrix_calloc( 3, 3);
      J45 = gsl_matrix_calloc( 3, 2);
    */

	oldW = gsl_vector_calloc(3);
	inertialW = gsl_vector_calloc(3);

	for(int i = 0; i < 8; i++)
		torsopos[i] = 0.0;

	egosphere.open(getName("/remoteEgoSphere"));


	if(_bLog)
		fp = fopen(strLogFilePath.c_str(),"w");

	cout << "finished opening" << endl;

	cout << "setting up the motor controllers" << endl;

	// Initializing amplifier interface
	iamp->enableAmp(0);
	yarp::os::Time().delay(0.1);printf(".");
	iamp->enableAmp(1);
	yarp::os::Time().delay(0.1);printf(".");
	iamp->enableAmp(2);
	yarp::os::Time().delay(0.1);printf(".");
	iamp->enableAmp(3);
	yarp::os::Time().delay(0.1);printf(".");
	iamp->enableAmp(4);
	yarp::os::Time().delay(0.1);printf(".");
	iamp->enableAmp(5);
	yarp::os::Time().delay(0.1);printf(".");

	// Initializing PID interface
	ipid->enablePid(0);
	yarp::os::Time().delay(0.1);printf(".");
	ipid->enablePid(1);
	yarp::os::Time().delay(0.1);printf(".");
	ipid->enablePid(2);
	yarp::os::Time().delay(0.1);printf(".");
	ipid->enablePid(3);
	yarp::os::Time().delay(0.1);printf(".");
	ipid->enablePid(4);
	yarp::os::Time().delay(0.1);printf(".");
	ipid->enablePid(5);
	yarp::os::Time().delay(0.1);printf(".");

    
	// Initializing Velocity Interface
	double accs[] = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
	ivel->setRefAccelerations(accs);
	yarp::os::Time().delay(0.1);printf(".");
	
	// Initializing Position Interface
	double spds[] = {2000, 200, 200, 200, 200, 200, 200, 200};
	ipos->setRefSpeeds(spds);
	yarp::os::Time().delay(0.1);printf(".");
	ipos->setRefAccelerations(accs);
	yarp::os::Time().delay(0.1);printf(".");
	
	//doing some test motions to check all motors are OK

	/*
      yarp::os::Time().delay(0.1);printf(".");
      ipos->positionMove( 0, 10);
      yarp::os::Time().delay(0.1);printf(".");
      ipos->positionMove( 1, 10);
      yarp::os::Time().delay(0.1);printf(".");
      ipos->positionMove( 2, 10);
      yarp::os::Time().delay(0.1);printf(".");
      ipos->positionMove( 3, 10);
      yarp::os::Time().delay(0.1);printf(".");
      ipos->positionMove( 4, 10);	
      yarp::os::Time().delay(0.1);printf(".");
      ipos->positionMove( 5, 10);	
      yarp::os::Time().delay(0.1);printf(".");


      yarp::os::Time().delay(2);printf(".");

      ipos->positionMove( 0, 0);
      yarp::os::Time().delay(0.1);printf(".");
      ipos->positionMove( 1, 0);
      yarp::os::Time().delay(0.1);printf(".");
      ipos->positionMove( 2, 0);
      yarp::os::Time().delay(0.1);printf(".");
      ipos->positionMove( 3, 0);
      yarp::os::Time().delay(0.1);printf(".");
      ipos->positionMove( 4, 0);	
      yarp::os::Time().delay(0.1);printf(".");
      ipos->positionMove( 5, 0);	
      yarp::os::Time().delay(0.1);printf(".");

	*/


	// GO !
	printf("\n> ");
	ncycles = 0;
    timesaccadeid = 0;
	start = Time::now();
   
    return true;
}

bool Control_GazeModule::close(){
    printf("closing Control_Gaze\n");

    if (_numAxes > 0){
		ipos->setRefSpeed( 0, 20);
		ipos->setRefSpeed( 1, 20);
		ipos->setRefSpeed( 2, 20);
		ipos->setRefSpeed( 3, 20);
		ipos->setRefSpeed( 4, 20);
		ipos->setRefSpeed( 5, 20);
	    ipos->positionMove( 0, 0);
	    ipos->positionMove( 1, 0);
	    ipos->positionMove( 2, 0);
	    ipos->positionMove( 3, 0);
	    ipos->positionMove( 4, 0);	
	    ipos->positionMove( 5, 0);	
    }
    printf("1\n");
	vels[0]=0;
	vels[1]=0;
	vels[2]=0;
	vels[3]=0;
	vels[4]=0;
	vels[5]=0;
    printf("2\n");
    if (_numAxes > 0){
	    ivel->velocityMove( vels );
    }
    printf("3\n");
	_smoothInput_port.close();
	_saccadeInput_port.close();
	_imageCoordPort.close();
	_disparityInput_port.close();
    printf("4\n");
	_inertialInput_port.close();
	_torsoInput_port.close();
    _configPort.close();
	_trackersignalOutput_port.close();
	_posdirOutput_port.close();
    _status_port.close();
    egosphere.close();
    printf("5\n");
	if(_bLog)
		fclose( fp );
    printf("6\n");
	gsl_vector_free( oldW );
	gsl_vector_free( inertialW );
	//gsl_vector_free( torsoW );
    printf("7\n");
	dd.close();
    printf("8\n");

    return true;
}

bool Control_GazeModule::interruptModule(){
    printf("\n\n\n\n\nControl_GazeModule::interruptModule()\n");
	/* 
	// WE REMOVED THIS TO TRY TO SOLVE THE PROBLEM OF CRASHING WHEN WE 'QUIT'
	
	_mutex.wait();

	_trackersignalOutput_port.interrupt();
	_inertialInput_port.interrupt();
	_smoothInput_port.interrupt();
	_saccadeInput_port.interrupt();
	_imageCoordPort.interrupt();
	_disparityInput_port.interrupt();
    _posdirOutput_port.interrupt(); 
    _status.interrupt();
	
	_mutex.post();
	*/
    return true;
}

bool Control_GazeModule::respond(const Bottle &command,Bottle &reply){
	
    bool rec = false; // recognized
    bool ok = false; // command executed successfully
    double az, el;  // get azimuth/elevation
    string status;  // controller status
    double time;    // for time variables

    switch (command.get(0).asVocab()) {
    case CONTROLGAZE_VOCAB_SACCADE:
        switch(command.get(1).asVocab()){
        case CONTROLGAZE_VOCAB_COORD_ABS:
            rec = true;
            ok = saccadeAbsolute(command.get(2).asDouble(), command.get(3).asDouble());
            if (!ok) reply.addVocab(VOCAB_FAILED);
            else reply.addVocab(VOCAB_OK);
            break;
        case CONTROLGAZE_VOCAB_COORD_REL:
            rec = true;
            ok = saccadeRelative(command.get(2).asDouble(), command.get(3).asDouble());
            if (!ok) reply.addVocab(VOCAB_FAILED);
            else reply.addVocab(VOCAB_OK);
            break;
        case CONTROLGAZE_VOCAB_COORD_IMG:
            rec = true;
            ok = saccadeImageRef(command.get(2).asDouble(), command.get(3).asDouble());
            if (!ok) reply.addVocab(VOCAB_FAILED);
            else reply.addVocab(VOCAB_OK);
            break;
		case CONTROLGAZE_VOCAB_COORD_PIX:
            rec = true;
            ok = saccadePixelRef(command.get(2).asDouble(), command.get(3).asDouble());
            if (!ok) reply.addVocab(VOCAB_FAILED);
            else reply.addVocab(VOCAB_OK);
            break;
        default:
            break;
        }   
        break;
	case CONTROLGAZE_VOCAB_PURSUIT:
        switch(command.get(1).asVocab()){
        case CONTROLGAZE_VOCAB_COORD_ABS:
            rec = true;
            ok = pursuitAbsolute(command.get(2).asDouble(), command.get(3).asDouble());
            if (!ok) reply.addVocab(VOCAB_FAILED);
            else reply.addVocab(VOCAB_OK);
            break;
        case CONTROLGAZE_VOCAB_COORD_REL:
            rec = true;
            ok = pursuitRelative(command.get(2).asDouble(), command.get(3).asDouble());
            if (!ok) reply.addVocab(VOCAB_FAILED);
            else reply.addVocab(VOCAB_OK);
            break;
        case CONTROLGAZE_VOCAB_COORD_IMG:
            rec = true;
            ok = pursuitImageRef(command.get(2).asDouble(), command.get(3).asDouble());
            if (!ok) reply.addVocab(VOCAB_FAILED);
            else reply.addVocab(VOCAB_OK);
            break;
		case CONTROLGAZE_VOCAB_COORD_PIX:
            rec = true;
            ok = pursuitPixelRef(command.get(2).asDouble(), command.get(3).asDouble());
            if (!ok) reply.addVocab(VOCAB_FAILED);
            else reply.addVocab(VOCAB_OK);
            break;
		case CONTROLGAZE_VOCAB_INTERRUPT:
            rec = true;
            ok = pursuitInterrupt();
            if (!ok) reply.addVocab(VOCAB_FAILED);
            else reply.addVocab(VOCAB_OK);
            break;
        default:
            break;
        }   
        break;
	case CONTROLGAZE_VOCAB_GET:
        switch(command.get(1).asVocab()) {
		case CONTROLGAZE_VOCAB_DIR_HEAD:
            rec = true;
            ok = this->getDirectionHead(az, el);
            if (ok){
                reply.addDouble(az);
                reply.addDouble(el);
                reply.addVocab(VOCAB_OK);
            }
            else reply.addVocab(VOCAB_FAILED);
			break;
        case CONTROLGAZE_VOCAB_DIR_EYE_RIGHT:
            rec = true;
            ok = this->getDirectionEyeRight(az, el);
            if (ok){
                reply.addDouble(az);
                reply.addDouble(el);
                reply.addVocab(VOCAB_OK);
            }
            else reply.addVocab(VOCAB_FAILED);
			break;
        case CONTROLGAZE_VOCAB_DIR_EYE_LEFT:
            rec = true;
            ok = this->getDirectionEyeLeft(az, el);
            if (ok){
                reply.addDouble(az);
                reply.addDouble(el);
                reply.addVocab(VOCAB_OK);
            }
            else reply.addVocab(VOCAB_FAILED);
			break;
        case CONTROLGAZE_VOCAB_STATE:
            rec = true;
            ok = this->getControllerStatus(status);
            if (ok){
                reply.addString(status.c_str());
                reply.addVocab(VOCAB_OK);
            }
            else reply.addVocab(VOCAB_FAILED);
			break;
        case CONTROLGAZE_VOCAB_SACCADE_TIME:
            rec = true;
            ok = this->getSaccadeTime(time);
            if (ok){
                reply.addDouble(time);
                reply.addVocab(VOCAB_OK);
            }
            else reply.addVocab(VOCAB_FAILED);
			break;
        case CONTROLGAZE_VOCAB_REF:
            rec = true;
            ok = this->getReference(az, el);
            if (ok){
                reply.addDouble(az);
                reply.addDouble(el);
                reply.addVocab(VOCAB_OK);
            }
            else reply.addVocab(VOCAB_FAILED);
			break;
        case CONTROLGAZE_VOCAB_MIN_SACCADE_TIME:
            rec = true;
            ok = this->getMinSaccadeTime(time);
            if (ok){
                reply.addDouble(time);
                reply.addVocab(VOCAB_OK);
            }
            else reply.addVocab(VOCAB_FAILED);
			break;

        case CONTROLGAZE_VOCAB_LIMIT_RESET_TIME:
            rec = true;
            ok = this->getLimitResetTime(time);
            if (ok){
                reply.addDouble(time);
                reply.addVocab(VOCAB_OK);
            }
            else reply.addVocab(VOCAB_FAILED);
			break;
        
        case CONTROLGAZE_VOCAB_VERGENCE_GAIN:
            {
                rec = true;
                double gain=this->vergenceGain;
                reply.addDouble(gain);
                reply.addVocab(VOCAB_OK);
            }
            break;

		default:
            break;
		}
        break;
    case CONTROLGAZE_VOCAB_SET:
        {
            switch(command.get(1).asVocab()) {
            case CONTROLGAZE_VOCAB_VERGENCE_GAIN:
                this->vergenceGain=command.get(2).asDouble();
                rec = true;
                reply.addVocab(VOCAB_OK);
                break;
            case CONTROLGAZE_VOCAB_MIN_SACCADE_TIME:
                rec = true;
                ok = this->setMinSaccadeTime(command.get(2).asDouble());
                if (ok){
                    reply.addVocab(VOCAB_OK);
                }
                else reply.addVocab(VOCAB_FAILED);
			    break;
            case CONTROLGAZE_VOCAB_LIMIT_RESET_TIME:
                rec = true;
                ok = this->setLimitResetTime(command.get(2).asDouble());
                if (ok){
                    reply.addVocab(VOCAB_OK);
                }
                else reply.addVocab(VOCAB_FAILED);
			    break;
            }
            break;
        }
    case CONTROLGAZE_VOCAB_RESET:
        rec = true;
        ok = reset();
        if (!ok) reply.addVocab(VOCAB_FAILED);
        else reply.addVocab(VOCAB_OK);
        break;
	case CONTROLGAZE_VOCAB_HELP:
        rec = true;
        ok = help();
        if (!ok) reply.addVocab(VOCAB_FAILED);
        else reply.addVocab(VOCAB_OK);
        break;
	default:
        break;
	}
	
    if (!rec)
        ok = RFModule::respond(command,reply); // will add message 'not recognized' if not recognized
	return ok;
}


bool Control_GazeModule::updateModule() //this runs every 
{

    if (_numAxes == 0){
        cout << "Controlboard does not provide any axis. Idle..." << endl;
        yarp::os::Time::delay(1);
        return true;
    }

	_mutex.wait(); 
	//printf("Control_GazeModule::updateModule()\n");
    //cout << "update starts now..." << endl;
	
	// read commands from the ports
	Bottle *imgcoordinput = NULL;
	Vector *posinput = NULL;
	Vector *velinput = NULL;
	Vector *dispinput = NULL;
	

	// READ COMMAND PORTS
	imgcoordinput = _imageCoordPort.read(false);
	posinput = _saccadeInput_port.read( false );
	velinput = _smoothInput_port.read( false );
	dispinput = _disparityInput_port.read( false );
	
	

	//Update state
	_updateTime = Time::now();
	if (timesaccadeid == 0)  //for the initial saccade
        timesaccadeid = _updateTime;
	ncycles++;
	
	
    double disparity;
	if( dispinput )
		disparity = (*dispinput)(0);
	else
		disparity = 0;

	// COMMAND PROCESSOR
	//_cmd = NONE;   This goes to the end because the response() function may have received a command
    char coord_id;
	char behav_id;
	int temp_id;
	//Receiving pursuit commands
	//Only accept pursuit commands if in modes REST, LIMIT, START_PURSUIT or CONTINUE_PURSUIT
	if( _behav == REST || _behav == LIMIT || _behav == START_PURSUIT || _behav == CONTINUE_PURSUIT )
        {
            if (velinput)  
                {
                    _cmd = PURSUIT;
                    _command_x = (*velinput)(0);
                    _command_y = (*velinput)(1);
                    coord_id = getLabel((*velinput)(2));
                    if(coord_id == 'r') 
                        {
                            //check the limits 
                            if( _command_x > 1 || _command_x < -1 || _command_y > 1 || _command_y < -1)
                                {
                                    //this is invalid, means a tracker failure - lets hold on the current position
                                    _coord = RELATIVE_ANGLE;
                                    _command_x = 0.0;
                                    _command_y = 0.0;
                                }
                            else
                                {
                                    _coord = NORMALIZED_PIXEL;	
                                }
                        }
                    else if(coord_id == 'i')
                        {
                            //check the limits 
                            if( _command_x > _cam.width() || _command_x < 0 || _command_y > _cam.height() || _command_y < 0)
                                {
                                    //this is invalid, means a tracker failure - lets hold on the current position
                                    _coord = RELATIVE_ANGLE;
                                    _command_x = 0.0;
                                    _command_y = 0.0;
                                }
                            else
                                {
                                    _coord = IMAGE_PIXEL;
                                }
                        }
                }
        }
	//Receiving saccade commands - these have priority wrt pursuit
	if( _behav == REST || _behav == LIMIT || _behav == START_PURSUIT || _behav == CONTINUE_PURSUIT )
        {
            if(imgcoordinput) //this port was made purposefully to connect directly viewer clicks 
                {
                    _command_x = imgcoordinput->get(0).asInt();
                    _command_y = imgcoordinput->get(1).asInt();
                    if(_command_x >= 0 && _command_x < _cam.width() && _command_y >= 0 && _command_y < _cam.height())
                        {
                            _cmd = SACCADE;
                            _coord = IMAGE_PIXEL;
                            temp_id = saccadeid + 1;
                        }
                }
            if(posinput)
                {
                    _command_x = (*posinput)(0);
                    _command_y = (*posinput)(1);
                    if(posinput->length() == 5)
                        {
                            coord_id = getLabel((*posinput)(2));
                            behav_id = getLabel((*posinput)(3));
                            temp_id = (int)((*posinput)(4));
                        }
                    else if(posinput->length() == 4)
                        {
                            coord_id = getLabel((*posinput)(2));
                            behav_id = getLabel((*posinput)(3));
                            temp_id = saccadeid + 1;
                        }
                    else if(posinput->length() == 3)
                        {
                            coord_id = getLabel((*posinput)(2));
                            behav_id = 's';
                            temp_id = saccadeid + 1;
                        }
                    else if(posinput->length() == 3)
                        {
                            coord_id = 'i';
                            behav_id = 's';
                            temp_id = saccadeid + 1;
                        }	

                    switch(coord_id)
                        {
                        case 'r': _coord = RELATIVE_ANGLE; break;
                        case 'a': _coord = ABSOLUTE_ANGLE; break;
                        case 'p': _coord = NORMALIZED_PIXEL; break;
                        case 'i': _coord = IMAGE_PIXEL; break;
                        }
                    if(temp_id > saccadeid)
                        {
                            _cmd = SACCADE;
                            saccadeid = temp_id;
                        }		
                }
        }

	// measure the state of the system
	bool validData = false;
	while( ! (validData = ienc->getEncoders( headpos )) )
		yarp::os::Time::delay(0.002);

	Vector *inertialmeas = NULL;
	inertialmeas = _inertialInput_port.read( false );
	if( !inertialmeas ) {
		gsl_vector_set_zero( inertialW );
	}
	else {
		double aux = (*inertialmeas)(7);
		aux = saturatevalues( aux, 0.03);
		gsl_vector_set( inertialW, 0, -aux * 180/3.1415);
		
		aux = (*inertialmeas)(6);
		aux = saturatevalues( aux, 0.03);
		gsl_vector_set( inertialW, 1, aux * 180/3.1415);
		
		aux = (*inertialmeas)(8);
		aux = saturatevalues( aux, 0.03);
		gsl_vector_set( inertialW, 2, aux * 180/3.1415);

		gsl_print_vector( inertialW, "inertialW");
	}

	Vector *torsopos_temp = NULL;
	torsopos_temp = _torsoInput_port.read( false );
	if(torsopos_temp) //update torsopos
	{
		for(int i = 0; i < torsopos_temp->length(); i++ )
		{
			torsopos[i] = (*torsopos_temp)[i];
		}
	}

	/*DEBUGGING*/
	/*for(int i = 0; i < 8; i++)
	{
		headpos[i] = 0;
		torsopos[i] = 0;
	}
	torsopos[2] = 10;*/


	head.HeadGaze( &_head_azy, &_head_elev, headpos);
	head.Gaze( &_eye_azy, &_eye_elev, headpos);

	// If we had a valid timely command, update the desired gaze

	//JUST FOR DEBUGGING
	/*_cmd = SACCADE;
	_command_x = 0;
	_command_y = 0;
	_coord = ABSOLUTE_ANGLE;*/
	
	if(_cmd != NONE )
        {
            /*updateAbsoluteGazeReference(_command_x, _command_y, headpos, _eye_azy, _eye_elev,
										_targ_azy, _targ_elev, _cmd, _coord);*/

			updateAbsoluteGazeReference2(_command_x, _command_y, headpos, torsopos, _eye_azy, _eye_elev,
										_abs_ref_az, _abs_ref_el, _cmd, _coord);
        }
	
	//Angles are in degrees.
	//Here I have to convert from absolute coordinates to neck based coordinates.
	//The head controller works on neck based coordinates,
	//but the target coordinates are computed in absolute.
	Matrix DesRotWaist = head.getRotationMatrixFromRollPitchYawAngles(0, _abs_ref_el, _abs_ref_az);
	Matrix Neck2Waist = head.getNeck2WaistTransf(torsopos);
	Matrix Waist2Neck = Neck2Waist.transposed();
	//this matrix is to rotate the neck such that roll, pitch, yaw angles can be computed as usual.
	Matrix NeckNorm(3,3);
	NeckNorm(0,0) = -1; NeckNorm(0,1) = 0; NeckNorm(0,2) = 0;
	NeckNorm(1,0) = 0; NeckNorm(1,1) = 0; NeckNorm(1,2) = 1;
	NeckNorm(2,0) = 0; NeckNorm(2,1) = 1; NeckNorm(2,2) = 0;
	Matrix DesRotNeck = NeckNorm * Waist2Neck * DesRotWaist; 
	double temp;
	head.getRollPitchYawAnglesFromRotationMatrix(DesRotNeck, temp, _targ_elev, _targ_azy);

	desazy = _targ_azy;
	deselev = _targ_elev;

	if(_cmd != NONE)
        {
            printf("Received Command: ");
            switch(_cmd) 
                {
                case SACCADE:   printf("Saccade "); break;
                case PURSUIT:   printf("Pursuit "); break;
                case INTERRUPT: printf("Interrupt "); break;
                }
            switch(_coord)
                {
                case ABSOLUTE_ANGLE: printf("absolute "); break;
                case RELATIVE_ANGLE: printf("relative "); break;
                case NORMALIZED_PIXEL: printf("normalized "); break;
                case IMAGE_PIXEL: printf("image "); break;
                }
            printf("x: %02.2f, y: %02.2f.\n", _command_x, _command_y);
        }
		
    // if nothing is received, or commands are invalid, 
	// do not update desired heading nor the state machine

	gsl_vector* pert;	pert = gsl_vector_calloc( 2 );
	gsl_vector* X;	X = gsl_vector_calloc( 3 );

	/*
      {	// see free direction spacial saliency
		
      Vector &vec = _posdirOutput_port.prepare();
      vec.resize(16);
      head.pred( headpos, vec.data(), 16 ); 

      _posdirOutput_port.write();

      }
	*/
	
    // THE STATE MACHINE
	//   TRANSITIONS DUE TO COMMANDS
	switch( _behav )
        {
		case REST: 
		case LIMIT:
			if( _cmd == SACCADE )
                {		
                    _behav = SACCADE_1; //goes to saccade mode
                }
			else if( _cmd == PURSUIT )
                {		
                    _behav = START_PURSUIT; //goes to smooth pursuit mode
                }
			// else remains in the same state
			break;
		case SACCADE_1:
			//this is not interruptible - is an atomic state
			break;
		case SACCADE_2:
			// Do not accept any other commands - maybe later this can be interrupted 
			// or use a flag "interruptable".
			break;
		case START_PURSUIT:
			// This is not interruptible - is an atomic state
			break;
		case CONTINUE_PURSUIT: //can be interrupted by a saccade
			if( _cmd == SACCADE )
			    _behav = SACCADE_1; //goes to saccade mode
			else if( _cmd == INTERRUPT )
                {
                    _behav = REST; 
                    desazy = _eye_azy;
                    deselev = _eye_elev;
                }			
			break;
		default:
			break;
        }

	//for the smooth pursuit predictors
	double realpos[3];
	realpos[0] = _targ_azy;
	realpos[1] = _targ_elev;
	realpos[2] = 1;				

    // THE STATE EXECUTION
    switch( _behav )
        {
		case REST: 
		case LIMIT:
			currenterror = head.HeadGazeController(desazy, deselev, X, pert, oldW, vels, headpos, inertialW, 0,controlType);
			//printf("%f %f %f %f \n", vels[0], vels[2], vels[3], vels[4] );

            vels[5]=-vergenceGain*disparity;
			velmove(vels); //velocity control is smoother
			break;
		case SACCADE_1:  //saccade initiated eye only phase
			//Does not accept commands here
			printf("STARTING SACCADE");
			egosphere.setSaccadicSuppression( true ); // does not wait for a response (if connection not established: GoTo Nirvana)
			head.setGazeControllerGain(  controlrate );		
			currenterror = 1000; //this **stupid** flag is to indicade eye only mode !
			currenterror = head.HeadSaccade( desazy, deselev, vels, headpos, currenterror);
            timesaccadeid = _updateTime; 
			posmove( vels ); //position control
			_behav = SACCADE_2; //next time it will go to the neck+eye saccade 

			/*
              if(1) { // I THINK THIS SHOULD BE IN THE END OF SACCADE_2 - ALEX
              // inform the tracker about the new target
              printf("SENDING RESET TO TRACKER\n");
              Bottle &bot = _trackersignalOutput_port.prepare();
              bot.addVocab( Vocab::encode("SET") );
              bot.addVocab( Vocab::encode("POS") );    
              _trackersignalOutput_port.write();
              }*/
			break;

		case SACCADE_2: //saccade continuation (combined neck/eye phase)
			currenterror = head.HeadGazeController(desazy, deselev, X, pert, oldW, vels, headpos, inertialW, 0,controlType);
			

            vels[5]=-vergenceGain*disparity;
			//printf("%f %f %f %f \n", vels[0], vels[2], vels[3], vels[4] );

			velmove(vels); //velocity control is smoother

			//check if it is time to end saccade
			if( timesaccadeid + _headSaccadeDelay < _updateTime ) 
                {	
                    //check if we have reached a close to final position
                    if( (currenterror < egosphereVisualUpdateThreshold) )
                        {
                            printf("ENDING SACCADE\n");
							egosphere.setSaccadicSuppression( false ); // does not wait for a response (if connection not established: GoTo Nirvana)
                            _behav = REST; //goes to a controlled waiting state
                        } 
                    else if( (timesaccadeid + _limitResetTime) < _updateTime)
                        //It is at too much time in this state. Maybe stuck in a limit.	
                        {
                            printf("REACHED LIMIT\n");
                            egosphere.setSaccadicSuppression( false ); // does not wait for a response (if connection not established: GoTo Nirvana)
                            _behav = LIMIT; 
                            //Basically the same as rest but must reset the desired gaze
                            desazy = _eye_azy;
                            deselev = _eye_elev;
                        }
                }
			break;

		case START_PURSUIT: //start pursuit mode
			printf("STARTING PURSUIT");
			#if USE_PREDICTIVE_CONTROL
			_pred.reset(realpos); 
			#endif
			head.setGazeControllerGain(  framerate );
			_behav = CONTINUE_PURSUIT;
			//Initialize here other pursuit initialization stuff (filters, predictors);
			//break;
		case CONTINUE_PURSUIT: //continues pursuit mode
			//This can be interrupted
			if( _cmd == PURSUIT)
                {
                    
					#if USE_PREDICTIVE_CONTROL
                            _prediction = _pred.predict(realpos, 1, 1);
                            desazy = _prediction[0];
                            deselev = _prediction[1];
					#else                        
                            desazy = _targ_azy;
                            deselev = _targ_elev;
					#endif
                 }
			currenterror = head.HeadGazeController(desazy, deselev, X, pert, oldW, vels, headpos, inertialW, 0,controlType);
			//printf("%f %f %f %f \n", vels[0], vels[2], vels[3], vels[4] );

            vels[5]=-vergenceGain*disparity;
			velmove(vels); //velocity control is smoother
			break;
		
		default:
			printf(" ERROR STATE \n");
        }

	

	//printf("time write %f\n", Time::now() - _updateTime);
	double cycpersec;
	if( ncycles % ((int)controlrate*10) == 0) //reports only once every 10 seconds
        {
            // the frame rate
            cycpersec =  ncycles / (Time::now() - start);
            printf("%f fps\n", cycpersec);
            printf("*************************************\n");
            // the current mode
            printf("MODE = ");
            switch( _behav )
                {
                case REST: printf("REST"); break;
                case LIMIT: printf("LIMIT"); break;
                case SACCADE_1: printf("START_SACCADE"); break;
                case SACCADE_2: printf("CONTINUE_SACCADE"); break;
                case START_PURSUIT: printf("START_PURSUIT"); break;
                case CONTINUE_PURSUIT: printf("CONTINUE_PURSUIT"); break;
                }
            printf("\n");

            // the last received command 
            printf("LAST COMMAND: ");
            if(_lastcmd == SACCADE)
                printf("SACCADE ");
            else if (_lastcmd == PURSUIT)
                printf("PURSUIT ");
            else if (_lastcmd == INTERRUPT)
                printf("INTERRUPT");
            else
                printf("UNKNOWN CMD ");

            // the associated coordinates
            switch(_lastcoord )
                {
                case ABSOLUTE_ANGLE:   printf("ABSOLUTE ANGLE "); break;
                case RELATIVE_ANGLE:   printf("RELATIVE ANGLE "); break;
                case NORMALIZED_PIXEL: printf("NORMALIZED IMAGE "); break;
                case IMAGE_PIXEL:      printf("IMAGE PIXEL "); break;
                default:               printf("UNKNOWN COORDS "); break;
                }
            printf("\n");
            // and the command values
            printf("x: %02.2f y: %02.2f \n", _command_x, _command_y);

            // the current state
            printf("ref azy:%02.2f head azy:%02.2f eye azy:%02.2f ref elev:%02.2f head elev:%02.2f eye elev:%02.2f\n", desazy, _head_azy, _eye_azy, deselev, _head_elev, _eye_elev);
        }

    //status port (required by attentionSelection)
    Bottle &bot = _status_port.prepare();
    bot.clear();
    bot.addInt(_behav);
    bot.addDouble(_eye_azy);
    bot.addDouble(_eye_elev);
	_status_port.write();

	if( _bLog )
        {
            //report status
            fprintf(fp, "sta: %05d %09.3f ", ncycles, _updateTime );
            switch( _behav )
                {
                case REST: fprintf(fp, "RST"); break;
                case LIMIT: fprintf(fp, "LIM"); break;
                case SACCADE_1: fprintf(fp, "SC1"); break;
                case SACCADE_2: fprintf(fp, "SC2"); break;
                case START_PURSUIT: fprintf(fp, "SPR"); break;
                case CONTINUE_PURSUIT: fprintf(fp, "CPR"); break;
                }
            fprintf(fp, "\n");

            //report command
            fprintf(fp, "cmd: ");
            switch( _cmd )
                {
                case NONE: fprintf(fp, "NON "); break;
                case PURSUIT: fprintf(fp, "PUR "); break;
                case SACCADE: fprintf(fp, "SAC "); break;
                case INTERRUPT: fprintf(fp, "INT "); break;
                }
            switch( _coord )
                {
                case ABSOLUTE_ANGLE:   fprintf(fp, "ABS "); break;
                case RELATIVE_ANGLE:   fprintf(fp, "REL "); break;
                case NORMALIZED_PIXEL: fprintf(fp, "NOR "); break;
                case IMAGE_PIXEL:      fprintf(fp, "PIX "); break;
                }
            fprintf(fp, "%03.3f %03.3f\n", _command_x, _command_y);
            //report gaze directions
            fprintf(fp, "gaz: %03.3f %03.3f %03.3f %03.3f %03.3f %03.3f\n", _head_azy, _head_elev, _eye_azy, _eye_elev, _targ_azy, _targ_elev);
            //report encoder readings
            fprintf(fp, "enc: %03.3f %03.3f %03.3f %03.3f %03.3f %03.3f\n", headpos[0], headpos[1], headpos[2], headpos[3], headpos[4], headpos[5] );
            //report motor commands
            fprintf(fp, "mot: %03.3f %03.3f %03.3f %03.3f %03.3f %03.3f\n", vels[0], vels[1], vels[2], vels[3], vels[4], vels[5]);
            //report inertial measurements
            fprintf(fp, "imu: %03.3f %03.3f %03.3f\n", gsl_vector_get(inertialW,0), gsl_vector_get(inertialW,1), gsl_vector_get(inertialW,2));
			//report torso velocity measurements
			//fprintf(fp, "tor: %03.3f %03.3f %03.3f\n", gsl_vector_get(torsoW,0), gsl_vector_get(inertialW,1), gsl_vector_get(inertialW,2));
            //report prediction values
            if(USE_PREDICTIVE_CONTROL)
                fprintf(fp, "pre: %03.3f %03.3f %03.3f\n", _prediction[0], _prediction[1], _prediction[2]);
        }
	
	gsl_vector_free( X );
	gsl_vector_free( pert );

	//save last received command for reporting
	if(_cmd != NONE)
        {
            _lastcmd = _cmd;
            _lastcoord = _coord;
            _lastcommand_x = _command_x;
            _lastcommand_y = _command_y;
        }
	
	//finished processing the command - prepare for next cycle
	_cmd = NONE; 
	
	//printf("Just before mutex");
    _mutex.post(); //NOT SURE IF REQUIRED UNCOMMENT OTHERWISE
	//printf("Just after mutex");

    return true;
}


//// converts relative angle of pixel based measurements in absolute angle
//int Control_GazeModule::processposinput(double coord1, double coord2, double *headpos, char type, char behav)
//{	
//    switch( type )
//        {
//        case 'a': _coord = ABSOLUTE_ANGLE; break;
//        case 'r': _coord = RELATIVE_ANGLE; break;
//        case 'p': _coord = NORMALIZED_PIXEL; break;
//        case 'i': _coord = IMAGE_PIXEL; break;
//        }
//
//    switch( behav )
//        {
//        case 's': behav = SACCADE_1; break;
//        case 'S': behav = SACCADE_2; break;
//        case 'p': behav = PURSUIT; break;
//        case 'r': behav = REST; break;
//        }
//
//	desazy = coord1;
//	deselev = coord2;
//
//	if( type == 'p' || type == 'i' )  // camera based coords
//        {		
//            double xmetric, ymetric;
//            //going to represent a direction as a vector (xmetric,ymetric,1)
//            if( type == 'p') // normalized pixels
//                {
//                    // for the case of normalized pixels
//                    double xnorm,ynorm;
//                    xnorm = coord1;  //inputs are in fact normalized pixel coordinates
//                    ynorm = coord2;
//                    _cam.norm2metric(xnorm,ynorm,xmetric,ymetric);
//                }
//            else //type == 'i'
//                {
//                    // for the case of image pixels
//                    double xpix,ypix;
//                    xpix = coord1;  //inputs are in fact normalized pixel coordinates
//                    ypix = coord2;
//                    _cam.pixel2metric(xpix,ypix,xmetric,ymetric);
//                    printf("px: %f, py: %f mx: %f my: %f\n",xpix,ypix,xmetric,ymetric);
//                }
//
//            // Direction is now represented by a vector
//
//            //printf("convertion from camera to absolute\n");
//            RobMatrix T05 = head.fkine( headpos, 'l');
//            //convert error in image (frame 5) to body (frame 0) 
//            RobMatrix des5 = RobMatrix( 1, xmetric, ymetric, 0);
//            RobMatrix desM0 = T05 * des5;
//            gsl_vector *des0 = desM0.getvector(0,3,3);
//            head.gazevector2azyelev( des0, &desazy, &deselev);
//            gsl_vector_free( des0 );
//
//
//
//            //Now for the other eye
//            RobMatrix T05r = head.fkine( headpos, 'r');
//            RobMatrix des5r = RobMatrix( 1, desazy_oe, deselev_oe, 0);
//            RobMatrix desM0r = T05r * des5r;
//            gsl_vector *des0r = desM0r.getvector(0,3,3);
//            head.gazevector2azyelev( des0r, &desazy_oe, &deselev_oe);
//            gsl_vector_free( des0r );
//
//            //now coords are in absolute mode
//            _coord = ABSOLUTE_ANGLE;
//        }   
//    else if( type == 'r' ) // relative angles - this is for sound input
//        {
//            double azy ,elev;
//            head.Gaze( &azy, &elev, headpos);
//
//            // THIS IS NOT RIGHT - MUST CONVERT FROM NECK TO BASE 
//            desazy = desazy + azy;
//            deselev = deselev + elev;
//
//            _coord = ABSOLUTE_ANGLE;
//        
//        }
//
//	currenterror = 1000;
//		
//	cout << "desired gaze" << desazy << ' ' << deselev << endl;
//	
//	egosphere.setSaccadicSuppression( true );
//
//	return 0;
//}

bool Control_GazeModule::updateAbsoluteGazeReference(double coord1, 
													 double coord2, 
													 double *headpos, 
													 double cur_azy,
													 double cur_elev,
													 double &new_azy,
													 double &new_elev,
													 Command cmd, 
													 Coordinate coord)

{
	if(coord == ABSOLUTE_ANGLE)
        {
            new_azy = coord1;
            new_elev = coord2;
        }
	else if(coord == RELATIVE_ANGLE)
        {
            new_azy = cur_azy + coord1;
            new_elev = cur_elev + coord2; 
        }
	else
        {
            double xmetric, ymetric;
            //going to represent a direction as a vector (xmetric,ymetric,1)	
            if(coord == NORMALIZED_PIXEL)  
                {
                    _cam.norm2metric(coord1,coord2,xmetric,ymetric);
                }  
            else if(coord == IMAGE_PIXEL)
                {
                    _cam.pixel2metric(coord1,coord2,xmetric,ymetric);
                }
            // Direction is now represented by a vector
            // TESTING : SHOULD FIND A BETTER WAY TO DO THIS LATTER
            if(cmd == PURSUIT)
                {
                    xmetric *= 1.0;
                    ymetric *= 1.0;
                    //printf("DANGER: EMPIRICAL TESTING MODE\n");
                }


            //printf("conversion from camera to absolute\n");
            RobMatrix T05 = head.fkine( headpos, 'l');
			//T05.print();

            //convert error in image (frame 5) to base (frame 0) 
            RobMatrix des5 = RobMatrix( 1, xmetric, ymetric, 0);
            RobMatrix desM0 = T05 * des5;
            gsl_vector *des0 = desM0.getvector(0,3,3);
            head.gazevector2azyelev( des0, &new_azy, &new_elev);
            gsl_vector_free( des0 );

            //Now for the other eye - DO WE NEED THIS
            /*RobMatrix T05r = head.fkine( headpos, 'r');
            RobMatrix des5r = RobMatrix( 1, desazy_oe, deselev_oe, 0);
            RobMatrix desM0r = T05r * des5r;
            gsl_vector *des0r = desM0r.getvector(0,3,3);
            head.gazevector2azyelev( des0r, &desazy_oe, &deselev_oe);
            gsl_vector_free( des0r );*/
        }
	return true;
}

bool Control_GazeModule::updateAbsoluteGazeReference2(double coord1, 
													 double coord2, 
													 double *headpos, 
													 double *torsopos,
													 double cur_azy,
													 double cur_elev,
													 double &new_azy,
													 double &new_elev,
													 Command cmd, 
													 Coordinate coord)

{
	if(coord == ABSOLUTE_ANGLE)
        {
            new_azy = coord1;
            new_elev = coord2;
        }
	else if(coord == RELATIVE_ANGLE)
        {
            new_azy = cur_azy + coord1;
            new_elev = cur_elev + coord2; 
        }
	else
        {
            double xmetric, ymetric;
            //going to represent a direction as a vector (xmetric,ymetric,1)	
            if(coord == NORMALIZED_PIXEL)  
                {
                    _cam.norm2metric(coord1,coord2,xmetric,ymetric);
                }  
            else if(coord == IMAGE_PIXEL)
                {
                    _cam.pixel2metric(coord1,coord2,xmetric,ymetric);
                }
            // Direction is now represented by a vector
            // TESTING : SHOULD FIND A BETTER WAY TO DO THIS LATTER
            if(cmd == PURSUIT)
                {
                    xmetric *= 1.0;
                    ymetric *= 1.0;
                    //printf("DANGER: EMPIRICAL TESTING MODE\n");
                }


            //printf("conversion from camera to absolute\n");
            /*RobMatrix T05 = head.fkine( headpos, 'l');
			T05.print();

            //convert error in image (frame 5) to base (frame 0) 
            RobMatrix des5 = RobMatrix( 1, xmetric, ymetric, 0);
            RobMatrix desM0 = T05 * des5;
            gsl_vector *des0 = desM0.getvector(0,3,3);
            head.gazevector2azyelev( des0, &new_azy, &new_elev);
            gsl_vector_free( des0 );*/

			Vector cyclopData(8);
			// units shall be in radians
			// remind that the torso is in reverse order:
			// their joints are sent assuming the neck as kinematic origin
			// and not the waist, hence we've got to invert them!
			cyclopData[0]=(M_PI/180.0)*torsopos[2];	
			cyclopData[1]=(M_PI/180.0)*torsopos[1];
			cyclopData[2]=(M_PI/180.0)*torsopos[0];
			// neck part
			cyclopData[3]=(M_PI/180.0)*headpos[0];
			cyclopData[4]=(M_PI/180.0)*headpos[1];
			cyclopData[5]=(M_PI/180.0)*headpos[2];
			// eyes part
			// fbHead[3]=gaze tilt
			// fbHead[4]=gaze version
			// fbHead[5]=gaze vergence
			cyclopData[6] = (M_PI/180.0)*headpos[3];	// eye tilt
			cyclopData[7] = (M_PI/180.0)*cyclopData[4]; // eye version
			Matrix DesPoseEye(4,4);
			DesPoseEye(0,0)= 1; DesPoseEye(0,1)= 0; DesPoseEye(0,2)= 0; DesPoseEye(0,3)= xmetric;
			DesPoseEye(1,0)= 0; DesPoseEye(1,1)= 1; DesPoseEye(1,2)= 0; DesPoseEye(1,3)= ymetric;
			DesPoseEye(2,0)= 0; DesPoseEye(2,1)= 0; DesPoseEye(2,2)= 1; DesPoseEye(2,3)= 1;
			DesPoseEye(3,0)= 0; DesPoseEye(3,1)= 0; DesPoseEye(3,2)= 0; DesPoseEye(3,3)= 0;
			Matrix Eye2Waist = head.getCyclopPose(cyclopData);
			Matrix DesPoseWaist = Eye2Waist * DesPoseEye;
			Vector DesOrient = DesPoseWaist.getCol(3);
			head.gazeVector2AzimuthElevation(DesOrient, new_azy, new_elev);
			/*cout << "DesPoseEye: "<< DesPoseEye.toString() << endl;
			cout << "Eye2Waist: "<< Eye2Waist.toString() << endl;
			cout << "DesPoseWaist: "<< DesPoseWaist.toString() << endl;
			cout << "DesOrient: " << DesOrient.toString() << endl;*/

			new_azy = new_azy*(180/M_PI);
			new_elev = new_elev*(180/M_PI);
        }
	return true;
}

//initiates a position move
//blocks until the eyes reach the final position
int Control_GazeModule::posmove(double *pos)
{
    bool ret = false;
		
    ipos->positionMove( 0, pos[0]);	ipos->setRefSpeed( 0, 200);	
    ipos->positionMove( 1, pos[1]);	ipos->setRefSpeed( 1, 200);
    ipos->positionMove( 2, pos[2]);	ipos->setRefSpeed( 2, 200);	
		
    // HACK FOR JOINT LIMITS -- REMOVED: Produces strange eye behavior
    //pos[3] = saturatevalues( pos[3], 20);
    ipos->positionMove( 3, pos[3]);		
    //pos[4] = saturatevalues( pos[4], 20);		
    ipos->positionMove( 4, pos[4]);	
    // HACK LIMITS
    //ipos->positionMove( 5, pos[4]);	

    ipos->setRefSpeed( 3, 200);
    ipos->setRefSpeed( 4, 200);	
    ipos->setRefSpeed( 5, 200);	

    //Psychologists say eyes and neck start at the same time. 
    //Do not need to way for eyes to finish...
    /*while(!ret)
      {
      printf("checkmotion %f %f; ",pos[3],pos[4]);
      Time::delay(0.01);
      ret = mycheckmotion( 4, pos[4]);	
      ret += mycheckmotion( 3, pos[3]);
      }
      printf("done\n");*/

    return ret;
}

//sets a velocity reference and does velocity control
int Control_GazeModule::velmove(double *vels)
{
    vels[1] = 0; // no swing
    // vels[5] = -2;

	if(_fake_velocity_control) //useful for using the simulator
	{
		relmove(vels);
	}
	else
	{
		for(int i = 0; i < 6; i++ )
			vels[i] *= _velocity_gain;
		ivel->velocityMove( vels );
	}

    return 0;
}

int Control_GazeModule::relmove(double *delta)
{
    bool ret = false;
	double speeds[] = {200,200,200,200,200,200,200,200};
	double newpos[6];

	ipos->setRefSpeeds(speeds);

	for(int i = 0; i < 5; i++)
	{
		newpos[i] = headpos[i]+_fake_velocity_gain*delta[i]/controlrate;
	}

	ipos->positionMove( newpos );

    return ret;
}


/****** INTERFACE FUNCTIONS *********/

bool Control_GazeModule::saccadeAbsolute(double azimuth, double elevation){
    _mutex.wait();
	//Is the state accepting saccades ?
	if( _behav == SACCADE_1 || _behav == SACCADE_2 )
        {
            _mutex.post();
            return false;
        }

	//else
	_cmd = SACCADE;
	_coord = ABSOLUTE_ANGLE;
	_command_x = azimuth;
	_command_y = elevation;
	_mutex.post();
    return true;
}

bool Control_GazeModule::saccadeRelative(double azimuth, double elevation){
    _mutex.wait();
	//Is the state accepting saccades ?
	if( _behav == SACCADE_1 || _behav == SACCADE_2 )
        {
            _mutex.post();
            return false;
        }
	//else
	_cmd = SACCADE;
	_coord = RELATIVE_ANGLE;
	_command_x = azimuth;
	_command_y = elevation;
	_mutex.post();
    return true;
}

bool Control_GazeModule::saccadeImageRef(double pnx, double pny){
    /* added by paulfitz, Wed Aug 15 16:36:43 CEST 2007 */
	/* modified by alex bernardino, Sun Jun 1 */
    _mutex.wait();
    //Is the state accepting saccades ?
	if( _behav == SACCADE_1 || _behav == SACCADE_2 )
        {
            _mutex.post();
            return false;
        }
	//else
	_cmd = SACCADE;
	_coord = NORMALIZED_PIXEL;
	_command_x = pnx;
	_command_y = pny;
	_mutex.post();
    return true;
}

bool Control_GazeModule::saccadePixelRef(double px, double py){
    _mutex.wait();
    //Is the state accepting saccades ?
	if( _behav == SACCADE_1 || _behav == SACCADE_2 )
        {
            _mutex.post();
            return false;
        }
	//else
	_cmd = SACCADE;
	_coord = IMAGE_PIXEL;
	_command_x = px;
	_command_y = py;
	_mutex.post();
    return true;
}

bool Control_GazeModule::pursuitAbsolute(double azimuth, double elevation){
    _mutex.wait();
	//Is the state accepting pursuit commands ?
	if( _behav == SACCADE_1 || _behav == SACCADE_2 )
        {
            _mutex.post();
            return false;
        }
	//else
	_cmd = PURSUIT;
	_coord = ABSOLUTE_ANGLE;
	_command_x = azimuth;
	_command_y = elevation;
	_mutex.post();
    return true;
}

bool Control_GazeModule::pursuitRelative(double azimuth, double elevation){
    _mutex.wait();
	//Is the state accepting pursuit commands ?
	if( _behav == SACCADE_1 || _behav == SACCADE_2 )
        {
            _mutex.post();
            return false;
        }
	//else
	_cmd = PURSUIT;
	_coord = RELATIVE_ANGLE;
	_command_x = azimuth;
	_command_y = elevation;
	_mutex.post();
    return true;
}

bool Control_GazeModule::pursuitImageRef(double pnx, double pny){
    _mutex.wait();
    //Is the state accepting pursuit commands ?
	if( _behav == SACCADE_1 || _behav == SACCADE_2 )
        {
            _mutex.post();
            return false;
        }
	//else
	_cmd = PURSUIT;
	_coord = NORMALIZED_PIXEL;
	_command_x = pnx;
	_command_y = pny;
	_mutex.post();
    return true;
}

bool Control_GazeModule::pursuitPixelRef(double px, double py){
    _mutex.wait();
    //Is the state accepting pursuit commands ?
	if( _behav == SACCADE_1 || _behav == SACCADE_2 )
        {
            _mutex.post();
            return false;
        }
	//else
	_cmd = PURSUIT;
	_coord = IMAGE_PIXEL;
	_command_x = px;
	_command_y = py;
	_mutex.post();
    return true;
}

bool Control_GazeModule::pursuitInterrupt(){
    _mutex.wait();
    //Are we in pursuit state ?
	if( _behav != CONTINUE_PURSUIT )
        {
            _mutex.post();
            return false;
        }
	//else
	_cmd = SACCADE;
	_coord = NORMALIZED_PIXEL;
	_command_x = 0.0;
	_command_y = 0.0;
	_mutex.post();
    return true;
}

bool Control_GazeModule::reset(){
    _mutex.wait();
    // TODO this method might want to reset some internal states/data
    if( _behav == SACCADE_1 || _behav == SACCADE_2 )
        {
            _mutex.post();
            return false;
        }
	//else
	_cmd = SACCADE;
	_coord = ABSOLUTE_ANGLE;
	_command_x = 0.0;
	_command_y = 0.0;
    _mutex.post();
    return true;
}

bool Control_GazeModule::help(){
	cout << "help : this command" << endl;
    cout << endl << "-- ACTUATION COMMANDS" << endl;
	cout << "sac abs <x> <y> : saccade in absolute angles" << endl;
	cout << "set pos <x> <y> : same as before" << endl;
	cout << "sac rel <x> <y> : saccade in head relative angles" << endl;
	cout << "sac img <x> <y> : saccade in camera normalized relative coordinastes" << endl;
	cout << "sac pix <x> <y> : saccade in image pixel relative coordinates" << endl;
	cout << "pur abs <x> <y> : pursuit in absolute angles" << endl;
	cout << "pur rel <x> <y> : pursuit in head relative angles" << endl;
	cout << "pur img <x> <y> : pursuit in camera normalized relative coordinastes" << endl;
	cout << "pur pix <x> <y> : pursuit in image pixel relative coordinates" << endl;
    cout << endl << "-- STATUS COMMANDS" << endl;
    cout << "rset            : resets the state of the controller" << endl;
	cout << "get st          : get time from last saccade" << endl;
	cout << "get stat        : get controller status" << endl;
	cout << "get ref         : get current gaze reference" << endl;
	cout << "get dh          : get current head gaze" << endl;
	cout << "get der         : get current right eye gaze" << endl;
	cout << "get del         : get current left eye gaze" << endl;
    cout << endl << "-- CONFIGURATION COMMANDS" << endl;
    cout << "set mst         : set minimum saccade time (a.k.a head saccade delay)" << endl;
    cout << "set lrt         : ser joint limit reset time" << endl;
    cout << "get mst         : get minimum saccade time (a.k.a head saccade delay)" << endl;
    cout << "get lrt         : get joint limit reset time" << endl;
    cout << "set verg <gain> : set vergence gain" << endl;
	cout << "get verg <gain> : get vergence gain" << endl;

	return true;
}

bool Control_GazeModule::getControllerStatus(string &status){
    _mutex.wait();
    switch( _behav ) 
        {
        case SACCADE_1:			 status = "sacc eyes"; break;
        case SACCADE_2:			 status = "sacc head"; break;
		case START_PURSUIT:		 status = "purs init"; break;
		case CONTINUE_PURSUIT:	 status = "purs cont"; break;
		case LIMIT:				 status = "limit";  break;
        case REST:				 status = "rest"; break;
        }
	_mutex.post();
    return true;
}

bool Control_GazeModule::getSaccadeTime(double &time){
    // TODO take into account end time to calculate actual saccading time
	_mutex.wait();
    time = Time::now() - timesaccadeid;
	_mutex.post();
    return true;
}

bool Control_GazeModule::getReference(double &azimuth, double &elevation){
	_mutex.wait();
    azimuth = desazy;
    elevation = deselev;
	_mutex.post();
    return true;
}

bool Control_GazeModule::getDirectionEyeRight(double &azimuth, double &elevation){
    // TODO add here right eye if available at some point...
	_mutex.wait();
    azimuth = _eye_azy;
    elevation = _eye_elev;
	_mutex.post();
    return true;
}

bool Control_GazeModule::getDirectionEyeLeft(double &azimuth, double &elevation){
    // TODO add here left eye if available at some point...
	_mutex.wait();
    azimuth = _eye_azy;
    elevation = _eye_elev;
	_mutex.post();
    return true;
}

bool Control_GazeModule::getDirectionHead(double &azimuth, double &elevation){
	_mutex.wait();
    azimuth = _head_azy;
    elevation = _head_elev;
	_mutex.post();
    return true;
}

bool Control_GazeModule::getMinSaccadeTime(double &time ){
	_mutex.wait();
    time = _headSaccadeDelay;
    _mutex.post();
    return true;
}

bool Control_GazeModule::getLimitResetTime(double &time ){
	_mutex.wait();
    time = _limitResetTime;
    _mutex.post();
    return true;
}


bool Control_GazeModule::setLimitResetTime(double time ){
    bool ret = true;
	_mutex.wait();
    if(time >= 0.0) {
        _limitResetTime = time;
        ret = false;
    }
    _mutex.post();
    return ret;
}

bool Control_GazeModule::setMinSaccadeTime(double time ){
    bool ret = true;
	_mutex.wait();
    if(time >= 0.0) {
        _headSaccadeDelay = time;
        ret = true;
    }
	else{
		ret = false;
	}
    _mutex.post();
    return ret;
}
