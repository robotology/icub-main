// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Manuel Lopes
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */


// capitalization problem on linux
#include <iCub/control_gaze.h>

using namespace std;



using namespace iCub::contrib;

int Control_GazeModule::mycheckmotion( int axis, double target)
{
double curr;
double min, max;

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
}

Control_GazeModule::~Control_GazeModule(){

}


bool Control_GazeModule::open(Searchable& config){
   
    if (config.check("help","if present, display usage message")) {
        printf("Call with --name /module_prefix --file configFile.ini --group CONFIGURATION_GROUP --motorboard\n");
        return false;
    }

    /* Added by Alex 20/7/2007 */
    _cam.open(config);
    /* End Addition ************/
    

    ConstString str = config.check("motorboard","/controlboard","Name of the control board").asString();
    // check default --group options e.g. [EGO_SPHERE]
    Bottle botConfig(config.toString().c_str());
    botConfig.setMonitor(config.getMonitor());
    // is group option present?
    Value *valGroup; // check assigns pointer to reference (no delete required (?))
    if(config.check("group", valGroup, "Configuration group to load module options from (string).")){
        string strGroup = valGroup->asString().c_str();
        // is group a valid bottle?
        if (!config.findGroup(strGroup.c_str()).isNull()){
            botConfig.clear();
            botConfig.fromString(config.findGroup(strGroup.c_str(), string("Loading configuration from group " + strGroup).c_str()).toString().c_str());
        }
        else{
            cout << endl << "Group " << strGroup << " not found." << endl;
            return false;
        }
    }

	controlType = visON;
	int a = botConfig.check("pidON", 0, "use integral term in tracker?").asInt();

	controlType = controlType | (pidON * botConfig.check("pidON", 0, "use integral term in tracker?").asInt());
	double pidGAIN = botConfig.check("pidGAIN", 5.0, "integral gain").asDouble();

	head.setintgain( pidGAIN);

	controlType = controlType | (vorON * botConfig.check("vorON", 0, "use inertial sensor").asInt());
	
	dolog = botConfig.check("log", 0, "write controller log information? (0/1)").asInt();

    // added JR 070723 (to control look back to center hack)
    _limitResetTime = botConfig.check("limitResetTime",
                                    Value(4.0),
                                    "Time from start of saccade until look back to center if saccade reaches a joint limit (double).").asDouble();

    // used a reference here - otherwise check for null doesn't work
    // (cannot copy a null bottle)
	framerate = botConfig.check("FrameRate",
                                    Value(20.0),
                                    "FrameRate").asDouble();
	
	head.setGazeControllerGain(  framerate );

	Bottle& K = botConfig.findGroup("K", "controller gain");

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
		botConfig.check("egosphereVisualUpdateThreshold", 1, "threshold for ending saccade").asDouble();

	//	head.setGazeControllerGain(gsl_matrix *K)
	Property propBoard;


	propBoard.put("device", "remote_controlboard");
	propBoard.put("remote" , str);
	cout << str << endl;
	propBoard.put("local", getName("controlboard"));
	dd.open(propBoard);
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

	_smoothInput_port.open( getName("vel"));
	_saccadeInput_port.open( getName("pos"));

	_inertialInput_port.open( getName("imu"));

	_disparityInput_port.open( getName("dis"));

	_trackersignalOutput_port.open( getName("trackersignal/bot:o"));

	_posdirOutput_port.open( getName("possibledirections/vec:o") );

    // open config port
    _configPort.open(getName("conf"));
    attach(_configPort, true);

	
	desazy = 0;	deselev = 0;
	desazy_oe = 0;	deselev_oe = 0;
	currenterror=0;

	saccadeid = 0;

	targtype = 'a';
	behavior = 's';
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

	if( !egosphere.open(getName("/remoteEgoSphere")))
	{
		cout << "Could not connect to the egosphere" << endl;
		egosphcom = false;

	}
	else
		egosphcom = true;


	if(dolog)
		fp = fopen("log.txt","w");

	ncycles = 0;
    timesaccadeid = 0;
	start = Time::now();
   

	cout << "finished opening" << endl;

	iamp->enableAmp(0);
	iamp->enableAmp(1);
	iamp->enableAmp(2);
	iamp->enableAmp(3);
	iamp->enableAmp(4);
	iamp->enableAmp(5);

	ipid->enablePid(0);
	ipid->enablePid(1);
	ipid->enablePid(2);
	ipid->enablePid(3);
	ipid->enablePid(4);
	ipid->enablePid(5);

	ipos->positionMove( 0, 0);
	ipos->positionMove( 1, 0);
	ipos->positionMove( 2, 0);
	ipos->positionMove( 3, 0);
	ipos->positionMove( 4, 0);	
	ipos->positionMove( 5, 0);	

	ivel->velocityMove( 0, 0);
	ivel->velocityMove( 1, 0);
	ivel->velocityMove( 2, 0);
	ivel->velocityMove( 3, 0);
	ivel->velocityMove( 4, 0);
	ivel->velocityMove( 5, 0);

	ivel->setRefAcceleration(0, 1000);
	ivel->setRefAcceleration(1, 1000);
	ivel->setRefAcceleration(2, 1000);
	ivel->setRefAcceleration(3, 1000);
	ivel->setRefAcceleration(4, 1000);
	ivel->setRefAcceleration(5, 1000);

	ipos->setRefSpeed(0, 200);
	ipos->setRefSpeed(1, 200);
	ipos->setRefSpeed(2, 200);
	ipos->setRefSpeed(3, 200);
	ipos->setRefSpeed(4, 200);
	ipos->setRefSpeed(5, 200);

	ipos->setRefAcceleration(0, 1000);
	ipos->setRefAcceleration(1, 1000);
	ipos->setRefAcceleration(2, 1000);
	ipos->setRefAcceleration(3, 1000);
	ipos->setRefAcceleration(4, 1000);
	ipos->setRefAcceleration(5, 1000);

	




    return true;
}

bool Control_GazeModule::close(){
printf("closing Control_Gaze\n");

    if (_numAxes > 0){
	    ipos->positionMove( 0, 0);
	    ipos->positionMove( 1, 0);
	    ipos->positionMove( 2, 0);
	    ipos->positionMove( 3, 0);
	    ipos->positionMove( 4, 0);	
	    ipos->positionMove( 5, 0);	
    }
printf("1\n");
	double vels[6];
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
	_disparityInput_port.close();
printf("4\n");
	_inertialInput_port.close();
    _configPort.close();
	_trackersignalOutput_port.close();
	_posdirOutput_port.close();
    egosphere.close();
printf("5\n");
	if(dolog)
		fclose( fp );
printf("6\n");
	gsl_vector_free( oldW );
	gsl_vector_free( inertialW );
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
	_disparityInput_port.interrupt();
    _posdirOutput_port.interrupt(); 
	
	_mutex.post();
	*/
    return true;
}

bool Control_GazeModule::respond(const Bottle &command,Bottle &reply){
	
    bool rec = false; // recognized
    bool ok = false; // command executed successfully
    double az, el;  // get azimuth/elevation
    string status;  // controller status
    double stime; // saccade time

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
        default:
            break;
        }   
        break;
	case CONTROLGAZE_VOCAB_SET:
        switch(command.get(1).asVocab()) {
		case CONTROLGAZE_VOCAB_POS:
            rec = true;
            ok = saccadeAbsolute(command.get(2).asDouble(), command.get(3).asDouble());
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
            ok = this->getSaccadeTime(stime);
            if (ok){
                reply.addDouble(stime);
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
		default:
            break;
		}
        break;
    case CONTROLGAZE_VOCAB_RESET:
        rec = true;
        ok = reset();
        if (!ok) reply.addVocab(VOCAB_FAILED);
        else reply.addVocab(VOCAB_OK);
        break;
	default:
        break;
	}
	
    if (!rec)
        ok = Module::respond(command,reply); // will add message 'not recognized' if not recognized
	return ok;
}


bool Control_GazeModule::updateModule(){

    if (_numAxes == 0){
        cout << "Controlboard does not provide any axis. Idle..." << endl;
        yarp::os::Time::delay(1);
        return true;
    }

    _mutex.wait(); //NOT SURE IF REQUIRED UNCOMMENT OTHERWISE
	//printf("Control_GazeModule::updateModule()\n");

    double vels[] = { 0, 0, 0, 0, 0, 0};
    double disparity;

    //cout << "update starts now..." << endl;
	
	// read from the port
	Vector *posinput = _saccadeInput_port.read( false );
	Vector *velinput = _smoothInput_port.read( false );

	Vector *dispinput = _disparityInput_port.read( false );
	Vector *inertialmeas = _inertialInput_port.read( false );

	double t2 = Time::now();

	if( dispinput )
		disparity = (*posinput)(0);
	else
		disparity = 0;


	ncycles++;
	ienc->getEncoders( headpos );
	//printf("time read %f\n", Time::now() - t2);
	
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

	if( currenterror < egosphereVisualUpdateThreshold )
	{
		if( egosphcom == true )
			egosphere.setSaccadicSuppression( false );
	}

	if (posinput)
		cout << "posinput" << ' ' << (*posinput)(0) << ' ' << (*posinput)(1) << ' ' << getLabel((*posinput)(2)) << endl ;

	gsl_vector* pert;	pert = gsl_vector_calloc( 2 );
	gsl_vector* X;	X = gsl_vector_calloc( 3 );


	_updateTime = Time::now();

	{	// see free direction spacial saliency
		
		Vector &vec = _posdirOutput_port.prepare();
		vec.resize(16);
		head.pred( headpos, vec.data(), 16 ); 

		_posdirOutput_port.write();

	}

	// hack for joint limit attention problem
    if (timesaccadeid == 0)
        timesaccadeid = Time::now();
	if( behavior == 'S')
		if( (timesaccadeid + _limitResetTime) < _updateTime)
		{
			timesaccadeid = Time::now();
			desazy = 0;
			deselev = 0;
			currenterror = 1000;
			targtype = 'a';
			behavior = 's';
		}

	printf("behavior %c type %c \n", behavior, targtype);
	switch( behavior ) {

		case 's':
			behavior = 'S';
			head.setGazeControllerGain(  framerate );		

			currenterror = head.HeadSaccade( desazy, deselev, vels, headpos, currenterror);
			
            timesaccadeid = Time::now(); 

			posmove( vels );
			
			if(1) {
				// inform the tracker about the new target
				printf("SENDING RESET TO TRACKER\n");
				Bottle &bot = _trackersignalOutput_port.prepare();
				bot.addVocab( Vocab::encode("SET") );
				bot.addVocab( Vocab::encode("POS") );    
				_trackersignalOutput_port.write();
			}
			
			break;

		case 'p':
			head.setGazeControllerGain(  framerate/5 );
			currenterror = head.HeadGazeController(desazy, deselev, X, pert, oldW, vels, headpos, inertialW, disparity,controlType);

			if( velinput )
				processposinput( velinput, headpos, 'p' );
			if( posinput )
			{
                /* Added by Alex 24/7/2007 */
                char coord_id = getLabel((*posinput)(2));
                if( (coord_id != 'a' ) &&  //absolute angle
                    (coord_id != 'r' ) &&  //relative angle
                    (coord_id != 'p' )     //norm pixel (relative)
                    ) 
                {
                    cout << "Invalid argument in position command mode: " << coord_id << endl;
                    return 0;
                }
                /* END Added by Alex 24/7/2007 */

				int	tempid = (int) (*posinput)(4);
				if(tempid > saccadeid)
				{
                    /* Changed by Alex 24/7/2007 */
					//processposinput( posinput, headpos, 'a' );
                    processposinput( posinput, headpos, coord_id );
                    behavior = 's';
                    /* END Changed by Alex 24/7/2007 */
					saccadeid = tempid;
				}				
			}

			//if(pospixel) 				processposinput( pospixel, headpos, 'p' );

			velmove(vels);
			break;

		case 'S':
			currenterror = head.HeadGazeController(desazy, deselev, X, pert, oldW, vels, headpos, inertialW, 0,controlType);
			if( (currenterror < egosphereVisualUpdateThreshold) )
			{
				printf(" END SACCADE\n");
				behavior = 'p';
			}

			// with this the neck phase of the saccade is interruptable
			if( posinput )
			{
                /* Added by Alex 24/7/2007 */
                char coord_id = getLabel((*posinput)(2));
                if( (coord_id != 'a' ) &&  //absolute angle
                    (coord_id != 'r' ) &&  //relative angle
                    (coord_id != 'p' )     //norm pixel (relative)
                    ) 
                {
                    cout << "Invalid argument in position command mode: " << coord_id << endl;
                    return 0;
                }
                /* END Added by Alex 24/7/2007 */
				int	tempid = (int) (*posinput)(4);

				if( (tempid < (saccadeid-10) ) || (tempid > saccadeid))
				{
                    /* Changed by Alex 24/7/2007 */
					//processposinput( posinput, headpos, 'a' );
                    processposinput( posinput, headpos, coord_id );
                    behavior = 's';
                    /* END Changed by Alex 24/7/2007 */
					saccadeid = tempid;
					timesaccadeid = Time::now(); // hack for joint limit attention problem
				}	
			}

			velmove(vels);

			break;

		//case 'P':			break;

		case 'r':
			gsl_vector_set_zero( oldW );

			if( velinput )
				processposinput( velinput, headpos, 'p' );
			if( posinput )
				processposinput( posinput, headpos, 'a' );

			velmove(vels);

			break;

		default:
			printf(" ERROR STATE \n");
	}







		

	//printf("time write %f\n", Time::now() - _updateTime);

	double cycpersec =  ncycles / (Time::now() - start);
	
	printf("\t  %f fps %f seconds %f current error\n", cycpersec, 1/cycpersec, currenterror);

	// LOG
	
	head.HeadGaze( &_neck_azy, &_neck_elev, headpos);
	head.Gaze( &_eye_azy, &_eye_elev, headpos);

	printf( "headpos > %2.2f %2.2f %2.2f %2.2f %2.2f \n ", headpos[0], headpos[1], headpos[2], headpos[3], headpos[4]);

	printf("vels > %2.2f %2.2f %2.2f %2.2f %2.2f %2.2f \n ", vels[0], vels[1], vels[2], vels[3], vels[4], vels[5]);

	double a1,a2;
	a1 = a2 = -999;
	if(posinput)
	{
		a1 = (*posinput)(0);
		a2 = (*posinput)(1);
	}
	printf( "desired>%2.2f %2.2f %2.2f %2.2f %2.2f %2.2f %2.2f %2.2f\n", desazy, _neck_azy, _eye_azy, deselev, _neck_elev, _eye_elev, a1, a2);
	
	if( dolog )
	{
		fprintf(fp, "%f %f %f %f %f %f 0 ", Time::now(), headpos[0], headpos[1], headpos[2], headpos[3], headpos[4]);
		fprintf(fp, "%f %f %f %f %f %f 0 ", vels[0], vels[1], vels[2], vels[3], vels[4], vels[5]);
		fprintf(fp, "%f %f %f %f %f %f %f %f 0 ", desazy, _neck_azy, _eye_azy, deselev, _neck_elev, _eye_elev, a1, a2);
		if(!inertialmeas)
			fprintf(fp, "99 99 99 99 99 99 99 99 99 99 99 99 \n");
		else
			fprintf(fp, "%f %f %f %f %f %f %f %f %f %f %f %f \n", (*inertialmeas)(0), (*inertialmeas)(1),(*inertialmeas)(2),
																  (*inertialmeas)(3),(*inertialmeas)(4),(*inertialmeas)(5),
																  (*inertialmeas)(6),(*inertialmeas)(7),(*inertialmeas)(8),
																  (*inertialmeas)(9),(*inertialmeas)(10),(*inertialmeas)(11));
	}
	
	gsl_vector_free( X );
	gsl_vector_free( pert );

    _mutex.post(); //NOT SURE IF REQUIRED UNCOMMENT OTHERWISE

    return true;
}


int Control_GazeModule::processposinput(Vector *input, double *headpos, char type)
{
		
	targtype = getLabel ((*input)(2));
	behavior = getLabel ((*input)(3));

	desazy = (*input)(0);
	deselev = (*input)(1);

	if( type == 'p') // normalized pixels
	{
		// for the case of normalized pixels
        /* Added by Alex 20/7/2007 */
        double x,y;
        x = desazy;  //inputs are in fact normalized pixel coordinates
        y = deselev;
        _cam.norm2metric(x,y,desazy,deselev);
        /* End Addition ************/

        /* Changed by Alex 24/7/2007 */
		//if( (targtype == 'r'))
		//{
        // Input in pixel coordinates is always relative 
        /* END Changed by Alex 24/7/2007 */
		
		printf(" convertion from r to a\n");
		RobMatrix T05 = head.fkine( headpos, 'l');
		//convert error in image (frame 5) to frame 0 
		RobMatrix des5 = RobMatrix( 1, desazy, deselev,	0);
		RobMatrix desM0 = T05 * des5;
		gsl_vector *des0 = desM0.getvector(0,3,3);
		head.gazevector2azyelev( des0, &desazy, &deselev);
		gsl_vector_free( des0 );

		RobMatrix T05r = head.fkine( headpos, 'r');
		RobMatrix des5r = RobMatrix( 1, desazy_oe, deselev_oe, 0);
		RobMatrix desM0r = T05r * des5r;
		gsl_vector *des0r = desM0r.getvector(0,3,3);
		head.gazevector2azyelev( des0r, &desazy_oe, &deselev_oe);
		gsl_vector_free( des0r );

		targtype = 'a';

        /* Changed by Alex 24/7/2007 */
		//} 
        /*    else if( (targtype == 'r'))
		    {
			// convert pixels to angles
		    }*/
        // Input in pixel coordinates is always relative 
        /* END Changed by Alex 24/7/2007 */
		
		
	}
    /* Changed by Alex 24/7/2007 */
	//else if( type == 'a' ) // angles
    else if( type == 'r' ) // relative angles
    /* END Changed by Alex 24/7/2007 */
	{
		// for the case of azimuth elevation
        /* Changed by Alex 24/7/2007 */
		//if( (targtype == 'r') )
		//{
        /* END Changed by Alex 24/7/2007 */
		double azy ,elev;
		head.Gaze( &azy, &elev, headpos);

		desazy = desazy + azy;
		deselev = deselev + elev;

		targtype = 'a';
        /* Changed by Alex 24/7/2007 */
		//}
        /* END Changed by Alex 24/7/2007 */
	}

		if(desazy>50)
			desazy = 50;
		if(desazy<-50)
			desazy = -50;
		
		if(deselev>50)
			deselev = 50;
		if(deselev<-50)
			deselev = -50;

		currenterror = 1000;
		
		cout << "desired gaze" << desazy << ' ' << deselev << endl;
		if( egosphcom == true )
			egosphere.setSaccadicSuppression( true );
	
		return 0;
}



int Control_GazeModule::posmove(double *pos)
{
		bool ret = false;
		
		ipos->positionMove( 0, pos[0]);	ipos->setRefSpeed( 0, 200);	
		ipos->positionMove( 1, pos[1]);	ipos->setRefSpeed( 1, 200);
		ipos->positionMove( 2, pos[2]);	ipos->setRefSpeed( 2, 200);	
		
		// HACK FOR JOINT LIMITS
		pos[3] = saturatevalues( pos[3], 20);
		ipos->positionMove( 3, pos[3]);		
		pos[4] = saturatevalues( pos[4], 20);		
		ipos->positionMove( 4, pos[4]);	
		// HACK LIMITS
		//ipos->positionMove( 5, pos[4]);	

		ipos->setRefSpeed( 3, 200);
		ipos->setRefSpeed( 4, 200);	
		ipos->setRefSpeed( 5, 200);	

		while(!ret)
		{
			printf("checkmotion %f %f\n",pos[3],pos[4]);
			Time::delay(0.01);
			ret = mycheckmotion( 4, pos[4]);	
			ret += mycheckmotion( 3, pos[3]);
		}
		printf("done\n");

		return ret;
}

int Control_GazeModule::velmove(double *vels)
{
		vels[1] = 0; // no swing
		//vels[4] = 0;

		ivel->velocityMove( vels );

		return 0;
}



bool Control_GazeModule::saccadeAbsolute(double azimuth, double elevation){
    _mutex.wait();
	desazy = azimuth;
	desazy_oe = azimuth; // other eye
	deselev = elevation;
	deselev_oe = elevation; // other eye
	currenterror = 1000;
	targtype = 'a';
	behavior = 's';
    _mutex.post();
    return true;
}

bool Control_GazeModule::saccadeRelative(double azimuth, double elevation){
    _mutex.wait();
	cout << "Control_GazeModule::saccadeRelative() not implemented" << endl;
    _mutex.post();
    return false;
}

bool Control_GazeModule::saccadeImageRef(double pnx, double pny){
    /* added by paulfitz, Wed Aug 15 16:36:43 CEST 2007 */
    _mutex.wait();
    Vector v(4);
    v(0) = pnx;
    v(1) = pny;
    v(2) = 'r'; // this is dodgy stuff! --paulfitz
    v(3) = 'p'; // this is dodgy stuff! --paulfitz
	targtype = 'p';
	behavior = 'p';
    processposinput(&v,headpos,'p');
    _mutex.post();
    printf("This doesn't actually work right :(\n");
    return false;
}

bool Control_GazeModule::reset(){
    _mutex.wait();
    // TODO this method might want to reset some internal states/data
    saccadeAbsolute(0.0,0.0);
    _mutex.post();
    return true;
}

bool Control_GazeModule::getControllerStatus(string &status){
    status = behavior; // string = char
    return true;
}

bool Control_GazeModule::getSaccadeTime(double &time){
    // TODO take into account end time to calculate actual saccading time
    time = Time::now() - timesaccadeid;
    return true;
}

bool Control_GazeModule::getReference(double &azimuth, double &elevation){
    azimuth = desazy;
    elevation = deselev;
    return true;
}

bool Control_GazeModule::getDirectionEyeRight(double &azimuth, double &elevation){
    // TODO add here right eye if available at some point...
    azimuth = _eye_azy;
    elevation = _eye_elev;
    return true;
}

bool Control_GazeModule::getDirectionEyeLeft(double &azimuth, double &elevation){
    // TODO add here left eye if available at some point...
    azimuth = _eye_azy;
    elevation = _eye_elev;
    return true;
}

bool Control_GazeModule::getDirectionHead(double &azimuth, double &elevation){
    azimuth = _neck_azy;
    elevation = _neck_elev;
    return true;
}
