// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
*	Copyright (C) 2008 Boris Duran
*	Control module for iCub's head motors.
*	
*	Information is expected from:
*		- a 4-dimensional vector with a 2 pairs of position-magnitud of 
*			both stimuli coming from a port labeled '/abTracker/data:o'
*		- an image coming from a port labeled '/abTracker/img:o'
*/

#include "ctrlModule.h"

CTRLmodule::CTRLmodule(){
	U = ippsMalloc_64f(dim_head);
	S = ippsMalloc_64f(dim_head);
	head_enc = NULL;
	head_pos = NULL;
	head_vel = NULL;
	head_pid = NULL;
	head_amp = NULL;
}

CTRLmodule::~CTRLmodule(){ 
	// memory release
	ippsFree( U );
	ippsFree( S );
}

bool CTRLmodule::open( Searchable& config ){
    if (config.check("help","if present, display usage message")) {
        printf("Call with --name /module_prefix --file dftFile.ini\n");
        return false;
    }
	// read more configuration values
	gain_eyes_yaw = (Ipp64f) config.check("gainEyesYaw", Value(70),
		"Gain for yaw motors in eyes").asDouble();
	gain_eyes_pitch = (Ipp64f) config.check("gainEyesPitch", Value(40.0), 
		"Gain for pitch motor in eyes").asDouble();
	gain_neck_yaw = (Ipp64f) config.check("gainNeckYaw", Value(40.0),
		"Gain for yaw motor in neck").asDouble();
	gain_neck_pitch = (Ipp64f) config.check("gainNeckPitch", Value(20.0), 
		"Gain for pitch motor in neck").asDouble();
	verb = config.check("verbose");
	if(verb)
		file_m = fopen("posMotors.txt","w");	// Saves motor positions

	// Data port
	inPort.open(getName("data:i"));
	Network::connect("/boris/dft/data:o", getName("data:i"), "udp");

	// Configuration
	Property conf_head;
	conf_head.put("device", "remote_controlboard");
	conf_head.put("remote", "/icub/head" );//remote_head.c_str());
	conf_head.put("local", "/icub/head/control" );//local_head.c_str());

	//Driver Creation
	head_driver.open(conf_head);
	if (!head_driver.isValid()){
		cout << "Device not available.  Here are the known devices:" << endl;
        cout << Drivers::factory().toString().c_str() << endl;
        //Network::fini();
		return false;
	}
	bool ok;
    ok = head_driver.view(head_pos);
    ok &= head_driver.view(head_vel);
    ok &= head_driver.view(head_enc);
    ok &= head_driver.view(head_pid);
    ok &= head_driver.view(head_amp);
    if(!ok){ cout << "Problems acquiring interfaces" << endl; return false; }

	int nAxes = 0;
	head_pos->getAxes(&nAxes);
	if(dim_head != nAxes){
		cout << "Problems detecting all axes:" << endl;
		cout << "Detected " << nAxes << " axes" << endl;
		return false;
	}

	ippsZero_64f( U, dim_head );
	ippsZero_64f( S, dim_head );

	k = 0;

	setZeroPosition();
	setZeroVelocity();

    return true;
}

bool CTRLmodule::close(){
	if(verb)
		fclose(file_m);

	while(fabs(S[0])>1.0 || fabs(S[2])>1.0 || 
		fabs(S[3])>1.0 || fabs(S[4])>1.0 || fabs(S[5])>1.0){
		setZeroPosition();
		Time::delay(0.1);
		head_enc->getEncoders((double*)S);
	}
	for(int j=0; j<dim_head; j++){
		head_amp->disableAmp(j);
		head_pid->disablePid(j);
	}
	head_driver.close();

	inPort.close();

    return true;
}

bool CTRLmodule::interruptModule(){
   	inPort.interrupt();
 
    return true;
}

bool CTRLmodule::updateModule(){

	head_enc->getEncoders((double*)S);

	ippsZero_64f( U, dim_head );
	Vector *v = inPort.read(); // Read from the port.
	if(k > 100 && v != NULL ){
//		cout << (*v)[0] << "\t" << (*v)[1] << "\t" << (*v)[2] << endl;
		//Test function:	10*sin(2*IPP_PI*k/200);
		U[5] = -gain_eyes_yaw*(*v)[0];	//nU[7];	// 5: Yaw Left Eye
		U[4] = gain_eyes_yaw*(*v)[1];	//nU[8];	// 4: Yaw Right Eye 
		U[3] = 0.0;	//nU[9];	// 3: Pitch Eyes 
		U[2] = gain_neck_yaw*(*v)[2];	//nU[10];	// 2: Yaw Neck
		U[1] = 0.0;	// 1: Roll Neck
		U[0] = 0.0;	//nU[11];	// 0: Pitch Neck
		if( fabs(S[2])>2.0 && (*v)[2]==0.0){
			head_pos->positionMove(2,0.0);
			Time::delay(0.1);
		}
	}

	for( int j=0; j<dim_head; j++)
		head_vel->velocityMove(j, U[j]);

	k = k + 1;

	// ------------------- Saving data... -------------------
	if(verb){
		for(int i = 0; i<dim_head; i++)
			fprintf(file_m, "%2.8f\t",	S[i]);
		fprintf(file_m, "\n");
	}

    return true;
}


bool CTRLmodule::setZeroPosition(){
	for(int j=0; j<dim_head; j++){
		head_pos->setRefSpeed(j, 40.0);
		head_amp->enableAmp(j);
		head_pid->enablePid(j);
		head_pos->positionMove(j, 0.0);
		Time::delay(0.1);
	}

	return true;
}
bool CTRLmodule::setZeroVelocity(){
	for(int j=0; j<dim_head; j++)
		head_pos->setRefAcceleration(j, 60.0);

	return true;
}