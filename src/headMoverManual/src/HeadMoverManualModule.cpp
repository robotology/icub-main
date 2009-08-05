// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iCub/HeadMoverManualModule.h>

HeadMoverManualModule::HeadMoverManualModule(){
	_encoders = NULL;
	_ienc = NULL;
	_ipos = NULL;
}

HeadMoverManualModule::~HeadMoverManualModule(){ 
}


bool HeadMoverManualModule::open(Searchable& config){
   
    if (config.check("help","if present, display usage message")) {
        printf("Call with --name /module_prefix --remote /server_controlboard/port --file configFile.ini\n");
        return false;
    }

	// read key map
	key_neck_til_u = config.check("key_neck_til_u", Value((int)'2'), "key code: neck tilt up").asInt();
	key_neck_til_d = config.check("key_neck_til_d", Value((int)'5'), "key code: neck tilt down").asInt();
	key_neck_swi_r = config.check("key_neck_swi_r", Value((int)'3'), "key code: neck swing right").asInt();
	key_neck_swi_l = config.check("key_neck_swi_l", Value((int)'1'), "key code: neck swing left").asInt();
	key_neck_pan_r = config.check("key_neck_pan_r", Value((int)'6'), "key code: neck pan right").asInt();
	key_neck_pan_l = config.check("key_neck_pan_l", Value((int)'4'), "key code: neck pan left").asInt();
	key_eyes_til_u = config.check("key_eyes_til_u", Value((int)'k'), "key code: eyes tilt up").asInt();
	key_eyes_til_d = config.check("key_eyes_til_d", Value((int)'i'), "key code: eyes tilt down").asInt();
	key_eyes_ax4_r = config.check("key_eyes_ax4_r", Value((int)'l'), "key code: eyes axis 4 right").asInt();
	key_eyes_ax4_l = config.check("key_eyes_ax4_l", Value((int)'j'), "key code: eyes axis 4 left").asInt();
	key_eyes_ax5_r = config.check("key_eyes_ax5_r", Value((int)'o'), "key code: eyes axis 5 right").asInt();
	key_eyes_ax5_l = config.check("key_eyes_ax5_l", Value((int)'u'), "key code: eyes axis 5 left").asInt();
	key_quit = config.check("key_quit", Value((int)'q'), "key code: quit module").asInt();

	// read more configuration values
	step_neck = config.check("step_neck", Value(5.0), "step size neck joints (double degrees)").asDouble();
	step_eyes = config.check("step_eyes", Value(5.0), "step size eye joints (double degrees)").asDouble();
	refVelocityNeck = config.check("refVelocityNeck", Value(5.0), "reference velocity for neck joints (double)").asDouble();
	refAccelerationNeck = config.check("refAccelerationNeck", Value(10.0), "reference acceleration for neck joints (double)").asDouble();
	refVelocityEyes = config.check("refVelocityEyes", Value(5.0), "reference velocity for eye joints (double)").asDouble();
	refAccelerationEyes = config.check("refAccelerationEyes", Value(10.0), "reference acceleration for eye joints (double)").asDouble();


	// open controlboard
    Value *valControlboard;
    if(!config.check("remote",
                     valControlboard,
                     "Port name of the server controlboard to connect to (string).")){
        cout << endl;                         
        cout << "Please specify the port name of the server controlboard to connect to (--remote option)." << endl;
        return false;
    }
    Bottle botControlboard(string(
                           string("(device remote_controlboard) ") +
                           string("(local ") + string(this->getName("controlboard").c_str()) + string(") ") +
                           string("(remote ") + string(valControlboard->asString().c_str()) + string(") ")
                           ).c_str());
	if(!_dd.open(botControlboard)){
		cout << "Failed to open remote controlboard. Check if server controlboard available and port name passed correctly in --remote option." << endl;
		return false;
	}
    if(!_dd.view(_ienc)){
        cout << "Motor controlboard does not implement IEncoders!" << endl;
        return false;
    }
    if(!_dd.view(_ipos)){
        cout << "Motor controlboard does not implement IPositionControl!" << endl;
        return false;
    }
	if(!_dd.view(_iamp)){
		cout << "No amplifier.. " << endl;
		return false;
	}
	if(!_dd.view(_ipid)){
		cout << "No pid.. " << endl;
		return false;
	}
    _numAxes = 0;
    _ienc->getAxes(&_numAxes);
    if (_numAxes == 0){
        cout << "Controlboard opened correctly but provides 0 axes. Aborting..." << endl;
		return false;
    }
    else if (_numAxes < 6){
        cout << "Number of motor axes outside expected range (available: " << _numAxes << " required 6)." << endl;
        return false;
    }
    else{
        _encoders = new double[_numAxes];
    }
	for (int i = 0; i < 3; i++){
		_iamp->enableAmp(i);
		_ipid->enablePid(i);
		_ipos->setRefSpeed(i, refVelocityNeck);
		_ipos->setRefAcceleration(i, refAccelerationNeck);
		_ipos->positionMove( i, 0);	
	}
	for (int i = 3; i < 6; i++){
		_iamp->enableAmp(i);
		_ipid->enablePid(i);
		_ipos->setRefSpeed(i, refVelocityEyes);
		_ipos->setRefAcceleration(i, refAccelerationEyes);
		_ipos->positionMove( i, 0);	
	}

	// print help
	cout << endl;
	cout << "****************** Keys *******************" << endl;
	cout << "neck tilt up:\t\t '" << (char)key_neck_til_u << "' ascii code " << key_neck_til_u << endl;
	cout << "neck tilt down:\t\t '" << (char)key_neck_til_d << "' ascii code " << key_neck_til_d << endl;
	cout << "neck swing right:\t '" << (char)key_neck_swi_r << "' ascii code " << key_neck_swi_r << endl;
	cout << "neck swing left:\t '" << (char)key_neck_swi_l << "' ascii code " << key_neck_swi_l << endl;
	cout << "neck pan right:\t\t '" << (char)key_neck_pan_r << "' ascii code " << key_neck_pan_r << endl;
	cout << "neck pan left:\t\t '" << (char)key_neck_pan_l << "' ascii code " << key_neck_pan_l << endl;
	cout << "eyes tilt up:\t\t '" << (char)key_eyes_til_u << "' ascii code " << key_eyes_til_u << endl;
	cout << "eyes tilt down:\t\t '" << (char)key_eyes_til_d << "' ascii code " << key_eyes_til_d << endl;
	cout << "eyes axis 4 right:\t '" << (char)key_eyes_ax4_r << "' ascii code " << key_eyes_ax4_r << endl;
	cout << "eyes axis 4 left:\t '" << (char)key_eyes_ax4_l << "' ascii code " << key_eyes_ax4_l << endl;
	cout << "eyes axis 5 right:\t '" << (char)key_eyes_ax5_r << "' ascii code " << key_eyes_ax5_r << endl;
	cout << "eyes axis 5 left:\t '" << (char)key_eyes_ax5_l << "' ascii code " << key_eyes_ax5_l << endl;
	cout << "exit/quit:\t\t '" << (char)key_quit << "' ascii code " << key_quit << endl;
	cout << "*******************************************" << endl;
	cout << endl;
	//cout << "The key map is defined in $ICUB_DIR/conf/headMoverManual.ini" << endl;
	//cout << endl;

    return true;
}

bool HeadMoverManualModule::close(){
	_dd.close();
    if (_encoders != NULL)
        delete [] _encoders;
    _encoders = NULL;
    return true;
}

bool HeadMoverManualModule::interruptModule(){
    
    return true;
}

bool HeadMoverManualModule::updateModule(){

	_ienc->getEncoders(_encoders);
	//for (int i = 0; i < _numAxes; i++)
	//	cout << _encoders[i] << "\t";
	//cout << endl;

	if(kbhit()){
        int ascii = (int)getch();
		//cout << "hit key: " << (char)ascii << " code: " << ascii << endl;
		if (ascii == key_quit){
			return false;
		}
		else if(ascii == key_neck_til_u){
			_ipos->positionMove( 0, _encoders[0]+step_neck);
			//cout << "neck tilt up" << endl;
		}
		else if(ascii == key_neck_til_d){
			_ipos->positionMove(0, _encoders[0]-step_neck);
			//cout << "neck tilt down" << endl;
		}
		else if(ascii == key_neck_swi_r){
			_ipos->positionMove(1, _encoders[1]+step_neck);
			//cout << "neck swing right" << endl;
		}
		else if(ascii == key_neck_swi_l){
			_ipos->positionMove(1, _encoders[1]-step_neck);
			//cout << "neck swing left" << endl;
		}
		else if(ascii == key_neck_pan_r){
			_ipos->positionMove(2, _encoders[2]-step_neck);
			//cout << "neck pan right" << endl;
		}
		else if(ascii == key_neck_pan_l){
			_ipos->positionMove(2, _encoders[2]+step_neck);
			//cout << "neck pan left" << endl;
		}
		else if(ascii == key_eyes_til_u){
			_ipos->positionMove(3, _encoders[3]+step_neck);
			//cout << "eyes tilt up" << endl;
		}
		else if(ascii == key_eyes_til_d){
			_ipos->positionMove(3, _encoders[3]-step_neck);
			//cout << "eyes tilt down" << endl;
		}
		else if(ascii == key_eyes_ax4_r){
			_ipos->positionMove(4, _encoders[4]+step_neck);
			//cout << "eyes axis 4 right" << endl;
		}
		else if(ascii == key_eyes_ax4_l){
			_ipos->positionMove(4, _encoders[4]-step_neck);
			//cout << "eyes axis 4 left" << endl;
		}
		else if(ascii == key_eyes_ax5_r){
			_ipos->positionMove(5, _encoders[5]+step_neck);
			//cout << "eyes axis 5 right" << endl;
		}
		else if(ascii == key_eyes_ax5_l){
			_ipos->positionMove(5, _encoders[5]-step_neck);
			//cout << "eyes axis 5 left" << endl;
		}
		else{
			cout << "no function assigned to key '" << (char)ascii << "'" << " code " << ascii << endl;
		}
	}
	yarp::os::Time::delay(0.1);
    return true;
}
