// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch, Alexandre Bernardino, Dario Figueira
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#include <iCub/EgoSphereModule.h>

EgoSphereModule::EgoSphereModule(){

    _encoders = NULL;
	_azREye = 0.0;
	_elREye = 0.0;
	_azLEye = 0.0;
	_elLEye = 0.0;
	_azHead = 0.0;
	_elHead = 0.0;
	_gazeREyeX = 0;
    _gazeREyeY = 0;
	_gazeLEyeX = 0;
	_gazeLEyeY = 0;
	_rotREye_W2C = new double[9]; // 1x9
	_rotLEye_W2C = new double[9]; // 1x9
	_rotHead_W2C = new double[9]; // 1x9
	_rotREye_C2W = new double[9]; // 1x9
	_rotLEye_C2W = new double[9]; // 1x9
	_rotHead_C2W = new double[9]; // 1x9
	_gazeREye = new double[3]; // 1x3
	_gazeLEye = new double[3]; // 1x3
	_gazeHead = new double[3]; // 1x3
    _framerate.init(50);
}

EgoSphereModule::~EgoSphereModule(){
   
	delete [] _rotREye_W2C;
	_rotREye_W2C = NULL;
	delete [] _rotLEye_W2C;
	_rotLEye_W2C = NULL;
	delete [] _rotHead_W2C;
	_rotHead_W2C = NULL;
	delete [] _rotREye_C2W;
	_rotREye_C2W = NULL;
	delete [] _rotLEye_C2W;
	_rotLEye_C2W = NULL;
	delete [] _rotHead_C2W;
	_rotHead_C2W = NULL;
	delete [] _gazeREye;
	_gazeREye = NULL;
	delete [] _gazeLEye;
	_gazeLEye = NULL;
	delete [] _gazeHead;
	_gazeHead = NULL;
}

bool EgoSphereModule::open(Searchable& config){

	// locate configuration file
    ResourceFinder rf;        
	if (config.check("context")){
        rf.setDefaultContext(config.find("context").asString());
	}
	if (config.check("from")){
        rf.setDefaultConfigFile(config.find("from").asString());
	}
	else if (config.check("file")){
        rf.setDefaultConfigFile(config.find("file").asString());
	}
	else{
        rf.setDefaultConfigFile("icubEgoSphere.ini");
	}
    rf.configure("ICUB_ROOT",0,NULL);
	Property prop(rf.toString());
	prop.fromString(config.toString(), false);
	prop.setMonitor(config.getMonitor());

    // check for toplevel configuration options (not contained in configuration group)
    // name of the server controlboard to connect to
	_strControlboard = prop.check("controlboard",
                                    Value("not_available"),
                                    "Port name of the server controlboard to connect to (string).").asString().c_str();
	if(_strControlboard == std::string("not_available")){
		_blnControlboard = false;
		std::cout << endl << "***No server controlboard specified, encoder values will be fixed for zero azimuth and elevation.***" << endl << endl;
	}
	else{
		_blnControlboard = true;
	}

    // Options from the EGO_SPHERE configuration group
    Bottle botConfig(prop.toString().c_str());
    botConfig.setMonitor(prop.getMonitor());
    if (!prop.findGroup("EGO_SPHERE").isNull()){ // read options from group [EGO_SPHERE] or if not present directly from file
        botConfig.clear();
        botConfig.fromString(prop.findGroup("EGO_SPHERE", "Loading configuration from group EGO_SPHERE.").toString());
    }
    _egoImgSize.width = botConfig.check("resx",
                                        Value(800),
                                        "Width of ego-sphere image (int)").asInt();
    _egoImgSize.height = botConfig.check("resy",
                                        Value(600),
                                        "Height of ego-sphere image (int)").asInt();
    _blnSaccadicSuppression = (bool)botConfig.check("saccadicSuppression",
                                        Value(0),
                                        "Inhibit projection of visual input into sphere and writing of output map (int [0|1]).").asInt();
    _refreshTime = botConfig.check("refreshTime",
                                    Value(0.05),
                                    "Target refresh time of the module update loop (double in sec.).").asDouble();
    _thresholdSalience = (float)botConfig.check("thresholdSalience",
                                    Value(20.0),
                                    "Threshold applied when updating salience map (double).").asDouble();
    _activateIOR = (bool)botConfig.check("activateIOR",
                                    Value(1),
                                    "Activate inhibition of return (int [0|1]).").asInt();
    _activateModAuditory = (bool)botConfig.check("activateModAuditory",
                                    Value(0),
                                    "Activate auditory channel (int [0|1]).").asInt();
    _activateModVision = (bool)botConfig.check("activateModVision",
                                    Value(1),
                                    "Activate visual channel (int [0|1]).").asInt();
    _activateModObjects = (bool)botConfig.check("activateModObjects",
                                    Value(0),
                                    "Activate object channel (int [0|1]).").asInt();

	// open remote controlboard
    if(_blnControlboard){        
		Bottle botControlboard(std::string(
			std::string("(device remote_controlboard) ") +
			std::string("(local ") + std::string(this->getName("controlboard").c_str()) + std::string(") ") +
			std::string("(remote ") + _strControlboard + std::string(") ")).c_str());

        _dd.open(botControlboard);
        if(!_dd.view(_ienc)){
            cout << "Motor controlboard does not implement IEncoders!" << endl;
            return false;
        }
        if(!_dd.view(_ipos)){
            cout << "Motor controlboard does not implement IPositionControl!" << endl;
            return false;
        }
        _numAxes = 0;
        _ienc->getAxes(&_numAxes);
        if (_numAxes == 0){
            cout << "Error retrieving number of axes from server controlboard at " << _strControlboard << endl;
            return false;
        }
        else if (_numAxes < 6){
            cout << "Number of motor axes outside expected range (available: " << _numAxes << " required 6)." << endl;
            return false;
        }
        else{
            _encoders = new double[_numAxes];
        }
    }
    else{
        _numAxes=6;
        _encoders = new double[_numAxes];
		for (int i = 0; i < 6; i++){
            _encoders[i] = 0.0;
		}
    }   

    // Visual Map
	if(_activateModVision){
		IModalityMap *map = new VisualMap();
		if(map->open(prop)){
			_vctMap.push_back(map);
		}
		else{
			cout << "Error opening visual modality map! Removing map..." << endl;
			delete map;
			map = NULL;
		}		
	}

    // Objects Map 
	if(_activateModObjects){
		IModalityMap *map = new ObjectMap();
		if(map->open(prop)){
			_vctMap.push_back(map);
		}
		else{
			cout << "Error opening object map! Removing map..." << endl;
			delete map;
			map = NULL;
		}		
	}

    // AcousticMap
    if(_activateModAuditory){
		IModalityMap *map = new AcousticMap();
		if(map->open(prop)){
			_vctMap.push_back(map);
		}
		else{
			cout << "Error opening auditory modality map! Removing map..." << endl;
			delete map;
			map = NULL;
		}		
	}

    // IOR
    if (_activateIOR){
        if(!_ior.open(prop)){ // will look for [IOR] group itself
            cout << "Error opening IOR! Inhibition of return will be turned off..." << endl;
            _activateIOR = false;
        }
    }

    // open output map port
    _prtImgEgoFloat.open(getName("map_out"));

    // open config port
    _configPort.open(getName("conf"));
    attach(_configPort, true);
 
    Time::turboBoost();
    return true;
}

bool EgoSphereModule::close(){
        
	// close main map port
    _prtImgEgoFloat.close();

	// close configuration port
    _configPort.close();

	// close IOR
    _ior.close();

	// close and delete maps
	while(_vctMap.size() > 0){
		IModalityMap *map = _vctMap.back();
		map->close();
		delete map;
		map = NULL;
		_vctMap.pop_back();
	}
    
	// close controlboard
    _dd.close();
    if (_encoders != NULL)
        delete [] _encoders;
    _encoders = NULL;

    return true;
}

bool EgoSphereModule::interruptModule(){
    
	for(int i = 0; i < (int)_vctMap.size(); i++){
		_vctMap[i]->interrupt();
	}
    _prtImgEgoFloat.interrupt();
    _configPort.interrupt();    
    return true;
}

bool EgoSphereModule::reset(){
    _ior.reset();
	for(int i = 0; i < (int)_vctMap.size(); i++){
		_vctMap[i]->reset();
	}
    if (_yrpImgFloatEgo.width() > 0 && _yrpImgFloatEgo.height() > 0)
        cvSet(((IplImage*)_yrpImgFloatEgo.getIplImage()), cvScalar(0.0f));
	std::cout << "***reset***" << std::endl;
    return true;
}

bool EgoSphereModule::updateModule(){

    _framerate.addStartTime(yarp::os::Time::now());

    double timeStart = Time::now();

    // map read
	for(int i = 0; i < (int)_vctMap.size(); i++){
		_vctMap[i]->read();
	}
    
	// read encoders
    if(_blnControlboard){
		_ienc->getEncoders(_encoders); 
    } // else encoders are set to 0
	// convert from vergence/common direction to indepentent eye positions
	double aux1,aux2;
	aux1 = _encoders[4];
	aux2 = _encoders[5];
	_encoders[5] = aux1 + aux2/2.0;
	_encoders[4] = aux1 - aux2/2.0;

    _semaphore.wait(); // do not move before (quit not possible if waiting for input)

	// init/clear ego-sphere map if size changed (and on start-up)
    _yrpImgFloatEgo.resize(_egoImgSize.width, _egoImgSize.height); // nothing done if ok already
    if (_yrpImgFloatEgo.width() != _egoImgSize.width || _yrpImgFloatEgo.height() != _egoImgSize.height){
        cvSet(((IplImage*)_yrpImgFloatEgo.getIplImage()), cvScalar(0.0f));
    }

    // map decay
	for(int i = 0; i < (int)_vctMap.size(); i++){
		_vctMap[i]->updateDecay();
	}

    // kinematics
	_rEyeMatrix = _headKin.fkine(_encoders, 'r'); // 'r' -> use right eye kinematics
	_lEyeMatrix = _headKin.fkine(_encoders, 'l'); // 'l' -> use right eye kinematics
	//std::cout << "right" << std::endl; _rEyeMatrix.print();
	//std::cout << "left" << std::endl; _lEyeMatrix.print();
	_headMatrix = _headKin.fkine(_encoders, 3); // 3-> use third joint from neck base
	convertRobMatrix(_rEyeMatrix, _rotREye_W2C); // robmatrix to world to camera
	convertRobMatrix(_lEyeMatrix, _rotLEye_W2C); // robmatrix to world to camera
	convertRobMatrix(_headMatrix, _rotHead_W2C); // robmatrix to world to camera
	transpose(_rotREye_W2C, _rotREye_C2W); // world to camera to camera to world
	transpose(_rotLEye_W2C, _rotLEye_C2W); // world to camera to camera to world
	transpose(_rotHead_W2C, _rotHead_C2W); // world to camera to camera to world
	calcGazeVector(_rotREye_C2W, _gazeREye); // matrix to gaze vector
	calcGazeVector(_rotLEye_C2W, _gazeLEye); // matrix to gaze vector
	calcGazeVector(_rotHead_C2W, _gazeHead); // matrix to gaze vector
    calcAngles(_gazeREye, _azREye, _elREye); // gaze vector to azimuth/elevation
	calcAngles(_gazeLEye, _azLEye, _elLEye); // gaze vector to azimuth/elevation
	calcAngles(_gazeHead, _azHead, _elHead); // gaze vector to azimuth/elevation
	calcGazePixel(_azREye, _elREye, _gazeREyeX, _gazeREyeY); // elevation/azimuth to egoSphere-pixels
	calcGazePixel(_azLEye, _elLEye, _gazeLEyeX, _gazeLEyeY); // elevation/azimuth to egoSphere-pixels
	calcGazePixel(_azHead, _elHead, _gazeHeadX, _gazeHeadY); // elevation/azimuth to egoSphere-pixels

	// process observation
	for(int i = 0; i < (int)_vctMap.size(); i++){
		_vctMap[i]->process(_azREye, _elREye, _gazeREyeX, _gazeREyeY, _rotREye_W2C,
							_azLEye, _elLEye, _gazeLEyeX, _gazeLEyeY, _rotLEye_W2C,
							_azHead, _elHead, _gazeHeadX, _gazeHeadY, _rotHead_W2C, 
							_blnSaccadicSuppression);
	}

	// merge modalities
	_yrpImgFloatEgo.zero(); // (decay is handled by each map individually)
	for(int i = 0; i < (int)_vctMap.size(); i++){
		ImageOf<PixelFloat>& yrpImgFloatModality = _vctMap[i]->getSalienceMap(_egoImgSize.width,_egoImgSize.height);
		cvMax(	_yrpImgFloatEgo.getIplImage(), 
				 yrpImgFloatModality.getIplImage(), 
				_yrpImgFloatEgo.getIplImage());
	}

    // inhibition of return
    if (_activateIOR){
        _ior.applyIOR(_yrpImgFloatEgo);
        _ior.updateIOR(_gazeREyeX, _gazeREyeY);
    }

    // threshold on final map
	if (_thresholdSalience > 0.0){
        cvThreshold(_yrpImgFloatEgo.getIplImage(), _yrpImgFloatEgo.getIplImage(), _thresholdSalience, 0.0, CV_THRESH_TOZERO);
	}

	// write final map to yarp port
    if (!_blnSaccadicSuppression){
        // prepare output images
        ImageOf<PixelFloat>& yrpImgFloatOut = _prtImgEgoFloat.prepare();
        // resize (nothing is done if size is correct)
        yrpImgFloatOut.resize(_egoImgSize.width, _egoImgSize.height);
        // copy ego img to out img 
        yrpImgFloatOut.copy(_yrpImgFloatEgo); // this is required because port manages a _set_ of images returned by prepare() / we need to be sure to paint always on the same image yrpImgFloatEgo
		// write
        _prtImgEgoFloat.write();
    }
	// let individual maps write any additional data (e.g. visualization)
	for(int i = 0; i < (int)_vctMap.size(); i++){
		_vctMap[i]->write(_blnSaccadicSuppression);
	}

    _semaphore.post();

    // wait if update rate too high
    double timeEnd = Time::now();
    if ((timeEnd - timeStart) < _refreshTime)
        Time::delay(_refreshTime - (timeEnd - timeStart));

    _framerate.addEndTime(yarp::os::Time::now());
    if(_framerate.hasNewFramerate()){
        cout << _framerate.getFramerate() << " fps" << endl; 
    }

    return true;
}

bool EgoSphereModule::setThresholdSalience(float thr){
    _thresholdSalience = thr;
    return true;
}

float EgoSphereModule::getThresholdSalience(){
    return _thresholdSalience;
}

bool EgoSphereModule::setSaccadicSuppression(bool on){
    _blnSaccadicSuppression = on;
	std::cout << "Setting saccadic suppression: " << on << std::endl;
    return true;
}

bool EgoSphereModule::getSaccadicSuppression(){
    return _blnSaccadicSuppression;
}

bool EgoSphereModule::setSalienceDecay(double rate){
    /*_mapVisual.setSalienceDecay(rate);
    _mapAcoustic.setSalienceDecay(rate);*/
	cout << "EgoSphereModule::setSalienceDecay() not implemented!" << endl;
    return true;
}

double EgoSphereModule::getSalienceDecay(){
    cout << "EgoSphereModule::getSalienceDecay() not implemented!" << endl;
    return -99.0;
}

bool EgoSphereModule::addIORRegion(double azimuth, double elevation){
    int x,y;
    calcGazePixel(-azimuth, -elevation, x, y); // takes care of limits
    _ior.addIORRegion(x,y);
	std::cout << "Added IOR region at: " << x << " " << y << endl;
    return true;
}

bool EgoSphereModule::respond(const Bottle &command,Bottle &reply){
        
    bool ok = false;
    bool rec = false; // is the command recognized?

    _semaphore.wait();
    switch (command.get(0).asVocab()) {
	case EGOSPHERE_VOCAB_HELP:
		rec = true;
		{
			reply.addString("help");

			reply.addString("\n");
			reply.addString("rset \t: reset the ego-sphere");
			reply.addString("get op \t: query for module output on/off");
			reply.addString("get sd \t: get salience decay rate");
			reply.addString("get ts \t: get salience threshold");
			reply.addString("get ma \t: query for auditory map on/off");
			reply.addString("get mv \t: query for visual map on/off");
			reply.addString("get mo \t: query for object map on/off");
			reply.addString("get ior \t: query for inhibition of return on/off");
		
			reply.addString("\n");
			reply.addString("set op <b> \t: set module output on/off ((bool)b[1/0])");
			reply.addString("set sd <d> \t: set salience decay rate ((double)d[0.0-1.0])");
			reply.addString("set ts <i> \t: set salience threshold ((int)i[0-255])");
			reply.addString("set ma <b> \t: set auditory map on/off ((bool)b[1/0])");
			reply.addString("set mv <b> \t: set visual map on/off ((bool)b[1/0])");
			reply.addString("set mo <b> \t: set object map on/off ((bool)b[1/0])");
			reply.addString("air <da> <de> \t: add an inhibition blob at position azimuth (da), elevation (de) ((double)da[-180.0-180.0]) ((double)de[-90.0-90.0])");

			ok = true;
		}
		break;
    case EGOSPHERE_VOCAB_RESET:
        rec = true;
        {
            ok = reset();
        }
        break;
    case EGOSPHERE_VOCAB_ADDIORREGION:
        rec = true;
        {
            if(command.size() < 3){
                ok = false;
            }
            else{
                double az = command.get(1).asDouble();
                double el = command.get(2).asDouble();
                ok = this->addIORRegion(az, el);
            }
        }
        break;
    case EGOSPHERE_VOCAB_SET:
        rec = true;
        {
            switch(command.get(1).asVocab()) {
            case EGOSPHERE_VOCAB_THRESHOLD_SALIENCE:{
                float thr = (float)command.get(2).asDouble();
                ok = setThresholdSalience(thr);
                break;
            }
            case EGOSPHERE_VOCAB_SACCADICSUPPRESSION:{
                bool on = (bool)command.get(2).asInt();
                ok = setSaccadicSuppression(on);
                break;
            }
            case EGOSPHERE_VOCAB_SALIENCE_DECAY:{
                double rate = command.get(2).asDouble();
                ok = setSalienceDecay(rate);
                break;
            }
            case EGOSPHERE_VOCAB_IOR:{
                _activateIOR = (bool)command.get(2).asInt();
				std::cout << "Setting IOR: " << _activateIOR << std::endl;
                ok = true;
                break;
            }
            case EGOSPHERE_VOCAB_MODALITY_AUDITORY:{
                _activateModAuditory = (bool)command.get(2).asInt();
                ok = true;
                break;
            }
            case EGOSPHERE_VOCAB_MODALITY_VISION:{
                _activateModVision = (bool)command.get(2).asInt();
                ok = true;
                break;
            }
            case EGOSPHERE_VOCAB_MODALITY_OBJECTS:{
                _activateModObjects = (bool)command.get(2).asInt();
                ok = true;
                break;
            }
            default:
                cout << "received an unknown request after a VOCAB_SET" << endl;
                break;
            }
        }
        break;
    case EGOSPHERE_VOCAB_GET:
        rec = true;
        {
            reply.addVocab(VOCAB_IS);
            reply.add(command.get(1));
            switch(command.get(1).asVocab()) {
            case EGOSPHERE_VOCAB_THRESHOLD_SALIENCE:{
                float thr = getThresholdSalience();
                reply.addDouble((double)thr);
                ok = true;
                break;
            }
            case EGOSPHERE_VOCAB_SACCADICSUPPRESSION:{
                int v = (int)getSaccadicSuppression();
                reply.addInt(v);
                ok = true;
                break;
            }
            case EGOSPHERE_VOCAB_SALIENCE_DECAY:{
                double rate = getSalienceDecay();
                reply.addDouble(rate);
                ok = true;
                break;
            }
            case EGOSPHERE_VOCAB_IOR:{
                reply.addInt((int)_activateIOR);
                ok = true;
                break;
            }
            case EGOSPHERE_VOCAB_MODALITY_AUDITORY:{
                reply.addInt((int)_activateModAuditory);
                ok = true;
                break;
            }
            case EGOSPHERE_VOCAB_MODALITY_VISION:{
                reply.addInt((int)_activateModVision);
                ok = true;
                break;
            }
            case EGOSPHERE_VOCAB_MODALITY_OBJECTS:{
                reply.addInt((int)_activateModObjects);
                ok = true;
                break;
            }

            break;
            default:
                cout << "received an unknown request after a VOCAB_GET" << endl;
                break;
            }
        }
        break;

    }
    _semaphore.post();

    if (!rec)
        ok = Module::respond(command,reply);
    
    if (!ok) {
        reply.clear();
        reply.addVocab(VOCAB_FAILED);
    }
    else
        reply.addVocab(VOCAB_OK);

    return ok;
}     

// helper function to pass correct matrix to projectors
void EgoSphereModule::convertRobMatrix(RobMatrix &robMatrix, double *matrix){
    double tmp[9];
    for (int b = 0; b < 3; b++)
        for (int a =0; a < 3; a++)
            tmp[b*3 + a] = robMatrix.M[b][a];
    // convert to 'intuitive' reference frame (axis switching)
    matrix[0] = -tmp[4];
    matrix[1] = -tmp[7];
    matrix[2] = tmp[1];
    matrix[3] = -tmp[5];
    matrix[4] = -tmp[8];
    matrix[5] = tmp[2];
    matrix[6] = -tmp[3];
    matrix[7] = -tmp[6];
    matrix[8] = tmp[0];
}

void EgoSphereModule::calcGazeVector(double *rotCamera2World, double *gaze){
    double vct[3]; 
    vct[0] = 0.0; vct[1] = 0.0; vct[2] = 1.0; // 1 0 0
    gaze[0] = rotCamera2World[0] * vct[0] + rotCamera2World[1] * vct[1] + rotCamera2World[2] * vct[2];
    gaze[1] = rotCamera2World[3] * vct[0] + rotCamera2World[4] * vct[1] + rotCamera2World[5] * vct[2];
    gaze[2] = rotCamera2World[6] * vct[0] + rotCamera2World[7] * vct[1] + rotCamera2World[8] * vct[2];  
}

void EgoSphereModule::calcAngles(double *gaze, double &azimuth, double &elevation) {
   // THIS IS AN IMPLEMENTATION OF ATAN2, FOR OUR CASE
   if( gaze[0] > 0.0 )
      azimuth = -atan(gaze[1]/gaze[0]) * 180.0 / M_PI;
   else if (gaze[0] < 0.0)
       if (gaze[1] > 0.0)
           azimuth = -180.0 - atan(gaze[1]/gaze[0]) * 180.0 / M_PI;
       else
           azimuth = 180.0 - atan(gaze[1]/gaze[0]) * 180.0 / M_PI;
   else // gaze[0]== 0
       if (gaze[1] > 0.0)
           azimuth = -90.0;
       else if (gaze[1] < 0.0)
           azimuth = 90.0;
       else // gaze[1]==0
           azimuth = 0.0; //INDETERMINATE CASE - SHOULD NEVER HAPPEN

    elevation = -atan(gaze[2]/sqrt(pow(gaze[0],2) + pow(gaze[1],2))) * 180 / M_PI;
}

void EgoSphereModule::calcGazePixel(double azimuth, double elevation, int &gazeX, int &gazeY){
    gazeX = (int)( ((double)(_egoImgSize.width))/360.0 * azimuth + ((double)(_egoImgSize.width))/2.0);
    gazeY = (int)( ((double)(_egoImgSize.height))/180.0 * elevation + ((double)(_egoImgSize.height))/2.0);
    if (gazeX < 0){
        cout << "EgoSphereModule::calcGazePixel(): " << gazeX << "!" << endl;
        gazeX = 0;
    }
    if (gazeY < 0){
        cout << "EgoSphereModule::calcGazePixel(): " << gazeY << "!" << endl;
        gazeY = 0;
    }
    if (gazeX >= _egoImgSize.width){
        cout << "EgoSphereModule::calcGazePixel(): " << gazeX << "!" << endl;
        gazeX = _egoImgSize.width - 1;
    }
    if (gazeY >= _egoImgSize.height){
        cout << "EgoSphereModule::calcGazePixel(): " << gazeY << "!" << endl;
        gazeY = _egoImgSize.height - 1;
    }
}


//void EgoSphereModule::getPeak(ImageOf<PixelFloat> &img, int &i, int &j, float &v){
//    //v = -FLT_MAX;
//    v = 0.0f;
//    i = img.width()/2;
//    j = img.height()/2;
//    IplImage *image = (IplImage*)img.getIplImage();
//    float *data = (float*)image->imageData;
//    for (int y = 0; y < image->height; y++){
//        for (int x = 0; x < image->width; x++){
//            if (v < *data){
//                v = *data;
//                i = x;
//                j = y;
//            }
//            data++;
//        }
//    }
//}

