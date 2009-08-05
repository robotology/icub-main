// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iCub/VisualMap.h>
// #include <highgui.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace iCub::contrib;

template <class T>
T portable_min(T x, T y) {
    return (x<y)?x:y;
}

VisualMap::VisualMap(){
	_imgFloatIn = NULL;
	_imgRgbIn = NULL;
	_botClick = NULL;
	_drawGazeDirectionREye = false;
	_drawGazeDirectionLEye = false;
	_gazePixREye = PixelRgb(0,255,0);
	_gazePixLEye = PixelRgb(255,0,0);
	_clickPix = PixelFloat(255.0);
	_clickPoint.x = -100;
	_clickPoint.y = -100;
}

VisualMap::~VisualMap(){

}

bool VisualMap::open(Searchable& config){

	bool ok = true;

	if(!config.check("name")){
		std::cout << "VisualMap: Error, module base name not found in configuration." << std::endl;
		return false;
	}

	// module base name
	std::string strModuleName = std::string(config.find("name").asString().c_str());

	// look for group EGO_SPHERE
    Bottle botConfig(config.toString().c_str());
    botConfig.setMonitor(config.getMonitor());
    if (!config.findGroup("EGO_SPHERE").isNull()){ // read options from group [EGO_SPHERE] or if not present directly from file
        botConfig.clear();
        botConfig.fromString(config.findGroup("EGO_SPHERE", "Loading configuration from group EGO_SPHERE.").toString());
    }
    _egoImgSize.width = botConfig.check("resx",
                                        Value(800),
                                        "Width of ego-sphere image (int)").asInt();
    _egoImgSize.height = botConfig.check("resy",
                                        Value(600),
                                        "Height of ego-sphere image (int)").asInt();

    // look for group EGO_SPHERE_VISUAL_MAP
    Bottle botConfigVisual(config.toString().c_str());
    botConfigVisual.setMonitor(config.getMonitor());
    if (!config.findGroup("EGO_SPHERE_VISUAL_MAP").isNull()){
        botConfigVisual.clear();
        botConfigVisual.fromString(config.findGroup("EGO_SPHERE_VISUAL_MAP", "Loading visual map configuration  from group EGO_SPHERE_VISUAL_MAP.").toString());
    }

    // ego_sphere_visual_map configuration
    _salienceDecayRate = botConfigVisual.check("decayVisual",
                                      Value(0.95),
                                      "Decay for the acoustic saliency map (double).").asDouble();
    _resXVisual = botConfigVisual.check("resXVisual",
                                Value(80),
                                "Width of internal visual map (int)").asInt();
    _resYVisual = botConfigVisual.check("resYVisual",
                                Value(60),
                                "Height of internal visual map (int)").asInt();
	std::string _strUseEye = botConfigVisual.check("useEye",
								Value("right"),
								"Input image originates from which eye string['right'/'left']").asString().c_str();
	if(_strUseEye == std::string("right")){
		_useRightEye = true;
	}
	else{
		_useRightEye = false;
	}
	_drawGazeDirectionREye = (bool)botConfigVisual.check("drawGazeDirectionRightEye",
                                Value(0),
                                "Draw a crosshair at the currently attended location in the RGB visualization of the sphere (int [0|1]).").asInt();
	_drawGazeDirectionLEye = (bool)botConfigVisual.check("drawGazeDirectionLeftEye",
                                Value(0),
                                "Draw a crosshair at the currently attended location in the RGB visualization of the sphere (int [0|1]).").asInt();
    _imgRemapX.resize(_resXVisual,_resYVisual);
    _imgRemapY.resize(_resXVisual,_resYVisual);
    _imgMapResA.resize(_resXVisual,_resYVisual);
    _imgMapResARgb.resize(_resXVisual, _resYVisual);
    // set to 0
    cvSet(((IplImage*)_imgMapResA.getIplImage()), cvScalar(0.0f));
    cvSet(((IplImage*)_imgMapResARgb.getIplImage()), cvScalar(0.0f));

    // camera calibration configuration
    string strCalibGroup = botConfigVisual.check("cameraCalibration",
                                        Value("CAMERA_CALIBRATION"),
                                        "Configuration group to read camera calibration values from (string).").asString().c_str();
    Bottle botConfigCamCalib;
    if (!config.findGroup(strCalibGroup.c_str()).isNull()){
        botConfigCamCalib.clear();
        botConfigCamCalib.fromString(config.findGroup(strCalibGroup.c_str(), string(string("Loading camera calibration configuration from group ") + strCalibGroup).c_str()).toString());
    }
    else {
        cout << "VisualMap::open() Camera calibration configuration group " << strCalibGroup << " not found!" << endl;
        return false;
    }
    // Defaults will correspond to a view field of 90 deg.
    _calibImgSize.width = botConfigCamCalib.check("w",
                                      Value(320),
                                      "Image width for which calibration parameters were calculated (int)").asInt();
    _calibImgSize.height = botConfigCamCalib.check("h",
                                      Value(240),
                                      "Image height for which calibration parameters were calculated (int)").asInt();
    _fx = botConfigCamCalib.check("fx", Value(320.0), "Focal distance (on horizontal pixel size units) (double)").asDouble();
    _fy = botConfigCamCalib.check("fy", Value(240.0), "Focal distance (on vertical pixel size units) (double)").asDouble();
    _cx = botConfigCamCalib.check("cx", Value(320.0), "Image center (on horizontal pixel size units) (double)").asDouble();
    _cy = botConfigCamCalib.check("cy", Value(240.0), "Image center (on vertical pixel size units) (double)").asDouble();
    _k1 = botConfigCamCalib.check("k1", Value(0.0), "Radial distortion (first parameter) (double)").asDouble();
    _k2 = botConfigCamCalib.check("k2", Value(0.0), "Radial distortion (second parameter) (double)").asDouble();
    _p1 = botConfigCamCalib.check("p1", Value(0.0), "Tangential distortion (first parameter) (double)").asDouble();
    _p2 = botConfigCamCalib.check("p2", Value(0.0), "Tangential distortion (second parameter) (double)").asDouble();
    
    _fx_scaled_flt = _fx_scaled_rgb = _fx;
    _fy_scaled_flt = _fy_scaled_rgb = _fy;
    _cx_scaled_flt = _cx_scaled_rgb = _cx;
    _cy_scaled_flt = _cy_scaled_rgb = _cy;

    _oldImgSizeFlt.width = -1;
    _oldImgSizeFlt.height = -1;
	_oldImgSizeRgb.width = -1;
    _oldImgSizeRgb.height = -1;

	// open ports
	ok = ok && _prtImgRgbIn.open(std::string(strModuleName + std::string("/mapVisual/rgb_in")).c_str());
	ok = ok && _prtImgFloatIn.open(std::string(strModuleName + std::string("/mapVisual/map_in")).c_str());
	ok = ok && _prtImgRgbClickIn.open(std::string(strModuleName + std::string("/mapVisual/click_in")).c_str());
	ok = ok && _prtImgRgbOut.open(std::string(strModuleName + std::string("/mapVisual/rgb_out")).c_str());

    return ok;
}

bool VisualMap::close(){	
	_prtImgRgbIn.close();
	_prtImgRgbOut.close();
	_prtImgFloatIn.close();
	_prtImgRgbClickIn.close();
    return true;
}

bool VisualMap::interrupt(){
	_prtImgRgbIn.interrupt();
	_prtImgRgbOut.interrupt();
	_prtImgFloatIn.interrupt();
	_prtImgRgbClickIn.interrupt();
	return true;
}

bool VisualMap::reset(){
    if (_imgMapResA.width() > 0 && _imgMapResA.height()  > 0)
        cvSet(((IplImage*)_imgMapResA.getIplImage()), cvScalar(0.0f));
    if (_imgMapResB.width() > 0 && _imgMapResB.height()  > 0)
        cvSet(((IplImage*)_imgMapResB.getIplImage()), cvScalar(0.0f));
    return true;
}


bool VisualMap::read(){
	_imgFloatIn  = _prtImgFloatIn.read(false); // non-blocking, returns NULL if nothing read
	_imgRgbIn = _prtImgRgbIn.read(false); // non-blocking, returns NULL if nothing read
	_botClick = _prtImgRgbClickIn.read(false); // non-blocking, returns NULL if nothing read
	return true;
}

bool VisualMap::process(	double azREye, double elREye, int xREye, int yREye, double *rotREye,
							double azLEye, double elLEye, int xLEye, int yLEye, double *rotLEye,
							double azHead, double elHead, int xHead, int yHead, double *rotHead,
							bool blnSaccadicSuppression){

	// FLOATING POINT IMAGE:
	if (_imgFloatIn != NULL && blnSaccadicSuppression != true){ 

		// check if initialization required
		if (_imgFloatIn->width()  != _oldImgSizeFlt.width || 
			_imgFloatIn->height() != _oldImgSizeFlt.height){
			this->initFlt(	cvSize(_imgFloatIn->width(), _imgFloatIn->height()), _calibImgSize);
		}

		if(_useRightEye){
			compute_egosp_map( _imgFloatIn->height(), _imgFloatIn->width(),
							_resYVisual, _resXVisual,
							_fx_scaled_flt, _fy_scaled_flt, // azimuth elevation
							_cx_scaled_flt,  _cy_scaled_flt,
							rotREye, //world to camera rotation
							(float*)((IplImage*)_imgRemapX.getIplImage())->imageData, 
							(float*)((IplImage*)_imgRemapY.getIplImage())->imageData);
		}
		else{
			compute_egosp_map( _imgFloatIn->height(), _imgFloatIn->width(),
							_resYVisual, _resXVisual,
							_fx_scaled_flt, _fy_scaled_flt, // azimuth elevation
							_cx_scaled_flt,  _cy_scaled_flt,
							rotLEye, //world to camera rotation
							(float*)((IplImage*)_imgRemapX.getIplImage())->imageData, 
							(float*)((IplImage*)_imgRemapY.getIplImage())->imageData);
		}

		cvRemap(_imgFloatIn->getIplImage(), 
				_imgMapResA.getIplImage(), 
				_imgRemapX.getIplImage(),
				_imgRemapY.getIplImage(),
				CV_INTER_LINEAR);

		if (_clickPoint.x > 0 && _clickPoint.y > 0){
			yarp::sig::draw::addCircle(_imgMapResA, _clickPix, _clickPoint.x, _clickPoint.y, 3);
		}

		_oldImgSizeFlt.width = _imgFloatIn->width();
		_oldImgSizeFlt.height = _imgFloatIn->height();
	}
    
	// RGB IMAGE:
	if (_imgRgbIn != NULL && blnSaccadicSuppression != true){ 
		 
		// check if initialization required
		if ( _imgRgbIn->width()  != _oldImgSizeRgb.width || 
			 _imgRgbIn->height() != _oldImgSizeRgb.height){
			this->initRgb(cvSize(_imgRgbIn->width(), _imgRgbIn->height()), _calibImgSize);
		}

		if(_useRightEye){
			compute_egosp_map( _imgRgbIn->height(), _imgRgbIn->width(),
							_resYVisual, _resXVisual,
							_fx_scaled_rgb, _fy_scaled_rgb, // azimuth elevation
							_cx_scaled_rgb,  _cy_scaled_rgb,
							rotREye, //world to camera rotation
							(float*)((IplImage*)_imgRemapX.getIplImage())->imageData, 
							(float*)((IplImage*)_imgRemapY.getIplImage())->imageData);
		}
		else{
			compute_egosp_map( _imgRgbIn->height(), _imgRgbIn->width(),
							_resYVisual, _resXVisual,
							_fx_scaled_rgb, _fy_scaled_rgb, // azimuth elevation
							_cx_scaled_rgb,  _cy_scaled_rgb,
							rotLEye, //world to camera rotation
							(float*)((IplImage*)_imgRemapX.getIplImage())->imageData, 
							(float*)((IplImage*)_imgRemapY.getIplImage())->imageData);
		}

		cvRemap(_imgRgbIn->getIplImage(), 
				_imgMapResARgb.getIplImage(), 
				_imgRemapX.getIplImage(),
				_imgRemapY.getIplImage(),
				CV_INTER_LINEAR);

		// draw gaze direction
		if (_drawGazeDirectionREye){
			yarp::sig::draw::addCrossHair(_imgMapResARgb, _gazePixREye, xREye, yREye, 10);
		}
		if (_drawGazeDirectionLEye){
			yarp::sig::draw::addCrossHair(_imgMapResARgb, _gazePixLEye, xLEye, yLEye, 10);
		}

		_oldImgSizeRgb.width = _imgRgbIn->width();
		_oldImgSizeRgb.height = _imgRgbIn->height();

	}

	// 'mouse-click salience'
	processClick();

	return true;
}

bool VisualMap::write(bool saccadicSuppression){
	// write the rgb visualization of the sphere
	ImageOf<PixelRgb>& imgRgbOut = _prtImgRgbOut.prepare();
	imgRgbOut.resize(_egoImgSize.width, _egoImgSize.height);
	imgRgbOut.copy(this->getVisualizationMap(_egoImgSize.width, _egoImgSize.height)); // convert from possibly different internal resolution to sphere resolution
	_prtImgRgbOut.write();
	return true;
}

bool VisualMap::updateDecay(){
	if (_salienceDecayRate < 1.0){
        cvScale(((IplImage*)_imgMapResA.getIplImage()),((IplImage*)_imgMapResA.getIplImage()),_salienceDecayRate);
	}
	return true;
}

ImageOf<PixelFloat>& VisualMap::getSalienceMap(int width, int height){
    _imgMapResB.resize(width, height);
    cvResize(_imgMapResA.getIplImage(), _imgMapResB.getIplImage(), CV_INTER_LINEAR );
    return _imgMapResB;
}

ImageOf<PixelRgb>& VisualMap::getVisualizationMap(int width, int height){
    _imgMapResBRgb.resize(width, height);
    cvResize(_imgMapResARgb.getIplImage(), _imgMapResBRgb.getIplImage(), CV_INTER_LINEAR );
    return _imgMapResBRgb;
}

bool VisualMap::setSalienceDecay(double rate){
    _salienceDecayRate = rate;
    return true;
}

double VisualMap::getSalienceDecay(){
    return _salienceDecayRate;
}


// helper function to pass correct matrix to projectors
void VisualMap::convertRobMatrix(RobMatrix &robMatrix, double *matrix){
    double tmp[9];
	// copy from matrix to array
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

void VisualMap::initFlt(CvSize currImgSizeFlt, CvSize calibImgSize){

	// Scale the intrinsics if required:
    // if current image size is not the same as the size for
    // which calibration parameters are specified we need to
    // scale the intrinsic matrix components.
	// floating point image calibration
    if (currImgSizeFlt.width != calibImgSize.width ||
        currImgSizeFlt.height != calibImgSize.height){
        float scaleX = (float)currImgSizeFlt.width / (float)calibImgSize.width;
        float scaleY = (float)currImgSizeFlt.height / (float)calibImgSize.height;
        _fx_scaled_flt = _fx * scaleX;
        _fy_scaled_flt = _fy * scaleY;
        _cx_scaled_flt = _cx * scaleX;
        _cy_scaled_flt = _cy * scaleY;
    }
    else{
        _fx_scaled_flt = _fx;
        _fy_scaled_flt = _fy;
        _cx_scaled_flt = _cx;
        _cy_scaled_flt = _cy;
    }

    compute_sp_params( currImgSizeFlt.height, currImgSizeFlt.width, 
                       currImgSizeFlt.height, currImgSizeFlt.width,
                       _fx_scaled_flt, _fy_scaled_flt, _cx_scaled_flt, _cy_scaled_flt, 
                       //_fx, _fy, _cx, _cy,
                       _k1, _k2,
                       _p2, _p2,
                       &_fa_flt, &_fe_flt,
                       &_ca_flt, &_ce_flt);

    _azSpanFlt = 2.0 * atan(_ca_flt/_fa_flt);
    _elSpanFlt = 2.0 * atan(_ce_flt/_fe_flt);
    cout << "fa float: " << _fa_flt << " ca float: " << _ca_flt << endl;
    cout << "fe float: " << _fe_flt << " ce float: " << _ce_flt << endl;
    cout << "_azSpan float: " << _azSpanFlt * 180.0 / M_PI << " _elSpan float: " << _elSpanFlt * 180.0 / M_PI << endl;
}

void VisualMap::initRgb(CvSize currImgSizeRgb, CvSize calibImgSize){

	 // Scale the intrinsics if required:
    // if current image size is not the same as the size for
    // which calibration parameters are specified we need to
    // scale the intrinsic matrix components.
	// rgb image calibration
	if (currImgSizeRgb.width != calibImgSize.width ||
        currImgSizeRgb.height != calibImgSize.height){
        float scaleX = (float)currImgSizeRgb.width / (float)calibImgSize.width;
        float scaleY = (float)currImgSizeRgb.height / (float)calibImgSize.height;
        _fx_scaled_rgb = _fx * scaleX;
        _fy_scaled_rgb = _fy * scaleY;
        _cx_scaled_rgb = _cx * scaleX;
        _cy_scaled_rgb = _cy * scaleY;
    }
    else{
        _fx_scaled_rgb = _fx;
        _fy_scaled_rgb = _fy;
        _cx_scaled_rgb = _cx;
        _cy_scaled_rgb = _cy;
    }

	compute_sp_params( currImgSizeRgb.height, currImgSizeRgb.width, 
                       currImgSizeRgb.height, currImgSizeRgb.width,
                       _fx_scaled_rgb, _fy_scaled_rgb, _cx_scaled_rgb, _cy_scaled_rgb, 
                       //_fx, _fy, _cx, _cy,
                       _k1, _k2,
                       _p2, _p2,
                       &_fa_rgb, &_fe_rgb,
                       &_ca_rgb, &_ce_rgb);

	_azSpanRgb = 2.0 * atan(_ca_rgb/_fa_rgb);
    _elSpanRgb = 2.0 * atan(_ce_rgb/_fe_rgb);
	cout << "fa rgb: " << _fa_rgb << " ca rgb: " << _ca_rgb << endl;
    cout << "fe rgb: " << _fe_rgb << " ce rgb: " << _ce_rgb << endl;
    cout << "_azSpan rgb: " << _azSpanFlt * 180.0 / M_PI << " _elSpan rgb: " << _elSpanFlt * 180.0 / M_PI << endl;
}

void VisualMap::processClick(){
    if (_botClick != NULL){
        float distance, removeDistance = 5.0f;
        CvPoint oldClickPoint;
        oldClickPoint.x = _clickPoint.x;
        oldClickPoint.y = _clickPoint.y;
        _clickPoint.x = _botClick->get(0).asInt();
        _clickPoint.y = _botClick->get(1).asInt();
        distance = (float)sqrt(pow((double)(_clickPoint.x - oldClickPoint.x),2) + pow((double)(_clickPoint.y - oldClickPoint.y),2));
        if (distance < removeDistance){
            _clickPoint.x = -100;
            _clickPoint.y = -100;
        }
	}
}
