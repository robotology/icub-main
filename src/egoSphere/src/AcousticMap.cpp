// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iCub/AcousticMap.h>

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

template <class T>
T portable_min(T x, T y) {
    return (x<y)?x:y;
}

AcousticMap::AcousticMap(){
    _matRotS2W = new double[9];
    _matRotW2S = new double[9];
	_imgMapRgb.resize(0,0);
}

AcousticMap::~AcousticMap(){
    delete [] _matRotS2W;
    delete [] _matRotW2S;
}

bool AcousticMap::open(Searchable& config){

	bool ok = true;

	if(!config.check("name")){
		std::cout << "AuditoryMap: Error, module base name not found in configuration." << std::endl;
		return false;
	}

	// module base name
	std::string strModuleName = std::string(config.find("name").asString().c_str());

    // look for group EGO_SPHERE_ACOUSTIC_MAP
    Bottle botConfigAcoustic(config.toString().c_str());
    botConfigAcoustic.setMonitor(config.getMonitor());
    if (!config.findGroup("EGO_SPHERE_ACOUSTIC_MAP").isNull()){
        botConfigAcoustic.clear();
        botConfigAcoustic.fromString(config.findGroup("EGO_SPHERE_ACOUSTIC_MAP", "Loading visual map configuration  from group EGO_SPHERE_ACOUSTIC_MAP.").toString());
    }

    _salienceDecayRate = botConfigAcoustic.check("decayAcoustic",
                                      Value(0.95),
                                      "Decay for the acoustic saliency map (double).").asDouble();
    _resXAcoustic = botConfigAcoustic.check("resXAcoustic",
                                Value(80),
                                "Width of internal acoustic map (int)").asInt();
    _resYAcoustic = botConfigAcoustic.check("resYAcoustic",
                                Value(60),
                                "Height of internal acoustic map (int)").asInt();
    _imgCart.resize(_resXAcoustic,_resYAcoustic);
    _imgRemapX.resize(_resXAcoustic,_resYAcoustic);
    _imgRemapY.resize(_resXAcoustic,_resYAcoustic);
    _imgSpher.resize(_resXAcoustic,_resYAcoustic);
    _imgMapResA.resize(_resXAcoustic,_resYAcoustic);
    
	ok = ok && _prtVctSound.open(std::string(strModuleName + std::string("/mapAuditory/vct_in")).c_str());

	return ok;
}

bool AcousticMap::interrupt(){
	_prtVctSound.interrupt();
	return true;
}

bool AcousticMap::close(){
	_prtVctSound.close();
    return true;
}

bool AcousticMap::reset(){
    if (_imgMapResA.width() > 0 && _imgMapResA.height()  > 0)
        cvSet(((IplImage*)_imgMapResA.getIplImage()), cvScalar(0.0f));
    if (_imgMapResB.width() > 0 && _imgMapResB.height()  > 0)
        cvSet(((IplImage*)_imgMapResB.getIplImage()), cvScalar(0.0f));
    return true;
}

bool AcousticMap::read(){
	_vctSound = _prtVctSound.read(false);
	return true;
}

bool AcousticMap::updateDecay(){
	if (_salienceDecayRate < 1.0){
        cvScale(((IplImage*)_imgMapResA.getIplImage()),((IplImage*)_imgMapResA.getIplImage()),_salienceDecayRate);
	}
	return true;
}

bool AcousticMap::process(	double azREye, double elREye, int xREye, int yREye, double *rotREye,
							double azLEye, double elLEye, int xLEye, int yLEye, double *rotLEye,
							double azHead, double elHead, int xHead, int yHead, double *rotHead,
							bool blnSaccadicSuppression){


	if (_vctSound != NULL){ // saccadic suppression does not influence audition
		if (_vctSound->size() > 4){

			// i) render distribution to cartesian map
			// ii) project cartesian map to spherical map
			// iii) add temporary spherical map to final spherical map

			// cartesian image size
			int hCart = _imgCart.height();
			int wCart = _imgCart.width();

			// extract sound localization values
			float azimuth = (float)(*_vctSound)(0);
			float elevation = (float)(*_vctSound)(2);
			float intensity = (float)255.0f; // _vctSound(4);
			float sigmaXDeg = (float)(*_vctSound)(1);
			float sigmaYDeg = (float)(*_vctSound)(3);
			float sigmaXPix = (float)(_resXAcoustic * sigmaXDeg/360.0);
			float sigmaYPix = (float)(_resYAcoustic * sigmaYDeg/180.0);
			int sizeXPix = (int)(4.0f*sigmaXPix);
			int sizeYPix = (int)(4.0f*sigmaYPix);
			((sizeXPix-1)%2==0)?sizeXPix:sizeXPix++; // make odd
			((sizeYPix-1)%2==0)?sizeYPix:sizeYPix++; // make odd

			std::cout << "Adding sound source at: az " << azimuth << " el " << elevation << std::endl;

			// i)
			// render sound distribution at location (0,0) to imgCart
			cvSet(_imgCart.getIplImage(), cvScalar(0.0f)); // zero
			//PixelFloat pix = 255.0f;
			//yarp::sig::draw::addRectangle(_imgCart, pix, wCart/2, hCart/2, wCart, hCart);
			//yarp::sig::draw::addCircle(_imgCart, pix, _imgCart.width()/2, _imgCart.height()/2, 10);
			this->renderGaussianDistribution(_imgCart,
										 _imgCart.width()/2, _imgCart.height()/2, 
										 sizeXPix, sizeYPix, 
										 sigmaXPix, sigmaYPix, 
										 intensity);
		   

			RobMatrix rotHeadRob (	rotHead[0], rotHead[3], rotHead[6], 
									rotHead[1], rotHead[4], rotHead[7], 
									rotHead[2], rotHead[5], rotHead[8]); // transpose (back)
			// calculate rotation matrix world2sound-centered-coordinates
			_robMatRot = rotHeadRob * _headKin.getRotationFromAzimuthElev(azimuth, elevation);
			/*cout << "Mat All: " << endl;
			_robMatRot.print();*/
			EgoSphereModule::convertRobMatrix(_robMatRot, _matRotS2W);
			/*cout << "_matRotS2W: " << endl;
			cout << _matRotS2W[0] << " " << _matRotS2W[1] << " " << _matRotS2W[2] << endl;
			cout << _matRotS2W[3] << " " << _matRotS2W[4] << " " << _matRotS2W[5] << endl;
			cout << _matRotS2W[6] << " " << _matRotS2W[7] << " " << _matRotS2W[8] << endl;*/
			EgoSphereModule::transpose(_matRotS2W, _matRotW2S);
			/*cout << "_matRotW2S: " << endl;
			cout << _matRotW2S[0] << " " << _matRotW2S[1] << " " << _matRotW2S[2] << endl;
			cout << _matRotW2S[3] << " " << _matRotW2S[4] << " " << _matRotW2S[5] << endl;
			cout << _matRotW2S[6] << " " << _matRotW2S[7] << " " << _matRotW2S[8] << endl;*/

			// ii)
			// compute remap maps
			double focalLength = 0.92*(double)min(_resYAcoustic,_resXAcoustic); // corresponds ~ what dragonfly's have
			//cout << "x: " << _resXAcoustic << "y: " << _resYAcoustic << " focal: " << focalLength << " cx: " << (double)wCart/2.0 << " cy: " << (double)hCart/2.0 << endl;
			compute_egosp_map( _resYAcoustic, _resXAcoustic,
							   _resYAcoustic, _resXAcoustic,
								focalLength, focalLength, // fx, fy
								(double)_resXAcoustic/2.0,  (double)_resYAcoustic/2.0,
								_matRotW2S, //world to sound rotation
							   (float*)((IplImage*)_imgRemapX.getIplImage())->imageData, 
							   (float*)((IplImage*)_imgRemapY.getIplImage())->imageData);
			// remap cartesian map to spherical map
			cvRemap(_imgCart.getIplImage(), _imgSpher.getIplImage(),
					_imgRemapX.getIplImage(), _imgRemapY.getIplImage(), 
					CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS,
					cvScalarAll(0)); // remap and fill outliers with 0's

			// iii)
			// add new temporary map to final spherical map
			// spherFinal(x,y) = max(spherTemp(x,y),spherFinal(x,y))
			cvMax(_imgSpher.getIplImage(), _imgMapResA.getIplImage(), _imgMapResA.getIplImage());
		}
		else{
			cout << "Error AcousticMap::process(): acoustic observation vector size < 5." << endl;
		}
	}
    return true;
}

bool AcousticMap::write(bool saccadicSuppression){
	// acoustic map does not write additional output
	return true;
}

yarp::sig::ImageOf<yarp::sig::PixelFloat>& AcousticMap::getSalienceMap(int width, int height){
    _imgMapResB.resize(width, height);
    cvResize(_imgMapResA.getIplImage(), _imgMapResB.getIplImage(), CV_INTER_LINEAR );
    return _imgMapResB;
}

yarp::sig::ImageOf<yarp::sig::PixelRgb>& AcousticMap::getVisualizationMap(int width, int height){
	// not implemented
	return _imgMapRgb;
}

bool AcousticMap::setWeightAcoustic(float w){
    _weightAcoustic = w;
    return true;
}

float AcousticMap::getWeightAcoustic(){
    return _weightAcoustic;
}


bool AcousticMap::setSalienceDecay(double rate){
    _salienceDecayRate = rate;
    return true;
}

double AcousticMap::getSalienceDecay(){
    return _salienceDecayRate;
}


void AcousticMap::renderGaussianDistribution(ImageOf<PixelFloat> &img,
                                             int posX, int posY, 
                                             int sizex, int sizey, 
                                             float sigmaX, float sigmaY, 
                                             float height){
    IplImage *ipl = (IplImage*)img.getIplImage();   
    //cout << "going to render.." << posX << " " << posY << " " << sizex << " " << sizey << " " << sigmaX << " " << sigmaY << " " << height << endl;
	if (((sizex-1)%2 == 0) && ((sizey-1)%2 == 0)){
		int extx = (sizex-1)/2;
		int exty = (sizey-1)/2;
        int currX, currY;
		for(int y=-exty;y<=exty;y++){
			for(int x=-extx;x<=extx;x++){
                currX = x+posX;
                currY = y+posY;
                // if inside acoustic map
                //cout << "rendering at: " << posX << " " << posY << endl;
                if (posX >= 0 && posX < img.width() &&
                    posY >= 0 && posY < img.height()){
                        //cout << "rendering at: " << posX << " " << posY << " value: " << height*(float)(exp(-0.5*(((float)x/sigmaX)*((float)x/sigmaX)+((float)y/sigmaY)*((float)y/sigmaY)))) << endl;
                    ((float*)(ipl->imageData + ipl->widthStep*(currY)))[currX] = height*(float)(exp(-0.5*(((float)x/sigmaX)*((float)x/sigmaX)+((float)y/sigmaY)*((float)y/sigmaY))));
                }
			}
		}
    }
	else
	{
		cout << "AcousticMap::renderGaussianDistribution not possible due to invalid size (has to be odd)" << endl;
	}
}
