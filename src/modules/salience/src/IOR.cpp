// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iCub/IOR.h>

using namespace iCub::vis;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

template <class T>
T portable_min(T x, T y) {
    return (x<y)?x:y;
}

template <class T>
T portable_max(T x, T y) {
    return (x>y)?x:y;
}

IOR::IOR(){
    iorKernel = NULL;
}

IOR::~IOR(){
    release();
    close();
}

bool IOR::reset(){
	if (attMap.width() > 0 && attMap.height() > 0)
		cvSet(((IplImage*)attMap.getIplImage()), cvScalar(0.0f));
	if (iorMap.width() > 0 && iorMap.height() > 0)
		cvSet(((IplImage*)iorMap.getIplImage()), cvScalar(1.0f));
	return true;
}

bool IOR::addIORRegion(int x, int y){
	this->addIORKernel(iorMap,x,y);
	return true;
}

bool IOR::applyIOR(yarp::sig::ImageOf<yarp::sig::PixelFloat> &map){
	if (width != map.width() || height != map.height()){
		//cout << "*** Initing IOR image size!" << endl;
        init(map.width(), map.height());
	}
    width = map.width();
    height = map.height();
    cvMul(iorMap.getIplImage(),map.getIplImage(), map.getIplImage());
    return true;
}

bool IOR::updateIOR(int gazeX, int gazeY){

    // update according to:
    // b: decay rate
    // ubound: upper bound of maps (1.0 in our case)
    // f(n) = f(n-1) + trueOrfalse*ubound*b*kernelValue - b*f(n-1); 

    if (iorMap.width() <= 0 || iorMap.height() <= 0)
        return false;

    // update IOR map (grow back with rate iorDecay to 1.0f)
    IMGFOR(iorMap, x, y){
        iorMap(x,y) = iorMap(x,y) * (1.0f - iorDecay) + iorDecay; //more intuitive: iorMap(x,y) = iorMap(x,y) + iorDecay * (1.0f - iorMap(x,y));
    }

    // update attention map
    // if a valid new gazepoint is specified
    if (gazeX > -1 && gazeY > -1 && 
        gazeX < attMap.width() && gazeY < attMap.height()){
            float attIncrease;
            IMGFOR(attMap, x, y){
                attIncrease = 0.0f;
                // calculate attIncrease if inside attention kernel area
                if (x >= (gazeX-attRad) && x <= (gazeX+attRad) &&
                    y >= (gazeY-attRad) && y <= (gazeY+attRad)){
                    attIncrease = iorKernel[x - gazeX + attRad][y - gazeY + attRad];
                }
                attMap(x,y) = attMap(x,y) + attDecay*(attIncrease - attMap(x,y));
                if (attMap(x,y) > threshold){
					//std::cout << "Adding IOR region at: pix x " << x << " pix y " << y << std::endl;
                    //addIORKernel(attMap, x, y);
                    addIORKernel(iorMap, x, y);
                }
            }
    }
    // simple update (decay only, no new gazepoint)
    else{
        IMGFOR(attMap, x, y)
            attMap(x,y) = attMap(x,y)*(1.0f - attDecay);
    }

    // for testing
    if(debug){
        ImageOf<PixelRgb> &outAttMap = prtAttMap.prepare();
        ImageOf<PixelRgb> &outIorMap = prtIorMap.prepare();
        outAttMap.resize(attMap.width(), attMap.height());
        outIorMap.resize(iorMap.width(), iorMap.height());
        float2Rgb((IplImage*)attMap.getIplImage(), (IplImage*)outAttMap.getIplImage(), 255.0);
        float2Rgb((IplImage*)iorMap.getIplImage(), (IplImage*)outIorMap.getIplImage(), 255.0);
        prtAttMap.write();
        prtIorMap.write();
    }
    return true;
}

void IOR::addIORKernel(yarp::sig::ImageOf<yarp::sig::PixelFloat> &map, int iorX, int iorY){
    for (int x = portable_max((iorX - attRad),0); x < portable_min((iorX + attRad), map.width()); x++)
        for (int y = portable_max((iorY - attRad),0); y < portable_min((iorY + attRad), map.height()); y++)
                map(x,y) = portable_min(1.0f - iorKernel[x - iorX + attRad][y - iorY + attRad], map(x,y));
}

void IOR::init(int w, int h){
    release();
    iorMap.resize(w,h);
    attMap.resize(w,h);
    //create ior kernel
    attDiam = (int)(attentionDiameterFraction*(float)portable_min(w,h));
    if ((attDiam-1)%2 != 0) // make it odd
		attDiam++;
    attRad = attDiam/2;
    iorKernel = new float*[attDiam]; // create kernel
	for (int j = 0; j < attDiam; j++)
		iorKernel[j] = new float[attDiam];
	createGaussianFilter2D(iorKernel, attDiam, attDiam, (float)attDiam/4.0f, (float)attDiam/4.0f, false);
    // init maps
    for (int y = 0; y < h; y++){
        for (int x = 0; x < w; x++){
            iorMap(x,y)=1.0f;
            attMap(x,y)=0.0f;
        }
    }
}

void IOR::release(){
    // delete kernel
    if (iorKernel != NULL){
	    for (int j = 0; j < attDiam; j++)
		    delete [] iorKernel[j];
	    delete [] iorKernel;
        iorKernel = NULL;
    }
}

bool IOR::open(Searchable &config){

    Bottle botConfig(config.toString().c_str());
    botConfig.setMonitor(config.getMonitor());
    // is [IOR] group present?
    string strGroup = "IOR";
    if (!config.findGroup(strGroup.c_str()).isNull()){
        botConfig.clear();
        botConfig.fromString(config.findGroup(strGroup.c_str(), string("Loading configuration from group " + strGroup).c_str()).toString().c_str());
    }
   
    attentionDiameterFraction = (float)botConfig.check("attentionDiameterFraction",
                                    Value(0.1666),
                                    "attentionDiameterFraction * min(imgWidth, imgHeight) = attentionDiameter (double).").asDouble();
    attDecay = (float)botConfig.check("attDecay",
                                    Value(0.95),
                                    "decay of attention map (float [0.0 ... 1.0]).").asDouble();
    iorDecay = (float)botConfig.check("iorDecay",
                                    Value(0.95),
                                    "decay of inhibited regions on the IOR map (float [0.0 ... 1.0]).").asDouble();
    threshold = (float)botConfig.check("attentionInhibitionThreshold",
                                    Value(0.85),
                                    "If this threshold is reached in the internal attention map the IOR map is updated at that location (inhibition)(float [0.0...1.0])").asDouble();
    debug = (bool)botConfig.check("debug",
                                Value(0),
                                "Switch on/off writing internal maps to output ports (converted/scaled to 255 grayscale) (int [0|1]).").asInt();

    if (debug){
        prtIorMap.open("/debug/ior/iorMap");
        prtAttMap.open("/debug/ior/attMap");
    }

    width = -1;
    height = -1;
    return true;
}

bool IOR::close(){
    if (debug){
        if (!prtIorMap.isClosed())
            prtIorMap.close();
        if (!prtAttMap.isClosed())
            prtAttMap.close();
    }
    return true;
}

void IOR::createGaussianFilter2D(float **filter, int sizex, int sizey, float ex, float ey, bool normalize){

	if (((sizex-1)%2 == 0) && ((sizey-1)%2 == 0)){
		int extx = (sizex-1)/2;
		int exty = (sizey-1)/2;
        int y;
		if (normalize){
			double sum=0.0;	
			for(y=-exty;y<=exty;y++){
				for(int x=-extx;x<=extx;x++){
					sum+=(exp(-0.5*(((double)x/ex)*((double)x/ex)+((double)y/ey)*((double)y/ey))));
				}
			}
			for(y=-exty;y<=exty;y++){
				for(int x=-extx;x<=extx;x++){
					filter[x+extx][y+exty]=(float)((exp(-0.5*(((double)x/ex)*((double)x/ex)+((double)y/ey)*((double)y/ey))))/sum);
					}
			}
		}
		else{
			for(y=-exty;y<=exty;y++){

				for(int x=-extx;x<=extx;x++){
					filter[x+extx][y+exty]=(float) (exp(-0.5*(((double)x/ex)*((double)x/ex)+((double)y/ey)*((double)y/ey))));
					}
			}
		}
	}
	else
	{
		cout << "AilImageProcessing::createGaussianFilter2D not possible due to invalid size (has to be odd)" << endl;
	}
}

void IOR::float2Rgb(IplImage* imgFloat, IplImage* imgRgb, float scaleFactor){
    float flValue = 0.0f;
    int   arraypos = 0;
    for (int y = 0; y < imgFloat->height; y++){
    	arraypos = imgRgb->widthStep*y;
        for (int x = 0; x < imgFloat->width; x++){
            flValue = scaleFactor * ((float*)(imgFloat->imageData + imgFloat->widthStep*y))[x];
            if (flValue < 0.0f)
                flValue = -flValue;
            if (flValue > 255.0f)
                flValue = 255.0f;
            ((uchar*)(imgRgb->imageData + arraypos))[x*3+0] = (uchar)flValue;
            ((uchar*)(imgRgb->imageData + arraypos))[x*3+1] = (uchar)flValue;
            ((uchar*)(imgRgb->imageData + arraypos))[x*3+2] = (uchar)flValue;
        }
    }                
}
