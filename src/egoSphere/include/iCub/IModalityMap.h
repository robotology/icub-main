// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2008 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 */

#ifndef __IMODALITYMAP__
#define __IMODALITYMAP__

 // std
#include <string>
#include <iostream>

// opencv
#include <cv.h>

// yarp
#include <yarp/String.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

// iCub
#include <iCub/kinematics/robmatrix.h>

namespace iCub {
    namespace contrib {
        class IModalityMap;
    }
}

/**
 *
 * Interface for modality maps
 *
 */
class iCub::contrib::IModalityMap {

public:
   
	virtual bool open(yarp::os::Searchable& config) = 0;

	/**
	 * Read from ports (should be non-blocking).
	 */ 
	virtual bool read() = 0; 

	virtual bool updateDecay() = 0;

	/**
	 * Process previously read data. Add observations to the salience internal map.
	 * Azimuth / Elevation in degrees, matrices are forward kinematic matrices
	 */
	virtual bool process(	double azREye, double elREye, int xREye, int yREye, double *rotREye,
							double azLEye, double elLEye, int xLEye, int yLEye, double *rotLEye,
							double azHead, double elHead, int xHead, int yHead, double *rotHead,
							bool blnSaccadicSuppression) = 0;


	virtual bool write(bool saccadicSuppression) = 0;

	/** 
	 * Get the salience map for this modality.\n
	 * The returned map is a scaled version if passed size does not fit inernal size.
	 */
	virtual yarp::sig::ImageOf<yarp::sig::PixelFloat>& getSalienceMap(int width, int height) = 0;

	/** 
	 * Get a visualization image for this modality.\n
	 * The returned map is a scaled version if passed size does not fit inernal size.
	 */
	virtual yarp::sig::ImageOf<yarp::sig::PixelRgb>& getVisualizationMap(int width, int height) = 0;

	virtual bool close() = 0;

	virtual bool interrupt() = 0;

	virtual bool reset() = 0;
};

#endif

    
//
//
//// sound
//    _prtVctSound.open(getName("sound"));
//        BufferedPort<VectorOf<double> >       _prtVctSound;
//            _prtVctSound.close();
//            _prtVctSound.interrupt();
//       
//// read
// yarp::VectorOf<double> *vctSound = _prtVctSound.read(false);       
// 
//// visual
//

//     // click point ports
//    BufferedPort<Bottle>                _prtImgEgoRgbClick;
//    

//    
//    // read

//    
//    
//// objects:
//    // open object locations port
//    _prtObjects.open(getName("objects")); //added Dario 2008
//     _prtObjects.close(); //added Dario 2008
//     
//     _prtObjects.interrupt(); //added Dario 2008
//     
//      yarp::os::Bottle *ObjectPositions; //added Dario 2008
//      
//     /*    Bottle bawtle; 
//    ImageOf<PixelRgb> black;*/
//    if(!_simulation){       //added Dario 2008
//        ObjectPositions = _prtObjects.read(false); 
//    }
////     else{
////     
////         //cout << bawtle.toString()<<endl;
//// //         ObjectPositions = &bawtle;
////     }
//
//if(ObjectPositions != NULL){ //added Dario 2008
//        addObjects(ObjectPositions);
//    }
//
//     void addObjects(yarp::os::Bottle *ObjectPositions); //added Dario 2008
//     
//     
//     
//     void EgoSphereModule::addObjects(yarp::os::Bottle *ObjectPositions){
//    VectorOf<double> objs(5);
//    objs[1] = 30; //"variance", width of the blob
//    objs[3] = 15; //"variance", height of the blob
//
//    //cout << "bottle: "<< ObjectPositions->toString() << endl;
//        
//    if(!ObjectPositions->find("size").isNull()){
//        double encoders[6];
//
//        int size = ObjectPositions->find("size").asInt();
//        char num[10];
//
//        Bottle encoders_b = ObjectPositions->findGroup("encoders");
//        encoders[0] = encoders_b.get(1).asDouble();
//        encoders[1] = encoders_b.get(2).asDouble();
//        encoders[2] = encoders_b.get(3).asDouble();
//        encoders[3] = encoders_b.get(4).asDouble();
//        encoders[4] = encoders_b.get(5).asDouble();
//        encoders[5] = encoders_b.get(6).asDouble();
//
//        for(int i=0; i< size; i++){
//            sprintf(num,"%d\0",i); 
//            Bottle object = ObjectPositions->findGroup(num); 
//            if(!object.isNull()){
//                yarp::os::ConstString label = object.find("label").asString();
//
//                double azimuth = object.find("azimuth").asDouble();
//                double elevation = object.find("elevation").asDouble();
//                objs[0] = azimuth;
//                objs[2] = elevation;
//                    // cout << "Object "<<label<<" in {"<< azimuth <<","<< elevation <<"}";
//                
////                 cvNamedWindow( "getMap before" );
////                 cvShowImage("getMap before", _mapObjects.getMap(_egoImgSize.width, _egoImgSize.height).getIplImage());
//
//                if (_activateModObjects){
//                    _mapObjects.addObservation(objs, encoders);
//                }
////                 
//            }
//        }
//        //cout << " encoders: "<<encoders[0]<<" "<<encoders[1]<<" "<<encoders[2] <<" "<<encoders[3] <<" "<<encoders[4]<<" "<<encoders[5] <<endl; 
//    }
//}
//
