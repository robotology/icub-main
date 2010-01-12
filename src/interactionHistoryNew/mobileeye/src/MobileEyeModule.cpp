#include <iCub/iha/MobileEyeModule.h>
#include <iCub/iha/iha_utils.h>

using namespace iCub::iha;


 /**
  * @addtogroup icub_iha2_MobileEye

This module is used to combine information from a gaze tracker and a face detection 
algorithm so as to allow information about the relationship between the current
gaze direction and the location of a face in the gaze tracker's scene image to be used
by other programs. In the \ref icub_iha2 application, the face in the scene image is
assumed to be the iCub's face, as the gaze tracker used in the demo is worn by a
person engaged in a one-on-one interaction with the robot.

This module takes as input:
 - gaze direction (in scene image pixel coordinates)
 - face detected in the scene image (in output format given by /ref icub_iha2_IhaFaceDetect)
 - the scene image

The scene image is displayed with both the gaze direction and face detection information
on it, and outputs the gaze and face coordinates in a single message.

Drivers for a specific gaze tracking system are not provided, as they are likely to 
contain propriety source code. This module is written in to be able to interface 
with any gaze tracking system that can provide the required input.

Note that this module does not take in timestamped messages for the gaze and face 
detection inputs (because not all gaze trackers output timestamped data and images). As
a result, the assumption is made that the face detection can run in real time. If this
assumption is violated, the face detection coordinates and gaze direction coordinates
may be out of sync and incorrectly associated.

\section lib_sec Libraries
- YARP libraries.
- IHA Debug Library

\section parameters_sec Parameters
\verbatim
--dbg [INT]   : debug printing level
--name [STR]  : process name for ports
--file [STR]  : configuration file

--NTSC [TRUE/FALSE]  : input image from NTSC device (false assumes PAL)

--drawMobileEyeGaze  : draw gaze location crosshair on image 
\endverbatim

\section portsa_sec Ports Accessed

 - assumes a yarp framegrabber device outputs gaze tracker scene images
 - assumes \ref icub_iha2_IhaFaceDetect is run on these scene images
 - assumes gaze coordinate port from gaze tracker

\section portsc_sec Ports Created
 
 - mobileeye:in - input scene image from gaze tracker
 - mobileeye:gaze - input gaze direction coordinates from gaze tracker
 - mobileeye:face - input face detection coordinates from 
 - mobileeye:out - output image with gaze and face coordinates drawn on
 - mobileeye:coords - output gaze and face coordinates in single message



\section tested_os_sec Tested OS
Linux

\section example_sec Example Instantiation of the Module
 ihaNewMobileEye --name /iha/mobileeye --NTSC TRUE --drawMobileEyeGaze

See also the script $ICUB_ROOT/app/ihaNew/mobileeye.sh

\see \ref icub_iha2_IhaFaceDetect

\author Frank Broz 

Copyright (C) 2009 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at \in src/interactionHistoryNew/mobileeye/src/MobileEyeModule.cpp.
 *
 */

MobileEyeModule::MobileEyeModule(){

}

MobileEyeModule::~MobileEyeModule(){

}

bool MobileEyeModule::open(Searchable& config)    {
    

	if (config.check("dbg")) { IhaDebug::setLevel(config.find("dbg").asInt()); }
 	ACE_OS::fprintf(stderr, "Debug level : %d\n",IhaDebug::getLevel());

    if (config.check("help","if present, display usage message")) {
        cerr << "Usage : " << "\n"
		<< "------------------------------------------" << "\n"
		<< "  --dbg [INT]   : debug printing level" << "\n"
		<< "  --name [STR]  : process name for ports" << "\n"
		<< "  --file [STR]  : config file" << "\n"
		<< "---------------------------------------------------------------------------" << "\n"
        << "  --NTSC [TRUE/false]  : input image from NTSC device (false assumes PAL)"<< "\n"
		<< "---------------------------------------------------------------------------" << "\n"
        << "  --drawMobileEyeGaze  : draw gaze location crosshair on image "<< "\n"
		<< "---------------------------------------------------------------------------" << "\n"
		<< "\n";
        return false;
    }

	NTSC = boolStringTest(config.check("NTSC",Value("FALSE")).asString());

    drawMobileEyeGaze = (bool)config.check("drawMobileEyeGaze",
                                    Value(1),
                                    "Draw a crosshair at MobileEye gaze location onto the output visualization image (int [0|1]).").asInt();

    imgInPort.open(getName("mobileeye:in"));
    imgOutPort.open(getName("mobileeye:out"));
	gazePort.open(getName("mobileeye:gaze")); 
    facePort.open(getName("mobileeye:face"));
    coordsPort.open(getName("mobileeye:coords"));
    //coordsPort.setStrict(true);
    oldSizeX = -1;
    oldSizeY = -1;
    needInit = true;

	fflush(stdout);
    return true;
}

bool MobileEyeModule::close() {

    //for (int i = 0; i < tmpBuffer.size(); i++){
    //    //cout << "deleting buffered image: " << i << endl;
    //    delete tmpBuffer[i];
    //}
    imgInPort.close();
    imgOutPort.close();
	gazePort.close(); 
	facePort.close(); 
    coordsPort.close();
	fflush(stdout);
    return true;
}

bool MobileEyeModule::interruptModule() {
    imgInPort.interrupt();
    imgOutPort.interrupt();
	gazePort.interrupt(); 
	facePort.interrupt();
    coordsPort.interrupt();
	fflush(stdout);
    return true;
}

bool MobileEyeModule::updateModule(){
	
	//_framerate.addStartTime(yarp::os::Time::now());
    //printf("In update module \n");
    // read image from port
    ImageOf<PixelRgb> *img = imgInPort.read();
    if (img==NULL) return false;
    //printf("Got a fresh image \n");
    //printf("1 Image in width %d height %d \n", img->width(),img->height());

    //read gaze coords from the port
    Bottle *gbot = gazePort.read();   
    static int gazeX = -1;
    static int gazeY = -1;
    static int gazeDia = -1;
    //float gazeV = -1;
    if(gbot!=NULL) {
        //the skipped indices contain label strings for the data
        gazeX = gbot->get(5).asInt();
        gazeY = (gbot->get(7).asInt()); 
        gazeDia = gbot->get(3).asInt();
        //gazeV = -1.0f;
    }

    printf("%s \n", (gbot->toString()).c_str());

    //read a vector of face coordinates from the face detector
    //static int faceX = -1;
    //static int faceY = -1;
    //static int faceW = -1;
    static int lastFace = 0;
    static int faceXS = -1;
    static int faceXE = -1;
    static int faceYS = -1;
    static int faceYE = -1;
    //do something here to make face readings somewhat persistent,
    //as the frame rate is lower than for the gaze information
    Vector *fv = facePort.read(false);
    if (fv!=NULL) {
        if ((*fv)[0] != 0) {
            //faceX = ((*fv)[1] + (*fv)[3])/2;
            //faceY = ((*fv)[2] + (*fv)[4])/2;
            //faceW = ((*fv)[3] - (*fv)[1])/2;
            faceXS = (*fv)[1];
            faceYS = (*fv)[2];
            faceXE = (*fv)[3];
            faceYE = (*fv)[4];
            //printf("Got a fresh face reading %d \n", lastFace);
            lastFace = 0;
        } else
            lastFace++;
            //printf("Face struct empty \n", lastFace);

    }// else {
    //  lastFace++;
    //}
    if (lastFace > 5) {  //used to be 3
        //faceX = -1;
        //faceY = -1;
        //faceW = -1;
        faceXS = -1;
        faceXE = -1;
        faceYS = -1;
        faceYE = -1;

    }

    //printf("gaze x %d gaze y %d dia %d\n", gazeX, gazeY, gazeDia);
    // if image size changes, need to resize buffered images
    if (img->width() != oldSizeX || img->height() != oldSizeY || needInit){
        //resizeBufferedImages(img->width(), img->height());
        needInit = false;
    }
    
    
    ImageOf<PixelRgb> &imgView = imgOutPort.prepare();
    //ImageOf<PixelFloat> &imgMap = filteredPort.prepare();
    mutex.wait();
    
    //copy the input image
    //printf("2a Image in width %d height %d \n", img->width(),img->height());
    imgView.copy(*img);
    printf("Image in width %d height %d \n", img->width(),img->height());
    printf("Image out width %d height %d \n", imgView.width(),imgView.height());
    // draw crosshair at most salient location to imgView if requestd
    static int scaledGX = -1;
    static int scaledGY = -1;
    static int unscaledGX = -1;
    static int unscaledGY = -1;
    static int lastgaze = 0;
    double scalew = 1.0;
    double scaleh = 1.0;
    //the 720x576 for gaze coords is hardcoded into the mobileeye software
    //correction 09/16/09, should be 768x576
    //090929, these reported values seem to be incorrect, trying to guess
    //the range by trial and error
    if (NTSC) {
        scalew = img->width()/640.0;
        scaleh = img->height()/480.0;
    } else {
        scalew = img->width()/768.0;
        scaleh = img->height()/576.0;     
    }
    unscaledGX = gazeX;
    unscaledGY = gazeY;

    printf("Unscaled gaze x %d y %d \n", unscaledGX,unscaledGY);
    printf("Image scale width %lf height %lf \n", scalew,scaleh);


    if( (gazeX > 0) &&((gazeX*scalew) < img->width()) &&
        (gazeY > 0) &&((gazeY*scaleh) < img->height())) {
            //(gazeDia < 1000)) { //don't check this anymore
        scaledGX = int(gazeX*scalew);
        scaledGY = int(gazeY*scaleh);
        lastgaze = 0;
    } else {
        lastgaze++;
    }
    
    if (lastgaze > 10) { //used to be 5
        scaledGX = -1;
        scaledGY = -1;
        unscaledGX = -1;
        unscaledGY = -1;
    }

    if (drawMobileEyeGaze){
        PixelRgb pix = PixelRgb(255,0,0);
        //yarp::sig::draw::addCrossHair(imgView, pix, scaledGX, scaledGY, 50);
        yarp::sig::draw::addCrossHair(imgView, pix, unscaledGX, unscaledGY, 50);
    }
    if( (faceXS >= 0) &&(faceXE <= img->width()) &&
        (faceYS >= 0) &&(faceYE <= img->height())) {
        PixelRgb pix = PixelRgb(0,255,0);
        //yarp::sig::draw::addCrossHair(imgView, pix, int((faceXS+faceXE)/2), int((faceYS+faceYE)/2), int((faceXE-faceXS)/2));
        yarp::sig::draw::addRectangleOutline(imgView, pix, int((faceXS+faceXE)/2), int((faceYS+faceYE)/2), int((faceXE-faceXS)/2),int((faceYE-faceYS)/2));
        printf("Face in center x %d center y %d width %d height y %d\n", 
               //faceW,faceH, faceW/4, faceH/4);
               int((faceXS+faceXE)/2), int((faceYS+faceYE)/2), int((faceXE-faceXS)/2),int((faceYE-faceYS)/2));
        printf("xs %d ys %d  xe %d  ye %d \n", faceXS, faceYS, faceXE, faceYE);
        PixelRgb pix2 = PixelRgb(0,0,255);
        yarp::sig::draw::addCrossHair(imgView, pix2, faceXS, faceYS, 50);
    }
    
    mutex.post();
    // write image to ports
    imgOutPort.write();
    

    //timestamp the coordinate output
    timeval tim;
    gettimeofday(&tim, NULL);
    double t1=tim.tv_sec+(tim.tv_usec/1000000.0);
    printf("time: %.6lf seconds elapsed\n", t1);


    //write the gaze and face coordinates out to the port
    Bottle &coords = coordsPort.prepare();
    coords.clear();
    //send the x, y coordinates for the gaze
    coords.addDouble(t1);
    //coords.addInt(unscaledGX);
    //coords.addInt(unscaledGY);
    coords.addInt(gazeX);
    coords.addInt(gazeY);
    //send the bounding box for the face
    coords.addInt(faceXS);
    coords.addInt(faceYS);
    coords.addInt(faceXE);
    coords.addInt(faceYE);
    coords.addString("\n");
    coordsPort.write();
    //printf("bottle size %d \n",coords.size());

    //printf("3 Image in width %d height %d \n", img->width(),img->height());
    oldSizeX = img->width();
    oldSizeY = img->height();
    
	//_framerate.addEndTime(yarp::os::Time::now());
    //if(_framerate.hasNewFramerate()){
    //   cout << _framerate.getFramerate() << " fps" << endl; 
    // }
    
	fflush(stdout);
    
    return true;
}



