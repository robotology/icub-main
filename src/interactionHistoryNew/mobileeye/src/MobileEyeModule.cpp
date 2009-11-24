// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 *
 */
 
#include <iCub/iha/MobileEyeModule.h>

#include <iCub/iha/iha_utils.h>

//using namespace iCub::iha;

MobileEyeModule::MobileEyeModule(){

}

MobileEyeModule::~MobileEyeModule(){

}

bool MobileEyeModule::open(Searchable& config)    {
    
    if (config.check("help","if present, display usage message")) {
        printf("Call with --name /prefix ");
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
    coordsPort.open(getName("mobileeye:coordsout"));
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



