#include <iCub/imageReaderThread.h>
#include <iCub/convert_bitdepth.h>
#include <ipps.h>
#include <iostream>


using namespace yarp::sig;
using namespace yarp::os;
using namespace std;

imageReaderThread::imageReaderThread()//:RateThread(THREAD_RATE_IMAGE)
{
    reinit_flag=false;
        
    inputImg=0;
    img=0;

    redPlane=0;
    greenPlane=0;
    bluePlane=0;

    redGreen_yarp=0;
    greenRed_yarp=0;
    blueYellow_yarp=0;

    yPlane=0;
    uPlane=0;
    vPlane=0;
    uvPlane=0;
}

imageReaderThread::~imageReaderThread()
{
    delete img;
    delete inputImg;

    delete redPlane;
    delete greenPlane;
    delete bluePlane;

    delete redGreen_yarp;
    delete greenRed_yarp;
    delete blueYellow_yarp;

    delete yPlane;
    delete uPlane;
    delete vPlane;
    delete uvPlane;
}

/*processorThread::processorThread(Property &op):processorThread(){
        
        
}*/

void imageReaderThread::reinitialise(int width, int height){
    srcsize.width=width;
    srcsize.height=height;

    inputImg=new ImageOf<PixelRgb>;
    inputImg->resize(width,height);
}

void imageReaderThread::setName(const char* str){
    this->name=str; 
}


std::string imageReaderThread::getName(const char* p){
    string str(name);
    str.append(p);
    printf("name: %s", name.c_str());
    return str;
}


bool imageReaderThread::threadInit(){
    printf("Thread initialisation.. \n");    
    inputPort.open(getName("/image:i").c_str());
        
    redPort.open(getName("/red:o").c_str());
    greenPort.open(getName("/green:o").c_str());
    bluePort.open(getName("/blue:o").c_str());

    rgPort.open(getName("/rg:o").c_str());
    grPort.open(getName("/gr:o").c_str());
    byPort.open(getName("/by:o").c_str());

    yPort.open(getName("/ychannel:o").c_str());
    uPort.open(getName("/uchannel:o").c_str());
    vPort.open(getName("/vchannel:o").c_str());
    uvPort.open(getName("/uvchannel:o").c_str());

   

    return true;
}

/**
* function called when the module is poked with an interrupt command
*/
void imageReaderThread::interrupt(){
    inputPort.interrupt();
    
    redPort.interrupt();
    greenPort.interrupt();
    bluePort.interrupt();

    rgPort.interrupt();
    grPort.interrupt();
    byPort.interrupt();
    
    yPort.interrupt();
    uPort.interrupt();
    vPort.interrupt();
    uvPort.interrupt();
}


void imageReaderThread::run(){
    img = this->inputPort.read(false);
    if(0==img)
        return;

    if(!reinit_flag){
        
	    srcsize.height=img->height();
	    srcsize.width=img->width();
        reinitialise(img->width(), img->height());
        reinit_flag=true;
        

    }

    //copy the inputImg into a buffer
    ippiCopy_8u_C3R(img->getRawImage(), img->getRowSize(),inputImg->getRawImage(), inputImg->getRowSize(),srcsize);
   
    outPorts();   
}
   
void imageReaderThread::outPorts(){
    //port2.prepare() = *img;	
    //rgb section
    if((redPlane!=0)&&(redPort.getOutputCount())){
        redPort.prepare() = *(redPlane);		
        redPort.write();
    }
    if((bluePlane!=0)&&(bluePort.getOutputCount())){
        bluePort.prepare() = *(bluePlane);		
        bluePort.write();
    }
    if((greenPlane!=0)&&(greenPort.getOutputCount())){
        greenPort.prepare() = *(greenPlane);		
        greenPort.write();
    }
    
    if((yPlane!=0)&&(yPort.getOutputCount())){
        yPort.prepare() = *(yPlane);		
        yPort.write();
    }
    if((uPlane!=0)&&(uPort.getOutputCount())){
        uPort.prepare() = *(uPlane);		
        uPort.write();
    }
    if((vPlane!=0)&&(vPort.getOutputCount())){
        vPort.prepare() = *(vPlane);		
        vPort.write();
    }
    if((uvPlane!=0)&&(uvPort.getOutputCount())){
        uvPort.prepare() = *(uvPlane);		
        uvPort.write();
    }
    
    //colour opponency section
    if((redGreen_yarp!=0)&&(rgPort.getOutputCount())){
        rgPort.prepare()=*(redGreen_yarp);
        rgPort.write();
    }
    if((greenRed_yarp!=0)&&(grPort.getOutputCount())){
        grPort.prepare()=*(greenRed_yarp);
        grPort.write();
    }
    if((blueYellow_yarp!=0)&&(byPort.getOutputCount())){
        byPort.prepare()=*(blueYellow_yarp);
        byPort.write();
    }
    
}

void imageReaderThread::threadRelease(){
   
    printf("Thread releasing.. \n");
    printf("input port closing .... \n");
    inputPort.close();
    
    printf("red channel port closing .... \n");
    redPort.close();
    printf("green channel port closing .... \n");
    greenPort.close();
    printf("blue channel port closing .... \n");
    bluePort.close();

    printf("R+G- colourOpponency port closing .... \n");
    rgPort.close();
    printf("G+R- colourOpponency port closing .... \n");
    grPort.close();
    printf("B+Y- colourOpponency port closing .... \n");
    byPort.close();

    printf("intensity channel port closing .... \n");
    yPort.close();
    printf("chrominance channel port closing .... \n");
    uPort.close();
    vPort.close();
    uvPort.close();
}



/*void imageReaderThread::setInputImage(ImageOf<PixelRgb>* inputImage){
    this->img=inputImage;
    this->width=inputImage->width();
    this->height=inputImage->height();
    reinitialise();
}*/


//----- end-of-file --- ( next line intentionally left blank ) ------------------
