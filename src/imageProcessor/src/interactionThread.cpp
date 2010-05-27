#include <iCub/interactionThread.h>
#include <iCub/convert_bitdepth.h>
#include <ipps.h>
#include <iostream>


using namespace yarp::sig;
using namespace yarp::os;
using namespace std;

interactionThread::interactionThread():RateThread(THREAD_RATE_IMAGE)
{
   this->tmp=0;
   this->inputImg=0;

   reinit_flag=false;
   redGreen_flag=new int;
   *redGreen_flag=0;
   greenRed_flag=new int;
   *greenRed_flag=0;
   blueYellow_flag=new int;
   *blueYellow_flag=0;

   redPlane=0;
   greenPlane=0;
   bluePlane=0;

    redGreen_yarp=new ImageOf<PixelMono>;
    greenRed_yarp=new ImageOf<PixelMono>;
    blueYellow_yarp=new ImageOf<PixelMono>;

}

interactionThread::~interactionThread()
{
   delete tmp;
   delete inputImg;

   delete redPlane;
   delete greenPlane;
   delete bluePlane;
   delete redGreen_yarp;
   delete greenRed_yarp;
   delete blueYellow_yarp;
}

/*processorThread::processorThread(Property &op):processorThread(){
        
        
}*/

void interactionThread::reinitialise(int width, int height){
    srcsize.width=width;
    srcsize.height=height;

    inputImg=new ImageOf<PixelRgb>;
    inputImg->resize(width,height);

    redGreen_yarp->resize(width,height);
    greenRed_yarp->resize(width,height);
    blueYellow_yarp->resize(width,height);

    
    
}

void interactionThread::setName(std::string str){
    this->name=str; 
}


std::string interactionThread::getName(const char* p){
    string str(name);
    str.append(p);
    printf("name: %s", name.c_str());
    return str;
}


bool interactionThread::threadInit(){
    printf("Thread initialisation.. \n");    
    openPorts();
    return true;
}

/**
* function called when the module is poked with an interrupt command
*/
void interactionThread::interrupt(){
    redPlanePort.interrupt();
    bluePlanePort.interrupt();
    greenPlanePort.interrupt();
    
    rgPort.interrupt();
    grPort.interrupt();
    byPort.interrupt();
    rgEdgesPort.interrupt();
    grEdgesPort.interrupt();
    byEdgesPort.interrupt();

    edgesPort.interrupt();

    inImagePort.interrupt();
}


void interactionThread::run(){
    
    /*this->inputImg = this->inImagePort.read(false);
    if(0==inputImg)
        return true;*/

    tmp=rgPort.read(false);
    if(tmp==0){
        return;
    }
    //outPorts();
    
    if(!reinit_flag){
        //srcsize.height=img->height();
        //srcsize.width=img->width();
        reinitialise(tmp->width(), tmp->height());
        reinit_flag=true;
        
        //startImageProcessor();
    }
    ippiCopy_8u_C1R(tmp->getRawImage(),tmp->getRowSize(),redGreen_yarp->getRawImage(),redGreen_yarp->getRowSize(),srcsize);
    //this->redGreen_yarp=tmp;
    this->redPlane=redPlanePort.read(false);
    if(0!=redGreen_yarp)
        *redGreen_flag=1;
    
    
    this->bluePlane=bluePlanePort.read(false);
    tmp=byPort.read(false);
    if(0!=tmp){
        ippiCopy_8u_C1R(tmp->getRawImage(),tmp->getRowSize(),blueYellow_yarp->getRawImage(),blueYellow_yarp->getRowSize(),srcsize);   
        *blueYellow_flag=1;
    }
    
    
    this->greenPlane=greenPlanePort.read(false);
    tmp=grPort.read(false);
    if(0!=tmp){
        ippiCopy_8u_C1R(tmp->getRawImage(),tmp->getRowSize(),greenRed_yarp->getRawImage(),greenRed_yarp->getRowSize(),srcsize);
        *greenRed_flag=1;
    }
    
    

    //check for any possible command
    
    /*Bottle* command=(Bottle)cmdPort.read(reader,false);
    if(command!=0){
        //Bottle* tmpBottle=cmdPort.read(false);
        ConstString str= command->toString();
        printf("command received: %s \n", str.c_str());
        Bottle* reply=new Bottle();
        this->respond(*command,*reply);
        command->clear();
    }*/
    outPorts();

}
   

void interactionThread::threadRelease(){
   
    printf("Thread releasing.. \n");
    printf("input port closing .... \n");
    closePorts();
}

bool interactionThread::openPorts(){
	
    bool ret = false;
    bool ok=true;
    //input ports
    inImagePort.open(getName("/image:i").c_str());
	redPlanePort.open(getName("/red:i").c_str());
    bluePlanePort.open(getName("/blue:i").c_str());
    greenPlanePort.open(getName("/green:i").c_str());

    rgPort.open(getName("/rg:i").c_str());
    grPort.open(getName("/gr:i").c_str());
    byPort.open(getName("/by:i").c_str());

    rgEdgesPort.open(getName("/rgEdges:o").c_str());
    grEdgesPort.open(getName("/grEdges:o").c_str());
    byEdgesPort.open(getName("/byEdges:o").c_str());

    edgesPort.open(getName("/edges:o").c_str());
    

	return true;
}

bool interactionThread::outPorts(){
    
    if((0!=edges_yarp)&&(edgesPort.getOutputCount())){
        edgesPort.prepare()=*(edges_yarp);
        edgesPort.write();
    }
    return true;
}

bool interactionThread::closePorts(){
	bool ret = false;
	//int res = 0;
	// Closing Port(s)
    //reduce verbosity --paulfitz

    //closing input ports
    inImagePort.close();
    redPlanePort.close();
    bluePlanePort.close();
    greenPlanePort.close();
    
    rgPort.close();
    grPort.close();
    byPort.close();
    rgEdgesPort.close();
    grEdgesPort.close();
    byEdgesPort.close();

    edgesPort.close();

	return ret;
}



/*void interactionThread::setInputImage(ImageOf<PixelRgb>* inputImage){
    this->img=inputImage;
    this->width=inputImage->width();
    this->height=inputImage->height();
    reinitialise();
}*/


//----- end-of-file --- ( next line intentionally left blank ) ------------------
