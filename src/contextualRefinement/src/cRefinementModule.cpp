// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <iCub/cRefinementModule.h>


/**
*function that opens the module
*/
bool cRefinementModule::open(Searchable& config) {
    ct = 0;
    portInImage.open(getName("inImage"));
	portInProbClassA.open(getName("inProbClassA"));
	portInProbOther.open(getName("inProbOther"));
	portOut.open(getName("out"));
    cmdPort.open(getName("cmd")); // optional command port
    attach(cmdPort); // cmdPort will work just like terminal
	
    return true;
}

/** 
* tries to interrupt any communications or resource usage
*/
bool cRefinementModule::interruptModule() {
    portInImage.interrupt();
	portInProbClassA.interrupt();
	portInProbOther.interrupt();
	portOut.interrupt();
	cmdPort.interrupt();
    return true;
}


bool cRefinementModule::close(){
	portInImage.close();
	portInProbClassA.close();
	portInProbOther.close();
	portOut.close();
	cmdPort.close();
	cThread->stop();
    return true;
}

void cRefinementModule::setProperties(MultiClass::Parameters prop){
	   //properties=prop;
}

void cRefinementModule::setOptions(yarp::os::Property opt ){
	options	=opt;
	// definition of the mode
	
	// definition of the name of the module
	ConstString name=opt.find("name").asString();
	printf("Module named as :%s \n", name.c_str());
	if(name.c_str()!=NULL)
		this->setName(name.c_str());
	printf("\n");
}

bool cRefinementModule::updateModule() {
	//set in grays the image
	imgInput = portInImage.read(false);
	if(imgInput==NULL){
		return true;	
	}
	cThread->imgProbClassA = portInProbClassA.read(false);
	if(cThread->imgProbClassA==NULL){
		return true;	
	}
	/*imgProbOther = portInProbOther.read(false);
	if(imgProbOther==NULL){
		return true;	
	}*/
    
	//prepare the output to the ports
	//ippiCopy_8u_C1R(p_pr[1],320,imgOut->getPixelAddress(0,0),320,isize);
	portOut.prepare() = *imgOut;		
	portOut.write();
	
    return true;
}


/** 
* function that istantiate the TrackerThread
* @param property of the thread
*/
void cRefinementModule::istantiateThread(Property options){
	cThread=new classifierThread(options);
}