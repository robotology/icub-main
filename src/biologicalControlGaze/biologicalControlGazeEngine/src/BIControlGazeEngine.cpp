#include <iCub/BIControlGazeEngine.h>

//
static BIControlGazeEngine *engineModule;

// Image Receiver
static YARPImgRecv *ptr_imgRecv;
#define _imgRecv (*(ptr_imgRecv))

//Image been read
static yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputImg;
#define _inputImg (*(ptr_inputImg))

// Semaphore
static yarp::os::Semaphore *ptr_semaphore;
#define _semaphore (*(ptr_semaphore))

/**
* function that opens the port where the inputImage is read
*/
bool openPortImage(){
	bool ret = false;
	//int res = 0;
	// Registering Port(s)
    //reduce verbosity --paulfitz
	printf("Registering port %s on network %s...\n", "/rea/BIControlGazeEngine/inputImage","default");
	ret = _imgRecv.Connect("/rea/BIControlGazeEngine/inputImage","default");
	if (ret == true)
        {
            //reduce verbosity --paulfitz
            printf("Port registration succeed!\n");
        }
	else
        {
            printf("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
	}
}


bool getImage(){
	bool ret = false;
	ret = _imgRecv.Update();

	if (ret == false){
		return false;
	}

	_semaphore.wait();
	ret = _imgRecv.GetLastImage(&_inputImg);
	engineModule->ptr_inputImage=&_inputImg;
	_semaphore.post();
	
	//printf("GetImage: out of the semaphore \n");
	return ret;
}


void createObjects() {
	ptr_imgRecv = new YARPImgRecv;
	ptr_inputImg = new yarp::sig::ImageOf<yarp::sig::PixelRgb>;
	//ptr_inputImg2= new yarp::sig::ImageOf<yarp::sig::PixelRgb>;
	ptr_semaphore = new yarp::os::Semaphore;
}

/*BIControlGazeEngine::BIControlGazeEngine(){
	
	}*/

bool BIControlGazeEngine::open(Searchable& config) {
        count=0;
		scaleFactorX=5;
		scaleFactorY=10;
		clampingThreshold=1;
		countLayer=2;

		if(!thread->start()){
			fprintf(stderr,"Error opening interfaces");
		}
	

		createObjects();
		openPortImage();

		port.open(getName("in")); 
		port0.open(getName("out0"));
		port1.open(getName("out1"));
        port2.open(getName("out2")); 
		portCmd.open(getName("inCmd"));
		portCmd.setStrict();
		
		img0=new ImageOf<PixelRgb>;
		img0->resize(320,240);
		img2=new ImageOf<PixelRgb>;
		img2->resize(320,240);
		ptr_inputImage2=new ImageOf<PixelRgb>;
		ptr_inputImage2->resize(320,240);

		runFreely=false;
		runClamped=false;

		//im_out = ippiMalloc_8u_C1(320,240,&psb);		
		//im_tmp_tmp= ippiMalloc_8u_C1(320,240,&psb);
		//im_tmp[0]=im_out;
		//im_tmp[1]=im_out;
		//im_tmp[2]=im_out;
		//red_tmp= ippiMalloc_8u_C1(320,240,&psb);
		//blue_tmp= ippiMalloc_8u_C1(320,240,&psb);
		//green_tmp= ippiMalloc_8u_C1(320,240,&psb);
		/*for(int i=0;i<320*240;i++){
			red_tmp[i]=255;
			blue_tmp[i]=255;
			green_tmp[i]=255;
		}*/
			

		engineModule=this;
        return true;
    }

bool  BIControlGazeEngine::interruptModule() {
		port.interrupt();
		port2.interrupt();
		return true;
	}

bool BIControlGazeEngine::close() {
		port.close();
		port2.close();
		return true;
	}


/** 
 *Function updateModule is executed when the Module is updated, 
 * 1.It reads the command from a port, convertes it into parameters
 * 2.It evolves the Boltzmann machine 
 * 3.Extract the output images of the active layers of the Boltzmann machine
 *
 * @param none
 * @return true if there were no errors in the funciton
 *
*/
bool BIControlGazeEngine::updateModule() {

	getImage();

	Bottle *bot=portCmd.read(false);
	if(bot!=NULL){
		string *commandTOT=new string(bot->toString().c_str());
		string command,option;
		string optionName1,optionValue1,optionName2, optionValue2;
		printf("Bottle  is: %s\n",commandTOT->c_str());
		unsigned int parOpen=commandTOT->find("(");
		printf("parOpen: %d \n",parOpen);
		if(parOpen==-1){
			command=commandTOT->substr(0,commandTOT->size());
		}
		else
		{	
		    command=commandTOT->substr(0,parOpen-1);
			option=commandTOT->substr(parOpen+1,commandTOT->size()-parOpen);
			
			
			unsigned int parPos1=option.find("(");
			unsigned int parPos2=option.find(")");
			unsigned int spacePos=option.find(" ");
			
			if(spacePos!=string::npos){
				optionName1=option.substr(parPos1+1,spacePos-parPos1);
				optionValue1= option.substr(spacePos+1,parPos2-spacePos-1);
				unsigned int dim=option.size();
				option=option.substr(parPos2+2,dim-2-parPos2);
				parPos1=option.find("(");
				if(parPos1!=string::npos){
					parPos2=option.find(")");
					spacePos=option.find(" ");
					optionName2=option.substr(parPos1+1,spacePos-parPos1);
					optionValue2= option.substr(spacePos+1,parPos2-spacePos-1);
					option=option.substr(parPos2,option.size()-parPos2);
				}
			
				//string name=option.substr(1,spacePos-1);
				//string value=option.substr(spacePos+1,option.size()-spacePos);
			}
		}

		printf("command: |%s| \n",command.c_str());
		printf("option: |%s| \n",option.c_str());
		printf("name1: |%s| \n",optionName1.c_str());
		printf("value1: |%s| \n",optionValue1.c_str());
		

		if(!strcmp(command.c_str(),"EvolveFreely")){
			printf("ExecuteFreely \n");
			runFreely=true;
			runClamped=false;
		}
		else if(!strcmp(command.c_str(),"EvolveClamped")){
			printf("ExecuteClamped \n");
			runFreely=false;
			runClamped=true;
		}
		else if(!strcmp(command.c_str(),"outHeadBehaviour")){
			printf("outHeadBehaviour \n");
			printf("option1:%s, value1:%s",optionName1,optionValue1);
			this->setJustEyes(false);
			this->setTrackerVector(1,1,0);
		}
		else if(!strcmp(command.c_str(),"outEyesBehaviour")){
			printf("outEyesBehaviour \n");
			printf("outHeadBehaviour \n");
			printf("option1:%s, value1:%s",optionName1,optionValue1);
			this->setJustEyes(true);
			this->setTrackerVector(1,1,0);
		}
		else if(!strcmp(command.c_str(),"Head_up_left")){
			printf("Up Left \n");
			this->setJustEyes(false);
			this->setTrackerVector(1,1,0);
		}
		else if(!strcmp(command.c_str(),"Head_left")){
			printf("Left \n");
			this->setJustEyes(false);
			this->setTrackerVector(1,0,0);
		}
		else if(!strcmp(command.c_str(),"Head_down_left")){
			printf("Down Left \n");
			this->setJustEyes(false);
			this->setTrackerVector(1,-1,0);
		}
		else if(!strcmp(command.c_str(),"Head_up_right")){
			printf("Up Right \n");
			this->setJustEyes(false);
			this->setTrackerVector(-1,1,0);
		}
		else if(!strcmp(command.c_str(),"Head_right")){
			printf("Right \n");
			this->setJustEyes(false);
			this->setTrackerVector(-1,0,0);
		}
		else if(!strcmp(command.c_str(),"Head_down_right")){
			printf("Down Right \n");
			this->setJustEyes(false);
			this->setTrackerVector(-1,-1,0);
		}
		else if(!strcmp(command.c_str(),"Head_up")){
			printf("Up \n");
			this->setJustEyes(false);
			this->setTrackerVector(0,1,0);
		}
		else if(!strcmp(command.c_str(),"Head_down")){
			printf("down \n");
			this->setJustEyes(false);
			this->setTrackerVector(0,-1,0);
		}
		else if(!strcmp(command.c_str(),"Head_stop")){
			printf("Stop \n");
			this->setJustEyes(false);
			this->stopTracker();
		}
		else if(!strcmp(command.c_str(),"Eyes_up_left")){
			printf("Eyes Up Left \n");
			this->setJustEyes(true);
			this->setTrackerVector(1,1,0);
		}
		else if(!strcmp(command.c_str(),"Eyes_left")){
			printf("Eyes Left \n");
			this->setJustEyes(true);
			this->setTrackerVector(1,0,0);
		}
		else if(!strcmp(command.c_str(),"Eyes_down_left")){
			printf("Eyes Down Left \n");
			this->setJustEyes(true);
			this->setTrackerVector(1,-1,0);
		}
		else if(!strcmp(command.c_str(),"Eyes_up_right")){
			printf("Eyes Up Right \n");
			this->setJustEyes(true);
			this->setTrackerVector(-1,1,0);
		}
		else if(!strcmp(command.c_str(),"Eyes_right")){
			printf("Eyes Right \n");
			this->setJustEyes(true);
			this->setTrackerVector(-1,0,0);
		}
		else if(!strcmp(command.c_str(),"Eyes_down_right")){
			printf("Eyes Down Right \n");
			this->setJustEyes(true);
			this->setTrackerVector(-1,-1,0);
		}
		else if(!strcmp(command.c_str(),"Eyes_up")){
			printf("Eyes Up \n");
			this->setJustEyes(true);
			this->setTrackerVector(0,1,0);
		}
		else if(!strcmp(command.c_str(),"Eyes_down")){
			printf("Eyes down \n");
			this->setJustEyes(true);
			this->setTrackerVector(0,-1,0);
		}
		else if(!strcmp(command.c_str(),"Eyes_stop")){
			printf("Eyes Stop \n");
			this->stopTracker();
		}
		else if(!strcmp(command.c_str(),"ClampLayer")){
			printf("ClampLayer \n");
//			clampLayer(currentLayer);
		}
		else if((!strcmp(command.c_str(),"Learn")))
			printf("Learn \n");
		else if(!strcmp(command.c_str(),"Save")){
			printf("save Configuration \n");
			this->mb->saveConfiguration();
		}
		else if((!strcmp(command.c_str(),"Load"))){
			printf("loadConfiguration \n");
//			this->loadConfiguration("configuration.ini");
		}
		else if((!strcmp(command.c_str(),"AddLayer"))){
			printf("addLayer \n");
			int valueRow=atoi(optionValue1.c_str());
			int valueCol=atoi(optionValue2.c_str());
//			this->addLayer(countLayer,valueRow,valueCol);
			countLayer++;
		}
		else if((!strcmp(command.c_str(),"CurrentLayer"))){
			
			int valueLayer=atoi(optionValue1.c_str());
			printf("CurrentUnit %d \n",valueLayer);
			//int valueCol=atoi(optionValue2.c_str());
			this->setCurrentLayer(valueLayer);
		}	
	}	

	
    return true;
}

/** 
* function that set the left vector of the tracker thread
*/
void BIControlGazeEngine::setTrackerLeftVector(){
	thread->setLeftVector(1,0,0);
}

/** 
* function that set the left vector of the tracker thread
*/
void BIControlGazeEngine::setTrackerRightVector(){
	thread->setRightVector(-1,0,0);
}

/** 
* function that set the left vector of the tracker thread
*/
void BIControlGazeEngine::setTrackerVector(double a, double b, double c){
	thread->setRightVector(a,b,c);
}

/** 
* function that stop the tracker where it is
*/
void BIControlGazeEngine::stopTracker(){
	thread->resetVector();
}

/** 
* function that set and reset the boolean for the control of just the eyes
* @param value of the flag to be set
*/
void BIControlGazeEngine::setJustEyes(bool value){
	thread->just_eyes=value;
}


/** 
* function that sets the scaleFactorX
* @param value new value of the scaleFactorX
*/
void BIControlGazeEngine::setScaleFactorX(int value){
	this->scaleFactorX=value;
}

/** 
* function that sets the scaleFactorY
* @param value new value of the scaleFactorY
*/
void BIControlGazeEngine::setScaleFactorY(int value){
	this->scaleFactorY=value;
}

/**
* set the attribute options of class Property
*/
void BIControlGazeEngine::setOptions(Property options){
//	this->options=options;
}
/** 
* function that istantiate the TrackerThread
* @param property of the thread
*/
void BIControlGazeEngine::istantiateThread(Property options){
	thread=new TrackerThread(options);
}

/** 
* function that set the number of the layer active 
* @param value number of the layer actually active
*/
void BIControlGazeEngine::setCurrentLayer(int value){
	this->currentLayer=value;
}