// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include <iCub/ImageProcessModule.h>
#include <iostream>

using namespace std;

// Image Receiver
//static YARPImgRecv *ptr_imgRecv;
// Image to Display
static yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputImg=0;
// Semaphore
static yarp::os::Semaphore *ptr_semaphore;
// Timeout ID
//static guint timeout_ID;
static yarp::sig::ImageOf<yarp::sig::PixelRgb>* _outputImage;
static ImageProcessModule *imageProcessModule;


#define _imgRecv (*(ptr_imgRecv))
#define _inputImg (*(ptr_inputImg))
#define _semaphore (*(ptr_semaphore))


bool ImageProcessModule::open(Searchable& config) {
    ct = 0;
	inputImage_flag=false;
    reinit_flag=false;
	
	currentProcessor=0;

    this->openPorts();   
    //ConstString portName2 = options.check("name",Value("/worker2")).asString();
    

    return true;
}

// try to interrupt any communications or resource usage
bool ImageProcessModule::interruptModule() {
	redPlanePort.interrupt();
    bluePlanePort.interrupt();
    greenPlanePort.interrupt();
    
    rgPort.interrupt();
    grPort.interrupt();
    byPort.interrupt();

    inImagePort.interrupt();
    cmdPort.interrupt();
	return true;
}

bool ImageProcessModule::close() {
    this->currentProcessor->stop();
	this->closePorts();
	return true;
	}

void ImageProcessModule::setOptions(yarp::os::Property opt){
	//options	=opt;
    // definition of the name of the module
    ConstString name=opt.find("name").asString();
    if(name!=""){
        printf("|||  Module named as :%s \n", name.c_str());
        this->setName(name.c_str());
    }
}



void ImageProcessModule::createObjects() {
//    ptr_imgRecv = new YARPImgRecv;
    ptr_inputImg = new yarp::sig::ImageOf<yarp::sig::PixelRgb>;
    ptr_semaphore = new yarp::os::Semaphore;
}


bool getImage(){
	bool ret = false;
	//ret = _imgRecv.Update();
	if (ret == false){
		return false;
	}

	_semaphore.wait();
	//ret = _imgRecv.GetLastImage(&_inputImg);
	_semaphore.post();
    printf("Acquired a new image for _imgRecv /n ");
	
	//imageProcessModule->processor1->inImage=&_inputImg;
	//imageProcessModule->processor2->inImage=&_inputImg;
	//imageProcessModule->processor3->inImage=&_inputImg;
	//printf("GetImage: out of the semaphore \n");
	//imageProcessModule->inputImage_flag=true;
	return ret;
}

void ImageProcessModule::setUp()
{
	printf("Module setting up automatically ..../n");
}

bool ImageProcessModule::openPorts(){
	bool ret = false;
    bool ok=true;
    //input ports
    inImagePort.open(getName("image:i"));
	redPlanePort.open(getName("red:i"));
    bluePlanePort.open(getName("blue:i"));
    greenPlanePort.open(getName("green:i"));

    rgPort.open(getName("rg:i"));
    grPort.open(getName("gr:i"));
    byPort.open(getName("by:i"));

    rgEdgesPort.open(getName("rgEdges:o"));
    grEdgesPort.open(getName("grEdges:o"));
    byEdgesPort.open(getName("byEdges:o"));

    cmdPort.open(getName("cmd")); // optional command port
    attach(cmdPort); // cmdPort will work just like terminal
	//outputports
	if (false)
        {
		
            /*_pOutPort = new yarp::os::BufferedPort<ImageOf<PixelRgb> >;
			_pOutPort2 = new yarp::os::BufferedPort<ImageOf<PixelRgb> >;
			_pOutPort3 = new yarp::os::BufferedPort<ImageOf<PixelRgb> >;*/
            printf("Registering port %s on network %s...\n", "/rea/ImageProcessor/out","default");
			printf("Registering port %s on network %s...\n", "/rea/ImageProcessor/out2","default");
			printf("Registering port %s on network %s...\n", "/rea/ImageProcessor/out3","default");
			/*portRg = new yarp::os::BufferedPort<ImageOf<PixelMono> >;
			portGr = new yarp::os::BufferedPort<ImageOf<PixelMono> >;
			portBy = new yarp::os::BufferedPort<ImageOf<PixelMono> >;*/
            printf("Registering port %s on network %s...\n", "/rea/ImageProcessor/outRG","default");
			printf("Registering port %s on network %s...\n", "/rea/ImageProcessor/outGR","default");
			printf("Registering port %s on network %s...\n", "/rea/ImageProcessor/outBY","default");
			/*portRedPlane = new yarp::os::BufferedPort<ImageOf<PixelMono> >;
			portGreenPlane = new yarp::os::BufferedPort<ImageOf<PixelMono> >;
			portBluePlane = new yarp::os::BufferedPort<ImageOf<PixelMono> >;*/
            printf("Registering port %s on network %s...\n", "/rea/ImageProcessor/outRed","default");
			printf("Registering port %s on network %s...\n", "/rea/ImageProcessor/outGreen","default");
			printf("Registering port %s on network %s...\n", "/rea/ImageProcessor/outBlue","default");
            /*bool ok = _pOutPort->open("/rea/ImageProcessor/out");
			ok = _pOutPort2->open("/rea/ImageProcessor/out2");
			ok = _pOutPort3->open("/rea/ImageProcessor/out3");
			ok = portRg->open("/rea/ImageProcessor/outRG");
			ok = portGr->open("/rea/ImageProcessor/outGR");
			ok = portBy->open("/rea/ImageProcessor/outBY");
			ok = portRedPlane->open("/rea/ImageProcessor/outRed");
			ok = portGreenPlane->open("/rea/ImageProcessor/outGreen");
			ok = portBluePlane->open("/rea/ImageProcessor/outBlue");*/
            if  (ok)
                printf("Port registration succeed!\n");
            else 
                {
                    printf("ERROR: Port registration failed.\nQuitting, sorry.\n");
                    return false;
                }

        }

	return true;
}

bool ImageProcessModule::outPorts(){
	bool ret = false;
    if((0!=currentProcessor->redGreenEdges_yarp)&&(rgEdgesPort.getOutputCount())){
        rgEdgesPort.prepare() = *(currentProcessor->redGreenEdges_yarp);		
        rgEdgesPort.write();
        ret=true;
    }
    if((0!=currentProcessor->greenRed_yarp)&&(grEdgesPort.getOutputCount())){
        grEdgesPort.prepare() = *(currentProcessor->greenRed_yarp);		
        grEdgesPort.write();
        ret=true;
    }
    if((0!=currentProcessor->blueYellow_yarp)&&(byEdgesPort.getOutputCount())){
        byEdgesPort.prepare() = *(currentProcessor->blueYellow_yarp);		
        byEdgesPort.write();
        ret=true;
    } 
	return ret;
}

bool ImageProcessModule::closePorts(){
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

    cmdPort.close();

	//closing output ports
	if (false)
        {
		
            /*_pOutPort;
			_pOutPort2 = new yarp::os::BufferedPort<ImageOf<PixelRgb>>;
			_pOutPort3 = new yarp::os::BufferedPort<ImageOf<PixelRgb>>;*/
			
			
            printf("Closing port %s on network %s...\n", "/rea/ImageProcessor/in","default");
			//_imgRecv.Disconnect();
			printf("Closing port %s on network %s...\n", "/rea/ImageProcessor/out","dafult");
			printf("Closing port %s on network %s...\n", "/rea/ImageProcessor/out2","dafult");
			printf("Closing port %s on network %s...\n", "/rea/ImageProcessor/out3","dafult");
			/*_pOutPort->close(); //->open("/rea/ImageProcessor/out");
			_pOutPort2->close(); //open("/rea/ImageProcessor/out2");
			_pOutPort3->close(); //open("/rea/ImageProcessor/out3");*/
			printf("Closing port %s on network %s...\n", "/rea/ImageProcessor/outRG","dafult");
			printf("Closing port %s on network %s...\n", "/rea/ImageProcessor/outGR","dafult");
			printf("Closing port %s on network %s...\n", "/rea/ImageProcessor/outBY","dafult");
			/*portRg->close(); //open("/rea/ImageProcessor/outRG");
			portGr->close(); //open("/rea/ImageProcessor/outGR");
			portBy->close(); //open("/rea/ImageProcessor/outBY");*/
            printf("Closing port %s on network %s...\n", "/rea/ImageProcessor/outRed","dafult");
			printf("Closing port %s on network %s...\n", "/rea/ImageProcessor/outGreen","dafult");
			printf("Closing port %s on network %s...\n", "/rea/ImageProcessor/outBlue","dafult");
			/*portRedPlane->close(); //open("/rea/ImageProcessor/outRed");
			portGreenPlane->close(); //open("/rea/ImageProcessor/outGreen");
			portBluePlane->close(); //open("/rea/ImageProcessor/outBlue");*/

            if(true){
                printf("All ports closed succeed!\n");
            }
            else 
            {
                    printf("ERROR: Ports closing failed.\nQuitting, sorry.\n");
                    return false;
            }

        }

	return ret;
}



void ImageProcessModule::reinitialise(int weight, int height){
    inputImg=new ImageOf<PixelRgb>;
    inputImg->resize(weight,height);
}


bool ImageProcessModule::updateModule() {
    /*this->inputImg = this->inImagePort.read(false);
    if(0==inputImg)
        return true;*/
    tmp=rgPort.read(false);
    if(tmp==0){
        return true;
    }
    
    if(!reinit_flag){
        //srcsize.height=img->height();
        //srcsize.width=img->width();
        reinitialise(tmp->width(), tmp->height());
        reinit_flag=true;
        currentProcessor=new ImageProcessor();
        currentProcessor->resizeImages(tmp->width(),tmp->height());
        startImageProcessor();
    }
    currentProcessor->redGreen_yarp=tmp;

    currentProcessor->redPlane=redPlanePort.read(false);
    if(0!=currentProcessor->redGreen_yarp){
        currentProcessor->redGreen_flag=true;
        currentProcessor->redGreenEdges_yarp=currentProcessor->findEdgesRedOpponency();
    }
    
    currentProcessor->bluePlane=bluePlanePort.read(false);
    currentProcessor->blueYellow_yarp=byPort.read(false);
    if(0!=currentProcessor->blueYellow_yarp){
        currentProcessor->blueYellow_flag=true;
        currentProcessor->blueYellow_yarp=currentProcessor->findEdgesBlueOpponency();
    }
    
    currentProcessor->greenPlane=greenPlanePort.read(false);
    currentProcessor->greenRed_yarp=grPort.read(false);
    if(0!=currentProcessor->greenRed_yarp){
        currentProcessor->blueYellow_flag=true;
        currentProcessor->greenRedEdges_yarp=currentProcessor->findEdgesGreenOpponency();
    }

    outPorts();
    return true;
}

bool ImageProcessModule::startImageProcessor(){
    printf("image processor starting ..... \n");
    this->currentProcessor->start();
    return true;
}

bool ImageProcessModule::respond(const Bottle &command,Bottle &reply){
        
    bool ok = false;
    bool rec = false; // is the command recognized?

    mutex.wait();
    switch (command.get(0).asVocab()) {
    case COMMAND_VOCAB_HELP:
        rec = true;
        {
            reply.addString("help");

            reply.addString("\n");
            reply.addString("get fn \t: general get command \n");
            

            reply.addString("\n");
            reply.addString("set s1 <s> \t: general set command \n");

            reply.addString("\n");
            reply.addString("run rgb : run the rgb processor \n");
            reply.addString("run yuv : run the yuv processor");
            

            reply.addString("\n");


            ok = true;
        }
        break;
    case COMMAND_VOCAB_NAME:
        rec = true;
        {
            // check and change filter name to pass on to the next filter
            string fName(command.get(1).asString());
            string subName;
            Bottle subCommand;
            int pos=1;
            //int pos = fName.find_first_of(filter->getFilterName());
            if (pos == 0){
                pos = fName.find_first_of('.');
                if (pos  > -1){ // there is a subfilter name
                    subName = fName.substr(pos + 1, fName.size()-1);
                    subCommand.add(command.get(0));
                    subCommand.add(Value(subName.c_str()));
                }
                for (int i = 2; i < command.size(); i++)
                    subCommand.add(command.get(i));
                //ok = filter->respond(subCommand, reply);
            }
            else{
                printf("filter name  does not match top filter name ");
                ok = false;
            }
        }
        break;
    case COMMAND_VOCAB_SET:
        rec = true;
        {
            switch(command.get(1).asVocab()) {
            case COMMAND_VOCAB_SALIENCE_THRESHOLD:{
                double thr = command.get(2).asDouble();
            }
                break;
            case COMMAND_VOCAB_NUM_BLUR_PASSES:{
                int nb = command.get(2).asInt();
                //reply.addString("connection 2");
              
                ok=true;
            }
                break;
            /*case COMMAND_VOCAB_TEMPORAL_BLUR:{
                int size = command.get(2).asInt();
                ok = this->setTemporalBlur(size);
            }*/
                break;
            case COMMAND_VOCAB_NAME:{
                string s(command.get(2).asString().c_str());
                reply.addString("connection 1");
                ok=true;
            }
                break;
            case COMMAND_VOCAB_CHILD_NAME:{
                int j = command.get(2).asInt();
                string s(command.get(3).asString().c_str());
            }
                break;
            case COMMAND_VOCAB_WEIGHT:{
                double w = command.get(2).asDouble();
            }
                break;
            case COMMAND_VOCAB_CHILD_WEIGHT:{
                int j = command.get(2).asInt();
                double w = command.get(3).asDouble();
            }
                break;
            case COMMAND_VOCAB_CHILD_WEIGHTS:{
                Bottle weights;
                for (int i = 2; i < command.size(); i++)
                    weights.addDouble(command.get(i).asDouble());
            }
                break;
            default:
                cout << "received an unknown request after a SALIENCE_VOCAB_SET" << endl;
                break;
            }
        }
        break;
     case COMMAND_VOCAB_RUN:
        rec = true;
        {
            switch(command.get(1).asVocab()) {
            case COMMAND_VOCAB_RGB_PROCESSOR:{
                printf("RUN RGB \n");

                ok=true;
            }
                break;
            case COMMAND_VOCAB_YUV_PROCESSOR:{
                printf("RUN YUV \n");
                
                ok=true;
            }
                break;
            default:
                cout << "received an unknown request after a _VOCAB_RUN" << endl;
                break;
            }
        }
        break;
    case COMMAND_VOCAB_GET:
        rec = true;
        {
            reply.addVocab(COMMAND_VOCAB_IS);
            reply.add(command.get(1));
            switch(command.get(1).asVocab()) {
            case COMMAND_VOCAB_SALIENCE_THRESHOLD:{
                double thr=0.0;
                reply.addDouble(thr);
                ok = true;
            }
                break;
            case COMMAND_VOCAB_NUM_BLUR_PASSES:{
                int nb = 0;
                reply.addInt(nb);
                ok = true;
            }
                break;
            case COMMAND_VOCAB_NAME:{
                string s(" ");
                reply.addString(s.c_str());
                ok = true;
            }
                break;
            case COMMAND_VOCAB_CHILD_NAME:{
                int j = command.get(2).asInt();
                string s(" ");
                reply.addString(s.c_str());
                ok = true;
            }
                break;
            case COMMAND_VOCAB_CHILD_COUNT:{
                int count =0;
                reply.addInt(count);
                ok = true;
            }
                break;
            case COMMAND_VOCAB_WEIGHT:{
                double w = 0.0;
                reply.addDouble(w);
                ok = true;
            }
                break;
            case COMMAND_VOCAB_CHILD_WEIGHT:{
                int j = command.get(2).asInt();
                double w = 0.0;
                reply.addDouble(w);
                ok = true;
            }
                break;
            case COMMAND_VOCAB_CHILD_WEIGHTS:{
                Bottle weights;
                //ok = filter->getChildWeights(&weights);
                for (int k = 0; k < weights.size(); k++)
                    reply.addDouble(0.0);
            }
                break;
            default:
                cout << "received an unknown request after a SALIENCE_VOCAB_GET" << endl;
                break;
            }
        }
        break;

    }
    mutex.post();

    if (!rec)
        ok = Module::respond(command,reply);
    
    if (!ok) {
        reply.clear();
        reply.addVocab(COMMAND_VOCAB_FAILED);
    }
    else
        reply.addVocab(COMMAND_VOCAB_OK);

    return ok;
} 	


