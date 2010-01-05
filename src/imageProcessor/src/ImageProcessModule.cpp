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
    printf("interrupting the module.. \n");
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
    cmdPort.interrupt();
	return true;
}

bool ImageProcessModule::close() {
    printf("Closing the module ... \n");
    if(0!=currentProcessor){
        printf("Thread running! Closing the thread ... \n");
        this->currentProcessor->stop();
    }
	this->closePorts();
    printf("The module has been successfully closed ... \n");
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
    ConstString value=opt.find("mode").asString();
    if(value!=""){
       if(value=="OPENCVSOBEL")
        this->OPENCVSOBEL=true;
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

    edgesPort.open(getName("edges:o"));

    cmdPort.open(getName("cmd")); // optional command port
    attach(cmdPort); // cmdPort will work just like terminal
	

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
    if((0!=currentProcessor->edges_yarp)&&(edgesPort.getOutputCount())){
        edgesPort.prepare() = *(currentProcessor->edges_yarp);		
        edgesPort.write();
        ret=true;
    } 
	return ret;
}

bool ImageProcessModule::closePorts(){
    printf("Closing all the ports ... \n");
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

    cmdPort.close();
    
    printf("All the ports successfully closed ... \n");
	return ret;
}



void ImageProcessModule::reinitialise(int weight, int height){
    inputImg=new ImageOf<PixelRgb>;
    inputImg->resize(weight,height);
}


bool ImageProcessModule::updateModule() {
    //check for any possible command
    Bottle* command=cmdPort.read(false);
    if(command!=0){
        //Bottle* tmpBottle=cmdPort.read(false);
        ConstString str= command->toString();
        printf("command received: %s \n", str.c_str());
        Bottle* reply=new Bottle();
        this->respond(*command,*reply);
    }

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
        //passes the temporary variable for the mode
        currentProcessor->OPENCVSOBEL=OPENCVSOBEL;
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
        currentProcessor->blueYellowEdges_yarp=currentProcessor->findEdgesBlueOpponency();
    }
    
    currentProcessor->greenPlane=greenPlanePort.read(false);
    currentProcessor->greenRed_yarp=grPort.read(false);
    if(0!=currentProcessor->greenRed_yarp){
        currentProcessor->greenRed_flag=true;
        currentProcessor->greenRedEdges_yarp=currentProcessor->findEdgesGreenOpponency();
    }

    if((currentProcessor->redGreen_flag)&&(currentProcessor->greenRed_flag)&&(currentProcessor->blueYellow_flag)){
        currentProcessor->edges_yarp=currentProcessor->combineMax();
        //lineMax();
        outPorts();
    }

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


