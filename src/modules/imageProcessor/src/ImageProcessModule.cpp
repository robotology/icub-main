// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include <iCub/ImageProcessModule.h>
#include <iostream>
#include <time.h>

 
using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;

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

static void sleep(unsigned int mseconds)
{
    clock_t goal = mseconds + clock();
    while (goal > clock());
}


bool ImageProcessModule::configure(ResourceFinder &rf)
{
    ct = 0;
    linkct=0;
    inputImage_flag=false;
    reinit_flag=false;
    currentProcessor=0;
    double dif=0;
    time_t start,end;
    time (&start);
    interThread=new interactionThread();
    
    Time::turboBoost();
    this->openPorts();  
    printf("resource finder configuration after time turbo boosting \n");
    
    
    interThread->setName(getName().c_str());
    interThread->start();


    //inputImg=0;
    while((interThread->tmp==0)&&(linkct<20)){
        printf("time to automatic shut down:  %d (sec) ...   \n", 20-linkct);
        sleep(1000);
        time (&end);
        dif = difftime (end,start);
        linkct++;
    }
    if(linkct>=20)
        return false;

    while(interThread->redGreen_yarp->width()==0){
        
    }
    interThread->width=interThread->redGreen_yarp->width();
    interThread->height=interThread->redGreen_yarp->height();

    
    //ConstString portName2 = options.check("name",Value("/worker2")).asString();

    currentProcessor=new ImageProcessor();
    //passes the temporary variable for the mode
    currentProcessor->OPENCVSOBEL=OPENCVSOBEL;
    currentProcessor->IPPISOBEL=IPPISOBEL;
    currentProcessor->IPPICROSS=IPPICROSS;
    currentProcessor->resizeImages(interThread->width,interThread->height);
    

    //linking of the shared images
    interThread->redPlane=currentProcessor->redPlane;
    interThread->bluePlane=currentProcessor->bluePlane;
    interThread->greenPlane=currentProcessor->greenPlane;

    currentProcessor->redGreen_yarp=interThread->redGreen_yarp;
    currentProcessor->greenRed_yarp=interThread->greenRed_yarp;
    currentProcessor->blueYellow_yarp=interThread->blueYellow_yarp;

    currentProcessor->redGreen_flag=interThread->redGreen_flag;
    currentProcessor->greenRed_flag=interThread->redGreen_flag;
    currentProcessor->blueYellow_flag=interThread->blueYellow_flag;

    interThread->edges_yarp=currentProcessor->edges_yarp;

    currentProcessor->start();

    return true;       
}

/**

bool ImageProcessModule::open(Searchable& config) {
    ct = 0;
    double dif=0;
    time_t start,end;
    time (&start);
	inputImage_flag=false;
    reinit_flag=false;

    Time::turboBoost();
    printf("resource finder configuration after time turbo boosting \n");
    
    interThread->setName("/imagePU/");
    interThread->start();

	currentProcessor=0;
    //inputImg=0;
    
    
    while((interThread->tmp==0)&&(dif<20)){
        time (&end);
        dif = difftime (end,start);
    }
    if(dif>=20)
        return false;

    while(interThread->inputImg->width()==0){
        
    }

    this->openPorts();   
    attach(cmdPort);
    attachTerminal();

    //ConstString portName2 = options.check("name",Value("/worker2")).asString();
    
    return true;
}
*/

// try to interrupt any communications or resource usage
bool ImageProcessModule::interruptModule() {
    linkct=199;
    printf("interrupting the module.. \n");
    cmdPort.interrupt();
    close();
	return true;
}

bool ImageProcessModule::close() {
    
    printf("Closing the module ... \n");
    if(0!=currentProcessor){
        printf("Thread running! Closing the thread ... \n");
        this->currentProcessor->stop();
    }
    if(0!=interThread){
        printf(" Interaction thread running! Closing the thread ... \n");
        interThread->stop();
    }
	//closePorts();
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
       else
        this->OPENCVSOBEL=false;
       if(value=="IPPISOBEL")
        this->IPPISOBEL=true;
       else
        this->IPPISOBEL=false;
       if(value=="IPPICROSS")
        this->IPPICROSS=true;
       else
        this->IPPICROSS=false;

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




void ImageProcessModule::reinitialise(int weight, int height){
    
       
}


bool ImageProcessModule::updateModule() {
    return true;
}

bool ImageProcessModule::startImageProcessor(){
    printf("image processor starting ..... \n");
    this->currentProcessor->start();
    return true;
}

void ImageProcessModule::resetFlags(){
    currentProcessor->CONVFILTER=false;
    currentProcessor->CONVMAX=false;
    currentProcessor->CONVSEQ=false;
    currentProcessor->IPPISOBEL=false;
    currentProcessor->OPENCVSOBEL=false;
    currentProcessor->IPPICROSS=false;
}

bool ImageProcessModule::openPorts(){
	
    cmdPort.open(getName("/cmd").c_str()); // optional command port
    attach(cmdPort); // cmdPort will work just like terminal
    attachTerminal();

	return true;
}



bool ImageProcessModule::closePorts(){
    printf("Closing all the ports ... \n");
    cmdPort.close();
    printf("All the ports successfully closed ... \n");

    /*if(inputImg!=0)
        delete inputImg;
    if(currentProcessor!=0)
        delete currentProcessor;
    */

	return true;
}

/*void ImageProcessModule::setName(std::string str){
    this->name=str; 
}*/


/*std::string ImageProcessModule::getName(const char* p){
    string str(name);
    str.append(p);
    printf("name: %s", name.c_str());
    return str;
}*/

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
            reply.addString("run s1 <s> \t: general run command \n");

            reply.addString("\n");
            reply.addString("run rgb : run the rgb processor \n");
            reply.addString("run yuv : run the yuv processor");

            reply.addString("\n");
            reply.addString("set s1 <s> \t: general run command \n");

            reply.addString("\n");
            reply.addString("set fil : set the convolution filter mode \n");
            reply.addString("set max : set the convolution max filter mode \n");
            reply.addString("set ocv : set the sobel mode (opencv algorightm) \n");
            reply.addString("set ipp : set the sobel mode(ipp algorithm) \n");
            reply.addString("set seq : set convolution in sequence mode \n");
            
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
            case COMMAND_VOCAB_FIL:{
                resetFlags();
                this->currentProcessor->CONVFILTER=true;
                ok = true;
            }
            break;
            case COMMAND_VOCAB_MAX:{
                resetFlags();
                this->currentProcessor->CONVMAX=true;
                ok = true;
            }
            break;
            case COMMAND_VOCAB_SEQ:{
                resetFlags();
                this->currentProcessor->CONVSEQ=true;
                ok = true;
            }
            break;
            case COMMAND_VOCAB_IPP:{
                resetFlags();
                this->currentProcessor->IPPISOBEL=true;
                ok = true;
            }
            break;
            case COMMAND_VOCAB_OCV:{
                resetFlags();
                this->currentProcessor->OPENCVSOBEL=true;
                ok = true;
            }
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
        ok = RFModule::respond(command,reply);
    
    if (!ok) {
        reply.clear();
        reply.addVocab(COMMAND_VOCAB_FAILED);
    }
    else
        reply.addVocab(COMMAND_VOCAB_OK);

    return ok;
} 	


