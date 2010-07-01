#include <iCub/colourProcessorModule.h>
#include <yarp/os/Network.h>

#include <string.h>
#include <time.h>
#include <iostream>

using namespace std;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp;

colourProcessorModule::colourProcessorModule(){
    reinit_flag=false;
    startrgb_flag=true;
    startyuv_flag=false;
}
 

bool colourProcessorModule::configure(ResourceFinder &rf)
{
    //initialization
    ct=0;
    dif=0;
    initflag=false;
    time_t start;
    //time_t end;
    time (&start);

    Time::turboBoost();
    cmdPort.open(getName("/cmd:i"));
    attach(cmdPort);
    attachTerminal();
    printf("resource finder configuration after time turbo boosting \n");
    
    interThread.setName(this->getName().c_str());
    printf("name:%s \n",this->getName().c_str());
    interThread.start();

    printf("\n waiting for connection of the input port \n");

    return true;
}

/**
* ****** DEPRECATED ********
*function that opens the module
*/
bool colourProcessorModule::open(Searchable& config) {
    interThread.start();
    ct = 0;
    //ConstString portName2 = options.check("name",Value("/worker2")).asString();
    startRgbProcessor();
    if(startyuv_flag){
            startYuvProcessor();
    }

    cmdPort.open(getName("cmd:i"));
    attach(cmdPort);
    
    return true;
}
// ***** DEPRECATED *****


/** 
* tries to interrupt any communications or resource usage
*/
bool colourProcessorModule::interruptModule() {
    //interThread.interrupt();
    cmdPort.interrupt();
    interThread.interrupt();
    return true;
}


bool colourProcessorModule::close(){
    

    //rgbProcessor.threadRelease();
    //yuvProcessor.threadRelease();
    //interThread.threadRelease();

    rgbProcessor.stop();
    yuvProcessor.stop();
    interThread.stop();

    cmdPort.close();
   
    return true;
}

void colourProcessorModule::setOptions(yarp::os::Property opt){
    //options	=opt;
    // definition of the mode
    // definition of the name of the module
    ConstString name=opt.find("name").asString();
    if(name!=""){
        printf("|||  Module named as :%s \n", name.c_str());
        this->setName(name.c_str());
    }
    ConstString yuvoption=opt.find("yuvprocessor").asString();
    if(yuvoption!=""){
        printf("|||  Module named as :%s \n", yuvoption.c_str());
        if(!strcmp(yuvoption.c_str(),"ON")){
            printf(" yuv processor starting.... \n");
            startyuv_flag=true;
        }
        //this->setName(name.c_str());
    }
    ConstString rgboption=opt.find("rgbprocessor").asString();
    if(rgboption!=""){
        printf("|||  Module named as :%s \n", rgboption.c_str());
        //this->setName(name.c_str());
    }
    
}

bool colourProcessorModule::updateModule() {
   
    /*Bottle *bot=portTarget.read(false);
    if(bot!=NULL){
        int intValues[4];    
        for(int i=0;i<4;i++){
            yarp::os::Value v=bot->pop();
            printf("integer:%d \n", v.asInt());
            intValues[3-i]=v.asInt();
        }
        targetLeftX=intValues[0];
        targetLeftY=intValues[1];
        targetRightX=intValues[2];
        targetRightY=intValues[3];
        std::string *commandTOT=new string(bot->toString().c_str());
        printf("%s \n", commandTOT->c_str());
    }*/

    if((0!=interThread.inputImg)&&(this->initflag)){
    
        /*
        while(interThread.inputImg->width()==0){
            
        }
        */
	    printf("input port activated! starting the processes ....\n");    

        //ConstString portName2 = options.check("name",Value("/worker2")).asString();
        //starting rgb thread and linking all the images
        startRgbProcessor();
        interThread.redPlane=rgbProcessor.redPlane;
        interThread.greenPlane=rgbProcessor.greenPlane;
        interThread.bluePlane=rgbProcessor.bluePlane;
        interThread.redGreen_yarp=rgbProcessor.redGreen_yarp;
        interThread.greenRed_yarp=rgbProcessor.greenRed_yarp;
        interThread.blueYellow_yarp=rgbProcessor.blueYellow_yarp;

        //starting yuv thread and linking all the images
        if(startyuv_flag){
                startYuvProcessor();
        }
        interThread.uvPlane=yuvProcessor.uvPlane;
        interThread.uPlane=yuvProcessor.uPlane;
        interThread.vPlane=yuvProcessor.vPlane;
        interThread.yPlane=yuvProcessor.yPlane;

        initflag=true;
    }
    
    return true;
}



void colourProcessorModule::startRgbProcessor(){
    //rgbProcessorThread rgbProcessor();
    rgbProcessor.setInputImage(interThread.inputImg);
    //rgbProcessor.resize(width,height);
    rgbProcessor.start();
}


void colourProcessorModule::startYuvProcessor(){
    //rgbProcessorThread rgbProcessor();
    yuvProcessor.setInputImage(rgbProcessor.redPlane,rgbProcessor.greenPlane,rgbProcessor.bluePlane);
    //rgbProcessor.resize(width,height);
    yuvProcessor.start();
}
void colourProcessorModule::reinitialise(int weight, int height){
    
}


bool colourProcessorModule::respond(const Bottle &command,Bottle &reply){
        
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
                startRgbProcessor();
                ok=true;
            }
                break;
            case COMMAND_VOCAB_YUV_PROCESSOR:{
                printf("RUN YUV \n");
                startYuvProcessor();
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

//----- end-of-file --- ( next line intentionally left blank ) ------------------
