#include <iCub/colourProcessorModule.h>


#include <string.h>
#include <iostream>

using namespace std;
using namespace yarp::sig;

colourProcessorModule::colourProcessorModule(){
    reinit_flag=false;
    startrgb_flag=true;
    startyuv_flag=false;
}


/**
*function that opens the module
*/
bool colourProcessorModule::open(Searchable& config) {
    ct = 0;
    //ConstString portName2 = options.check("name",Value("/worker2")).asString();
    inputPort.open(getName("image:i"));
    
    redPort.open(getName("red:o"));
    greenPort.open(getName("green:o"));
    bluePort.open(getName("blue:o"));

    rgPort.open(getName("rg:o"));
    grPort.open(getName("gr:o"));
    byPort.open(getName("by:o"));

    yPort.open(getName("ychannel:o"));
    uPort.open(getName("uchannel:o"));
    vPort.open(getName("vchannel:o"));
    uvPort.open(getName("uvchannel:o"));

    cmdPort.open(getName("cmd:i"));
    attach(cmdPort);
    
    return true;
}

/** 
* tries to interrupt any communications or resource usage
*/
bool colourProcessorModule::interruptModule() {
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

    cmdPort.interrupt();
    
    return true;
}


bool colourProcessorModule::close(){
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

    cmdPort.close();

    rgbProcessor.threadRelease();
    yuvProcessor.threadRelease();
   
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
    
    
    img = this->inputPort.read(false);
    if(0==img)
        return true;

    if(!reinit_flag){
        
	    srcsize.height=img->height();
	    srcsize.width=img->width();
        reinitialise(img->width(), img->height());
        reinit_flag=true;
        startRgbProcessor();
        if(startyuv_flag){
            startYuvProcessor();
        }

    }

    //copy the inputImg into a buffer
    ippiCopy_8u_C3R(img->getRawImage(), img->getRowSize(),inputImg->getRawImage(), inputImg->getRowSize(),srcsize);
   
  
    outPorts();
    return true;
}

void colourProcessorModule::outPorts(){


    //port2.prepare() = *img;	
    if((this->rgbProcessor.redPlane!=0)&&(redPort.getOutputCount())){
        redPort.prepare() = *(this->rgbProcessor.redPlane);		
        redPort.write();
    }
    if((this->rgbProcessor.bluePlane!=0)&&(bluePort.getOutputCount())){
        bluePort.prepare() = *(this->rgbProcessor.bluePlane);		
        bluePort.write();
    }
    if((this->rgbProcessor.greenPlane!=0)&&(greenPort.getOutputCount())){
        greenPort.prepare() = *(this->rgbProcessor.greenPlane);		
        greenPort.write();
    }
    if((this->yuvProcessor.yPlane!=0)&&(yPort.getOutputCount())){
        yPort.prepare() = *(this->yuvProcessor.yPlane);		
        yPort.write();
    }
    if((this->yuvProcessor.uPlane!=0)&&(uPort.getOutputCount())){
        uPort.prepare() = *(this->yuvProcessor.uPlane);		
        uPort.write();
    }
    if((this->yuvProcessor.vPlane!=0)&&(vPort.getOutputCount())){
        vPort.prepare() = *(this->yuvProcessor.vPlane);		
        vPort.write();
    }
    if((this->yuvProcessor.uvPlane!=0)&&(uvPort.getOutputCount())){
        uvPort.prepare() = *(this->yuvProcessor.uvPlane);		
        uvPort.write();
    }
    if((this->rgbProcessor.redGreen_yarp!=0)&&(rgPort.getOutputCount())){
        rgPort.prepare()=*(this->rgbProcessor.redGreen_yarp);
        rgPort.write();
    }
    if((this->rgbProcessor.greenRed_yarp!=0)&&(grPort.getOutputCount())){
        grPort.prepare()=*(this->rgbProcessor.greenRed_yarp);
        grPort.write();
    }
    if((this->rgbProcessor.blueYellow_yarp!=0)&&(byPort.getOutputCount())){
        byPort.prepare()=*(this->rgbProcessor.blueYellow_yarp);
        byPort.write();
    }
}

void colourProcessorModule::startRgbProcessor(){
    //rgbProcessorThread rgbProcessor();
    rgbProcessor.setInputImage(inputImg);
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
    inputImg=new ImageOf<PixelRgb>;
    inputImg->resize(weight,height);
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
        ok = Module::respond(command,reply);
    
    if (!ok) {
        reply.clear();
        reply.addVocab(COMMAND_VOCAB_FAILED);
    }
    else
        reply.addVocab(COMMAND_VOCAB_OK);

    return ok;
} 	


