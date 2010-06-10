// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include <iCub/selectiveAttentionModule.h>


#include <iostream>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;


#define COMMAND_VOCAB_SET VOCAB3('s','e','t')
#define COMMAND_VOCAB_GET VOCAB3('g','e','t')
#define COMMAND_VOCAB_KBU VOCAB3('k','b','u') //weight of the bottom-up algorithm
#define COMMAND_VOCAB_KTD VOCAB3('k','t','d') //weight of top-down algorithm
#define COMMAND_VOCAB_RIN VOCAB3('r','i','n') //red intensity value
#define COMMAND_VOCAB_GIN VOCAB3('g','i','n') //green intensity value
#define COMMAND_VOCAB_BIN VOCAB3('b','i','n') //blue intensity value

// Image Receiver
//static YARPImgRecv *ptr_imgRecv;
// Image to Display
static yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputImg=0;
// Semaphore
static yarp::os::Semaphore *ptr_semaphore;
// Timeout ID
//static guint timeout_ID;
static yarp::sig::ImageOf<yarp::sig::PixelRgb>* _outputImage;
static selectiveAttentionModule *selectiveAttentionModule;


#define _imgRecv (*(ptr_imgRecv))
#define _inputImg (*(ptr_inputImg))
#define _semaphore (*(ptr_semaphore))


bool selectiveAttentionModule::open(Searchable& config) {
    ct = 0;
	inputImage_flag=false;
    reinit_flag=false;

	currentProcessor=0;
    inputImg=0;
    tmp=0;
    tmp2=0;

    targetRED=0;
    targetGREEN=0;
    targetBLUE=0;
    salienceTD=0.0;
    salienceBU=1.0;
    
    time (&start);


    openPorts();   
    //ConstString portName2 = options.check("name",Value("/worker2")).asString();
    
    return true;
}

// try to interrupt any communications or resource usage
bool selectiveAttentionModule::interruptModule() {
    printf("interrupting the module.. \n");
	map1Port.interrupt();
    map2Port.interrupt();
    map3Port.interrupt();
    
    map4Port.interrupt();
    map5Port.interrupt();
    map6Port.interrupt();
    
    selectedAttentionPort.interrupt();
    linearCombinationPort.interrupt();
    centroidPort.interrupt();
    feedbackPort.interrupt();
    
    inImagePort.interrupt();
    cmdPort.interrupt();
	return true;
}

bool selectiveAttentionModule::close() {
    
    printf("Closing the module ... \n");
    if(0!=currentProcessor){
        printf("Thread running! Closing the thread ... \n");
        this->currentProcessor->stop();
    }
	this->closePorts();
    printf("The module has been successfully closed ... \n");
	return true;
}

void selectiveAttentionModule::setOptions(yarp::os::Property opt){
	//options	=opt;
    // definition of the name of the module
    ConstString name=opt.find("name").asString();
    if(name!=""){
        printf("|||  Module named as :%s \n", name.c_str());
        this->setName(name.c_str());
    }
    ConstString value=opt.find("mode").asString();
    if(value!=""){
       
    }
}



void selectiveAttentionModule::createObjects() {
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
	
	//selectiveAttentionModule->processor1->inImage=&_inputImg;
	//selectiveAttentionModule->processor2->inImage=&_inputImg;
	//selectiveAttentionModule->processor3->inImage=&_inputImg;
	//printf("GetImage: out of the semaphore \n");
	//selectiveAttentionModule->inputImage_flag=true;
	return ret;
}

void selectiveAttentionModule::setUp()
{
	printf("Module setting up automatically ..../n");
}

bool selectiveAttentionModule::openPorts(){
	bool ret = false;
    bool ok=true;
    //input ports 
    inImagePort.open(getName("image:i"));
    map1Port.open(getName("map1:i")); //
    map2Port.open(getName("map2:i"));; //
    map3Port.open(getName("map3:i"));; //	 
    map4Port.open(getName("map4:i"));; 
    map5Port.open(getName("map5:i"));; 
    map6Port.open(getName("map6:i"));; 

    

    selectedAttentionPort.open(getName("attention:o"));
    linearCombinationPort.open(getName("combination:o"));
    centroidPort.open(getName("centroid:o"));
    feedbackPort.open(getName("feedback:o"));

    cmdPort.open(getName("cmd")); // optional command port
    attach(cmdPort); // cmdPort will work just like terminal

	return true;
}

bool selectiveAttentionModule::outPorts(){
	bool ret = false;
    if((0!=currentProcessor->linearCombinationImage)&&(linearCombinationPort.getOutputCount())){
        linearCombinationPort.prepare() = *(currentProcessor->linearCombinationImage);
        linearCombinationPort.write();
    }
    
    if((0!=currentProcessor->outputImage)&&(selectedAttentionPort.getOutputCount())){
        selectedAttentionPort.prepare() = *(currentProcessor->outputImage);
        selectedAttentionPort.write();
    }	

    if(centroidPort.getOutputCount()){  
        Bottle& commandBottle=centroidPort.prepare();
        commandBottle.clear();
        commandBottle.addVocab(Vocab::encode("sac"));
        commandBottle.addVocab(Vocab::encode("img"));
        commandBottle.addInt(currentProcessor->centroid_x);
        commandBottle.addInt(currentProcessor->centroid_y);
        centroidPort.write();
    }

    if(feedbackPort.getOutputCount()){  
        //Bottle& commandBottle=feedbackPort.prepare();
        Bottle in,commandBottle;
        commandBottle.clear();
        
        
        time (&end);
        double dif = difftime (end,start);
        if(dif>30+2){
                //restart the time interval
                 time(&start);
        }
        else if((dif>2)&&(dif<30+2)){
            //setting coefficients
            commandBottle.clear();
            commandBottle.addVocab(VOCAB3('s','e','t'));
            commandBottle.addVocab(VOCAB3('k','t','d'));
            salienceTD=salienceTD+0.1;
        
            //if(salienceTD>0.99)
                salienceTD=1.0;
            printf("salienceTD \n");
            commandBottle.addDouble((double)salienceTD);
            feedbackPort.write(commandBottle,in);
            commandBottle.clear();
            commandBottle.addVocab(VOCAB3('s','e','t'));
            commandBottle.addVocab(VOCAB3('k','b','u'));
            salienceBU=salienceBU-0.1;
            
            //if(salienceBU<=0)
                salienceBU=0;
            commandBottle.addDouble((double)salienceBU);
            feedbackPort.write(commandBottle,in);    
            printf("read: %f,%f,%f \n",(double)targetRED,(double)targetGREEN,(double)targetBLUE);
            
        }
        else{
            printf("salienceBU \n");
            commandBottle.addVocab(VOCAB3('s','e','t'));
            commandBottle.addVocab(VOCAB3('k','t','d'));
            salienceTD=0.0;
            commandBottle.addDouble((double)salienceTD);
            feedbackPort.write(commandBottle,in);
            commandBottle.clear();
            commandBottle.addVocab(VOCAB3('s','e','t'));
            commandBottle.addVocab(VOCAB3('k','b','u'));
            salienceBU=1.0;
            commandBottle.addDouble((double)salienceBU);
            feedbackPort.write(commandBottle,in);
            commandBottle.clear();
            commandBottle.addVocab(VOCAB3('s','e','t'));
            commandBottle.addVocab(VOCAB3('r','i','n'));
            commandBottle.addDouble((double)currentProcessor->targetRed);
            //commandBottle.addDouble(255.0);
            feedbackPort.write(commandBottle,in);
            commandBottle.clear();
            commandBottle.addVocab(VOCAB3('s','e','t'));
            commandBottle.addVocab(VOCAB3('g','i','n'));
            commandBottle.addDouble((double)currentProcessor->targetGreen);
            //commandBottle.addDouble(0.0);
            feedbackPort.write(commandBottle,in);
            commandBottle.clear();
            commandBottle.addVocab(VOCAB3('s','e','t'));
            commandBottle.addVocab(VOCAB3('b','i','n'));
            commandBottle.addDouble((double)currentProcessor->targetBlue);
            feedbackPort.write(commandBottle,in);
            commandBottle.clear();
            //commandBottle.addDouble(0.0);
            printf("%f,%f,%f \n",(double)currentProcessor->targetRed,(double)currentProcessor->targetGreen,(double)currentProcessor->targetBlue);
        }

        
    }

    
    return ret;
}

bool selectiveAttentionModule::closePorts(){
    printf("Closing all the ports ... \n");
	bool ret = false;
	//int res = 0;
	// Closing Port(s)
    //reduce verbosity --paulfitz

    //closing input ports
    inImagePort.close();
    map1Port.close();
    map2Port.close();
    map3Port.close();
    map4Port.close();
    map5Port.close();
    map6Port.close();

    selectedAttentionPort.close();
    linearCombinationPort.close();
    centroidPort.close();
    feedbackPort.close();
    
    cmdPort.close();
    printf("All the ports successfully closed ... \n");

	return ret;
}



void selectiveAttentionModule::reinitialise(int width, int height){
    inputImg=new ImageOf<PixelRgb>;
    inputImg->resize(width,height);
    tmp=new ImageOf<PixelMono>;
    tmp->resize(width,height);
    tmp2=new ImageOf<PixelRgb>;
    tmp2->resize(width,height);
}


bool selectiveAttentionModule::updateModule() {
    string name;

    /*this->inputImg = this->inImagePort.read(false);
    if(0==inputImg)
        return true;*/

    //-----------check for any possible command
    Bottle* command=cmdPort.read(false);
    if(command!=0){
        //Bottle* tmpBottle=cmdPort.read(false);
        ConstString str= command->toString();
        printf("command received: %s \n", str.c_str());
        Bottle* reply=new Bottle();
        this->respond(*command,*reply);
        command->clear();
    }
    //--------read value from the preattentive level
    if(feedbackPort.getOutputCount()){
        Bottle in,out;
        out.clear();
        out.addString("get");
        out.addString("ktd");
        feedbackPort.write(out,in);
        name=in.pop().asString();
        salienceTD=in.pop().asDouble();
        out.clear();
        in.clear();
        
        out.addString("get");
        out.addString("kbu");
        feedbackPort.write(out,in);
        name=in.pop().asString();
        salienceBU=in.pop().asDouble();
        out.clear();
        in.clear();
        
        out.addString("get");
        out.addString("rin");
        feedbackPort.write(out,in);
        name=in.pop().asString();
        targetRED=in.pop().asInt();
        out.clear();
        in.clear();
        
        out.addString("get");
        out.addString("gin");
        feedbackPort.write(out,in);
        name=in.pop().asString();
        targetGREEN=in.pop().asInt();
        out.clear();
        in.clear();
        
        out.addString("get");
        out.addString("bin");
        feedbackPort.write(out,in);
        name=in.pop().asString();
        targetBLUE=in.pop().asDouble();
        out.clear();
        in.clear();
    }

    //

    //-------------read input maps
    //if(map1Port.getInputCount()){
    //    tmp=map1Port.read(false);
    //}
    if(inImagePort.getInputCount()){
        tmp2=inImagePort.read(false);
    }
    if(tmp2==0){
        return true;
    }
    
    if(!reinit_flag){
        //srcsize.height=img->height();
        //srcsize.width=img->width();
        reinitialise(tmp2->width(), tmp2->height());
        reinit_flag=true;
        currentProcessor=new selectiveAttentionProcessor();
        //passes the temporary variable for the mode
        currentProcessor->resizeImages(tmp2->width(),tmp2->height());
        startselectiveAttentionProcessor();
        currentProcessor->setIdle(false);
    }
    
    currentProcessor->inImage=tmp2;

    if(map1Port.getInputCount()){    
        tmp=map1Port.read(false);
        if(tmp!=NULL)
            currentProcessor->map1_yarp=tmp;
    }
    if(map2Port.getInputCount()){    
        tmp=map2Port.read(false);
        if(tmp!=NULL)
            currentProcessor->map2_yarp=tmp;
    }
    if(map3Port.getInputCount()){
        tmp=map3Port.read(false);
        if(tmp!=NULL)
            currentProcessor->map3_yarp=tmp;
    }
    if(map4Port.getInputCount()){
        tmp=map4Port.read(false);
        if(tmp!=NULL)
            currentProcessor->map4_yarp=tmp;
    }
    
    if(map5Port.getInputCount()){
        tmp=map5Port.read(false);
        if(tmp==NULL)
            currentProcessor->map5_yarp=tmp;
    }
    if(map6Port.getInputCount()){
        tmp=map6Port.read(false);
        if(tmp!=NULL)
            currentProcessor->map6_yarp=tmp;
    }
    
    outPorts();

    return true;
}

bool selectiveAttentionModule::startselectiveAttentionProcessor(){
    printf("image processor starting ..... \n");
    this->currentProcessor->start();
    return true;
}

void selectiveAttentionModule::resetFlags(){
    
}

bool selectiveAttentionModule::respond(const Bottle &command,Bottle &reply){
        
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
            

            reply.addString("\n");
            

            reply.addString("\n");
            reply.addString("set k1 <double> \t: setting of linear combination coefficient (map1) \n");
            reply.addString("set k2 <double> \t: setting of linear combination coefficient (map2) \n");
            reply.addString("set k3 <double> \t: setting of linear combination coefficient (map3) \n");
            reply.addString("set k4 <double> \t: setting of linear combination coefficient (map4) \n");
            reply.addString("set k5 <double> \t: setting of linear combination coefficient (map5) \n");
            reply.addString("set k6 <double> \t: setting of linear combination coefficient (map6) \n");

            reply.addString("\n");
            
            
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
            case COMMAND_VOCAB_K1:{
                double w = command.get(2).asDouble();
                if(currentProcessor!=0)
                    currentProcessor->k1=w;
                ok = true;
            }
            break;
            case COMMAND_VOCAB_K2:{
                double w = command.get(2).asDouble();
                if(currentProcessor!=0)
                    currentProcessor->k2=w;
                ok = true;
            }
            break;
            case COMMAND_VOCAB_K3:{
                double w = command.get(2).asDouble();
                if(currentProcessor!=0)
                    currentProcessor->k3=w;
                ok = true;
            }
            break;
            case COMMAND_VOCAB_K4:{
                double w = command.get(2).asDouble();
                if(currentProcessor!=0)
                    currentProcessor->k4=w;
                ok = true;
            }
            break;
            case COMMAND_VOCAB_K5:{
                double w = command.get(2).asDouble();
                if(currentProcessor!=0)
                    currentProcessor->k5=w;
                ok = true;
            }
            break;
            case COMMAND_VOCAB_K6:{
                double w = command.get(2).asDouble();
                if(currentProcessor!=0)
                    currentProcessor->k6=w;
                ok = true;
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
                cout << "received an unknown request after a SET COMMAND" << endl;
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
                cout << "received an unknown request after a GET COMMAND" << endl;
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


