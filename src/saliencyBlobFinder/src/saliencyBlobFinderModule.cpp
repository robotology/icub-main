#include <iCub/saliencyBlobFinderModule.h>

#include <iostream>
using namespace std;

saliencyBlobFinderModule::saliencyBlobFinderModule(){
    reinit_flag=false;
    reply=new Bottle();
    blobFinder=0;
}


/**
*function that opens the module
*/
bool saliencyBlobFinderModule::open(Searchable& config) {
    ct = 0;
    //ConstString portName2 = options.check("name",Value("/worker2")).asString();
    inputPort.open(getName("image:i"));
    
    redPort.open(getName("red:i"));
    greenPort.open(getName("green:i"));
    bluePort.open(getName("blue:i"));

    rgPort.open(getName("rg:i"));
    grPort.open(getName("gr:i"));
    byPort.open(getName("by:i"));

    outputPort.open(getName("image:o"));

    cmdPort.open(getName("cmd"));
    attach(cmdPort);
    
    return true;
}

/** 
* tries to interrupt any communications or resource usage
*/
bool saliencyBlobFinderModule::interruptModule() {
    printf("module interrupted .... \n");
    inputPort.interrupt();
    
    redPort.interrupt();
    greenPort.interrupt();
    bluePort.interrupt();

    rgPort.interrupt();
    grPort.interrupt();
    byPort.interrupt();

    outputPort.interrupt();

    cmdPort.interrupt();
    
    return true;
}


bool saliencyBlobFinderModule::close(){
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

    printf("closing outputport .... \n");
    outputPort.close();

    printf("closing command port .... \n");
    cmdPort.close();
   
    return true;
}

void saliencyBlobFinderModule::setOptions(yarp::os::Property opt){
    //options	=opt;
    // definition of the mode
    // definition of the name of the module
    ConstString name=opt.find("name").asString();
    if(name!=""){
        printf("|||  Module named as :%s \n", name.c_str());
        this->setName(name.c_str());
    }
    ConstString value=opt.find("mode").asString();
    if(value!=""){
        printf("|||  Module operating mode :%s \n", value.c_str());
        if(value=="MEA"){
            printf("meancolour image as output selected \n");
        }
        
    }
}

bool saliencyBlobFinderModule::updateModule() {
    
    command=cmdPort.read(false);
    if(command!=0){
        //Bottle* tmpBottle=cmdPort.read(false);
        ConstString str= command->toString();
        printf("command received: %s \n", str.c_str());
        this->respond(*command,*reply);
        printf("module reply: %s \n",reply->toString().c_str());
    }
    
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
        this->height=img->height();
        this->width=img->width();
        reinitialise(img->width(), img->height());
        reinit_flag=true;
        //initialization of the main thread
        blobFinder=new blobFinderThread();
        blobFinder->reinitialise(img->width(), img->height());
        blobFinder->start();
        blobFinder->ptr_inputImg=img;
    }

    //copy the inputImg into a buffer
    ippiCopy_8u_C3R(img->getRawImage(), img->getRowSize(),blobFinder->ptr_inputImg->getRawImage(), blobFinder->ptr_inputImg->getRowSize(),srcsize);
    bool ret1=true,ret2=true;
    ret1=getOpponencies();
    ret2=getPlanes();
    if(ret1&&ret2)
        blobFinder->freetorun=true;
    outPorts();
    return true;
}

/**
* function that reads the ports for colour RGB opponency maps
*/
bool saliencyBlobFinderModule::getOpponencies(){
    bool temp;
    ImageOf<PixelMono> *tmpImage=new ImageOf<PixelMono>;
    tmpImage->resize(this->width,this->height);
    temp=rgPort.read(false);
    if(temp)
        ippiCopy_8u_C1R(tmpImage->getRawImage(),tmpImage->getRowSize(),blobFinder->ptr_inputImgRG->getRawImage(), blobFinder->ptr_inputImgRG->getRowSize(),srcsize);
    if(blobFinder->ptr_inputImgRG==0)
        return false;
   
    temp=grPort.read(false);
    if(temp)
        ippiCopy_8u_C1R(tmpImage->getRawImage(),tmpImage->getRowSize(),blobFinder->ptr_inputImgGR->getRawImage(), blobFinder->ptr_inputImgGR->getRowSize(),srcsize);
    if(blobFinder->ptr_inputImgGR==0)
        return false;
    temp=byPort.read(false);
    if(temp)
        ippiCopy_8u_C1R(tmpImage->getRawImage(),tmpImage->getRowSize(),blobFinder->ptr_inputImgBY->getRawImage(), blobFinder->ptr_inputImgBY->getRowSize(),srcsize);
    if(blobFinder->ptr_inputImgBY==0)
        return false;
    return true;
}

/**
* function that reads the ports for the RGB planes
*/
bool saliencyBlobFinderModule::getPlanes(){
    bool temp;
    ImageOf<PixelMono> *tmpImage=new ImageOf<PixelMono>;tmpImage->resize(this->width,this->height);
    temp=redPort.read(false);
    if(temp)
        ippiCopy_8u_C1R(tmpImage->getRawImage(),tmpImage->getRowSize(),blobFinder->ptr_inputImgRed->getRawImage(), blobFinder->ptr_inputImgRed->getRowSize(),srcsize);
    if(blobFinder->ptr_inputImgRed==0)
        return false;
    temp=greenPort.read(false);
    if(temp)
        ippiCopy_8u_C1R(tmpImage->getRawImage(),tmpImage->getRowSize(),blobFinder->ptr_inputImgGreen->getRawImage(), blobFinder->ptr_inputImgGreen->getRowSize(),srcsize);
    if(blobFinder->ptr_inputImgGreen==0)
        return false;
    temp=bluePort.read(false);
    if(temp)
        ippiCopy_8u_C1R(tmpImage->getRawImage(),tmpImage->getRowSize(),blobFinder->ptr_inputImgBlue->getRawImage(), blobFinder->ptr_inputImgBlue->getRowSize(),srcsize);
    if(blobFinder->ptr_inputImgBlue==0)
        return false;
    return true;
}


void saliencyBlobFinderModule::outPorts(){ 
    if(0!=blobFinder->image_out){  //&&(outputPort.getOutputCount()
        outputPort.prepare() = *(blobFinder->image_out);		
        outputPort.write();
    }
}


void saliencyBlobFinderModule::reinitialise(int weight, int height){
    img=new ImageOf<PixelRgb>;
    img->resize(weight,height);
}


bool saliencyBlobFinderModule::respond(const Bottle &command,Bottle &reply){
        
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
            case COMMAND_VOCAB_MAXSALIENCY:{
                int nb = command.get(2).asInt();
                printf("most salient blob as output has been selected \n");
                //reply.addString("connection 2");
                ok=true;
            }
                break;
            case COMMAND_VOCAB_CONTRASTLP:{
                printf("image of the saliency (grayscale) \n");

                ok =true;
            }
                break;
            case COMMAND_VOCAB_MEANCOLOURS:{
                string s(command.get(2).asString().c_str());
                printf("image composed by mean colour blobs selected as output\n");
                //reply.addString("connection 1");
                ok=true;
            }
                break;
            case COMMAND_VOCAB_TAGGED:{
                int j = command.get(2).asInt();
                printf("image of tags(unsigned char) given to blobs  \n");
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


