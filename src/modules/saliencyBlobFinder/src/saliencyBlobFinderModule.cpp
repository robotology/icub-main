#include <iCub/saliencyBlobFinderModule.h>

using namespace std;

#define NOTIMECONTROL true

#define centroidDispacementY 10;

saliencyBlobFinderModule::saliencyBlobFinderModule(){
  //interThread=new interactionThread();
    blobFinder=0;
    rateThread=0;

    reinit_flag=false;
    reply=new Bottle();
    

    //---------- flags --------------------------
    contrastLP_flag=false;
    meanColour_flag=false;
    blobCataloged_flag=false;
    foveaBlob_flag=false;
    colorVQ_flag=false;
    maxSaliencyBlob_flag=true;
    blobList_flag=false;
    tagged_flag=false;
    watershed_flag=false;

    timeControl_flag=true;
    filterSpikes_flag=false;
}

void saliencyBlobFinderModule::copyFlags(){

    blobFinder->contrastLP_flag=contrastLP_flag;
    blobFinder->meanColour_flag=meanColour_flag;
    blobFinder->blobCataloged_flag=blobCataloged_flag;
    blobFinder->foveaBlob_flag=foveaBlob_flag;
    blobFinder->colorVQ_flag=colorVQ_flag;
    blobFinder->maxSaliencyBlob_flag=maxSaliencyBlob_flag;
    blobFinder->blobList_flag=blobList_flag;
    blobFinder->tagged_flag=tagged_flag;
    blobFinder->watershed_flag=watershed_flag;
    blobFinder->filterSpikes_flag=filterSpikes_flag;

}



bool saliencyBlobFinderModule::configure(ResourceFinder &rf){
/* Process all parameters from both command-line and .ini file */

    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name", 
                           Value("/blobFinder/icub/left_cam"), 
                           "module name (string)").asString();

    /*
    * before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
    setName(moduleName.c_str());

    /* now, get the rest of the parameters */

    /*
    * get the ratethread which will define the period of the processing thread
    */
    rateThread             = rf.check("ratethread", 
                           Value(30), 
                           "processing ratethread (int)").asInt();
    cout<<"Module started with the parameter ratethread:"<<rateThread<<endl;

    /*
    * gets the minBounding area for blob neighbours definition
    */
    minBoundingArea             = rf.check("minBoundingArea", 
                           Value(225), 
                           "minBoundingArea (int)").asInt();

    if (!cmdPort.open(getName())) {           
      cout << getName() << ": Unable to open port " << endl;
      return false;
    }

    attach(cmdPort);                  // attach to port

    //initialization of the main thread
    if(rateThread==0)
        blobFinder=new blobFinderThread();
    else
        blobFinder=new blobFinderThread(rateThread);

    blobFinder->setName(this->getName().c_str());
    //blobFinder->reinitialise(interThread->img->width(),interThread->img->height());
    blobFinder->start();
    blobFinder->countSpikes=this->countSpikes;

    //passes the value of flags
    copyFlags();

    printf("\n waiting for connection of the input port \n");

    return true;
}

/** 
* tries to interrupt any communications or resource usage
*/
bool saliencyBlobFinderModule::interruptModule() {
    printf("module interrupted .... \n");
    cmdPort.interrupt();
    blobFinder->interrupt();
    
    return true;
}


bool saliencyBlobFinderModule::close(){
    printf("closing command port .... \n");
    cmdPort.close();
    blobFinder->stop();
    //interThread->stop();

    return true;
}

void saliencyBlobFinderModule::setOptions(yarp::os::Property opt){
    //options=opt;
    // definition of the mode
    // definition of the name of the module
    ConstString name=opt.find("name").asString();
    if(name!=""){
        printf("|||  Module named as :%s \n", name.c_str());
        this->setName(name.c_str());
    }
    int rate=opt.find("rateThread").asInt();
    if(rate!=0){
        printf("|||  Module rateThread as :%d \n", rate);
        this->rateThread=rate;
    }
    ConstString value=opt.find("mode").asString();
    if(value!=""){
        printf("|||  Module operating mode :%s \n", value.c_str());
        if(value=="MEA"){
            meanColour_flag=true;
            printf("meancolour image as output selected \n");
        }
        if(value=="MAX"){
            maxSaliencyBlob_flag=true;
            printf("max_saliency image as output selected \n");
        }
    }
    value=opt.find("filter").asString();
    if(value!=""){
        printf("|||  Module filter :%s \n", value.c_str());
        if(value=="spikes"){
            filterSpikes_flag=true;
            printf("stimuli filter ON \n");
        }
        if(value=="kalman"){
            printf("kalman filter ON \n");
        }
        if(value=="off"){
            filterSpikes_flag=false;
            printf("all the filters OFF \n");
        }
    }
    value=opt.find("timeControl").asString();
    if(value!=""){
        printf("|||  Module time control flag :%s \n", value.c_str());
        if(value=="ON"){
            this->timeControl_flag=true;
            printf("time control ON \n");
        }
        
        if(value=="OFF"){
            this->timeControl_flag=false;
            printf("time control OFF \n");
        }
    }
    int numValue=opt.find("xdisp").asInt();
    if(numValue!=0){
        printf("|||  Module x disp :%d \n", numValue);
        this->xdisp=numValue;
    }
    numValue=opt.find("ydisp").asInt();
    if(numValue!=0){
        printf("|||  Module y disp :%d \n", numValue);
        this->ydisp=numValue;
    }
    numValue=opt.find("countSpikes").asInt();
    if(numValue!=0){
        printf("|||  Module countSpikes :%d \n", numValue);
        this->countSpikes=numValue;
    }
    numValue=opt.find("thresholddArea").asInt();
    if(numValue!=0){
        printf("|||  Module threshold area :%d \n", numValue);
        this->thresholdArea=numValue;
    }
}

bool saliencyBlobFinderModule::updateModule() {

    /*if((0!=interThread->img) && (!this->reinit_flag)){

        //initialization of the main thread
        blobFinder=new blobFinderThread();
        blobFinder->reinitialise(interThread->img->width(),interThread->img->height());
        blobFinder->start();
        blobFinder->ptr_inputImg=interThread->img;
        blobFinder->ptr_inputImgRed=interThread->ptr_inputImgRed;
        blobFinder->ptr_inputImgGreen=interThread->ptr_inputImgGreen;
        blobFinder->ptr_inputImgBlue=interThread->ptr_inputImgBlue;
        blobFinder->ptr_inputImgRG=interThread->ptr_inputImgRG;
        blobFinder->ptr_inputImgGR=interThread->ptr_inputImgGR;
        blobFinder->ptr_inputImgBY=interThread->ptr_inputImgBY;
        blobFinder->image_out=interThread->image_out;
        blobFinder->countSpikes=this->countSpikes;
        

        //passes the value of flags
        copyFlags();
        this->reinit_flag=true;
    }
    */

    /*command=cmdPort.read(PortReader,false);
    if(command!=0){
        //Bottle* tmpBottle=cmdPort.read(false);
        ConstString str= command->toString();
        printf("command received: %s \n", str.c_str());
        this->respond(*command,*reply);
        printf("module reply: %s \n",reply->toString().c_str());
    }*/
    
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
    
    /*
    ct++;
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
        tmpImage->resize(this->width,this->height);
        //initialization of the main thread
        blobFinder=new blobFinderThread();
        blobFinder->reinitialise(img->width(), img->height());
        blobFinder->start();
        blobFinder->ptr_inputImg=img;
        blobFinder->countSpikes=this->countSpikes;
        //passes the value of flags
        copyFlags();
    }

    //copy the inputImg into a buffer
    ippiCopy_8u_C3R(img->getRawImage(), img->getRowSize(),blobFinder->ptr_inputImg->getRawImage(), blobFinder->ptr_inputImg->getRowSize(),srcsize);
    bool ret1=true,ret2=true;
    ret1=getOpponencies();
    ret2=getPlanes();
    if(ret1&&ret2)
        blobFinder->freetorun=true;
    outPorts();*/
    return true;
}



void saliencyBlobFinderModule::reinitialise(int weight, int height){
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
            reply.addString("set mea : plots the meancolour image \n");
            reply.addString("set clp : streams out the contrast LP image");
            reply.addString("set max : streams out the max saliency blob");
            reply.addString("set tag : streams out the image of blobs coloured by the associated tag value");
            reply.addString("\n");
            reply.addString("set kbu : set coefficient k of the bottom-up saliency calculation ");
            reply.addString("set ktd : set coefficient k of the top-down saliency calculation ");
            reply.addString("set Mdb : set maximum dimension allowed for blobs ");
            reply.addString("set mdb : set minimum dimension allowed for blobs");
            reply.addString("set rin : set red intensity value for the target to be sought");
            reply.addString("set gin : set green intensity value for the target to be sought");
            reply.addString("set bin : set blue intensity value for the target to be sought");
            reply.addString("set wax : set minumum bounding area");
            reply.addString("set par : set percentage of the blob dimension considered surrounding area");
            reply.addString("\n");
            reply.addString("set tcon : set the constantTimeGazeControl (ex.: format for iKinGazeCtrl) ");
            reply.addString("set tcen : set the constantTimeCentroidControl (ex.: format for controlGaze2) ");

            reply.addString("\n");

            reply.addString("get kbu : set coefficient k of the bottom-up saliency calculation ");
            reply.addString("get ktd : set coefficient k of the top-down saliency calculation ");
            reply.addString("get Mdb : set maximum dimension allowed for blobs ");
            reply.addString("get mdb : set minimum dimension allowed for blobs");
            reply.addString("get rin : set red intensity value for the target to be sought");
            reply.addString("get gin : set green intensity value for the target to be sought");
            reply.addString("get bin : set blue intensity value for the target to be sought");


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
                if(0!=blobFinder){
                    this->blobFinder->resetFlags();
                    this->blobFinder->maxSaliencyBlob_flag=true;
                }
                //reply.addString("connection 2");
                ok=true;
            }
                break;
            case COMMAND_VOCAB_CONTRASTLP:{
                printf("image of the saliency (grayscale) \n");
                if(0!=blobFinder){
                    this->blobFinder->resetFlags();
                    this->blobFinder->contrastLP_flag=true;
                }
                ok =true;
            }
                break;
            case COMMAND_VOCAB_WAT:{
                printf("image of the watershed \n");
                if(0!=blobFinder){
                    this->blobFinder->resetFlags();
                    this->blobFinder->watershed_flag=true;
                }
                ok =true;
            }
                break;
            case COMMAND_VOCAB_MEANCOLOURS:{
                string s(command.get(2).asString().c_str());
                printf("image composed by mean colour blobs selected as output\n");
                if(0!=blobFinder){
                    //this->blobFinder->resetFlags();
                    this->blobFinder->meanColour_flag=true;
                }
                //reply.addString("connection 1");
                ok=true;
            }
                break;
            case COMMAND_VOCAB_TAGGED:{
                //int j = command.get(2).asInt();
                printf("image of tags(unsigned char) given to blobs  \n");
                if(0!=blobFinder){
                    blobFinder->resetFlags();
                    blobFinder->tagged_flag=true;
                }
                string s(command.get(3).asString().c_str());
                ok=true;
            }
                break;
            case COMMAND_VOCAB_FOVEA:{
                //int j = command.get(2).asInt();
                printf("image of the fovea \n");
                if(0!=blobFinder){
                    blobFinder->resetFlags();
                    blobFinder->foveaBlob_flag=true;
                }
                string s(command.get(3).asString().c_str());
                ok=true;
            }
                break;
            case COMMAND_VOCAB_TCON:{
                //int j = command.get(2).asInt();
                printf("constantTimeGazeControl of the output   \n");
                double w= command.get(2).asDouble();
                if(0!=blobFinder)
                    this->blobFinder->constantTimeGazeControl=w;
                ok=true;
            }
                break;
            case COMMAND_VOCAB_TCEN:{
                //int j = command.get(2).asInt();
                printf("constantTimeCentroid of the output \n");
                double w= command.get(2).asDouble();
                if(0!=blobFinder)
                    this->blobFinder->constantTimeCentroid=w;
                ok=true;
            }
                break;
            case COMMAND_VOCAB_KBU:{
                double w = command.get(2).asDouble();
                printf("set kbu: %f \n", w);
                if(0!=blobFinder)
                    this->blobFinder->salienceBU=w;
                ok=true;
            }
                break;
            case COMMAND_VOCAB_KTD:{
                double w = command.get(2).asDouble();
                printf("set ktd: %f \n", w);
                if(0!=blobFinder)
                    blobFinder->salienceTD=w;
                ok=true;
            }
                break;
            case COMMAND_VOCAB_MBA:{
                double w = command.get(2).asDouble();
                printf("set mBA: %f \n", w);
                if(0!=blobFinder)
                    blobFinder->minBoundingArea=w;
                ok=true;
            }
                break;
            case COMMAND_VOCAB_PAR:{
                int w = command.get(2).asInt();
                printf("set PAR: %f \n", w);
                if(0!=blobFinder)
                    blobFinder->salience->pArea=w/100;
                ok=true;
            }
                break;
            case COMMAND_VOCAB_RIN:{
                double w = command.get(2).asDouble();
                printf("set rin: %f \n", w);
                if(0!=blobFinder)
                    blobFinder->targetRED=w;
                ok=true;
            }
                break;
            case COMMAND_VOCAB_GIN:{
                double w = command.get(2).asDouble();
                printf("set gin: %f \n", w);
                if(0!=blobFinder)
                    blobFinder->targetGREEN=w;
                ok=true;
            }
                break;
            case COMMAND_VOCAB_BIN:{
                double w = command.get(2).asDouble();
                printf("set bin: %f \n", w);
                if(0!=blobFinder)
                    blobFinder->targetBLUE=w;
                ok=true;
            }
                break;
            case COMMAND_VOCAB_MAXDB:{
                int w = command.get(2).asInt();
                if(0!=blobFinder)
                    blobFinder->maxBLOB=w;
                ok=true;
            }
                break;
            case COMMAND_VOCAB_MINDB:{
                int w = command.get(2).asInt();
                if(0!=blobFinder)
                    blobFinder->minBLOB=w;
                ok=true;
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
        case COMMAND_VOCAB_RSET:
        rec = true;
        {
            switch(command.get(1).asVocab()) {
            case COMMAND_VOCAB_FLT:{
                printf("reset filter");
                ok=true;
            }
                break;
            default:
                cout << "received an unknown request after a _VOCAB_RSET" << endl;
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
            case COMMAND_VOCAB_KBU:{
                if(blobFinder!=0){
                    double nb = blobFinder->salienceBU;
                    reply.addDouble(nb);
                    ok = true;
                }
            }
                break;
            case COMMAND_VOCAB_KTD:{
                if(blobFinder!=0){
                    double nb = blobFinder->salienceTD;
                    reply.addDouble(nb);
                    ok = true;
                }
            }
            break;
            case COMMAND_VOCAB_RIN:{
                if(blobFinder!=0){
                    int nb = blobFinder->targetRED;
                    reply.addInt(nb);
                    ok = true;
                }
            }
            break;
            case COMMAND_VOCAB_GIN:{
                if(blobFinder!=0){
                    int nb = blobFinder->targetGREEN;
                    reply.addInt(nb);
                    ok = true;
                }
            }
            break;
            case COMMAND_VOCAB_BIN:{
                if(blobFinder!=0){
                    int nb = blobFinder->targetBLUE;
                    reply.addInt(nb);
                    ok = true;
                }
            }
            break;
            case COMMAND_VOCAB_MAXDB:{
                int nb = blobFinder->maxBLOB;
                reply.addInt(nb);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_MINDB:{
                int nb = blobFinder->minBLOB;
                reply.addInt(nb);
                ok = true;
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


