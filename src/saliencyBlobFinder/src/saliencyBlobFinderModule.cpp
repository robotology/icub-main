#include <iCub/saliencyBlobFinderModule.h>

using namespace std;

#define NOTIMECONTROL true

#define centroidDispacementY 10;

saliencyBlobFinderModule::saliencyBlobFinderModule(){
    reinit_flag=false;
    tmpImage=new ImageOf<PixelMono>;
    reply=new Bottle();
    blobFinder=0;

    //---------- flags --------------------------
	contrastLP_flag=false;
	meanColour_flag=false;
	blobCataloged_flag=false;
	foveaBlob_flag=false;
	colorVQ_flag=false;
	maxSaliencyBlob_flag=false;
	blobList_flag=false;
	tagged_flag=false;
	watershed_flag=false;

    timeControl_flag=true;
    filterSpikes_flag=true;
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
    centroidPort.open(getName("centroid:o"));
    triangulationPort.open(getName("triangulation:o"));
    gazeControlPort.open(getName("gazeControl:o"));
    cmdPort.open(getName("cmd"));
    attach(cmdPort);

    time (&start);
    
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
    centroidPort.interrupt();
    triangulationPort.interrupt();
    gazeControlPort.interrupt();
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
    
    printf("closing communication ports .... \n");
    centroidPort.close();
    gazeControlPort.close();
    triangulationPort.close();
   
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
    outPorts();
    return true;
}

/**
* function that reads the ports for colour RGB opponency maps
*/
bool saliencyBlobFinderModule::getOpponencies(){

    tmpImage=rgPort.read(false);
    if(tmpImage!=NULL)
        ippiCopy_8u_C1R(tmpImage->getRawImage(),tmpImage->getRowSize(),blobFinder->ptr_inputImgRG->getRawImage(), blobFinder->ptr_inputImgRG->getRowSize(),srcsize);
   
    tmpImage=grPort.read(false);
    if(tmpImage!=NULL)
        ippiCopy_8u_C1R(tmpImage->getRawImage(),tmpImage->getRowSize(),blobFinder->ptr_inputImgGR->getRawImage(), blobFinder->ptr_inputImgGR->getRowSize(),srcsize);
    
    tmpImage=byPort.read(false);
    if(tmpImage!=NULL)
        ippiCopy_8u_C1R(tmpImage->getRawImage(),tmpImage->getRowSize(),blobFinder->ptr_inputImgBY->getRawImage(), blobFinder->ptr_inputImgBY->getRowSize(),srcsize);
    
    return true;
}

/**
* function that reads the ports for the RGB planes
*/
bool saliencyBlobFinderModule::getPlanes(){
    
    tmpImage=redPort.read(false);
    if(tmpImage!=NULL)
        ippiCopy_8u_C1R(tmpImage->getRawImage(),tmpImage->getRowSize(),blobFinder->ptr_inputImgRed->getRawImage(), blobFinder->ptr_inputImgRed->getRowSize(),srcsize);
   
    tmpImage=greenPort.read(false);
    if(tmpImage!=NULL)
        ippiCopy_8u_C1R(tmpImage->getRawImage(),tmpImage->getRowSize(),blobFinder->ptr_inputImgGreen->getRawImage(), blobFinder->ptr_inputImgGreen->getRowSize(),srcsize);
    
    tmpImage=bluePort.read(false);
    if(tmpImage!=NULL)
        ippiCopy_8u_C1R(tmpImage->getRawImage(),tmpImage->getRowSize(),blobFinder->ptr_inputImgBlue->getRawImage(), blobFinder->ptr_inputImgBlue->getRowSize(),srcsize);
    
    return true;
}


void saliencyBlobFinderModule::outPorts(){ 
    
    //printf("centroid:%f,%f \n",blobFinder->salience->centroid_x,blobFinder->salience->centroid_y);
    //printf("target:%f,%f \n",blobFinder->salience->target_x,blobFinder->salience->target_y);
    
    if((0!=blobFinder->image_out)&&(outputPort.getOutputCount())){ 
        outputPort.prepare() = *(blobFinder->image_out);		
        outputPort.write();
    }

    if(triangulationPort.getOutputCount()){
        Bottle in,bot;
        //Bottle &bot = triangulationPort.prepare(); 
        bot.clear();
        /*bot.addVocab( Vocab::encode("get") ); 
        bot.addVocab( Vocab::encode("3dpoint") );
        bot.addVocab( Vocab::encode("right") );*/
        bot.addString("get");
        bot.addString("3dpoint");
        bot.addString("right");
        bot.addDouble(blobFinder->salience->target_x);
        bot.addDouble(_logpolarParams::_ysize-blobFinder->salience->target_y);
        /*bot.addDouble(blobFinder->salience->centroid_x);
        bot.addDouble(_logpolarParams::_ysize-blobFinder->salience->centroid_y);*/
       
        bot.addDouble(1.5); //fixed distance in which the saccade takes place
        triangulationPort.write(bot,in); //stop here till it receives a response
        if (in.size()>0) {
            target_z=in.pop().asDouble()+0.1;
            target_y=in.pop().asDouble()+0.15;
            target_x=in.pop().asDouble();
            
        } else { 
            printf("No response\n");
        }
        bot.clear();
    }

    if(gazeControlPort.getOutputCount()){
        if(!this->timeControl_flag){
            Bottle &bot = gazeControlPort.prepare(); 
            bot.clear();
            int target_xmap,target_ymap, target_zmap;
            
            bot.addDouble(target_x);  
            bot.addDouble(target_y); 
            bot.addDouble(target_z);
            gazeControlPort.writeStrict();
        }
        else{
            time (&end);
            double dif = difftime (end,start);
            if(dif>blobFinder->constantTimeGazeControl+2){
                //restart the time intervall
                 time(&start);
            }
            else if((dif>blobFinder->constantTimeGazeControl)&&(dif<blobFinder->constantTimeGazeControl+2)){
                //output the command
                //finds the entries with a greater number of occurencies 
                std::map<const char*,int>::iterator iterMap;
                /*int previousValue=occurencesMap.begin()->second;
                std::string finalKey("");
                iterMap=occurencesMap.begin();
                for(;iterMap==occurencesMap.end();iterMap++){
                    if(iterMap->second>previousValue){
                        sprintf((char*)finalKey.c_str(),"%s",iterMap->first);
                    }
                }
                //estracts the strings of the target
                size_t found;
                string target_xmap_string("");
                string target_ymap_string("");
                string target_zmap_string("");
                string rest("");

                found=finalKey.find(",");
                target_xmap_string=finalKey.substr(0,found);
                rest=finalKey.substr(found,finalKey.length()-found);
                found=finalKey.find(",");
                target_ymap_string=rest.substr(0,found);
                rest=finalKey.substr(found,finalKey.length()-found);
                found=finalKey.find(",");
                target_zmap_string=rest.substr(0,finalKey.length());*/
                
                //subdived the string into x,y,z
                //send the command.
                Bottle &bot = gazeControlPort.prepare(); 
                bot.clear();
                int target_xmap,target_ymap, target_zmap;
                bot.addDouble(target_x);  
                bot.addDouble(target_y); 
                bot.addDouble(target_z);
                gazeControlPort.writeStrict();
                time(&start);
                //clear the map
            }
            else{
                //idle period
                //check if it is present and update the map
                //std::string positionName(" ");
                /*sprintf((char*)positionName.c_str(),"%f,%f,%f",target_x,target_y,target_z);
                printf((char*)positionName.c_str());*/
                /*std::map<const char*,int>::iterator iterMap;
                
                iterMap=occurencesMap.find(positionName.c_str());

                if(iterMap==0){
                    printf("new occurence!");
                }
                else{
                    iterMap->second++;
                }*/

            }
        }
    }

    if(centroidPort.getOutputCount()){
        Bottle &bot = centroidPort.prepare(); 
        bot.clear();
        
        // temporary implementation for opencvLogPolar
        /*bot.addDouble(blobFinder->salience->maxc); 
        bot.addDouble(blobFinder->salience->maxr); */
        //logPolarMapper iCub driver
        time (&end);
        double dif = difftime (end,start);
        if((dif>blobFinder->constantTimeCentroid)&&(dif<=blobFinder->constantTimeCentroid+2)){
            bot.addVocab( Vocab::encode("sac") ); 
            bot.addVocab( Vocab::encode("img") ); 
            double centroidDisplacementY=1.0;
            double xrel=(blobFinder->salience->target_x-_logpolarParams::_xsize/2)/(_logpolarParams::_xsize/2);
            double yrel=(blobFinder->salience->target_y-_logpolarParams::_ysize/2)/(-_logpolarParams::_ysize/2);
            //printf("%f>%f,%f \n",dif,xrel,yrel);
            bot.addDouble(xrel);  
            bot.addDouble(yrel); 
            centroidPort.write();
            
        }
        else if(dif>60){
            time (&start);
        }
        else{
            /*printf("%f.",dif);
            bot.addVocab( Vocab::encode("sac") ); 
            bot.addVocab( Vocab::encode("abs") ); 
            bot.addDouble(0);  
            bot.addDouble(0); 
            centroidPort.write();*/
        }
        
    }
     /*Bottle& _outBottle=_centroidPort->prepare();
     _outBottle.clear();
    //_outBottle.addString("centroid:");
    _outBottle.addInt(this->sasalience->centroid_x);
    _outBottle.addInt(this->salience->centroid_y);
    _outBottle.addInt(this->salience->centroid_x);
    _outBottle.addInt(this->salience->centroid_y);
    _centroidPort->writeStrict();*/
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
            reply.addString("\n");
            reply.addString("set rea : set the constantTimeGazeControl in terms of seconds ");

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
            case COMMAND_VOCAB_MEANCOLOURS:{
                string s(command.get(2).asString().c_str());
                printf("image composed by mean colour blobs selected as output\n");
                if(0!=blobFinder){
                    this->blobFinder->resetFlags();
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
                printf("constantTimeGazeControl of the output \n");
                double w= command.get(2).asDouble();
                if(0!=blobFinder)
                    this->blobFinder->constantTimeGazeControl=w/10;
                ok=true;
            }
                break;
            case COMMAND_VOCAB_TCEN:{
                //int j = command.get(2).asInt();
                printf("constantTimeCentroid of the output \n");
                double w= command.get(2).asDouble();
                if(0!=blobFinder)
                    this->blobFinder->constantTimeCentroid=w/10;
                ok=true;
            }
                break;
            case COMMAND_VOCAB_KBU:{
                double w = command.get(2).asDouble();
                if(0!=blobFinder)
                    this->blobFinder->salienceBU=w;
                ok=true;
            }
                break;
            case COMMAND_VOCAB_KTD:{
                double w = command.get(2).asDouble();
                if(0!=blobFinder)
                    blobFinder->salienceTD=w;
                ok=true;
            }
                break;
            case COMMAND_VOCAB_RIN:{
                double w = command.get(2).asDouble();
                if(0!=blobFinder)
                    blobFinder->targetRED=w;
                ok=true;
            }
                break;
            case COMMAND_VOCAB_GIN:{
                double w = command.get(2).asDouble();
                if(0!=blobFinder)
                    blobFinder->targetGREEN=w;
                ok=true;
            }
                break;
            case COMMAND_VOCAB_BIN:{
                double w = command.get(2).asDouble();
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
            /*case COMMAND_VOCAB_WEIGHT:{
                double w = 0.0;
                reply.addDouble(w);
                ok = true;
            }*/
                break;
            /*case COMMAND_VOCAB_CHILD_WEIGHT:{
                int j = command.get(2).asInt();
                double w = 0.0;
                reply.addDouble(w);
                ok = true;
            }*/
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


