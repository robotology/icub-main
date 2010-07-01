#include <iCub/interactionThread.h>
//#include <iCub/convert_bitdepth.h>
#include <ipps.h>
#include <iostream>


using namespace yarp::sig;
using namespace yarp::os;
using namespace std;

interactionThread::interactionThread():RateThread(INT_THREAD_RATE)
{
   blobFinder=0;

   tmpImage=new ImageOf<PixelMono>;
   img=new ImageOf<PixelRgb>;

   previous_target_x=0;
   previous_target_y=0;

   time (&startTimer);
}

interactionThread::~interactionThread()
{
   delete tmpImage;
   delete img;
}


void interactionThread::reinitialise(int width, int height){
    srcsize.width=width;
    srcsize.height=height;

    img->resize(width,height);
}

void interactionThread::setName(std::string str){
    this->name=str; 
}


std::string interactionThread::getName(const char* p){
    string str(name);
    str.append(p);
    printf("name: %s", name.c_str());
    return str;
}


bool interactionThread::threadInit(){
    printf("Thread initialisation.. \n");    
    openPorts();
    return true;
}

/**
* function called when the module is poked with an interrupt command
*/
void interactionThread::interrupt(){
    inputPort.interrupt();//(getName("image:i"));
    
    redPort.interrupt();//open(getName("red:i"));
    greenPort.interrupt();//open(getName("green:i"));
    bluePort.interrupt();//open(getName("blue:i"));

    rgPort.interrupt();//open(getName("rg:i"));
    grPort.interrupt();//open(getName("gr:i"));
    byPort.interrupt();//open(getName("by:i"));

    outputPort.interrupt();//open(getName("image:o"));
    centroidPort.interrupt();//open(getName("centroid:o"));
    triangulationPort.interrupt();//open(getName("triangulation:o"));
    gazeControlPort.interrupt();//open(getName("gazeControl:o"));
}


void interactionThread::run(){
    
    ct++;
    img = inputPort.read(false);
    if(0==img)
        return;

    if(!reinit_flag){    
	    srcsize.height=img->height();
	    srcsize.width=img->width();
        this->height=img->height();
        this->width=img->width();
        reinitialise(img->width(), img->height());
        reinit_flag=true;
        tmpImage->resize(this->width,this->height);
        
    }

    //copy the inputImg into a buffer
    ippiCopy_8u_C3R(img->getRawImage(), img->getRowSize(),blobFinder->ptr_inputImg->getRawImage(), blobFinder->ptr_inputImg->getRowSize(),srcsize);
    bool ret1=true,ret2=true;
    ret1=getOpponencies();
    ret2=getPlanes();
    if(ret1&&ret2)
        blobFinder->freetorun=true;
    outPorts();
    
   
}

/**
* function that reads the ports for colour RGB opponency maps
*/
bool interactionThread::getOpponencies(){

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
bool interactionThread::getPlanes(){
    
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


bool interactionThread::outPorts(){ 
    
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
        /*bot.addVocab( Vocab::encode("get") );x 
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
            target_z=in.pop().asDouble();
            target_y=in.pop().asDouble()+0.097;
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
            int target_xmap,target_ymap,target_zmap;
            
            bot.addDouble(target_x);  
            bot.addDouble(target_y); 
            bot.addDouble(target_z);
            gazeControlPort.writeStrict();
        }
        else{
            time (&endTimer);
            double dif = difftime (endTimer,startTimer);
            if(dif>blobFinder->constantTimeGazeControl+0.5){
                //restart the time intervall
                 time(&startTimer);
            }
            else if((dif>blobFinder->constantTimeGazeControl)&&(dif<blobFinder->constantTimeGazeControl+0.5)){
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
                time(&startTimer);
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
        
        
        /*bot.addDouble(blobFinder->salience->maxc); 
        bot.addDouble(blobFinder->salience->maxr); */
        
        time (&endTimer);
        double dif = difftime (endTimer,startTimer);
        if((dif>blobFinder->constantTimeCentroid)&&(dif<=blobFinder->constantTimeCentroid+0.5)){
            if((blobFinder->salience->target_x<previous_target_x+5)&&(blobFinder->salience->target_x>previous_target_x-5)){
                if((blobFinder->salience->target_y<previous_target_y+5)&&(blobFinder->salience->target_y>previous_target_y-5)){
                    //printf("same position \n");
                }
                else{
                    //printf("."); 
                    bot.addVocab( Vocab::encode("sac") ); 
                    bot.addVocab( Vocab::encode("img") ); 
                    double centroidDisplacementY=1.0;
                    double xrel=(blobFinder->salience->target_x-_logpolarParams::_xsize/2+xdisp)/(_logpolarParams::_xsize/2);
                    double yrel=(blobFinder->salience->target_y-_logpolarParams::_ysize/2+ydisp)/(-_logpolarParams::_ysize/2);
                    //printf("%f>%f,%f \n",dif,xrel,yrel);
                    bot.addDouble(xrel);  
                    bot.addDouble(yrel); 
                    centroidPort.write();

                    previous_target_x=blobFinder->salience->target_x;
                    previous_target_y=blobFinder->salience->target_y;
                }
            }
            else{
                //printf(".");
                bot.addVocab( Vocab::encode("sac") ); 
                bot.addVocab( Vocab::encode("img") ); 
                double centroidDisplacementY=1.0;
                double xrel=(blobFinder->salience->target_x-_logpolarParams::_xsize/2)/(_logpolarParams::_xsize/2);
                double yrel=(blobFinder->salience->target_y-_logpolarParams::_ysize/2)/(-_logpolarParams::_ysize/2);
                //printf("%f>%f,%f \n",dif,xrel,yrel);
                bot.addDouble(xrel);  
                bot.addDouble(yrel); 
                centroidPort.write();

                previous_target_x=blobFinder->salience->target_x;
                previous_target_y=blobFinder->salience->target_y;
            }
            


            /*bot.addVocab( Vocab::encode("sac") ); 
            bot.addVocab( Vocab::encode("img") ); 
            double centroidDisplacementY=1.0;
            double xrel=(blobFinder->salience->target_x-_logpolarParams::_xsize/2)/(_logpolarParams::_xsize/2);
            double yrel=(blobFinder->salience->target_y-_logpolarParams::_ysize/2)/(-_logpolarParams::_ysize/2);
            //printf("%f>%f,%f \n",dif,xrel,yrel);
            bot.addDouble(xrel);  
            bot.addDouble(yrel); 
            centroidPort.write();*/
            
        }
        else if(dif>blobFinder->constantTimeCentroid+0.5){
            time (&startTimer);
        }
        else{
           
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

    return true;
}

void interactionThread::threadRelease(){
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


    printf("Thread releasing.. \n");
    printf("input port closing .... \n");
    closePorts();
}

bool interactionThread::openPorts(){
	
    bool ret = false;
    bool ok=true;
    
    //ConstString portName2 = options.check("name",Value("/worker2")).asString();
    inputPort.open(getName("image:i").c_str());
    
    redPort.open(getName("red:i").c_str());
    greenPort.open(getName("green:i").c_str());
    bluePort.open(getName("blue:i").c_str());

    rgPort.open(getName("rg:i").c_str());
    grPort.open(getName("gr:i").c_str());
    byPort.open(getName("by:i").c_str());

    outputPort.open(getName("image:o").c_str());
    centroidPort.open(getName("centroid:o").c_str());
    triangulationPort.open(getName("triangulation:o").c_str());
    gazeControlPort.open(getName("gazeControl:o").c_str());

	return true;
}

bool interactionThread::closePorts(){
	bool ret = false;
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

    
    
    printf("closing communication ports .... \n");
    centroidPort.close();
    gazeControlPort.close();
    triangulationPort.close();

	return ret;
}



/*void interactionThread::setInputImage(ImageOf<PixelRgb>* inputImage){
    this->img=inputImage;
    this->width=inputImage->width();
    this->height=inputImage->height();
    reinitialise();
}*/


//----- end-of-file --- ( next line intentionally left blank ) ------------------
