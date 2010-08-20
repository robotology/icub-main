// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include <iCub/selectiveAttentionProcessor.h>



#include <ipp.h>
#include <ipps.h>
//#include <qimage.h>
#include <math.h>
#include <iostream>
#include <stdlib.h>
#include <cstdio>


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;

using namespace std;


// available methods for edges detection
//#define CONVMAX
//#define CONVFILTER
//#define IPPISOBEL
//#define OPENCVSOBEL
//#define CONVSEQ

selectiveAttentionProcessor::selectiveAttentionProcessor(int rateThread):RateThread(rateThread)
{
    this->inImage=new ImageOf<PixelRgb>;
    reinit_flag=false;
    maskSeed=4;
    maskTop=20;

    //flags
    inputImage_flag=0;
    idle=true;
    resized_flag=false;
    interrupted=false;
    
    k1=0.5;
    k2=0.0;
    k3=0.0;
    k4=0.0;
    k5=0;
    k6=0;

    // images
    edges_yarp=new ImageOf<PixelMono>;
    tmp=new ImageOf<PixelMono>;
    
    map1_yarp=new ImageOf<PixelMono>;
    map2_yarp=new ImageOf<PixelMono>;
    map3_yarp=new ImageOf<PixelMono>;
    map4_yarp=new ImageOf<PixelMono>;
    map5_yarp=new ImageOf<PixelMono>;
    map6_yarp=new ImageOf<PixelMono>;
    

    map1_ippi = 0;
    map2_ippi = 0;
    map3_ippi = 0;
    map4_ippi = 0;
    map5_ippi = 0;
    map6_ippi = 0;


    tmp=new ImageOf<PixelMono>;
    
    image_out=new ImageOf<PixelRgb>;
    image_tmp=new ImageOf<PixelMono>;
    outputImage=new ImageOf<PixelMono>;
    outputImage2=new ImageOf<PixelMono>;
    linearCombinationImage=new ImageOf<PixelMono>;
        
}

selectiveAttentionProcessor::~selectiveAttentionProcessor(){
    printf("Destructor /n");
    delete inImage;
   // delete portImage;
    delete edges_yarp;

    delete map1_yarp;
    delete map2_yarp;
    delete map3_yarp;
    delete map4_yarp;
    delete map5_yarp;
    delete map6_yarp;
    
    ippiFree(map1_ippi );
    ippiFree(map2_ippi );
    ippiFree(map3_ippi );
    ippiFree(map4_ippi );
    ippiFree(map5_ippi );
    ippiFree(map6_ippi );

    delete tmp;
    delete image_out;
    delete image_tmp;
    delete outputImage;
    delete outputImage2;
    
}

selectiveAttentionProcessor::selectiveAttentionProcessor(ImageOf<PixelRgb>* inputImage):RateThread(THREAD_RATE) {
    this->inImage=inputImage;
//    this->portImage=portImage;

    //edgesOutput=new ImageOf<PixelMono>;
    //portImage=new ImageOf<PixelRgb>;
    
    tmp=new ImageOf<PixelMono>;    
}

void selectiveAttentionProcessor::reinitialise(int width, int height){
    this->srcsize.width=width;
    this->srcsize.height=height;
    this->width=width;
    this->height=height;

    inputImg=new ImageOf<PixelRgb>;
    inputImg->resize(width,height);

    linearCombinationImage=new ImageOf<PixelMono>;
    linearCombinationImage->resize(width,height);

    outputImage=new ImageOf<PixelMono>;
    outputImage->resize(width,height);
    map1_yarp=new ImageOf<PixelMono>;
    map1_yarp->resize(width,height);
    map2_yarp=new ImageOf<PixelMono>;
    map2_yarp->resize(width,height);
    map3_yarp=new ImageOf<PixelMono>;
    map3_yarp->resize(width,height);
    map4_yarp=new ImageOf<PixelMono>;
    map4_yarp->resize(width,height);
    map5_yarp=new ImageOf<PixelMono>;
    map5_yarp->resize(width,height);
    map6_yarp=new ImageOf<PixelMono>;
    map6_yarp->resize(width,height);
    
}

void selectiveAttentionProcessor::resizeImages(int width,int height) {

    tmp->resize(width,height);
    //portImage->resize(width,height);
    /*map1_yarp->resize(width,height);
    map2_yarp->resize(width,height);
    map3_yarp->resize(width,height);
    map4_yarp->resize(width,height);
    map5_yarp->resize(width,height);
    map6_yarp->resize(width,height);*/

    inImage->resize(width,height);

    if(map1_ippi ==0){
        map2_ippi  = ippiMalloc_8u_C1(width,height,&psb);
        map3_ippi  = ippiMalloc_8u_C1(width,height,&psb);
        map4_ippi  = ippiMalloc_8u_C1(width,height,&psb);
        map5_ippi  = ippiMalloc_8u_C1(width,height,&psb);
        map6_ippi  = ippiMalloc_8u_C1(width,height,&psb);
    }

    tmp->resize(width,height);
    image_out->resize(width,height);
    image_tmp->resize(width,height);
    outputImage->resize(width,height);
    outputImage2->resize(width,height);
    linearCombinationImage->resize(width,height);

    cvImage16= cvCreateImage(cvSize(width,height),IPL_DEPTH_16S,1);
    cvImage8= cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,1);
    
    resized_flag=true;
}

/**
*	initialization of the thread 
*/
bool selectiveAttentionProcessor::threadInit(){
    printf("Thread initialization .... \n");
    //input ports 
    inImagePort.open(getName("/image:i").c_str());
    map1Port.open(getName("/map1:i").c_str()); 
    map2Port.open(getName("/map2:i").c_str());
    map3Port.open(getName("/map3:i").c_str()); 	 
    map4Port.open(getName("/map4:i").c_str()); 
    map5Port.open(getName("/map5:i").c_str()); 
    map6Port.open(getName("/map6:i").c_str()); 

    selectedAttentionPort.open(getName("/attention:o").c_str());
    linearCombinationPort.open(getName("/combination:o").c_str());
    centroidPort.open(getName("/centroid:o").c_str());
    feedbackPort.open(getName("/feedback:o").c_str());
    return true;
}

void selectiveAttentionProcessor::setName(std::string str){
    this->name=str; 
}


std::string selectiveAttentionProcessor::getName(const char* p){
    string str(name);
    str.append(p);
    //printf("name: %s", name.c_str());
    return str;
}

/**
* active loop of the thread
*/
void selectiveAttentionProcessor::run(){


        //synchronisation with the input image occuring
        if(!interrupted){
            
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
            //if(inImagePort.getInputCount()){
                
            tmp2=inImagePort.read(false);
            
            if(tmp2==0){
                return;
            }
            
            if(!reinit_flag){
                //srcsize.height=img->height();
                //srcsize.width=img->width();
                reinitialise(tmp2->width(), tmp2->height());
                reinit_flag=true;
                //currentProcessor=new selectiveAttentionProcessor();
                //passes the temporary variable for the mode
                //currentProcessor->resizeImages(tmp2->width(),tmp2->height());
                //startselectiveAttentionProcessor();
                //currentProcessor->setIdle(false);
            }
            
            //currentProcessor->inImage=tmp2;
            
            if(map1Port.getInputCount()){    
                tmp=map1Port.read(false);
                if(tmp!=0) {
                    ippiCopy_8u_C1R(tmp->getRawImage(),tmp->getRowSize(),map1_yarp->getRawImage(),map1_yarp->getRowSize(),this->srcsize);
                    idle=false;
                }
                
            }
            if(map2Port.getInputCount()){    
                tmp=map2Port.read(false);
                if(tmp!=0) {
                    ippiCopy_8u_C1R(tmp->getRawImage(),tmp->getRowSize(),map2_yarp->getRawImage(),map2_yarp->getRowSize(),this->srcsize);
                    idle=false;
                }
            }
            if(map3Port.getInputCount()){
                tmp=map3Port.read(false);
                if(tmp!=0) {
                    ippiCopy_8u_C1R(tmp->getRawImage(),tmp->getRowSize(),map3_yarp->getRawImage(),map3_yarp->getRowSize(),this->srcsize);
                    idle=false;
                }
            }
            if(map4Port.getInputCount()){
                tmp=map4Port.read(false);
                if(tmp!=0) {
                    ippiCopy_8u_C1R(tmp->getRawImage(),tmp->getRowSize(),map4_yarp->getRawImage(),map4_yarp->getRowSize(),this->srcsize);
                    idle=false;
                }
            }
            
            if(map5Port.getInputCount()){
                tmp=map5Port.read(false);
                if(tmp!=0) {
                    ippiCopy_8u_C1R(tmp->getRawImage(),tmp->getRowSize(),map5_yarp->getRawImage(),map5_yarp->getRowSize(),this->srcsize);
                    idle=false;
                }
            }
            if(map6Port.getInputCount()){
                tmp=map6Port.read(false);
                if(tmp!=0) {
                    ippiCopy_8u_C1R(tmp->getRawImage(),tmp->getRowSize(),map6_yarp->getRawImage(),map6_yarp->getRowSize(),this->srcsize);
                    idle=false;
                }
            }

            //2. processing of the input images
            int rowSize=map1_yarp->getRowSize();
            unsigned char maxValue=0;
            
            if(!idle){
                for(int y=0;y<height;y++){
                    for(int x=0;x<width;x++){
                        unsigned char value=0;
                        if(map1_yarp!=0)
                            value+=(unsigned char)map1_yarp->getRawImage()[x+y*rowSize]*k1;
                        if(map2_yarp!=0)
                            value+=(unsigned char)map2_yarp->getRawImage()[x+y*rowSize]*k2;
                        if(map3_yarp!=0)
                            value+=(unsigned char)map3_yarp->getRawImage()[x+y*rowSize]*k3;
                        if(map4_yarp!=0)
                            value+=(unsigned char)map4_yarp->getRawImage()[x+y*rowSize]*k4;
                        if(map5_yarp!=0)
                            value+=(unsigned char)map5_yarp->getRawImage()[x+y*rowSize]*k5;
                        if(map6_yarp!=0)
                            value+=(unsigned char)map6_yarp->getRawImage()[x+y*rowSize]*k6;
                        if((map1_yarp==0)&&(map2_yarp==0)&&(map3_yarp==0)&&(map4_yarp==0)&&(map5_yarp==0)&&(map6_yarp==0))
                            value=0;
                        else
                            linearCombinationImage->getRawImage()[x+y*rowSize]=value;
                        if(maxValue<value)
                            maxValue=value;
                        linearCombinationImage->getRawImage()[x+y*rowSize]=value;
                    }
                }

                for(int y=0;y<height;y++){
                    for(int x=0;x<width;x++){
                        if(maxValue==0)
                            outputImage->getRawImage()[x+y*rowSize]=0;
                        else if((linearCombinationImage->getRawImage()[x+y*rowSize]>=maxValue-10)&&((linearCombinationImage->getRawImage()[x+y*rowSize]<=maxValue+10))){
                            outputImage->getRawImage()[x+y*rowSize]=255;
                        }
                        else{    
                            outputImage->getRawImage()[x+y*rowSize]=0;
                        }
                    }   
                }

                ippiCopy_8u_C1R(outputImage->getRawImage(),outputImage->getRowSize(),outputImage2->getRawImage(),outputImage2->getRowSize(), srcsize);
                //extractContour(outputImage2,inImage,centroid_x,centroid_y);
                //printf("centroid_x %d, centroid_y %d", centroid_x, centroid_y);

                //ippiCopy_8u_C1R((const Ipp8u *)dst->imageData,dst->widthStep, outputImage->getRawImage(), outputImage->getRowSize(),srcsize);
                //get the colour of the inputImage starting in the centroid_x and centroid_y position
                //unsigned char* pColour=inImage->getPixelAddress(centroid_x,centroid_y);
                //3. sending the output on the ports
                outPorts();
            }
        }
}

bool selectiveAttentionProcessor::outPorts(){
    bool ret = false;
    if((0!=linearCombinationImage)&&(linearCombinationPort.getOutputCount())){
        linearCombinationPort.prepare() = *(linearCombinationImage);
        linearCombinationPort.write();
    }
    
    if((0!=outputImage)&&(selectedAttentionPort.getOutputCount())){
        selectedAttentionPort.prepare() = *(outputImage);
        selectedAttentionPort.write();
    }

    if(centroidPort.getOutputCount()){  
        Bottle& commandBottle=centroidPort.prepare();
        commandBottle.clear();
        commandBottle.addString("sac");
        commandBottle.addString("img");
        //commandBottle.addInt(centroid_x);
        //commandBottle.addInt(centroid_y);
        centroidPort.write();
    }

    if(feedbackPort.getOutputCount()){  
        //Bottle& commandBottle=feedbackPort.prepare();
        Bottle in,commandBottle;
        commandBottle.clear();
        
        
        time (&end2);
        double dif = difftime (end2,start2);
        if(dif>30+2){
                //restart the time interval
                 time(&start2);
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
            commandBottle.addDouble((double)targetRed);
            //commandBottle.addDouble(255.0);
            feedbackPort.write(commandBottle,in);
            commandBottle.clear();
            commandBottle.addVocab(VOCAB3('s','e','t'));
            commandBottle.addVocab(VOCAB3('g','i','n'));
            commandBottle.addDouble((double)targetGreen);
            //commandBottle.addDouble(0.0);
            feedbackPort.write(commandBottle,in);
            commandBottle.clear();
            commandBottle.addVocab(VOCAB3('s','e','t'));
            commandBottle.addVocab(VOCAB3('b','i','n'));
            commandBottle.addDouble((double)targetBlue);
            feedbackPort.write(commandBottle,in);
            commandBottle.clear();
            //commandBottle.addDouble(0.0);
            printf("%f,%f,%f \n",(double)targetRed,(double)targetGreen,(double)targetBlue);
        }
    }
    return true;
}


void selectiveAttentionProcessor::extractContour(ImageOf<PixelMono>* inputImage,ImageOf<PixelRgb>* inputColourImage,int& x,int& y){
    
    CvMemStorage* stor=cvCreateMemStorage(0);
    CvBox2D box;
    CvSeq* cont = cvCreateSeq(CV_SEQ_ELTYPE_POINT, sizeof(CvSeq), sizeof(CvPoint) , stor);
    cvFindContours(inputImage->getIplImage(), stor, &cont, sizeof(CvContour),
                CV_RETR_LIST, CV_CHAIN_APPROX_NONE, cvPoint(0,0));
    IplImage* dst = cvCreateImage( cvGetSize(outputImage->getIplImage()), 8, 3 );
    cvZero(dst);   
    double numObj = 0;
    float line[4];
    CvPoint center;
    CvPoint pt1;
    CvPoint pt2;

    for(;cont;cont = cont->h_next){
        numObj ++;
        //  int count = cont->total; // This is number point in contour
        box = cvMinAreaRect2(cont, stor);
        center.x = cvRound(box.center.x);
        center.y = cvRound(box.center.y);
        x=center.x;
        y=center.y;
        float v = box.size.width;
        float v1 = box.size.height;

        /*
        //unsigned char targetRed=0, targetGreen=0, targetBlue=0;
        unsigned char tmpRed=0, tmpGreen=0, tmpBlue=0;
        getPixelColour(inImage, x,y,targetRed,targetGreen,targetBlue);
        getPixelColour(inImage, x+1,y,tmpRed,tmpGreen, tmpBlue);
        targetRed=(targetRed+tmpRed)/2;targetGreen=(targetGreen+tmpGreen)/2;targetBlue=(targetBlue+tmpBlue)/2;
        //targetRed = (targetRed > tmpRed) ? targetRed : tmpRed;targetGreen = (targetGreen > tmpGreen) ? targetGreen : tmpGreen;targetBlue = (targetBlue > tmpBlue) ? targetBlue : tmpBlue;
        getPixelColour(inImage, x-1,y,tmpRed,tmpGreen, tmpBlue);
        targetRed=(targetRed+tmpRed)/2;targetGreen=(targetGreen+tmpGreen)/2;targetBlue=(targetBlue+tmpBlue)/2;
        //targetRed = (targetRed > tmpRed) ? targetRed : tmpRed;targetGreen = (targetGreen > tmpGreen) ? targetGreen : tmpGreen;targetBlue = (targetBlue > tmpBlue) ? targetBlue : tmpBlue;
        getPixelColour(inImage, x,y+1,tmpRed,tmpGreen, tmpBlue);
        targetRed=(targetRed+tmpRed)/2;targetGreen=(targetGreen+tmpGreen)/2;targetBlue=(targetBlue+tmpBlue)/2;
        //targetRed = (targetRed > tmpRed) ? targetRed : tmpRed;targetGreen = (targetGreen > tmpGreen) ? targetGreen : tmpGreen;targetBlue = (targetBlue > tmpBlue) ? targetBlue : tmpBlue;
        getPixelColour(inImage, x,y-1,tmpRed,tmpGreen, tmpBlue);
        targetRed=(targetRed+tmpRed)/2;targetGreen=(targetGreen+tmpGreen)/2;targetBlue=(targetBlue+tmpBlue)/2;
        //targetRed = (targetRed > tmpRed) ? targetRed : tmpRed;targetGreen = (targetGreen > tmpGreen) ? targetGreen : tmpGreen;targetBlue = (targetBlue > tmpBlue) ? targetBlue : tmpBlue;
        */
        
           
        cvDrawContours(dst,cont,CV_RGB(targetBlue,targetGreen,targetRed),CV_RGB(0,0,0),0,1,8);
        cvCircle (dst, center, 1, CV_RGB(targetBlue,targetGreen,targetRed));
           
        cvFitLine(cont, CV_DIST_L2, 1, 0.01, 0.01, line);
        float t = (v + v1)/2;
        pt1.x = cvRound(line[2] - line[0] *t );
        pt1.y = cvRound(line[3] - line[1] *t );
        pt2.x = cvRound(line[2] + line[0] *t );
        pt2.y = cvRound(line[3] + line[1] *t );

        //cvCircle(dst, pt1, 1, CV_RGB(targetRed,targetGreen,targetBlue));
        //cvCircle(dst, pt2, 1, CV_RGB(targetBlue,targetGreen,targetRed));
       
        /*
        //cvLine(dst, pt1, pt2, CV_RGB(targetRed,targetGreen,targetBlue), 3, CV_AA, 0);
        double theta = 0;
        // double theta = 180 / M_PI * atan2( (pt2.y - pt1.y) , (pt2.x - pt1.x) );
        CvFont font;
        double hScale=0.3;
        double vScale=0.3;
        int    lineWidth=1;

        cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale,vScale,0,lineWidth);
        char* Angles=new char;

        sprintf(Angles, "%d,%d,%d",(int)targetRed,(int)targetGreen,(int)targetBlue);
        cvPutText ( dst , Angles, cvPoint( 10, 10), &font, cvScalar(255,0,0) );
        */

    }
    //cvCopy(dst,outputImage->getIplImage());
    //ippiCopy_8u_C1R((const Ipp8u *)dst->imageData,dst->widthStep, outputImage->getRawImage(), outputImage->getRowSize(),srcsize);
    cvReleaseMemStorage(&stor);
    cvReleaseImage(&dst);
    //cvReleaseMemStorage(&storage);
}


void selectiveAttentionProcessor::getPixelColour(ImageOf<PixelRgb>* inputColourImage,int x ,int y, unsigned char &targetRed, unsigned char &targetGreen, unsigned char &targetBlue){
    //printf("max image dim:%d with rowsize %d \n",inImage->getRawImageSize(),inImage->getRowSize());
    unsigned char pColour=inImage->getRawImage()[(x*3)+y*inImage->getRowSize()];
    targetRed=pColour;
    //pColour++;
    pColour=inImage->getRawImage()[(x*3)+1+y*inImage->getRowSize()];
    targetGreen=pColour;
    //pColour++;
    pColour=inImage->getRawImage()[(x*3)+2+y*inImage->getRowSize()];
    targetBlue=pColour;
    //printf("colour found: %d %d %d \n",(int) targetBlue,(int)targetGreen,(int)targetRed);
}

/**
* function called when the module is poked with an interrupt command
*/
void selectiveAttentionProcessor::interrupt(){
    interrupted=true;
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
    
}

/**
*	releases the thread
*/
void selectiveAttentionProcessor::threadRelease(){
    printf("Thread realeasing .... \n");
    printf("Closing all the ports.. \n");
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
    
}

void selectiveAttentionProcessor::setIdle(bool value){
    mutex.wait();
    idle=value;
    mutex.post();
}






//----- end-of-file --- ( next line intentionally left blank ) ------------------

