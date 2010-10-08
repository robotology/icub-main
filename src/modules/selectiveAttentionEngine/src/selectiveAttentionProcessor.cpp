// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea
 * email:   francesco.rea@iit.it
 * website: www.robotcub.org 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

/**
 * @file selectiveAttentionProcessor.cpp
 * @brief Implementation of the thread of selective attention module (see header file).
 */

#include <iCub/selectiveAttentionProcessor.h>


#include <ipp.h>
#include <ipps.h>
#include <ippcore.h>

#include <math.h>
#include <iostream>
#include <stdlib.h>
#include <cstdio>


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace yarp::dev;
using namespace std;
using namespace iCub::logpolar;

#define XSIZE_DIM 640 //original mapping 
#define YSIZE_DIM 480 //original mapping
#define TIME_CONST 50 //number of times period rateThread to send motion command

selectiveAttentionProcessor::selectiveAttentionProcessor(int rateThread):RateThread(rateThread)
{
    this->inImage=new ImageOf<PixelRgb>;
    reinit_flag=false;
    inputImage_flag=0;
    idle=true;
    interrupted=false;
    gazePerform=true;
    xSizeValue=XSIZE_DIM;
    ySizeValue=YSIZE_DIM;

    cLoop=0;
    
    //default values of the coefficients
    k1=0.5;
    k2=0.1;
    k3=0.5;
    k4=0.1;
    k5=0.5;
    k6=0.5;
    kmotion=0.2;
    kc1=0.2;

    // images
    edges_yarp=new ImageOf<PixelMono>;
    tmp=new ImageOf<PixelMono>;
    
    map1_yarp=new ImageOf<PixelMono>;
    map2_yarp=new ImageOf<PixelMono>;
    map3_yarp=new ImageOf<PixelMono>;
    map4_yarp=new ImageOf<PixelMono>;
    map5_yarp=new ImageOf<PixelMono>;
    map6_yarp=new ImageOf<PixelMono>;
    motion_yarp=new ImageOf<PixelMono>;
    cart1_yarp=new ImageOf<PixelMono>;

    map1_ippi = 0;
    map2_ippi = 0;
    map3_ippi = 0;
    map4_ippi = 0;
    map5_ippi = 0;
    map6_ippi = 0;


    tmp=new ImageOf<PixelMono>;
    
    image_out=new ImageOf<PixelRgb>;
    image_tmp=new ImageOf<PixelMono>;
}

selectiveAttentionProcessor::~selectiveAttentionProcessor(){
    printf("Destructor \n");
    delete inImage;
   // delete portImage;
    delete edges_yarp;

    delete map1_yarp;
    delete map2_yarp;
    delete map3_yarp;
    delete map4_yarp;
    delete map5_yarp;
    delete map6_yarp;
    delete inputLogImage;
    delete motion_yarp;
    delete cart1_yarp;
    
    ippiFree(map1_ippi );
    ippiFree(map2_ippi );
    ippiFree(map3_ippi );
    ippiFree(map4_ippi );
    ippiFree(map5_ippi );
    ippiFree(map6_ippi );

    delete tmp;
    delete image_out;
    delete image_tmp;
    delete intermCartOut;
}

selectiveAttentionProcessor::selectiveAttentionProcessor(ImageOf<PixelRgb>* inputImage):RateThread(THREAD_RATE) {
    this->inImage=inputImage;
    tmp=new ImageOf<PixelMono>;
}

void selectiveAttentionProcessor::reinitialise(int width, int height){
    srcsize.width=width;
    srcsize.height=height;
    this->width=width;
    this->height=height;

    inImage=new ImageOf<PixelRgb>;
    inImage->resize(width,height);

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
    inputLogImage=new ImageOf<PixelRgb>;
    inputLogImage->resize(width,height);

    //fixed resize 640,480
    intermCartOut=new ImageOf<PixelRgb>;
    intermCartOut->resize(xSizeValue,ySizeValue);
    srcsizeCart.width=xSizeValue/2;
    srcsizeCart.height=ySizeValue/2;
    motion_yarp=new ImageOf<PixelMono>;
    motion_yarp->resize(xSizeValue/2,ySizeValue/2);
    cart1_yarp=new ImageOf<PixelMono>;
    cart1_yarp->resize(xSizeValue/2,ySizeValue/2);
}

void selectiveAttentionProcessor::resizeImages(int width,int height) {

    tmp->resize(width,height);
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

    cvImage16= cvCreateImage(cvSize(width,height),IPL_DEPTH_16S,1);
    cvImage8= cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,1);
}

/**
*	initialization of the thread 
*/
bool selectiveAttentionProcessor::threadInit(){
    printf("Thread initialization .... \n");
    //opening ports"
    vergencePort.open(getName("/vergence:o").c_str());
    inImagePort.open(getName("/image:i").c_str());
    map1Port.open(getName("/map1:i").c_str());
    map2Port.open(getName("/map2:i").c_str());
    map3Port.open(getName("/map3:i").c_str());
    map4Port.open(getName("/map4:i").c_str());
    map5Port.open(getName("/map5:i").c_str());
    map6Port.open(getName("/map6:i").c_str());

    cart1Port.open(getName("/cart1:i").c_str());
    motionPort.open(getName("/motion:i").c_str());

    linearCombinationPort.open(getName("/combination:o").c_str());
    centroidPort.open(getName("/centroid:o").c_str());
    feedbackPort.open(getName("/feedback:o").c_str());
    imageCartOut.open(getName("/cartesian:o").c_str());

    //initializing logpolar mapping
    cout << "||| initializing the logpolar mapping" << endl;

    if (!trsf.allocLookupTables(L2C, numberOfRings, numberOfAngles, xSizeValue, ySizeValue, overlap)) {
        cerr << "can't allocate lookup tables" << endl;
        return false;
    }
    cout << "||| lookup table allocation done" << endl;

    //initializing gazecontrollerclient
    Property option;
    option.put("device","gazecontrollerclient");
    option.put("remote","/iKinGazeCtrl");
    string localCon("/client/gaze");
    localCon.append(getName(""));
    option.put("local",localCon.c_str());

    clientGazeCtrl=new PolyDriver();
    clientGazeCtrl->open(option);
    igaze=NULL;

    if (clientGazeCtrl->isValid()) {
       clientGazeCtrl->view(igaze);
    }
    else
        return false;
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

template<class T>
inline int round(T a) {
    int ceilValue=(int)ceil(a);
    int floorValue=(int)floor(a);
    if(a-floorValue<=0.5)
        return floorValue;
    else
        return ceilValue;
}

/**
* active loop of the thread
*/
void selectiveAttentionProcessor::run(){
    cLoop++;
    //synchronisation with the input image occuring
    if(!interrupted){
        
        //--read value from the preattentive level
        if(feedbackPort.getOutputCount()){
            /*
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
            */
        }

        tmp2=inImagePort.read(false);
        if(tmp2==0){
            return;
        }
        if(!reinit_flag){
            reinitialise(tmp2->width(), tmp2->height());
            reinit_flag=true;
        }
        ippiCopy_8u_C3R(tmp2->getRawImage(),tmp2->getRowSize(),inImage->getRawImage(), inImage->getRowSize(),srcsize);
        
        if(map1Port.getInputCount()) {
            tmp=map1Port.read(false);
            if(tmp!=0) {
                ippiCopy_8u_C1R(tmp->getRawImage(),tmp->getRowSize(),map1_yarp->getRawImage(),map1_yarp->getRowSize(),this->srcsize);
                idle=false;
            }
            
        }
        if(map2Port.getInputCount()) {
            tmp=map2Port.read(false);
            if(tmp!=0) {
                ippiCopy_8u_C1R(tmp->getRawImage(),tmp->getRowSize(),map2_yarp->getRawImage(),map2_yarp->getRowSize(),this->srcsize);
                idle=false;
            }
        }
        if(map3Port.getInputCount()) {
            tmp=map3Port.read(false);
            if(tmp!=0) {
                ippiCopy_8u_C1R(tmp->getRawImage(),tmp->getRowSize(),map3_yarp->getRawImage(),map3_yarp->getRowSize(),this->srcsize);
                idle=false;
            }
        }
        if(map4Port.getInputCount()) {
            tmp=map4Port.read(false);
            if(tmp!=0) {
                ippiCopy_8u_C1R(tmp->getRawImage(),tmp->getRowSize(),map4_yarp->getRawImage(),map4_yarp->getRowSize(),this->srcsize);
                idle=false;
            }
        }
        
        if(map5Port.getInputCount()) {
            tmp=map5Port.read(false);
            if(tmp!=0) {
                ippiCopy_8u_C1R(tmp->getRawImage(),tmp->getRowSize(),map5_yarp->getRawImage(),map5_yarp->getRowSize(),this->srcsize);
                idle=false;
            }
        }
        if(map6Port.getInputCount()) {
            tmp=map6Port.read(false);
            if(tmp!=0) {
                ippiCopy_8u_C1R(tmp->getRawImage(),tmp->getRowSize(),map6_yarp->getRawImage(),map6_yarp->getRowSize(),this->srcsize);
                idle=false;
            }
        }
        if(motionPort.getInputCount()) {
            tmp=motionPort.read(false);
            if(tmp!=0) {
                ippiCopy_8u_C1R(tmp->getRawImage(),tmp->getRowSize(),motion_yarp->getRawImage(),motion_yarp->getRowSize(),srcsizeCart);
                idle=false;
            }
        }
        if(cart1Port.getInputCount()) {
            tmp=cart1Port.read(false);
            if(tmp!=0) {
                ippiCopy_8u_C1R(tmp->getRawImage(),tmp->getRowSize(),cart1_yarp->getRawImage(),cart1_yarp->getRowSize(),srcsizeCart);
                idle=false;
            }
        }

        //2. processing of the input images
        unsigned char* pmap1= map1_yarp->getRawImage();
        unsigned char* pmap2= map2_yarp->getRawImage();
        unsigned char* pmap3= map3_yarp->getRawImage();
        unsigned char* pmap4= map4_yarp->getRawImage();
        unsigned char* pmap5= map5_yarp->getRawImage();
        unsigned char* pmap6= map6_yarp->getRawImage();
        ImageOf<PixelMono>& linearCombinationImage=linearCombinationPort.prepare();
        linearCombinationImage.resize(width,height);
        unsigned char* plinear=linearCombinationImage.getRawImage();
        int padding=map1_yarp->getPadding();
        int rowSize=map1_yarp->getRowSize();
        unsigned char maxValue=0;
        double sumK=k1+k2+k3+k4+k5+k6;
        // combination of all the saliency maps
        if(!idle){
            for(int y=0;y<height;y++){
                for(int x=0;x<width;x++){
                    unsigned char value=(unsigned char)ceil((double)(*pmap1 * (k1/sumK) + *pmap2 * (k2/sumK) + *pmap3 * (k3/sumK) + *pmap4 * (k4/sumK) + *pmap5 * (k5/sumK) + *pmap6 * (k6/sumK)));
                    pmap1++;pmap2++;pmap3++;
                    pmap4++;pmap5++;pmap6++;
                    *plinear=value;
                    plinear++;
                }
                pmap1+=padding;pmap2+=padding;pmap3+=padding;
                pmap4+=padding;pmap5+=padding;pmap6+=padding;
                plinear+=padding;
            }
            //trasform the logpolar to cartesian (the logpolar image has to be 3channel image)
            plinear=linearCombinationImage.getRawImage();
            unsigned char* pImage=inputLogImage->getRawImage();
            int padding3C=inputLogImage->getPadding();
            maxValue=0;
            for(int y=0;y<height;y++) {
                for(int x=0;x<width;x++) {
                    *pImage++=(unsigned char)*plinear;
                    *pImage++=(unsigned char)*plinear;
                    *pImage++=(unsigned char)*plinear;
                    plinear++;
                }
                pImage+=padding3C;
                plinear+=padding;
            }
            ImageOf<PixelRgb> &outputCartImage = imageCartOut.prepare();  //preparing the cartesian output
            int outputXSize=(int)floor((double)xSizeValue/2);
            int outputYSize=(int) floor((double)ySizeValue/2);
            outputCartImage.resize(outputXSize,outputYSize);
            trsf.logpolarToCart(*intermCartOut,*inputLogImage);
            //find the max in the cartesian image and downsample
            maxValue=0;
            float xm=0,ym=0;
            int countMaxes=0;
            pImage=outputCartImage.getRawImage();
            unsigned char* pInter=intermCartOut->getRawImage();
            unsigned char* pcart1= cart1_yarp->getRawImage();
            unsigned char* pmotion= motion_yarp->getRawImage();
            int paddingInterm=intermCartOut->getPadding(); //padding of the colour image (640,480)
            int rowSizeInterm=intermCartOut->getRowSize();
            double sumCart=0.5 + kmotion + kc1;
            int paddingCartesian=cart1_yarp->getPadding();
            int paddingOutput=outputCartImage.getPadding();
            for(int y=0;y<ySizeValue;y++) {
                if(y%2==0) {
                    for(int x=0;x<xSizeValue;x++) {
                        if(x%2==0) {
                            unsigned char value=(unsigned char)ceil((double)(*pcart1 * (kc1/sumCart) + *pInter * (0.5/sumCart) + *pmotion * (kmotion/sumCart)));
                            //unsigned char value=*pInter;
                            *pImage=value;
                            if(maxValue<*pImage) {
                                maxValue=*pImage;
                            }
                            pImage++;pInter++;
                            *pImage=value;
                            pImage++;pInter++;
                            *pImage=value;
                            pImage++;pInter++;
                            pcart1++;
                            pmotion++;
                        }
                        else {
                            pInter+=3;
                        }
                    }
                    pImage+=paddingOutput;
                    pInter+=paddingInterm;
                    pcart1+=paddingCartesian;
                    pmotion+=paddingCartesian;
                }
                else {
                    pInter+=paddingInterm+rowSizeInterm;
                }
            }
            
            pImage=outputCartImage.getRawImage();
            float distance=0;
            bool foundmax=false;
            for(int y=0;y<ySizeValue/2;y++) {
                for(int x=0;x<xSizeValue/2;x++) {
                    //*pImage=value;
                    if(*pImage==maxValue) {
                        if(!foundmax) {
                            *pImage=255;pImage++;*pImage=0;pImage++;*pImage=0;pImage-=2;
                            countMaxes++;
                            xm=(float)x;ym=(float)y;
                            foundmax=true;
                        }
                        else {
                            distance=sqrt((x-xm)*(x-xm)+(y-ym)*(y-ym));
                            if(distance<10) {
                                *pImage=255;pImage++;*pImage=0;pImage++;*pImage=0;pImage-=2;
                                countMaxes++;
                                xm+=x;
                                ym+=y;
                            }
                        }
                    }
                    pImage+=3;
                }
                pImage+=paddingOutput;
            }
            xm=xm/countMaxes; ym=ym/countMaxes;
            //representation of red lines where the WTA point is
            //representation of the vertical line
            pImage=outputCartImage.getRawImage();
            pImage+=round(xm)*3;
            for(int i=0;i<ySizeValue/2;i++) {
                *pImage=255;pImage++;*pImage=0;pImage++;*pImage=0;pImage++;
                pImage+=(xSizeValue/2-1)*3+paddingOutput;
            }
            //representation of the horizontal line
            pImage=outputCartImage.getRawImage();
            pImage+=round(ym) * (3 * (xSizeValue/2) + paddingOutput);
            for(int i=0;i<xSizeValue/2;i++) {
                *pImage=255;pImage++;*pImage=0;pImage++;*pImage=0;pImage++;
            }
            //controlling the heading of the robot
            if(cLoop>TIME_CONST) {
                //printf("cartesian: %f,%f \n", xm,ym);
                if(gazePerform) {
                    Vector px(2);
                    px[0]=round(xm);  //divided by two because the iKinGazeCtrl receives coordinates in image plane of 320,240
                    px[1]=round(ym);
                    //we still have one degree of freedom given by
                    //the distance of the object from the image plane
                    //if you do not have it, try to guess :)
                    double z=0.5;   // distance [m]
                    Bottle response;
                    if(vergencePort.getOutputCount()) {
                        //suspending any vergence control
                        Bottle& command=vergencePort.prepare();
                        command.clear();
                        command.addString("sus");
                        vergencePort.write();
                        printf("%s \n", response.toString().c_str());
                        //waiting for the ack from vergence
                        bool flag=false;
                        while(!igaze->checkMotionDone(&flag)){
                            printf("waiting vergence \n");
                        }
                    }
                    igaze->lookAtMonoPixel(camSel,px,z);
                    //waiting for the end of the saccadic event
                    bool flag=false;
                    while(!igaze->checkMotionDone(&flag)) {
                        printf("waiting saccade \n");
                    }
                    if(vergencePort.getOutputCount()) {
                        //suspending any vergence control
                        Bottle& command=vergencePort.prepare();
                        //resuming vergence
                        command.clear(); response.clear();
                        command.addString("res");
                        vergencePort.write();
                        printf("%s \n", response.toString().c_str());
                    }
                }
                cLoop=0;
            }
            outPorts();
        }
    }
}

void selectiveAttentionProcessor::setGazePerform(bool value) {
    gazePerform=value;
}

void selectiveAttentionProcessor::setCamSelection(int value) {
    camSel=value;
}

void selectiveAttentionProcessor::setXSize(int xSize) {
    xSizeValue=xSize;
}

void selectiveAttentionProcessor::setYSize(int ySize) {
    ySizeValue=ySize;
}

void selectiveAttentionProcessor::setOverlap(double _overlap) {
    overlap=_overlap;
}

void selectiveAttentionProcessor::setNumberOfRings(int _numberOfRings) {
    numberOfRings=_numberOfRings;
}

void selectiveAttentionProcessor::setNumberOfAngles(int _numberOfAngles) {
    numberOfAngles=_numberOfAngles;
}


bool selectiveAttentionProcessor::outPorts(){
    bool ret = false;
    if(linearCombinationPort.getOutputCount()){
        linearCombinationPort.write();
    }
    if(imageCartOut.getOutputCount()){
        imageCartOut.write();
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
    vergencePort.interrupt();
    map1Port.interrupt();
    map2Port.interrupt();
    map3Port.interrupt();
    map4Port.interrupt();
    map5Port.interrupt();
    map6Port.interrupt();

    cart1Port.interrupt();

    motionPort.interrupt();
    
    linearCombinationPort.interrupt();
    centroidPort.interrupt();
    feedbackPort.interrupt();
    
    inImagePort.interrupt();
    
}

/**
*	releases the thread
*/
void selectiveAttentionProcessor::threadRelease(){
    trsf.freeLookupTables();

    printf("Thread realeasing .... \n");
    printf("Closing all the ports.. \n");
    //closing input ports
    vergencePort.close();
    inImagePort.close();
    map1Port.close();
    map2Port.close();
    map3Port.close();
    map4Port.close();
    map5Port.close();
    map6Port.close();

    cart1Port.close();

    motionPort.close();

    linearCombinationPort.close();
    centroidPort.close();
    feedbackPort.close();
    imageCartOut.close();
    
    delete clientGazeCtrl;
}

void selectiveAttentionProcessor::setIdle(bool value){
    mutex.wait();
    idle=value;
    mutex.post();
}


//----- end-of-file --- ( next line intentionally left blank ) ------------------

