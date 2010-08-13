/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Vadim Tikhanoff, Andrew Dankers
 * email:   vadim.tikhanoff@iit.it
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

#include "iCub/yuvProc.h"
#include <stdio.h>
#include <stdlib.h>
 #include <string.h>
#define KERNSIZE 5
//#include "yarp/os/impl/NameClient.h"

bool yuvProc::configure(yarp::os::ResourceFinder &rf)
{    
    /* Process all parameters from both command-line and .ini file */

    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name", 
                           Value("yuvProc"), 
                           "module name (string)").asString();

   /*
    * before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
   
    setName(moduleName.c_str());

    /* get the name of the input and output ports, automatically prefixing the module name by using getName() */
 
    inputPortName           = "/";
    inputPortName           += getName(
                           rf.check("IMG_IN", 
                           Value("/image:i"),
                           "Input image port (string)").asString()
                           );
    
   
    outputPortNameY         = "/";
    outputPortNameY         += getName(
                           rf.check("YPort", 
                           Value("/Y/image:o"),
                           "Output Y port (string)").asString()
                           );

    outputPortNameUV       = "/";
    outputPortNameUV       += getName(
                           rf.check("UVPort", 
                           Value("/UV/image:o"),
                           "Output UV port (string)").asString()
                           );

    /* do all initialization here */
    /* open ports  */  
       
    if (!inputPort.open(inputPortName.c_str())) {
        cout << getName() << ": unable to open port " << inputPortName << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if (!outPortY.open(outputPortNameY.c_str())) {
        cout << getName() << ": unable to open port " << outputPortNameY << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!outPortUV.open(outputPortNameUV.c_str())) {
        cout << getName() << ": unable to open port " << outputPortNameUV << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

   /*
    * attach a port of the same name as the module (prefixed with a /) to the module
    * so that messages received from the port are redirected to the respond method
    */

    handlerPortName =  "/";
    handlerPortName += getName();         // use getName() rather than a literal 
 
    if (!handlerPort.open(handlerPortName.c_str())) {           
        cout << getName() << ": Unable to open port " << handlerPortName << endl;  
        return false;
    }

    attach(handlerPort);                  // attach to port
 
    //attachTerminal();                     // attach to terminal (maybe not such a good thing...)


    /* create the thread and pass pointers to the module parameters */

    yuvThread = new YUVThread(&inputPort, &outPortY, &outPortUV);

    /* now start the thread to do the work */
    yuvThread->start(); // this calls threadInit() and it if returns true, it then calls run()

    return true ;      // let the RFModule know everything went well
}


bool yuvProc::interruptModule()
{
    inputPort.interrupt();
    outPortY.interrupt();
    outPortUV.interrupt();
    handlerPort.interrupt();
    yuvThread->stop();
    return true;
}


bool yuvProc::close()
{
    inputPort.close();
    outPortY.close();
    outPortUV.close();   
    handlerPort.close();
    /* stop the thread */
    yuvThread->stop();
    delete yuvThread;
    return true;
}


bool yuvProc::respond(const Bottle& command, Bottle& reply) 
{
    string helpMessage =  string(getName().c_str()) + 
                        " commands are: \n" +  
                        "help \n" + 
                        "quit \n";

    reply.clear(); 

    if (command.get(0).asString()=="quit") {
        reply.addString("quitting");
        return false;     
    }
    else if (command.get(0).asString()=="help") {
        cout << helpMessage;
        reply.addString("ok");
    }
    else{
			cout << "command not known - type help for more info" << endl;
	}
    return true;
}

/* Called periodically every getPeriod() seconds */

bool yuvProc::updateModule()
{
    return true;
}

double yuvProc::getPeriod()
{
    /* module periodicity (seconds), called implicitly by myModule */
    
    return 0.1;
}

YUVThread::YUVThread(BufferedPort<ImageOf<PixelRgb> > *inputPort, BufferedPort<ImageOf<PixelMono> > *outPortY, BufferedPort<ImageOf<PixelMono> > *outPortUV)
{
    imageInputPort    = inputPort;
    imageOutPortY  = outPortY; 
    imageOutPortUV  = outPortUV;
    min = 0.0;
    max = 0.0;
}

bool YUVThread::threadInit() 
{
    /* initialize variables and create data-structures if needed */
    init = true;
    return true;
}

void YUVThread::run(){

//-----------------------------------REMEMBER THAT THE CONNECTION TO LOG IS OBSOLETE AS SHOULD CONNECT TO COLOUR PROCESSOR DIRECTLY!!!!!!

    while (isStopping() != true) { // the thread continues to run until isStopping() returns true
        
        gotImg =  ( imageInputPort->getInputCount() > 0 );

        Time::delay(0.1);

        if (gotImg > 0) {

            img = imageInputPort->read(false);

            if (init){//Get first RGB image to establish width, height:
                cout << "initializing" << endl;   
                initAll();
                cout << "done initializing" << endl;        
            }
            extender( img, KERNSIZE ); //here extendorig

            //create your own YUV image ( from RBG eg... with alpha channel and separate Y U and V channel 
            ippiCopy_8u_C3R( inputExtImage->getRawImage(),inputExtImage->getRowSize(), orig, img_psb, srcsize );

            //convert to RGBA:
            ippiCopy_8u_C3AC4R( orig, img_psb, colour, psb4, srcsize );
            //convert to Y,U,V image channels:
            ippiRGBToYUV_8u_AC4R( colour, psb4, yuva_orig, psb4, srcsize);
            //extract Y, U, V Images
            pyuva[0]= y_orig;
            pyuva[1]= u_orig;
            pyuva[2]= v_orig; 
            pyuva[3]= tmp; 
            ippiCopy_8u_C4P4R(yuva_orig,psb4,pyuva,psb,srcsize);

            //intensity process: performs centre-surround uniqueness analysis
            centerSurr->proc_im_8u( y_orig , psb );
            ippiCopy_8u_C1R( centerSurr->get_centsur_norm8u(), centerSurr->get_psb_8u(), ycs_out, ycs_psb , srcsize);
    //-----------------------------------------------------------------------------------------------------------------------------------------------------------

            //Colour process U performs centre-surround uniqueness analysis:
            centerSurr->proc_im_8u( u_orig , psb );
            ippiAdd_32f_C1IR(centerSurr->get_centsur_32f(),centerSurr->get_psb_32f(),cs_tot_32f,psb_32f,srcsize);
            //Colour process V:
            centerSurr->proc_im_8u( v_orig , psb );
            ippiAdd_32f_C1IR(centerSurr->get_centsur_32f(),centerSurr->get_psb_32f(),cs_tot_32f,psb_32f,srcsize);

            //get min max
            ippiMinMax_32f_C1R(cs_tot_32f,psb_32f,srcsize,&min,&max); 
            if (max==min){max=255.0;min=0.0;}
            ippiScale_32f8u_C1R(cs_tot_32f,psb_32f,colcs_out,col_psb,srcsize,min,max);

            ippiCopy_8u_C1R(ycs_out, ycs_psb, img_Y->getRawImage(), img_Y->getRowSize(), srcsize);
            ippiCopy_8u_C1R(colcs_out,col_psb, img_UV->getRawImage(), img_UV->getRowSize(), srcsize);

            unsigned char* imgY = img_Y->getPixelAddress( KERNSIZE, KERNSIZE );
            unsigned char* imgUV = img_UV->getPixelAddress( KERNSIZE, KERNSIZE );
        
            unsigned char* imgYo = img_out_Y->getRawImage();
            unsigned char* imgUVo = img_out_UV->getRawImage();

            int rowsize= img_out_Y->getRowSize();
            int rowsize2= img_Y->getRowSize();

            for(int row=0;row<origsize.height;row++) {
                for(int col=0;col<origsize.width;col++) {
                    *imgYo = *imgY;
                    *imgUVo = *imgUV;
                    imgYo++;imgUVo++;
                    imgY++;imgUV++;
                }    
                imgYo+=rowsize - origsize.width;
                imgUVo+=rowsize - origsize.width;
                imgY+=rowsize2 - origsize.width - KERNSIZE + KERNSIZE;
                imgUV+=rowsize2 - origsize.width - KERNSIZE + KERNSIZE;
            }
            //output Y centre-surround results to ports
            if (imageOutPortY->getOutputCount()>0){
                imageOutPortY->prepare() = *img_out_Y;	
                imageOutPortY->write();
            }

            //output UV centre-surround results to ports
            if (imageOutPortUV->getOutputCount()>0){
                imageOutPortUV->prepare() = *img_out_UV;	
                imageOutPortUV->write();
            }
            //reset 
            ippiSet_32f_C1R(0.0,cs_tot_32f,psb_32f,srcsize);
        }
    } //while
}

void YUVThread::threadRelease() 
{
    cout << "cleaning up things.." << endl;
    if (gotImg > 0){    
        delete centerSurr;
        delete img_out_Y;
        delete img_out_UV;
        delete img_Y;
        delete img_UV;
        delete inputExtImage;
        ippiFree(orig);
        ippiFree(colour); 
        ippiFree(tmp); 
        free (pyuva);
        ippiFree(yuva_orig);
        ippiFree(y_orig);
        ippiFree(u_orig);
        ippiFree(v_orig);
        ippiFree(cs_tot_32f);
        ippiFree(colcs_out);
        ippiFree(ycs_out);
    }
    cout << "finished cleaning up" << endl;
}

void YUVThread::initAll()
{
    init = false;
    origsize.width = img->width();
    origsize.height = img->height();

    srcsize.width = origsize.width + 2 * KERNSIZE;
    srcsize.height = origsize.height + 2 * KERNSIZE;

    cout << "Received input image dimensions: " << origsize.width << " " << origsize.height << endl;
    cout << "Will extend these to: " << srcsize.width << " " << srcsize.height << endl;

    orig    = ippiMalloc_8u_C3(srcsize.width,srcsize.height,&img_psb);
    colour  = ippiMalloc_8u_C4(srcsize.width,srcsize.height,&psb4);

    yuva_orig = ippiMalloc_8u_C1(srcsize.width*4,srcsize.height,&psb4);
    y_orig    = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&psb);
    u_orig    = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&psb);
    v_orig    = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&psb);
    
    tmp     = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&psb);// to separate alpha channel
    pyuva = (Ipp8u**) malloc(4*sizeof(Ipp8u*));

    cs_tot_32f  = ippiMalloc_32f_C1(srcsize.width,srcsize.height, &psb_32f);
    colcs_out   = ippiMalloc_8u_C1(srcsize.width,srcsize.height,  &col_psb);
    ycs_out     = ippiMalloc_8u_C1(srcsize.width,srcsize.height,  &ycs_psb);

    ncsscale = 4;

    centerSurr  = new CentSur( srcsize , ncsscale);
    
    inputExtImage=new ImageOf<PixelRgb>;
    inputExtImage->resize(srcsize.width, srcsize.height);

	img_Y = new ImageOf<PixelMono>;
	img_Y->resize(srcsize.width, srcsize.height);

	img_UV = new ImageOf<PixelMono>;
	img_UV->resize(srcsize.width, srcsize.height);

    img_out_Y = new ImageOf<PixelMono>;
	img_out_Y->resize(origsize.width, origsize.height);

	img_out_UV = new ImageOf<PixelMono>;
	img_out_UV->resize(origsize.width, origsize.height);
        
}

ImageOf<PixelRgb>* YUVThread::extender(ImageOf<PixelRgb>* inputOrigImage,int maxSize) {
    //copy of the image 
    ippiCopy_8u_C3R(inputOrigImage->getRawImage(),inputOrigImage->getRowSize(),inputExtImage->getPixelAddress(maxSize,maxSize),inputExtImage->getRowSize(),origsize);    
    //memcpy of the horizontal fovea lines (rows) 
    int sizeBlock=origsize.width/2;
    for( int i=0;i<maxSize;i++) {
        memcpy(inputExtImage->getPixelAddress(sizeBlock+maxSize,maxSize-1-i),inputExtImage->getPixelAddress(maxSize,maxSize+i),sizeBlock*sizeof(PixelRgb));
        memcpy(inputExtImage->getPixelAddress(maxSize,maxSize-1-i),inputExtImage->getPixelAddress(sizeBlock,maxSize+i),sizeBlock*sizeof(PixelRgb));
    }
    //copy of the block adiacent angular positions (columns)
    unsigned char* ptrDestRight;
    unsigned char* ptrOrigRight;
    unsigned char* ptrDestLeft;
    unsigned char* ptrOrigLeft;
    for( int row=0;row<height;row++ ) {
        ptrDestRight=inputExtImage->getPixelAddress(width-maxSize,row);
        ptrOrigRight=inputExtImage->getPixelAddress(maxSize,row);
        ptrDestLeft=inputExtImage->getPixelAddress(0,row);
        ptrOrigLeft=inputExtImage->getPixelAddress(width-maxSize-maxSize,row);
        for(int i=0;i<maxSize;i++) {
            //right block
            *ptrDestRight=*ptrOrigRight;
            ptrDestRight++;ptrOrigRight++;
            *ptrDestRight=*ptrOrigRight;
            ptrDestRight++;ptrOrigRight++;
            *ptrDestRight=*ptrOrigRight;
            ptrDestRight++;ptrOrigRight++;
            //left block
            *ptrDestLeft=*ptrOrigLeft;
            ptrDestLeft++;ptrOrigLeft++;
            *ptrDestLeft=*ptrOrigLeft;
            ptrDestLeft++;ptrOrigLeft++;
            *ptrDestLeft=*ptrOrigLeft;
            ptrDestLeft++;ptrOrigLeft++;
        }
    }
    return inputExtImage;
}

