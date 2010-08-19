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

const int KERNSIZEMAX = 9;

using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;

bool yuvProc::configure(yarp::os::ResourceFinder &rf)
{    
    /* Process all parameters from both command-line and .ini file */

    moduleName            = rf.check("name", 
                           Value("yuvProc"), 
                           "module name (string)").asString();

    imageType            = rf.check("image", 
                           Value("yuv"), 
                           "image type (string)").asString();
    
    period               = rf.check("period", Value(100), "thread period").asInt();

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

    attach(handlerPort);    // attach to port

    /* create the thread and pass pointers to the module parameters */
    yuvThread = new YUVThread(period, moduleName, imageType);

    /* now start the thread to do the work */
    yuvThread->start(); // this calls threadInit() and it if returns true, it then calls run()

    return true ;      // let the RFModule know everything went well
}

bool yuvProc::interruptModule()
{
    handlerPort.interrupt();
    return true;
}

bool yuvProc::close()
{
    handlerPort.close();
    /* stop the thread */
    yuvThread->stop();
    cout << "deleting thread " << endl;
    delete yuvThread;
    return true;
}

bool yuvProc::respond(const Bottle& command, Bottle& reply) 
{
    reply.clear(); 

    if (command.get(0).asString()=="quit") {
        reply.addString("quitting");
        return false;     
    }
    else if (command.get(0).asString()=="help") {
        cout << "Options:" << endl << endl;
        cout << "\t--name       name: module name (default: visUnique)"                    << endl;
        cout << "\t--image      type: image type to process (default: yuv)"                   << endl;
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

YUVThread::~YUVThread()
{
    
}

YUVThread::YUVThread( int period, string moduleName, string imgType ):RateThread(period) 
{
    isYUV = true;
    //set up module name
    this->moduleName = moduleName;

    //set up module name
    if (imgType == "yuv" || imgType == "YUV") {
        cout << "will run module using the YUV image colour space" << endl;
    }else if (imgType == "hsv" || imgType == "HSV") {
        isYUV = false;
        cout << "will run module using the HSV image colour space" << endl;
    }else{
        cout << "probably an error in the colour space will use default: YUV" << endl;
    }
    cout << "initialising Variables" << endl;

    img_out_Y = NULL;    
	img_out_UV = NULL;
    img_out_V = NULL;
    inputExtImage = NULL;
    img_Y = NULL;    
	img_UV = NULL;
    img_V = NULL;
    pyuva = NULL;
    centerSurr = NULL;
    cs_tot_32f = NULL;
    orig = NULL;
    colour = NULL;
    yuva_orig = NULL;
    first_plane = NULL; 
    second_plane  = NULL; 
    third_plane  = NULL; 
    tmp = NULL;
    ycs_out = NULL;
    scs_out = NULL; 
    vcs_out = NULL; 
    colcs_out = NULL;

    allocated = false;
}

bool YUVThread::threadInit() 
{
    string temp[3];
    if ( isYUV ){
        temp[0] = "Y";
        temp[1] = "UV";
    }else{
        temp[0] = "H";
        temp[1] = "S";
        temp[2] = "V";
    }
    //create all ports
    inputPortName = "/" + moduleName + "/image:i";
    imageInputPort.open( inputPortName.c_str() );

    outputPortName1 = "/" + moduleName + "/" + temp[0] + "/image:o";
    imageOutPort1.open( outputPortName1.c_str() );
    
    outputPortName2 = "/" + moduleName + "/" + temp[1] + "/image:o";
    imageOutPort2.open( outputPortName2.c_str() );
    
    if (!isYUV) {
        outputPortName3 = "/" + moduleName + "/" + temp[2] + "/image:o";
        imageOutPort3.open( outputPortName3.c_str() );
    }
   
    return true;
}

void YUVThread::run() 
{
    ImageOf<PixelRgb> *img = imageInputPort.read(false);
    
    if(img != NULL) {
        if( !allocated || img->width() != img_out_Y->width() || img->height() != img_out_Y->height() ) {
            deallocate();
            allocate( img );
        }
        extender( img, KERNSIZEMAX );

        ippiCopy_8u_C3AC4R( inputExtImage->getRawImage(), inputExtImage->getRowSize(), colour, psb4, srcsize );

        if ( isYUV )
            ippiRGBToYUV_8u_AC4R( colour, psb4, yuva_orig, psb4, srcsize );
        else
            ippiRGBToHSV_8u_AC4R( colour, psb4, yuva_orig, psb4, srcsize );

        //extract planes
        pyuva[0]= first_plane; //Y or H
        pyuva[1]= second_plane;//U or S
        pyuva[2]= third_plane; //V or V
        pyuva[3]= tmp; 
        ippiCopy_8u_C4P4R( yuva_orig, psb4, pyuva, psb, srcsize );

        //performs centre-surround uniqueness analysis on first plane
        centerSurr->proc_im_8u( first_plane , psb );
        ippiCopy_8u_C1R( centerSurr->get_centsur_norm8u(), centerSurr->get_psb_8u(), ycs_out, ycs_psb , srcsize );

        //performs centre-surround uniqueness analysis on second plane:
        centerSurr->proc_im_8u( second_plane , psb );
        if ( isYUV )
            ippiAdd_32f_C1IR( centerSurr->get_centsur_32f(), centerSurr->get_psb_32f(), cs_tot_32f, psb_32f, srcsize );
        else
            ippiCopy_8u_C1R( centerSurr->get_centsur_norm8u(), centerSurr->get_psb_8u(), scs_out, ycs_psb , srcsize ); 

        //Colour process V:performs centre-surround uniqueness analysis:
        centerSurr->proc_im_8u( third_plane , psb );
        if ( isYUV )
            ippiAdd_32f_C1IR( centerSurr->get_centsur_32f(), centerSurr->get_psb_32f(), cs_tot_32f, psb_32f, srcsize );
        else
            ippiCopy_8u_C1R( centerSurr->get_centsur_norm8u(), centerSurr->get_psb_8u(), vcs_out, ycs_psb , srcsize ); 

        if ( isYUV ){
            //get min max
            Ipp32f valueMin,valueMax;
            valueMin = 0.0;
            valueMax = 0.0;
            ippiMinMax_32f_C1R( cs_tot_32f, psb_32f, srcsize, &valueMin, &valueMax );
            //if ( valueMax == valueMin ){ valueMax = 255.0f; valueMin = 0.0f; }
            ippiScale_32f8u_C1R( cs_tot_32f,psb_32f,colcs_out,col_psb,srcsize, valueMin, valueMax );
        }
  
        //revert to yarp images
        ippiCopy_8u_C1R( ycs_out, ycs_psb, img_Y->getRawImage(), img_Y->getRowSize(), srcsize );
        
        if ( isYUV ){
            ippiCopy_8u_C1R( colcs_out,col_psb, img_UV->getRawImage(), img_UV->getRowSize(), srcsize );
        }else{
            ippiCopy_8u_C1R( scs_out, ycs_psb, img_UV->getRawImage(), img_UV->getRowSize(), srcsize );
            ippiCopy_8u_C1R( vcs_out, ycs_psb, img_V->getRawImage(), img_V->getRowSize(), srcsize );
        }

        //this is nasty, resizes the images...
        unsigned char* imgY = img_Y->getPixelAddress( KERNSIZEMAX, KERNSIZEMAX );
        unsigned char* imgUV = img_UV->getPixelAddress( KERNSIZEMAX, KERNSIZEMAX );
        unsigned char* imgV;
        unsigned char* imgVo;
        if (!isYUV){
           imgV = img_V->getPixelAddress( KERNSIZEMAX, KERNSIZEMAX );
           imgVo = img_out_V->getRawImage();
        }
        
        unsigned char* imgYo = img_out_Y->getRawImage();
        unsigned char* imgUVo = img_out_UV->getRawImage();
        int rowsize= img_out_Y->getRowSize();
        int rowsize2= img_Y->getRowSize();

        for(int row=0; row<origsize.height; row++) {
            for(int col=0; col<origsize.width; col++) {
                *imgYo  = *imgY;
                *imgUVo = *imgUV;
                if (!isYUV) {
                    *imgVo = *imgV;
                    imgVo++;  imgV++;          
                }
                imgYo++;  imgUVo++;
                imgY++;   imgUV++;
            }    
            imgYo+=rowsize - origsize.width;
            imgUVo+=rowsize - origsize.width;
            imgY+=rowsize2 - origsize.width;
            imgUV+=rowsize2 - origsize.width;
            if (!isYUV) {
                imgVo+=rowsize - origsize.width;
                imgV+=rowsize2 - origsize.width;       
            }
        }

        //output Y centre-surround results to ports
        if ( imageOutPort1.getOutputCount()>0 ){
            imageOutPort1.prepare() = *img_out_Y;	
            imageOutPort1.write();
        }

        //output UV centre-surround results to ports
        if ( imageOutPort2.getOutputCount()>0 ){
             imageOutPort2.prepare() = *img_out_UV;	
            imageOutPort2.write();
        }
        //output UV centre-surround results to ports
        if ( !isYUV && imageOutPort3.getOutputCount()>0 ){
            imageOutPort3.prepare() = *img_out_V;	
            imageOutPort3.write();
        }
    }
}

void YUVThread::allocate( ImageOf<PixelRgb> *img )
{
    origsize.width = img->width();
    origsize.height = img->height();

    srcsize.width = origsize.width + 2 * KERNSIZEMAX;
    srcsize.height = origsize.height + KERNSIZEMAX;

    cout << "Received input image dimensions: " << origsize.width << " " << origsize.height << endl;
    cout << "Will extend these to: " << srcsize.width << " " << srcsize.height << endl;

    colour  = ippiMalloc_8u_C4( srcsize.width, srcsize.height, &psb4);

    yuva_orig = ippiMalloc_8u_C1( srcsize.width *4, srcsize.height, &psb4);
    first_plane    = ippiMalloc_8u_C1( srcsize.width, srcsize.height, &psb);
    second_plane    = ippiMalloc_8u_C1( srcsize.width, srcsize.height, &psb);
    third_plane   = ippiMalloc_8u_C1( srcsize.width, srcsize.height, &psb);
    
    tmp     = ippiMalloc_8u_C1( srcsize.width, srcsize.height, &psb );// to separate alpha channel
    pyuva = (Ipp8u**) malloc(4*sizeof(Ipp8u*));

    cs_tot_32f  = ippiMalloc_32f_C1( srcsize.width, srcsize.height, &psb_32f );
    colcs_out   = ippiMalloc_8u_C1( srcsize.width, srcsize.height,  &col_psb );
    ycs_out     = ippiMalloc_8u_C1( srcsize.width, srcsize.height,  &ycs_psb );
    scs_out     = ippiMalloc_8u_C1( srcsize.width, srcsize.height,  &ycs_psb );
    vcs_out     = ippiMalloc_8u_C1( srcsize.width, srcsize.height,  &ycs_psb );

    ncsscale = 4;

    centerSurr  = new CentSur( srcsize , ncsscale );

    inputExtImage = new ImageOf<PixelRgb>;
    inputExtImage->resize( srcsize.width, srcsize.height );

	img_Y = new ImageOf<PixelMono>;
	img_Y->resize( srcsize.width, srcsize.height );

    img_V = new ImageOf<PixelMono>;
	img_V->resize( srcsize.width, srcsize.height );

	img_UV = new ImageOf<PixelMono>;
	img_UV->resize( srcsize.width, srcsize.height );

    img_out_Y = new ImageOf<PixelMono>;
	img_out_Y->resize( origsize.width, origsize.height );

	img_out_UV = new ImageOf<PixelMono>;
	img_out_UV->resize( origsize.width, origsize.height );

    img_out_V = new ImageOf<PixelMono>;
	img_out_V->resize( origsize.width, origsize.height );
        
    allocated = true;
    cout << "done allocating" << endl;
}

void YUVThread::deallocate( )
{
    delete centerSurr;
    delete img_out_Y;
    delete img_out_UV;
    delete img_out_V;
    delete img_Y;
    delete img_UV;
    delete img_V;
    delete inputExtImage;
    ippiFree( colour ); 
    ippiFree( tmp ); 
    free ( pyuva );
    ippiFree( yuva_orig );
    ippiFree( first_plane );
    ippiFree( second_plane );
    ippiFree( third_plane );
    ippiFree( cs_tot_32f );
    ippiFree( colcs_out );
    ippiFree( ycs_out );
    ippiFree( scs_out );
    ippiFree( vcs_out );
	img_out_Y = NULL;    
	img_out_UV = NULL;
    img_out_V = NULL;
    inputExtImage = NULL;
    img_Y = NULL;    
	img_UV = NULL;
    img_V = NULL;
    pyuva = NULL;
    centerSurr = NULL;
    //cs_tot_32f = NULL;
    //orig = NULL;
    colour = NULL;
    yuva_orig = NULL;
    first_plane = NULL; 
    second_plane = NULL; 
    third_plane = NULL; 
    tmp = NULL;
    ycs_out = NULL; 
    scs_out = NULL;
    vcs_out = NULL;
    colcs_out = NULL;
    
    allocated = false;
}

void YUVThread::threadRelease() 
{
    deallocate();
}

void YUVThread::onStop() 
{
    
    cout << "cleaning up..." << endl;
    cout << "attempting to close ports" << endl;
    imageInputPort.interrupt();
    imageOutPort1.interrupt();
    imageOutPort2.interrupt();
    imageInputPort.close();
    imageOutPort1.close();
    imageOutPort2.close();

    if ( !isYUV ) {
        imageOutPort3.close();
    }
    
    cout << "finished closing ports" << endl;
}

ImageOf<PixelRgb>* YUVThread::extender(ImageOf<PixelRgb>* inputOrigImage, int maxSize) {
    // copy of the image 
    ippiCopy_8u_C3R(inputOrigImage->getRawImage(),
                    inputOrigImage->getRowSize(),
                    inputExtImage->getPixelAddress(maxSize,maxSize),
                    inputExtImage->getRowSize(),
                    origsize);

    // memcpy of the horizontal fovea lines (rows) 
    const int sizeBlock = origsize.width / 2;
    for(int i = 0; i < maxSize; i++) {
        memcpy( inputExtImage->getPixelAddress( sizeBlock + maxSize, maxSize-1-i ),
               inputExtImage->getPixelAddress( maxSize, maxSize+i ),
               sizeBlock*sizeof(PixelRgb));
        memcpy( inputExtImage->getPixelAddress( maxSize, maxSize-1-i ),
               inputExtImage->getPixelAddress( sizeBlock + maxSize, maxSize+i ),
               sizeBlock*sizeof(PixelRgb));
    }

    // copy of the block adjacent angular positions (columns)
    const int px = maxSize * sizeof(PixelRgb);
    for (int row = 0; row < srcsize.height; row++) {
        memcpy ( inputExtImage->getPixelAddress( srcsize.width-maxSize, row ),
                inputExtImage->getPixelAddress( maxSize,row ), px);
        memcpy ( inputExtImage->getPixelAddress( 0, row ),
                inputExtImage->getPixelAddress( srcsize.width- maxSize-maxSize, row ), px);
    }

    return inputExtImage;
}
