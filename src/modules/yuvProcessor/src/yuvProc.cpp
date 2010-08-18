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
#include <cassert>

const int KERNSIZEMAX = 9;

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

bool yuvProc::configure(yarp::os::ResourceFinder &rf)
{    
    /* Process all parameters from both command-line and .ini file */

    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name", 
                           Value("yuvProc"), 
                           "module name (string)").asString();

    period = rf.check("period", Value(100), "").asInt();

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

    yuvThread = new YUVThread(&inputPort, &outPortY, &outPortUV, period);

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

YUVThread::YUVThread(BufferedPort<ImageOf<PixelRgb> > *inputPort, BufferedPort<ImageOf<PixelMono> > *outPortY, BufferedPort<ImageOf<PixelMono> > *outPortUV, int p=100) : RateThread(p)
{
    imageInputPort    = inputPort;
    imageOutPortY  = outPortY; 
    imageOutPortUV  = outPortUV;
	img_out_Y = NULL;    
	img_out_UV = NULL;
    inputExtImage = NULL;
    img_Y = NULL;    
	img_UV = NULL;
    pyuva = NULL;
    centerSurr = NULL;
    cs_tot_32f = NULL;
    orig = NULL;
    colour = NULL;
    yuva_orig = NULL;
    y_orig = NULL; 
    u_orig = NULL; 
    v_orig = NULL; 
    tmp = NULL;
    ycs_out = NULL; 
    colcs_out = NULL;

    allocated = false;
    img_psb = 0;
    psb4 = 0;
    psb = 0; 
    ycs_psb = 0;
    col_psb = 0;
    psb_32f = 0;
}

YUVThread::~YUVThread()
{
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

	img_out_Y = NULL;    
	img_out_UV = NULL;
    inputExtImage = NULL;
    img_Y = NULL;    
	img_UV = NULL;
    pyuva = NULL;
    centerSurr = NULL;
    cs_tot_32f = NULL;
    orig = NULL;
    colour = NULL;
    yuva_orig = NULL;
    y_orig = NULL; 
    u_orig = NULL; 
    v_orig = NULL; 
    tmp = NULL;
    ycs_out = NULL; 
    colcs_out = NULL;
}


bool YUVThread::threadInit() 
{
    /* initialize variables and create data-structures if needed */
    return true;
}

void YUVThread::run() {
    //
    if (imageInputPort->getInputCount()) {
        //
        ImageOf<PixelRgb> *img = imageInputPort->read(false);
        if (img != NULL) {
            if( !allocated || img->width() != img_out_Y->width() || img->height() != img_out_Y->height() ) {
                deallocate();
                allocate( img );
            }
                
            // extend logpolar input image
            extender( img, KERNSIZEMAX );
            //create our own YUV image ( from RBG eg... with alpha channel and separate Y U and V channel )
            ippiCopy_8u_C3R( inputExtImage->getRawImage(), inputExtImage->getRowSize(), orig, img_psb, srcsize );
            //convert to RGBA:
            ippiCopy_8u_C3AC4R( orig, img_psb, colour, psb4, srcsize );
            //convert to Y,U,V image channels:
            ippiRGBToYUV_8u_AC4R( colour, psb4, yuva_orig, psb4, srcsize);
            //extract Y, U, V Images
            pyuva[0]= y_orig;
            pyuva[1]= u_orig;
            pyuva[2]= v_orig; 
            pyuva[3]= tmp; 
            ippiCopy_8u_C4P4R( yuva_orig, psb4, pyuva, psb, srcsize );

            //intensity process: performs centre-surround uniqueness analysis
            centerSurr->proc_im_8u( y_orig , psb );
            ippiCopy_8u_C1R( centerSurr->get_centsur_norm8u(), centerSurr->get_psb_8u(), ycs_out, ycs_psb , srcsize );
               
            //Colour process U performs centre-surround uniqueness analysis:
            centerSurr->proc_im_8u( u_orig , psb );
            ippiAdd_32f_C1IR( centerSurr->get_centsur_32f(), centerSurr->get_psb_32f(), cs_tot_32f, psb_32f, srcsize );
            //Colour process V:performs centre-surround uniqueness analysis:
            centerSurr->proc_im_8u( v_orig , psb );
            ippiAdd_32f_C1IR( centerSurr->get_centsur_32f(), centerSurr->get_psb_32f(), cs_tot_32f, psb_32f, srcsize );

            //get min max
            Ipp32f valueMin,valueMax;
            valueMin = 0.0;
            valueMax = 0.0;
            ippiMinMax_32f_C1R( cs_tot_32f, psb_32f, srcsize, &valueMin, &valueMax );
            //if ( valueMax == valueMin ){ valueMax = 255.0f; valueMin = 0.0f; }
            ippiScale_32f8u_C1R( cs_tot_32f,psb_32f,colcs_out,col_psb,srcsize, valueMin, valueMax );
                
            //revert to yarp images
            ippiCopy_8u_C1R( ycs_out, ycs_psb, img_Y->getRawImage(), img_Y->getRowSize(), srcsize );
            ippiCopy_8u_C1R( colcs_out,col_psb, img_UV->getRawImage(), img_UV->getRowSize(), srcsize );

            //this is nasty, resizes the images...
            unsigned char* imgY = img_Y->getPixelAddress( KERNSIZEMAX, KERNSIZEMAX );
            unsigned char* imgUV = img_UV->getPixelAddress( KERNSIZEMAX, KERNSIZEMAX );
            unsigned char* imgYo = img_out_Y->getRawImage();
            unsigned char* imgUVo = img_out_UV->getRawImage();
            int rowsize= img_out_Y->getRowSize();
            int rowsize2= img_Y->getRowSize();

            for(int row=0; row<origsize.height; row++) {
                for(int col=0; col<origsize.width; col++) {
                    *imgYo  = *imgY;
                    *imgUVo = *imgUV;
                    imgYo++;  imgUVo++;
                    imgY++;   imgUV++;
                }    
                imgYo+=rowsize - origsize.width;
                imgUVo+=rowsize - origsize.width;
                imgY+=rowsize2 - origsize.width;
                imgUV+=rowsize2 - origsize.width;
            }

            //output Y centre-surround results to ports
            if ( imageOutPortY->getOutputCount()>0 ){
                imageOutPortY->prepare() = *img_out_Y;	
                imageOutPortY->write();
            }

            //output UV centre-surround results to ports
            if ( imageOutPortUV->getOutputCount()>0 ){
                imageOutPortUV->prepare() = *img_out_UV;	
                imageOutPortUV->write();
            }
            //reset 
            ippiSet_32f_C1R( 0.0, cs_tot_32f, psb_32f, srcsize );
        }
    }
}

void YUVThread::threadRelease() 
{
    cout << "cleaning up things.." << endl;
    deallocate();
    cout << "finished cleaning up" << endl;
}

void YUVThread::deallocate()
{
    delete centerSurr;
    delete img_out_Y;
    delete img_out_UV;
    delete img_Y;
    delete img_UV;
    delete inputExtImage;
    ippiFree( orig );
    ippiFree( colour ); 
    ippiFree( tmp ); 
    free ( pyuva );
    ippiFree( yuva_orig );
    ippiFree( y_orig );
    ippiFree( u_orig );
    ippiFree( v_orig );
    ippiFree( cs_tot_32f );
    ippiFree( colcs_out );
    ippiFree( ycs_out );

	img_out_Y = NULL;    
	img_out_UV = NULL;
    inputExtImage = NULL;
    img_Y = NULL;    
	img_UV = NULL;
    pyuva = NULL;
    centerSurr = NULL;
    cs_tot_32f = NULL;
    orig = NULL;
    colour = NULL;
    yuva_orig = NULL;
    y_orig = NULL; 
    u_orig = NULL; 
    v_orig = NULL; 
    tmp = NULL;
    ycs_out = NULL; 
    colcs_out = NULL;
    
    allocated = false;
}

void YUVThread::allocate( ImageOf<PixelRgb> *img )
{
    assert (allocated == false);
    origsize.width = img->width();
    origsize.height = img->height();

    srcsize.width = origsize.width + 2 * KERNSIZEMAX;
    srcsize.height = origsize.height + 2* KERNSIZEMAX;

    cout << "Received input image dimensions: " << origsize.width << " " << origsize.height << endl;
    cout << "Will extend these to: " << srcsize.width << " " << srcsize.height << endl;

    orig    = ippiMalloc_8u_C3( srcsize.width, srcsize.height, &img_psb );
    colour  = ippiMalloc_8u_C4( srcsize.width, srcsize.height, &psb4);

    yuva_orig = ippiMalloc_8u_C1( srcsize.width *4, srcsize.height, &psb4);
    y_orig    = ippiMalloc_8u_C1( srcsize.width, srcsize.height, &psb);
    u_orig    = ippiMalloc_8u_C1( srcsize.width, srcsize.height, &psb);
    v_orig    = ippiMalloc_8u_C1( srcsize.width, srcsize.height, &psb);
    
    tmp     = ippiMalloc_8u_C1( srcsize.width, srcsize.height, &psb );// to separate alpha channel
    pyuva = (Ipp8u**) malloc(4*sizeof(Ipp8u*));

    cs_tot_32f  = ippiMalloc_32f_C1( srcsize.width, srcsize.height, &psb_32f );
    colcs_out   = ippiMalloc_8u_C1( srcsize.width, srcsize.height,  &col_psb );
    ycs_out     = ippiMalloc_8u_C1( srcsize.width, srcsize.height,  &ycs_psb );

    ncsscale = 4;

    centerSurr  = new CentSur( srcsize , ncsscale );

    inputExtImage=new ImageOf<PixelRgb>;
    inputExtImage->resize( srcsize.width, srcsize.height );

	img_Y = new ImageOf<PixelMono>;
	img_Y->resize( srcsize.width, srcsize.height );

	img_UV = new ImageOf<PixelMono>;
	img_UV->resize( srcsize.width, srcsize.height );

    img_out_Y = new ImageOf<PixelMono>;
	img_out_Y->resize( origsize.width, origsize.height );

	img_out_UV = new ImageOf<PixelMono>;
	img_out_UV->resize( origsize.width, origsize.height );
        
    allocated = true;
    cout << "done allocating" << endl;
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


