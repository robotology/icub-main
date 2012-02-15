/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Vadim Tikhanoff, Andrew Dankers
 * email:   vadim.tikhanoff@iit.it
 * website: www.robotcub.org 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txtd
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include "iCub/lumaChroma.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const int KERNSIZEMAX = 9;

using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;

bool lumaChroma::configure(yarp::os::ResourceFinder &rf)
{    
    /* Process all parameters from both command-line and .ini file */

    moduleName            = rf.check("name", 
                           Value("lumaChroma"), 
                           "module name (string)").asString();

    setName(moduleName.c_str());

    imageType            = rf.check("image", 
                           Value("yuv"), 
                           "image type (string)").asString();
    
   /*
    * attach a port of the same name as the module (prefixed with a /) to the module
    * so that messages received from the port are redirected to the respond method
    */

    handlerPortName =  "/";
    handlerPortName += getName();         // use getName() rather than a literal 
 
    if (!handlerPort.open(handlerPortName.c_str())) 
    {           
        cout << getName() << ": Unable to open port " << handlerPortName << endl;  
        return false;
    }

    attach(handlerPort);    // attach to port

    /* create the thread and pass pointers to the module parameters */
    procThread = new PROCThread(moduleName, imageType);

    /* now start the thread to do the work */
    procThread->open();

    return true ;      // let the RFModule know everything went well
}

bool lumaChroma::interruptModule()
{
    handlerPort.interrupt();
    return true;
}

bool lumaChroma::close()
{
    handlerPort.close();
    /* stop the thread */
    cout << "starting the shutdown procedure " << endl;    
    procThread->interrupt();
    procThread->close();
    fprintf(stdout, "deleting thread \n");
    delete procThread;
    fprintf(stdout, "done deleting thread \n");
    return true;
}

bool lumaChroma::respond(const Bottle& command, Bottle& reply) 
{
    reply.clear(); 

    if (command.get(0).asString()=="quit") 
    {
        reply.addString("quitting");
        return false;     
    }
    else if (command.get(0).asString()=="help") 
    {
        cout << "Options:" << endl << endl;
        cout << "\t--name       name: module name (default: lumaChroma)"                      << endl;
        cout << "\t--image      type: image type to process (default: yuv)"                   << endl;
        reply.addString("ok");
    }
    else
    {
	    cout << "command not known - type help for more info" << endl;
	}
    return true;
}

/* Called periodically every getPeriod() seconds */

bool lumaChroma::updateModule() 
{
    return true;
}

double lumaChroma::getPeriod() 
{
    return 0.1;
}

PROCThread::~PROCThread()
{

}

PROCThread::PROCThread( string moduleName, string imgType )
{
    isYUV = true;
    //set up module name
    this->moduleName = moduleName;

    //set up module name
    if (imgType == "yuv" || imgType == "YUV") 
    {
        cout << "will run module using the YUV image colour space" << endl;
    }else if (imgType == "hsv" || imgType == "HSV") 
    {
        isYUV = false;
        cout << "will run module using the HSV image colour space" << endl;
    }else
    {
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
    centerSurr = NULL;
    allocated = false;
}

bool PROCThread::open() 
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

    this->useCallback();

    //create all ports
    inputPortName = "/" + moduleName + "/image:i";
    BufferedPort<ImageOf<PixelRgb> >::open( inputPortName.c_str() );

    outputPortName1 = "/" + moduleName + "/" + temp[0] + "/image:o";
    imageOutPort1.open( outputPortName1.c_str() );
    
    outputPortName2 = "/" + moduleName + "/" + temp[1] + "/image:o";
    imageOutPort2.open( outputPortName2.c_str() );
    
    if (!isYUV) 
    {
        outputPortName3 = "/" + moduleName + "/" + temp[2] + "/image:o";
        imageOutPort3.open( outputPortName3.c_str() );
    }

    check=false;
   
    return true;
}

void PROCThread::onRead(ImageOf<yarp::sig::PixelRgb> &img) 
{
    mutex.wait();
    if(check)
    {
        mutex.post();
        return;
    }

    if( !allocated || img.width() != img_out_Y->width() || img.height() != img_out_Y->height() ) 
    {
        deallocate();
        allocate( img );
    }
    extender( img, KERNSIZEMAX );
	if ( isYUV )
        cv::cvtColor( cv::Mat((IplImage*)inputExtImage->getIplImage()), orig, CV_RGB2YCrCb);
    else
		cv::cvtColor( cv::Mat((IplImage*)inputExtImage->getIplImage()), orig, CV_RGB2HSV);
    
    vector<cv::Mat> planes;
    cv::split(orig, planes);
    //performs centre-surround uniqueness analysis on first plane
    centerSurr->proc_im_8u( planes[0] );
	
    IplImage y_img = centerSurr->get_centsur_norm8u();
    cvCopyImage( &y_img, ( IplImage *)img_Y->getIplImage());
	csTot32f.setTo(cv::Scalar(0));

    //performs centre-surround uniqueness analysis on second plane:
    centerSurr->proc_im_8u( planes[1] );
    if ( isYUV )
		cv::add(centerSurr->get_centsur_32f(), csTot32f, csTot32f);
    else
    {
		IplImage s_img = centerSurr->get_centsur_norm8u();
        cvCopyImage( &y_img, ( IplImage *)img_UV->getIplImage());
    }
    //performs centre-surround uniqueness analysis on third plane:
    centerSurr->proc_im_8u( planes[2] );
    if ( isYUV )
		cv::add(centerSurr->get_centsur_32f(), csTot32f, csTot32f);
    else
    {
        IplImage v_img = centerSurr->get_centsur_norm8u();
        cvCopyImage( &v_img, ( IplImage *)img_V->getIplImage());
    }
    if ( isYUV )
    {
        //get min max   
        double valueMin = 0.0f;
        double  valueMax = 0.0f;
        cv::minMaxLoc(csTot32f, &valueMin, &valueMax);
        if ( valueMax == valueMin )
        {
            valueMax = 255.0f; valueMin = 0.0f;
        }
        cv::convertScaleAbs( csTot32f, uvimg, 255/(valueMax - valueMin), -255*valueMin/(valueMax-valueMin) );
        IplImage uv_img = uvimg;
        cvCopyImage( &uv_img, ( IplImage *)img_UV->getIplImage());
    }
    //this is nasty, resizes the images...
    unsigned char* imgY = img_Y->getPixelAddress( KERNSIZEMAX, KERNSIZEMAX );
    unsigned char* imgUV = img_UV->getPixelAddress( KERNSIZEMAX, KERNSIZEMAX );
    unsigned char* imgV;
    unsigned char* imgVo;

    if (!isYUV)
    {
       imgV = img_V->getPixelAddress( KERNSIZEMAX, KERNSIZEMAX );
       imgVo = img_out_V->getRawImage();
    }
    unsigned char* imgYo = img_out_Y->getRawImage();
    unsigned char* imgUVo = img_out_UV->getRawImage();
    int rowsize= img_out_Y->getRowSize();
    int rowsize2= img_Y->getRowSize();
    for(int row=0; row<origsize.height; row++) 
    {
        for(int col=0; col<origsize.width; col++) 
        {
            *imgYo  = *imgY;
            *imgUVo = *imgUV;
            if (!isYUV) 
            {
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
        if (!isYUV) 
        {
            imgVo+=rowsize - origsize.width;
            imgV+=rowsize2 - origsize.width;       
        }
    }
    
    //output Y or H centre-surround results to ports
    if ( imageOutPort1.getOutputCount()>0 )
    {
        imageOutPort1.prepare() = *img_out_Y;	
        imageOutPort1.write();
    }

    //output UV or V centre-surround results to ports
    if ( imageOutPort2.getOutputCount()>0 )
    {
         imageOutPort2.prepare() = *img_out_UV;	
        imageOutPort2.write();
    }
    //output H centre-surround results to ports
    if ( !isYUV && imageOutPort3.getOutputCount()>0 )
    {
        imageOutPort3.prepare() = *img_out_V;	
        imageOutPort3.write();
    }
    mutex.post();
}

void PROCThread::allocate( ImageOf<PixelRgb> &img )
{
    origsize.width = img.width();
    origsize.height = img.height();

    srcsize.width = origsize.width + 2 * KERNSIZEMAX;
    srcsize.height = origsize.height + KERNSIZEMAX;

    cout << "Received input image dimensions: " << origsize.width << " " << origsize.height << endl;
    cout << "Will extend these to: " << srcsize.width << " " << srcsize.height << endl;

    orig = cv::Mat(srcsize.width, srcsize.height, CV_8UC3);
	csTot32f = cv::Mat( srcsize.height, srcsize.width, CV_32FC1 );
    uvimg = cv::Mat( srcsize.height, srcsize.width, CV_32FC1 );

    ncsscale = 4;
    centerSurr  = new CentSur( srcsize , ncsscale );

    inputExtImage = new ImageOf<PixelRgb>;
    inputExtImage->resize( srcsize.width, srcsize.height );

	img_Y = new ImageOf<PixelMono>;
	img_Y->resize( srcsize.width, srcsize.height );

    img_out_Y = new ImageOf<PixelMono>;
	img_out_Y->resize( origsize.width, origsize.height );

    img_UV = new ImageOf<PixelMono>;
	img_UV->resize( srcsize.width, srcsize.height );

    img_out_UV = new ImageOf<PixelMono>;
	img_out_UV->resize( origsize.width, origsize.height );

    img_V = new ImageOf<PixelMono>;
	img_V->resize( srcsize.width, srcsize.height );

    img_out_V = new ImageOf<PixelMono>;
	img_out_V->resize( origsize.width, origsize.height );
        
    allocated = true;
    cout << "done allocating" << endl;
}

void PROCThread::deallocate( )
{
    delete img_out_Y;
    delete img_out_UV;
    delete img_out_V;
    delete img_Y;
    delete img_UV;
    delete img_V;
    delete inputExtImage;
	img_out_Y = NULL;    
	img_out_UV = NULL;
    img_out_V = NULL;
    inputExtImage = NULL;
    img_Y = NULL;    
	img_UV = NULL;
    img_V = NULL;
    delete centerSurr;
	centerSurr = NULL;
    
    orig.release();
    uvimg.release();
    csTot32f.release();

    allocated = false;
}

void PROCThread::close() 
{
    cout << "now closing ports..." << endl;
    mutex.wait();
    imageOutPort1.close();
    imageOutPort2.close();
    if ( !isYUV ) 
        imageOutPort3.close();
    deallocate();
    cout << "deallocated all attempting to close read port..." << endl;
    BufferedPort<ImageOf<PixelRgb> >::close();
    mutex.post();
    cout << "finished closing the read port..." << endl;
}

void PROCThread::interrupt() 
{
    mutex.wait();
    check=true;
    cout << "cleaning up..." << endl;
    cout << "attempting to interrupt ports" << endl;
    
    imageOutPort1.interrupt();
    imageOutPort2.interrupt();

    if ( !isYUV ) 
        imageOutPort3.interrupt();
    
    BufferedPort<ImageOf<PixelRgb> >::interrupt();
    cout << "finished interrupt ports" << endl;
    mutex.post();
}

ImageOf<PixelRgb>* PROCThread::extender(ImageOf<PixelRgb>& inputOrigImage, int maxSize) 
{
    // copy of the image 
	inputExtImage->copy( inputOrigImage, srcsize.width, srcsize.height );
    // memcpy of the horizontal fovea lines (rows) 
    const int sizeBlock = origsize.width / 2;
    for(int i = 0; i < maxSize; i++) 
    {
        memcpy( inputExtImage->getPixelAddress( sizeBlock + maxSize, maxSize-1-i ),
            inputExtImage->getPixelAddress( maxSize, maxSize+i ),
            sizeBlock*sizeof(PixelRgb));
        memcpy( inputExtImage->getPixelAddress( maxSize, maxSize-1-i ),
            inputExtImage->getPixelAddress( sizeBlock + maxSize, maxSize+i ),
            sizeBlock*sizeof(PixelRgb));
    }
    // copy of the block adjacent angular positions (columns)
    const int px = maxSize * sizeof(PixelRgb);

    for (int row = 0; row < srcsize.height; row++) 
    {
        memcpy ( inputExtImage->getPixelAddress( srcsize.width-maxSize, row ),
            inputExtImage->getPixelAddress( maxSize,row ), px);
        memcpy ( inputExtImage->getPixelAddress( 0, row ),
            inputExtImage->getPixelAddress( srcsize.width- maxSize-maxSize, row ), px);
    }
    return inputExtImage;
}

