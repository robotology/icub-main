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
 
    inputPortNameY           = "/";
    inputPortNameY           += getName(
                           rf.check("Y_IN", 
                           Value("/Y/image:i"),
                           "Input image port (string)").asString()
                           );

    inputPortNameU           = "/";
    inputPortNameU           += getName(
                           rf.check("U_IN", 
                           Value("/U/image:i"),
                           "Input image port (string)").asString()
                           );

    inputPortNameV           = "/";
    inputPortNameV           += getName(
                           rf.check("V_IN", 
                           Value("/V/image:i"),
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
       
    if (!inputPortY.open(inputPortNameY.c_str())) {
        cout << getName() << ": unable to open port " << inputPortNameY << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!inputPortU.open(inputPortNameU.c_str())) {
        cout << getName() << ": unable to open port " << inputPortNameU << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!inputPortV.open(inputPortNameV.c_str())) {
        cout << getName() << ": unable to open port " << inputPortNameV << endl;
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

    yuvThread = new YUVThread(&inputPortY,&inputPortU, &inputPortV, &outPortY, &outPortUV);

    /* now start the thread to do the work */
    yuvThread->start(); // this calls threadInit() and it if returns true, it then calls run()

    return true ;      // let the RFModule know everything went well
}


bool yuvProc::interruptModule()
{
    inputPortY.interrupt();
    inputPortU.interrupt();
    inputPortV.interrupt();
    outPortY.interrupt();
    outPortUV.interrupt();
    handlerPort.interrupt();
    yuvThread->stop();
    return true;
}


bool yuvProc::close()
{
    inputPortY.close();
    inputPortU.close();
    inputPortV.close();    
    outPortY.close();
    outPortUV.close();   
    handlerPort.close();
    /* stop the thread */
    yuvThread->stop();
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

YUVThread::YUVThread(BufferedPort<ImageOf<PixelMono> > *inputPortY, BufferedPort<ImageOf<PixelMono> > *inputPortU, BufferedPort<ImageOf<PixelMono> > *inputPortV, BufferedPort<ImageOf<PixelMono> > *outPortY, BufferedPort<ImageOf<PixelMono> > *outPortUV)
{
    imageInputPortY    = inputPortY;
    imageInputPortU    = inputPortU;
    imageInputPortV    = inputPortV;
    imageOutPortY   = outPortY;
    imageOutPortUV   = outPortUV;
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
        
        do {
         imgY = imageInputPortY->read(false);
      } while (imgY == NULL);

        do {
         imgU = imageInputPortU->read(false);
      } while (imgU == NULL);

        do {
         imgV = imageInputPortV->read(false);
      } while (imgV == NULL);

        if (init){//Get first RGB image to establish width, height:
            cout << "initializing" << endl;   
            initAll();     
        }

    /*  //used for testing: by creating your own YUV image ( from RBG eg...getting logpolar images straight from robot ) with alpha chanenl and separate Y U and V channel 
        ippiCopy_8u_C3R( img->getPixelAddress(0,0),img->getRowSize(), colour_in, psb3, srcsize);
        //convert to RGBA:
        ippiCopy_8u_C3AC4R(colour_in,psb3,colour,psb4,srcsize);
        //convert to Y,U,V image channels:
        ippiRGBToYUV_8u_AC4R(colour,psb4,yuva_orig,psb4,srcsize);
        //extract Y, U, V Images
        pyuva[0]= y_orig;
        pyuva[1]= u_orig;
        pyuva[2]= v_orig; 
        pyuva[3]= tmp; 
        ippiCopy_8u_C4P4R(yuva_orig,psb4,pyuva,psb,srcsize);
        */
        
        ippiCopy_8u_C1R( imgY->getRawImage(), imgY->getRowSize(), y_orig, y_psb, srcsize);
        ippiCopy_8u_C1R( imgU->getRawImage(), imgU->getRowSize(), u_orig, u_psb, srcsize);
        ippiCopy_8u_C1R( imgV->getRawImage(), imgV->getRowSize(), v_orig, v_psb, srcsize);

        //intensity process: performs centre-surround uniqueness analysis
        centerSurr->proc_im_8u( y_orig , y_psb );
        ippiCopy_8u_C1R(centerSurr->get_centsur_norm8u(),centerSurr->get_psb_8u(), ycs_out, ycs_psb , srcsize);
 
//-----------------------------------------------------------------------------------------------------------------------------------------------------------

        //Colour process U performs centre-surround uniqueness analysis:
        centerSurr->proc_im_8u( u_orig , u_psb );
        ippiAdd_32f_C1IR(centerSurr->get_centsur_32f(),centerSurr->get_psb_32f(),cs_tot_32f,psb_32f,srcsize);
        //Colour process V:
        centerSurr->proc_im_8u( v_orig , v_psb );
        ippiAdd_32f_C1IR(centerSurr->get_centsur_32f(),centerSurr->get_psb_32f(),cs_tot_32f,psb_32f,srcsize);

        //get min max
        ippiMinMax_32f_C1R(cs_tot_32f,psb_32f,srcsize,&min,&max); 
        if (max==min){max=255.0;min=0.0;}
        ippiScale_32f8u_C1R(cs_tot_32f,psb_32f,colcs_out,col_psb,srcsize,min,max);
 

		//revert to yarp image
		ippiCopy_8u_C1R(ycs_out, ycs_psb, img_out_Y->getRawImage(), img_out_Y->getRowSize(), srcsize);
		ippiCopy_8u_C1R(colcs_out, col_psb, img_out_UV->getRawImage(), img_out_UV->getRowSize(), srcsize);

        //output Y centre-surround results to ports
        if (imageOutPortY->getOutputCount()>0){
            imageOutPortY->prepare() = *imgY;	
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
}

void YUVThread::threadRelease() 
{
    /* for example, delete dynamically created data-structures */
}

void YUVThread::initAll()
{
    width = imgY->width();
    height = imgY->height();
    cout << "Received input image dimensions: " << imgY->width() << " " << imgY->height() << endl;
    init = false;
    srcsize.width = width;
    srcsize.height = height;
  
    //yuva_orig = ippiMalloc_8u_C1(width*4,height,&psb4);
    //colour_in   = ippiMalloc_8u_C3(width,height,&psb3);
    //colour      = ippiMalloc_8u_C4(width,height,&psb4);

    ycs_out     = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&ycs_psb);
    
    y_orig      = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&y_psb);// to separate Y channel
    u_orig      = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&u_psb);// to separate U channel
    v_orig      = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&v_psb);// to separate V channel

    //tmp         = ippiMalloc_8u_C1(width,height,&psb);// to separate alpha channel
    //pyuva = (Ipp8u**) malloc(4*sizeof(Ipp8u*));

    cs_tot_32f  = ippiMalloc_32f_C1(srcsize.width,srcsize.height,&psb_32f);
    colcs_out   = ippiMalloc_8u_C1(srcsize.width,srcsize.height,&col_psb);

    ncsscale = 4;
    centerSurr  = new CentSur(srcsize,ncsscale);

	img_out_Y = new ImageOf<PixelMono>;
	img_out_Y->resize(srcsize.width, srcsize.height);

	img_out_UV = new ImageOf<PixelMono>;
	img_out_UV->resize(srcsize.width, srcsize.height);
	
}
