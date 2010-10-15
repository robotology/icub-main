/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Vadim Tikhanoff Andrew Dankers
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

/** 
 * @ingroup icub_module
 *
 * \defgroup icub_zdfModule zdfMod
 *
 * Receives the left and right images from the robot and segments objects that are located in the fovea. Performs marker-less pixel-wise segmentation of an object located in the fovea.The output is an image of the segmented object in grayscale and a difference of gausian segmentation.
 *
 * 
 * \section lib_sec Libraries
 *
 * YARP
 * IPP
 * OPENCV
 *
 * \section parameters_sec Parameters
 * 
 * Command-line Parameters 
 * 
 * The following key-value pairs can be specified as command-line parameters by prefixing \c -- to the key 
 * (e.g. \c --from file.ini. The value part can be changed to suit your needs; the default values are shown below. 
 *
 * - \c from \c zdfMod.ini \n 
 *   specifies the configuration file
 *
 * - \c context \c zdfMod/conf \n
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c zdfMod \n   
 *   specifies the name of the module (used to form the stem of module port names)  
 *
 * Configuration File Parameters
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 *   
 * - \c inPortLeft \c /imageLeft:i \n    
 *   specifies the input port name (this string will be prefixed by \c /zdfMod 
 *   or whatever else is specifed by the name parameter
 *
 * - \c inPortRight \c /imageRight:i \n    
 *   specifies the input port name (this string will be prefixed by \c /zdfMod 
 *   or whatever else is specifed by the name parameter
 *
 * - \c outPortProb \c /imageProb:o \n    
 *   specifies the input port name (this string will be prefixed by \c /zdfMod 
 *   or whatever else is specifed by the name parameter
 *
 * - \c outPortSeg \c /imageSeg:o \n  
 *   specifies the output port name (this string will be prefixed by \c /zdfMod 
 *   or whatever else is specifed by the name parameter
 *
 * - \c outPortDog \c /imageDog:o \n  
 *   specifies the output port name (this string will be prefixed by \c /zdfMod 
 *   or whatever else is specifed by the name parameter

 * \section portsa_sec Ports Accessed
 * 
 * - None
 *                      
 * \section portsc_sec Ports Created
 *
 *  Input ports
 *
 *  - \c /zdfMod \n
 *    This port is used to change the parameters of the module at run time or stop the module. \n
 *    The following commands are available
 * 
 *    \c help \n
 *    \c quit \n
 *
 *    Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /zdfMod
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 *  - \c /zdfMod/imageLeft:i \n
 *  - \c /zdfMod/imageRight:i \n
 *
 * Output ports
 *
 *  - \c /zdfMod \n
 *    see above
 *
 *  - \c /zdfMod/imageProb:o \n
 *  - \c /zdfMod/imageSeg:o \n
 *  - \c /zdfMod/imageDog:o \n
 *
 * Port types
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 * \c BufferedPort<ImageOf<PixelBgr> >   \c inPortLeft; \n 
 * \c BufferedPort<ImageOf<PixelBgr> >   \c inPortRight; \n 
 * \c BufferedPort<ImageOf<PixelMono> >   \c outPortProb; \n 
 * \c BufferedPort<ImageOf<PixelMono> >   \c outPortSeg;   
 * \c BufferedPort<ImageOf<PixelMono> >   \c outPortDog;       
 *
 * \section in_files_sec Input Data Files
 *
 * None
 *
 * \section out_data_sec Output Data Files
 *
 * None
 *
 * \section conf_file_sec Configuration Files
 *
 * \c zdfMod.ini  in \c $ICUB_ROOT/app/zdfMod/conf \n
 * 
 * \section tested_os_sec Tested OS
 *
 * Linux: Ubuntu 9.10 and Debian Stable 
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>zdfMod --name zdfMod --context zdfMod/conf --from zdfMod.ini </tt>
 *
 * \author 
 * 
 * Vadim Tikhanoff, Andrew Dankers
 * 
 * Copyright (C) 2009 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * This file can be edited at \c $ICUB_ROOT/src\modules\zdfModule\src\main.cpp
 * 
 */

#include "iCub/zdfMod.h"
#include <math.h>
#include <cmath>
#include <cassert>

using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;

bool zdfMod::configure(yarp::os::ResourceFinder &rf)
{    
    /* Process all parameters from both command-line and .ini file */
    
    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name", 
                           Value("zdfMod"), 
                           "module name (string)").asString();

    setName(moduleName.c_str());
    parameters.iter_max = rf.findGroup("PARAMS").check("max_iteration",Value(1),"what did the user select?").asInt();
    parameters.randomize_every_iteration = rf.findGroup("PARAMS").check("randomize_iteration",Value(1),"what did the user select?").asInt();
    parameters.smoothness_penalty_base = rf.findGroup("PARAMS").check("smoothness_penalty_base",Value(1),"what did the user select?").asInt();
    parameters.smoothness_penalty = rf.findGroup("PARAMS").check("smoothness_penalty",Value(1),"what did the user select?").asInt();
    parameters.data_penalty  = rf.findGroup("PARAMS").check("data_penalty",Value(1),"what did the user select?").asInt();
    parameters.smoothness_3sigmaon2 = rf.findGroup("PARAMS").check("smoothness_3sigmaon2",Value(1),"what did the user select?").asInt();
    parameters.bland_dog_thresh = rf.findGroup("PARAMS").check("bland_dog_thresh",Value(1),"what did the user select?").asInt();
    parameters.radial_penalty = rf.findGroup("PARAMS").check("radial_penalty",Value(1),"what did the user select?").asInt();
    parameters.acquire_wait = rf.findGroup("PARAMS").check("acquire_wait",Value(1),"what did the user select?").asInt();
    parameters.min_area = rf.findGroup("PARAMS").check("min_area",Value(1),"what did the user select?").asInt();
    parameters.max_area = rf.findGroup("PARAMS").check("max_area",Value(1),"what did the user select?").asInt();
    parameters.max_spread = rf.findGroup("PARAMS").check("max_spread",Value(1),"what did the user select?").asInt();
    parameters.cog_snap = rf.findGroup("PARAMS").check("cog_snap",Value(1),"what did the user select?").asDouble();
    parameters.bland_prob = rf.findGroup("PARAMS").check("bland_prob",Value(1),"what did the user select?").asDouble();

   /*
    * attach a port of the same name as the module (prefixed with a /) to the module
    * so that messages received from the port are redirected to the respond method
    */

    handlerName =  "/";
    handlerName += getName();         // use getName() rather than a literal 
 
    if (!handlerPort.open(handlerName.c_str())) {           
        cout << getName() << ": Unable to open port " << handlerName << endl;  
        return false;
    }

    attach(handlerPort);               // attach to port
    //attachTerminal();                // attach to terminal (maybe not such a good thing...)

    /* create the thread and pass pointers to the module parameters */
    zdfThread = new ZDFThread( &parameters );

    /*pass the name of the module in order to create ports*/
    zdfThread->setName(moduleName);

    /* now start the thread to do the work */
    zdfThread->start(); // this calls threadInit() and it if returns true, it then calls run()

    return true ;
}

/* Called periodically every getPeriod() seconds */

bool zdfMod::updateModule() 
{

    return true;
}

bool zdfMod::interruptModule() 
{

    handlerPort.interrupt();
    return true;
}

bool zdfMod::close() 
{
    handlerPort.close();
    zdfThread->stop();
    cout << "deleteing thread " << endl;
    delete zdfThread;
    return true;
}

double zdfMod::getPeriod() 
{
    return 0.1;
}


bool zdfMod::respond(const Bottle& command, Bottle& reply) 
{

    bool ok = false;
    bool rec = false; // is the command recognized?

   // mutex.wait();
    switch (command.get(0).asVocab()) {
    case COMMAND_VOCAB_HELP:
        rec = true;
        {

        string helpMessage =  string(getName().c_str()) + 
                        " commands are: \n" +  
                        "help \n" + 
                        "quit \n";

        reply.clear();
        ok = true;
    } 
    break;

/*  cvNamedWindow( "Settings", 0) ; 
    cvCreateTrackbar( "DATA_PENALTY", "Settings", &params->data_penalty, 255, 0 );
    cvCreateTrackbar( "SMOOTHNESS_PENALTY_BASE", "Settings", &params->smoothness_penalty_base, 255, 0 );
    cvCreateTrackbar( "SMOOTHNESS_PENALTY", "Settings", &params->smoothness_penalty, 1000, 0 );
    cvCreateTrackbar( "RADIAL_PENALTY", "Settings", &params->radial_penalty, 255, 0 );
    cvCreateTrackbar( "SMOOTHNESS_3SIGMAON2", "Settings", &params->smoothness_3sigmaon2, 255, 0 );
    cvCreateTrackbar( "BLAND_DOG_THRESH", "Settings", &params->bland_dog_thresh, 255, 0 );*/

    case COMMAND_VOCAB_SET:
        rec = true;
        {
        switch(command.get(1).asVocab()) {
            case COMMAND_VOCAB_K1:{
                double w = command.get(2).asDouble();
                zdfThread->params->data_penalty = w;
                ok = true;
            }
            break;
            case COMMAND_VOCAB_K2:{
                double w = command.get(2).asDouble();
                zdfThread->params->smoothness_penalty_base = w;
                ok = true;
            }
            break;
            case COMMAND_VOCAB_K3:{
                double w = command.get(2).asDouble();
                zdfThread->params->smoothness_penalty = w;
                ok = true;
            }
            break;
            case COMMAND_VOCAB_K4:{
                double w = command.get(2).asDouble();
                zdfThread->params->radial_penalty = w;
                ok = true;
            }
            break;
            case COMMAND_VOCAB_K5:{
                double w = command.get(2).asDouble();
                zdfThread->params->smoothness_3sigmaon2 = w;
                ok = true;
            }
            break;
            case COMMAND_VOCAB_K6:{
                double w = command.get(2).asDouble();
                zdfThread->params->bland_dog_thresh = w;
                ok = true;
            }
            break;
        }
    case COMMAND_VOCAB_GET:
        rec = true;
        {
        reply.addVocab(COMMAND_VOCAB_IS);
        reply.add(command.get(1));
        switch(command.get(1).asVocab()) {
            case COMMAND_VOCAB_K1:{
                double w = zdfThread->params->data_penalty;
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_K2:{
                double w = zdfThread->params->smoothness_penalty_base;
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_K3:{
                double w = zdfThread->params->smoothness_penalty;
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_K4:{
                double w = zdfThread->params->radial_penalty;
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_K5:{
                double w = zdfThread->params->smoothness_3sigmaon2;
                reply.addDouble(w);
                ok = true;
            }
            break;
            case COMMAND_VOCAB_K6:{
                double w = zdfThread->params->bland_dog_thresh;
                reply.addDouble(w);
                ok = true;
            }
            break;
        
        default: {
            }
                break;
        }
        
    }
    break;
    }
    }
    return true;
}

ZDFThread::ZDFThread( MultiClass::Parameters *parameters ) 
{
    
    params = parameters;
    img_out_prob = NULL;    
	img_out_seg = NULL;
	img_out_dog = NULL;
    img_out_temp = NULL;
    res_t = NULL; 
  	out= NULL, seg_im = NULL, seg_dog = NULL, fov_l = NULL, fov_r = NULL, zd_prob_8u = NULL, o_prob_8u = NULL, tempImg = NULL;
	p_prob = NULL;
  	//templates:
  	temp_l = NULL, temp_r = NULL;
  	//input:
  	rec_im_ly = NULL;
  	rec_im_ry = NULL;
	//Difference of Gaussian:
	dl = NULL;
  	dr = NULL;
	m = NULL;
    l_orig = NULL, r_orig = NULL;
    allocated = false;
}

ZDFThread::~ZDFThread( )
{
    delete dl;
    delete dr;
    delete m;
    ippiFree(l_orig);
    ippiFree(r_orig);
    ippiFree(rec_im_ly);
    ippiFree(rec_im_ry);
    ippiFree(res_t);
    ippiFree(out);
    ippiFree(seg_im);
    ippiFree(seg_dog);
    ippiFree(fov_l);
    ippiFree(fov_r);
    ippiFree(zd_prob_8u);
    ippiFree(o_prob_8u);
    free(p_prob);
    ippiFree(temp_l);
    ippiFree(temp_r);
    delete img_out_prob;
    delete img_out_seg;
    delete img_out_dog; 
}

void ZDFThread::setName(string module) 
{
    this->moduleName = module;
}

bool ZDFThread::threadInit() 
{
    /* initialize variables and create data-structures if needed */

    //create all ports
    inputNameLeft = "/" + moduleName + "/imageLeft:i";
    imageInLeft.open( inputNameLeft.c_str() );
    
    inputNameRight = "/" + moduleName + "/imageRight:i";
    imageInRight.open( inputNameRight.c_str() );

    outputNameProb = "/" + moduleName + "/imageProb:o";
    imageOutProb.open( outputNameProb.c_str() );

    outputNameSeg = "/" + moduleName + "/imageSeg:o";
    imageOutSeg.open( outputNameSeg.c_str() );
    
    outputNameDog = "/" + moduleName + "/imageDog:o";
    imageOutDog.open( outputNameDog.c_str() );

    outputNameTemp = "/" + moduleName + "/imageTemp:o";
    imageOutTemp.open( outputNameTemp.c_str() );
    
    return true;
}

void ZDFThread::run()
{

    while (isStopping() != true) { // the thread continues to run until isStopping() returns true
        
        ImageOf<PixelBgr> *img_in_left = imageInLeft.read(true);
        ImageOf<PixelBgr> *img_in_right = imageInRight.read(true);

        if(img_in_left != NULL && img_in_right != NULL) {

            if( !allocated || img_in_left->width() != insize.width || img_in_left->height() != insize.height) {
                deallocate();
                allocate(img_in_left);
            }

		    //processing for zdf
		    if (scale==1.0){ //resize the images if needed
			    //copy yarp image to IPP
			    ippiCopy_8u_C3R( img_in_left->getRawImage(),  img_in_left->getRowSize(), l_orig, psb, srcsize);
			    ippiCopy_8u_C3R( img_in_right->getRawImage(), img_in_right->getRowSize(), r_orig, psb, srcsize);
          	}else{
			    //scale to width,height:
                ippiResizeGetBufSize(inroi, inroi, 3, IPPI_INTER_CUBIC, &BufferSize);
                Ipp8u* pBuffer=ippsMalloc_8u(BufferSize);
                ippiResizeSqrPixel_8u_C3R( img_in_left->getRawImage(), insize, psb, inroi, l_orig, psb, inroi, scale, scale, 0, 0, IPPI_INTER_CUBIC, pBuffer);   
                ippiResizeSqrPixel_8u_C3R( img_in_right->getRawImage(), insize, psb, inroi, r_orig, psb, inroi, scale, scale, 0, 0, IPPI_INTER_CUBIC, pBuffer);     
                ippsFree(pBuffer);
                //the following is deprecated...use previous
			    //ippiResize_8u_C3R( img_in_left->getRawImage(), insize, img_in_left->width() * 3, inroi, l_orig, psb, srcsize, scale, scale, IPPI_INTER_CUBIC);
			    //ippiResize_8u_C3R( img_in_right->getRawImage(), insize, img_in_right->width() * 3, inroi, r_orig, psb, srcsize, scale, scale, IPPI_INTER_CUBIC);
		    }
		    //copy to grayscale
		    ippiRGBToGray_8u_C3C1R( l_orig, psb , rec_im_ly, psb_in, srcsize );
		    ippiRGBToGray_8u_C3C1R( r_orig, psb , rec_im_ry, psb_in, srcsize );

		    if (acquire){
			    ippiCopy_8u_C1R(&rec_im_ly [(( srcsize.height - tsize.height ) / 2 ) * psb_in + ( srcsize.width - tsize.width ) /2 ], psb_in, temp_l, psb_t, tsize);			
			    ippiCopy_8u_C1R(&rec_im_ry [(( srcsize.height - tsize.height ) / 2 ) * psb_in + ( srcsize.width - tsize.width ) /2 ], psb_in, temp_r, psb_t, tsize);
		    }

		    //Create left fovea and find left template in left image
		    ippiCrossCorrValid_NormLevel_8u32f_C1R(&rec_im_ly[(( srcsize.height - tisize.height )/2)*psb_in + ( srcsize.width - tisize.width )/2],
				           psb_in, tisize,
				           temp_l,
				           psb_t, tsize,
				           res_t, psb_rest);

		    ippiMaxIndx_32f_C1R( res_t, psb_rest, trsize, &max_t, &sx, &sy); 
		    //ippiCopy_8u_C1R( rec_im_ly, psb_in, fov_l, psb_m, srcsize);
		    ippiCopy_8u_C1R( &rec_im_ly [ ( mid_y + tl_y ) * psb_in + mid_x + tl_x], psb_in, fov_l, psb_m, msize ); //original

		    //**************************
		    //Create right fovea and find right template in right image:
		    ippiCrossCorrValid_NormLevel_8u32f_C1R(&rec_im_ry[((srcsize.height-tisize.height)/2 + dpix_y )*psb_in + (srcsize.width-tisize.width)/2],
					    psb_in,tisize,
					    temp_r,
					    psb_t,tsize,
					    res_t, psb_rest);

		    ippiMaxIndx_32f_C1R(res_t,psb_rest,trsize,&max_t,&sx,&sy);
		    //ippiCopy_8u_C1R( rec_im_ry, psb_in, fov_r, psb_m, srcsize);
		    ippiCopy_8u_C1R(&rec_im_ry[(mid_y+tr_y+dpix_y)*psb_in + mid_x+tr_x],psb_in,fov_r,psb_m,msize); // original

            //Star diffence of gaussian on foveated images
		    dl->proc( fov_l, psb_m );
		    dr->proc( fov_r, psb_m );

		    //**************************
		    //SPATIAL ZD probability map from fov_l and fov_r:
		    //perform RANK or NDT kernel comparison:	            	
		    for (int j=koffsety;j<msize.height-koffsety;j++){
      			c.y=j;
      			for (int i=koffsetx;i<msize.width-koffsetx;i++){
	        			c.x=i;
	        			//if either l or r textured at this retinal location: 
	        		if (dl->get_dog_onoff()[i + j*dl->get_psb()] >= params->data_penalty || dr->get_dog_onoff()[i + j*dr->get_psb()] >= params->bland_dog_thresh ){      
	          			if (RANK0_NDT1==0){
						    //use RANK:
						    get_rank(c,fov_l,psb_m,rank1);
						    get_rank(c,fov_r,psb_m,rank2);
						    cmp_res = cmp_rank(rank1,rank2);
	          			}
	        			else{ 
						    //use NDT:
						    get_ndt(c,fov_l,psb_m,ndt1);
						    get_ndt(c,fov_r,psb_m,ndt2);
						    cmp_res = cmp_ndt( ndt1, ndt2 );
	          			}
	          			zd_prob_8u[ j * psb_m + i] = (int)(cmp_res * 255.0);
	        		}
	        		else{
	          				//untextured in both l & r, so set to bland prob (ZD):
	          			zd_prob_8u[j*psb_m+i] = (int)(params->bland_prob * 255.0);//bland_prob
	        		} 

	        		//RADIAL PENALTY:
	        		//The further from the origin, less likely it's ZD, so reduce zd_prob radially:
	        		//current radius:
	        		r = sqrt((c.x-msize.width/2.0)*(c.x-msize.width/2.0)+(c.y-msize.height/2.0)*(c.y-msize.height/2.0));
	        		rad_pen =  (int) ( (r/rmax)* params->radial_penalty );//radial_penalty
	        		max_rad_pen = zd_prob_8u[j*psb_m+i];
	        		if(max_rad_pen < rad_pen) {
	          			rad_pen=max_rad_pen;
	        		}
	        		//apply radial penalty
	        		zd_prob_8u[j*psb_m+i]-= rad_pen;
	        
	        		//manufacture NZD prob (other):
	        		o_prob_8u[psb_m*j+i] = 255 - zd_prob_8u[psb_m*j+i];
	      		}
		    }

		    //Do MRF optimization:
		    m->proc( fov_r, p_prob ); //provide edge map and probability map
		    //cache for distribution:
		    ippiCopy_8u_C1R( m->get_class(), m->get_psb(), out, psb_m, msize);
		
		    //evaluate result:
		    getAreaCoGSpread(out, psb_m , msize, &area, &cog_x, &cog_y, &spread); 	
	
		    cog_x_send = cog_x;
		    cog_y_send = cog_y;
		
		    //we have mask and image  (out)   [0/255].
		    //construct masked image  (fov_l) [0..255]:
		    for (int j=0;j<msize.height;j++){
      			for (int i=0;i<msize.width;i++){
        			if (out[i + j*psb_m  ]==0){
          				seg_im[ j * psb_m + i] = 0;
          				seg_dog[ j * psb_m + i] = 0;
        			}
        			else{
						    //cout << "here also " << endl;
         				seg_dog [ j * psb_m + i] = dr->get_dog_onoff()[j*psb_m + i];
          				seg_im [ j * psb_m + i] = fov_r[j * psb_m + i];
        			}
      			}
		    }
			
		    //If nice segmentation:
		    if (area >= params->min_area && area <= params->max_area && spread<= params->max_spread){ 
      			//don't update templates to image centre any more as we have a nice target
       			acquire = false;
                //update templates towards segmentation CoG:
      			printf("area:%d spread:%f cogx:%f cogy:%f - UPDATING TEMPLATE\n",area,spread,cog_x,cog_y);
      			//Bring cog of target towards centre of fovea:
      			//SNAP GAZE TO OBJECT:
      			cog_x*= params->cog_snap;
      			cog_y*= params->cog_snap;
			    //floor(val + 0.5) instead of round
			    ippiCopy_8u_C1R(&fov_l[( mid_x_m + ( (int) floor ( cog_x + 0.5 ) ) ) + ( mid_y_m + ( ( int ) floor ( cog_y + 0.5) ) ) * psb_m], psb_m, temp_l, psb_t, tsize );
			    ippiCopy_8u_C1R(&fov_r[( mid_x_m + ( (int) floor ( cog_x + 0.5 ) ) ) + ( mid_y_m + ( ( int ) floor ( cog_y + 0.5) ) ) * psb_m], psb_m, temp_r, psb_t, tsize );
			    //ippiCopy_8u_C1R(&fov_l[( mid_x_m + ( (int) round ( cog_x ) ) ) + ( mid_y_m + ( ( int ) round ( cog_y) ) ) * psb_m], psb_m, temp_l, psb_t, tsize );
      			//ippiCopy_8u_C1R(&fov_r[( mid_x_m + ( (int) round ( cog_x ) ) ) + ( mid_y_m + ( ( int ) round ( cog_y) ) ) * psb_m], psb_m, temp_r, psb_t, tsize );
                
                //retreive only the segmented object in order to send as template
                if (imageOutTemp.getOutputCount()>0){ 

                    int top = -1;
                    int left = -1;
                    int right = -1;
                    int bottom = -1;
                 
                    for (int j=0;j<msize.height * psb_m;j++){
                        
                        if ( (int)seg_im[ j ] > 0){
                            top = j/psb_m; 
                            break;
                        }
                
                    }
                    for (int j=msize.height * psb_m;j >0; j--){

                        if ( (int)seg_im[ j ] > 0){
                            bottom = j/psb_m; 
                            break;
                        }
                    }
                    bool out = false;
                    for (int i=0;i<msize.width;i++){
                        for (int j=0;j<msize.height;j++){
                            if ( (int)seg_im[i + j *psb_m] > 0){
                                left = i; 
                                out = true;
                                break;
                            }
                        }
                        if (out)break;
                    }
                    out = false;
                    for (int i=msize.width;i >0; i--){
                        for (int j=0;j<msize.height;j++){
                            if ( (int)seg_im[i + j *psb_m] > 0){
                                right = i; 
                                out = true;
                                break;
                            }
                        }
                        if (out)break;
                    }

                    int u = 0;
                    int v = 0;
                    tempSize.width = right-left + 1;
                    tempSize.height = bottom-top + 1;

                    tempImg = ippiMalloc_8u_C3( tempSize.width, tempSize.height, &psbtemp);
                  
                    img_out_temp = new ImageOf<PixelBgr>;
                    img_out_temp->resize(tempSize.width, tempSize.height);
                     
                   for (int j = top; j < bottom +1;j++){
      			        for (int i = left; i < right + 1;i++){
                            
                            if ( (int)seg_im[i + j *psb_m] > 0){
                                int x = srcsize.width/2 - msize.width/2 + i;
                                int y = srcsize.height/2 - msize.height/2 + j; 
                                
                                tempImg[u*3 + v * psbtemp] = r_orig[x*3 + y * psb];
                                tempImg[u*3 + v * psbtemp + 1] = r_orig[x*3 + y * psb + 1];
                                tempImg[u*3 + v * psbtemp + 2] = r_orig[x*3 + y * psb + 2];    
                            }else{
                                tempImg[u*3 + v * psbtemp] = 0;
                                tempImg[u*3 + v * psbtemp + 1] = 0;
                                tempImg[u*3 + v * psbtemp + 2] = 0;
                            }
                            u ++; 
                            
                        }     
                        u = 0;
                        v ++;
                    }
                    
                    ippiCopy_8u_C3R( tempImg, psbtemp, img_out_temp->getRawImage(), img_out_temp->getRowSize() , tempSize);

                    imageOutTemp.prepare() = *img_out_temp;	
               	    imageOutTemp.write();

                    delete img_out_temp;
                    ippiFree (tempImg);
                }
      			//We've updated, so reset waiting:
     			waiting=0;
       			//report that we-ve updated templates:
      			update = true;
		    }
		    //Otherwise, just keep previous templates:
		    else{
      			printf("area:%d spread:%f cogx:%f cogy:%f\n",area,spread,cog_x,cog_y);	
      			waiting++;
       			//report that we didn't update template:
      			update = false;
		    }
	
		    if(waiting >= 25 ){ //acquire_wait
      			printf("Acquiring new target until nice seg (waiting:%d >= acquire_wait:%d)\n",waiting, params->acquire_wait );//acquire_wait
      			acquire = true;
		    }

		
		    //send it all when connections are established
		    if (imageOutProb.getOutputCount()>0){ 
                ippiCopy_8u_C1R( zd_prob_8u, psb_m, img_out_prob->getRawImage(), img_out_prob->getRowSize(), msize );
			    imageOutProb.prepare() = *img_out_prob;	
               	imageOutProb.write();
            }

		    if (imageOutSeg.getOutputCount()>0){
                ippiCopy_8u_C1R( seg_im, psb_m, img_out_seg->getRawImage(), img_out_seg->getRowSize(), msize );
               	imageOutSeg.prepare() = *img_out_seg;	
               	imageOutSeg.write();
            }
		    if (imageOutDog.getOutputCount()>0){
                ippiCopy_8u_C1R( seg_dog, psb_m, img_out_dog->getRawImage(), img_out_dog->getRowSize(), msize );
               	imageOutDog.prepare() = *img_out_dog;	
               	imageOutDog.write();
            }
		}
	}
}

void ZDFThread::threadRelease() 
{
    
}

void ZDFThread::onStop()
{
    cout << "closing ports.." << endl;
    imageInLeft.close();
    imageInRight.close();
    imageOutProb.close();
    imageOutSeg.close();
    imageOutDog.close();
    imageOutTemp.close(); 
    cout << "finished cleaning.." << endl;
}

void ZDFThread::deallocate() {

    cout << "cleaning up dynamically created objects" << endl;
    delete dl;
    delete dr;
    delete m;
    ippiFree(l_orig);
    ippiFree(r_orig);
    ippiFree(rec_im_ly);
    ippiFree(rec_im_ry);
    ippiFree(res_t);
    ippiFree(out);
    ippiFree(seg_im);
    ippiFree(seg_dog);
    ippiFree(fov_l);
    ippiFree(fov_r);
    ippiFree(zd_prob_8u);
    ippiFree(o_prob_8u);
    free(p_prob);
    ippiFree(temp_l);
    ippiFree(temp_r);
    delete img_out_prob;
    delete img_out_seg;
    delete img_out_dog;

    allocated = false;
}
void ZDFThread::allocate(ImageOf<PixelBgr> *img) {
     assert (allocated == false);

/*  cvNamedWindow( "Settings", 0) ; 
    cvCreateTrackbar( "DATA_PENALTY", "Settings", &params->data_penalty, 255, 0 );
    cvCreateTrackbar( "SMOOTHNESS_PENALTY_BASE", "Settings", &params->smoothness_penalty_base, 255, 0 );
    cvCreateTrackbar( "SMOOTHNESS_PENALTY", "Settings", &params->smoothness_penalty, 1000, 0 );
    cvCreateTrackbar( "RADIAL_PENALTY", "Settings", &params->radial_penalty, 255, 0 );
    cvCreateTrackbar( "SMOOTHNESS_3SIGMAON2", "Settings", &params->smoothness_3sigmaon2, 255, 0 );
    cvCreateTrackbar( "BLAND_DOG_THRESH", "Settings", &params->bland_dog_thresh, 255, 0 );*/

    cout << "Received left input image dimensions: " << img->width() << " " << img->height() << endl;
    cout << "Received right input image dimensions: " << img->width() << " " << img->height() << endl;

    width = 320; 
    height = 240;
    scale = 1.0;
    scale  = ((double)width)/img->width();

    insize.width = img->width();
    insize.height = img->height();

    printf("Scaling to image dimensions: (%d,%d). Scale factor %f\n", width, height,scale);

    BufferSize=0;
    inroi.x=0;
    inroi.y=0;
    inroi.width  =  img->width();
    inroi.height = img->height();

    srcsize.width = img->width();
    srcsize.height = img->height();

    msize.width  = 128;//should be taken from ini file // was 128
    msize.height = 128;//should be taken from ini file // was 128
    tsize.width  = 32;//should be taken from ini file 
    tsize.height = 32;//should be taken from ini file
    imgsize.width  = 64;//should be taken from ini file 
    imgsize.height = 64;//should be taken from ini file

    t_lock_lr = 32;//should be taken from ini file
    t_lock_ud = 32;//should be taken from ini file

    tisize.width  = tsize.width  + 2 * t_lock_lr;
    tisize.height = tsize.height + 2 * t_lock_ud;
    trsize.width  = tisize.width  - tsize.width  + 1;
    trsize.height = tisize.height - tsize.height + 1;

    mid_x = (srcsize.width  - msize.width)/2;
    mid_y = (srcsize.height - msize.height)/2;
    mid_x_m = (msize.width  - tsize.width)/2;
    mid_y_m = (msize.height - tsize.height)/2;
    cog_x = 0.0;
    cog_y = 0.0;
    cog_x_send = 0.0;
    cog_y_send = 0.0;

    if (RANK0_NDT1==0){
        koffsetx = RANKX;
        koffsety = RANKY;
    }
    else{
        koffsetx = NDTX;
        koffsety = NDTY;
    }

    nclasses = 2;
    dpix_y = 0;

    l_orig      = ippiMalloc_8u_C3( srcsize.width, srcsize.height, &psb);
    r_orig      = ippiMalloc_8u_C3( srcsize.width, srcsize.height, &psb);

    rec_im_ly   = ippiMalloc_8u_C1( srcsize.width, srcsize.height, &psb_in);
    rec_im_ry   = ippiMalloc_8u_C1( srcsize.width, srcsize.height, &psb_in);

    res_t     = ippiMalloc_32f_C1(trsize.width,trsize.height,&psb_rest);
    out        = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
    seg_im     = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
    seg_dog    = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);

    fov_l      = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
    fov_r      = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
    zd_prob_8u = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
    o_prob_8u  = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
    p_prob    = (Ipp8u**) malloc(sizeof(Ipp8u*)*nclasses);

    ippiSet_8u_C1R( 0, zd_prob_8u, psb_m, msize );
    ippiSet_8u_C1R( 0, o_prob_8u, psb_m, msize );

    p_prob[0] = o_prob_8u;
    p_prob[1] = zd_prob_8u;

    //templates:
    temp_l     = ippiMalloc_8u_C1(tsize.width,tsize.height, &psb_t);
    temp_r     = ippiMalloc_8u_C1(tsize.width,tsize.height, &psb_t);

    dl = new DoG(msize);
    dr = new DoG(msize);

    m = new MultiClass( msize, psb_m, nclasses, params );

    tl_x = 0;
    tl_y = 0;
    tr_x = 0;
    tr_y = 0;
    waiting = 0;
    rmax = sqrt((msize.width/2.0)*(msize.width/2.0) 
    +(msize.height/2.0)*(msize.height/2.0));

    update = false;
    acquire = true;

    img_out_prob = new ImageOf<PixelMono>;
    img_out_prob->resize(msize.width, msize.height);

    img_out_seg = new ImageOf<PixelMono>;
    img_out_seg->resize(msize.width, msize.height);

    img_out_dog = new ImageOf<PixelMono>;
    img_out_dog->resize(msize.width, msize.height);

    cmp_res = 0.0;
    area = 0;
    allocated = true;
}


void ZDFThread::get_ndt(Coord c,Ipp8u * im, int w, int*list)
{

    Coord n;

    int ndt_ind = 0;
    n = c+Coord(1,0);     
    if (abs(im[n.y*w + n.x]-im[c.y*w + c.x])<=NDTEQ)
        list[ndt_ind]= 0;
    else if (im[n.y*w + n.x]-im[c.y*w + c.x]>NDTEQ)
        list[ndt_ind]= 1;
    else
        list[ndt_ind]= -1;  

    ndt_ind++;
    n = c+Coord(0,1);
    if (abs(im[n.y*w + n.x]-im[c.y*w + c.x])<=NDTEQ)
        list[ndt_ind]= 0;
    else if (im[n.y*w + n.x]-im[c.y*w + c.x]>NDTEQ)
        list[ndt_ind]= 1;
    else
        list[ndt_ind]= -1;

    ndt_ind++;
    n = c+Coord(-1,0);
    if (abs(im[n.y*w + n.x]-im[c.y*w + c.x])<=NDTEQ)
        list[ndt_ind]= 0;
    else if (im[n.y*w + n.x]-im[c.y*w + c.x]>NDTEQ)
        list[ndt_ind]= 1;
    else
        list[ndt_ind]= -1;

    ndt_ind++;
    n = c+Coord(0,-1);
    if (abs(im[n.y*w + n.x]-im[c.y*w + c.x])<=NDTEQ)
        list[ndt_ind]= 0;
    else if (im[n.y*w + n.x]-im[c.y*w + c.x]>NDTEQ)
        list[ndt_ind]= 1;
    else
        list[ndt_ind]= -1;


    if (NDTSIZE>4){

    //diagonals:
    ndt_ind++;
    n = c+Coord(1,1);     
    if (abs(im[n.y*w + n.x]-im[c.y*w + c.x])<=NDTEQ)
        list[ndt_ind]= 0;
    else if (im[n.y*w + n.x]-im[c.y*w + c.x]>NDTEQ)
        list[ndt_ind]= 1;
    else
        list[ndt_ind]= -1;  

    ndt_ind++;
    n = c+Coord(1,-1);
    if (abs(im[n.y*w + n.x]-im[c.y*w + c.x])<=NDTEQ)
        list[ndt_ind]= 0;
    else if (im[n.y*w + n.x]-im[c.y*w + c.x]>NDTEQ)
        list[ndt_ind]= 1;
    else
        list[ndt_ind]= -1;

    ndt_ind++;
    n = c+Coord(-1,1);
    if (abs(im[n.y*w + n.x]-im[c.y*w + c.x])<=NDTEQ)
        list[ndt_ind]= 0;
    else if (im[n.y*w + n.x]-im[c.y*w + c.x]>NDTEQ)
        list[ndt_ind]= 1;
    else
        list[ndt_ind]= -1;

    ndt_ind++;
    n = c+Coord(-1,-1);
    if (abs(im[n.y*w + n.x]-im[c.y*w + c.x])<=NDTEQ)
        list[ndt_ind]= 0;
    else if (im[n.y*w + n.x]-im[c.y*w + c.x]>NDTEQ)
        list[ndt_ind]= 1;
    else
        list[ndt_ind]= -1;
    }
}

double ZDFThread::cmp_ndt(int*ndt_l, int*ndt_r)
{

    int s=0;

    for (int count=0;count<NDTSIZE;count++){
        if(ndt_l[count]==ndt_r[count]){
            s++;
        }
    }
    return ((double)s)/((double)NDTSIZE);

}


void ZDFThread::get_rank(Coord c,Ipp8u *im, int w, int*list)
{
    Coord n;
    int i = 0;

    for (int x=-RANKX;x<=RANKX;x++){
        for (int y=-RANKY;y<=RANKY;y++){

        n = c+Coord(x,y);
        list[i] = im[n.y*w + n.x];
        i++;

        }
    }
}

double ZDFThread::cmp_rank(int*l1, int*l2)
{ 
    int n1 = 0; //number of non-ties for x
    int n2 = 0; //number of non-ties for y
    int is = 0;

    int a1,a2,aa;

    double tau;//,svar,z,prob;

    for(int j=0;j<RANKSIZE;j++) {
        for(int k=j+1;k<RANKSIZE;k++) {
            a1 = l1[j] - l1[k];
            a2 = l2[j] - l2[k];
            aa = a1*a2;
            if(aa) {
                ++n1;
                ++n2;

                aa > 0 ? ++is : --is;

            } else {
            if(a1) ++n1;
            if(a2) ++n2;
            }
        }
    }

    tau = (is) / (sqrt((float)n1) * sqrt((float)n2));
    // svar = (4.0 * n + 10.0) / (9.0 * n * (n - 1.0));
    // z = tau / sqrt(svar);
    // prob = erfcc(abs(z) / 1.4142136);

    if (tau < 0.0){tau=0.0;}

    return tau;
}



void ZDFThread::getAreaCoGSpread(Ipp8u*im_,int psb_,IppiSize sz_,int*parea,double*pdx,double*pdy,double*spread){

    double naccum = 0.0, xaccum = 0.0, yaccum = 0.0;
    *spread = 0.0;

    //cout << "GET AREA " << sz_.height << " " << sz_.width << endl;

    for (int j=0;j<sz_.height;j++){
        for (int i=0;i<sz_.width;i++){
            if (im_[j*psb_+i]!=0){
            //cout << "GET AREA DOUBLE LOOP" <<endl;
                xaccum+=(double)i; 
                yaccum+=(double)j; 
                naccum+=1.0;
        }
    }
}

    *parea = (int)naccum;
    //cout << "naccum " << naccum << endl;
    if (naccum > 0.0){
    *pdx = -(sz_.width/2.0 - xaccum/naccum - 1.0);
    *pdy = -(sz_.height/2.0 - yaccum/naccum - 1.0);  

        //get spread:
        for (int j=0;j<sz_.height;j++){
            for (int i=0;i<sz_.width;i++){
                if (im_[j*psb_+i]!=0){  
                    *spread += sqrt( fabs((i-sz_.width/2.0 - 1.0) - (*pdx)) * fabs((i-sz_.width/2.0 - 1.0) - (*pdx)) 
                    + fabs((j-sz_.height/2.0 - 1.0) -(*pdy)) * fabs((j-sz_.height/2.0 - 1.0) -(*pdy)) );
                }
            }
        }

    *spread/=naccum;
    }
    else {
        *pdx = 0.0;
        *pdy = 0.0;
        *spread = 0.0;
    }
}  
