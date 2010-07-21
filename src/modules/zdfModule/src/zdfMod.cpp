/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Vadim Tikhanoff
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

#include "iCub/zdfMod.h"
#include <math.h>

//OPENCV
#include <cv.h>
#include <highgui.h>

bool zdfMod::configure(yarp::os::ResourceFinder &rf)
{    
    /* Process all parameters from both command-line and .ini file */

    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name", 
                           Value("zdfMod"), 
                           "module name (string)").asString();

   /*
    * before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
   
    setName(moduleName.c_str());

    /* get the name of the input and output ports, automatically prefixing the module name by using getName() */
 
    inputNameLeft      = "/";
    inputNameLeft      += getName(
                       rf.check("inPortLeft", 
                       Value("/imageLeft:i"),
                       "Input image port (string)").asString()
                       );

	inputNameRight     = "/";
    inputNameRight     += getName(
                       rf.check("inPortRight", 
                       Value("/imageRight:i"),
                       "Input image port (string)").asString()
                       );
   
    outputNameProb     = "/";
    outputNameProb     += getName(
                       rf.check("outPortProb", 
                       Value("/imageProb:o"),
                       "Output image port (string)").asString()
                       );

	outputNameSeg     = "/";
    outputNameSeg     += getName(
                      rf.check("outPortSeg", 
                      Value("/imageSeg:o"),
                      "Output image port (string)").asString()
                       );
	
	outputNameDog     = "/";
    outputNameDog     += getName(
                      rf.check("outPortDog", 
                      Value("/imageDog:o"),
                      "Output image port (string)").asString()
                       );


    /* do all initialization here */
    /* open ports  */ 
       
    if (!inputLeft.open(inputNameLeft.c_str())) {
        cout << getName() << ": unable to open port " << inputNameLeft << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
	if (!inputRight.open(inputNameRight.c_str())) {
        cout << getName() << ": unable to open port " << inputNameRight << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!outputProb.open(outputNameProb.c_str())) {
        cout << getName() << ": unable to open port " << outputNameProb << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
	if (!outputSeg.open(outputNameSeg.c_str())) {
        cout << getName() << ": unable to open port " << outputNameSeg << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
	if (!outputDog.open(outputNameDog.c_str())) {
        cout << getName() << ": unable to open port " << outputNameDog << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

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

    attach(handlerPort);                  // attach to port
    //attachTerminal();                     // attach to terminal (maybe not such a good thing...)

    /* create the thread and pass pointers to the module parameters */

    zdfThread = new ZDFThread( &inputLeft, &inputRight, &outputProb, &outputSeg, &outputDog );

    /* now start the thread to do the work */
    zdfThread->start(); // this calls threadInit() and it if returns true, it then calls run()

    return true ;      // let the RFModule know everything went well
}


bool zdfMod::interruptModule()
{
    inputLeft.interrupt();
    inputRight.interrupt();
	outputProb.interrupt();
    outputSeg.interrupt();
    handlerPort.interrupt();
    zdfThread->stop();
    return true;
}

bool zdfMod::close()
{
    inputLeft.close();    
    inputRight.close();
	outputProb.close();    
    outputSeg.close();
	handlerPort.interrupt();
    zdfThread->stop();
    return true;
}

bool zdfMod::respond(const Bottle& command, Bottle& reply) 
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

bool zdfMod::updateModule()
{
    return true;
}

double zdfMod::getPeriod()
{
    /* module periodicity (seconds), called implicitly by myModule */
    
    return 0.1;
}

ZDFThread::ZDFThread(BufferedPort<ImageOf<PixelBgr> > *inputLeft, BufferedPort<ImageOf<PixelBgr> > *inputRight, BufferedPort<ImageOf<PixelMono> > *outputProb, BufferedPort<ImageOf<PixelMono> > *outputSeg, BufferedPort<ImageOf<PixelMono> > *outputDog )
{
    imageInLeft   = inputLeft;
    imageInRight  = inputRight;
	imageOutProb  = outputProb;
    imageOutSeg   = outputSeg;
	imageOutDog   = outputDog;
}

bool ZDFThread::threadInit() 
{
    /* initialize variables and create data-structures if needed */
    init = true;
    return true;
}

void ZDFThread::run(){

    while (isStopping() != true) { // the thread continues to run until isStopping() returns true
        
		leftPort =  ( imageInLeft->getInputCount() > 0 );
    	rightPort = ( imageInRight->getInputCount() > 0 );
        
		if ( leftPort + rightPort > 1){
			do {
        		img_in_left = imageInLeft->read(false);
      		} while (img_in_left == NULL );
		
			do {
        		img_in_right = imageInRight->read(false);
      		} while (img_in_right == NULL);

			if (init){//Get first RGB image to establish width, height:
            	cout << "initializing" << endl;   
            	initAll();     
        	}

		cvShowImage("Settings", NULL);  
    	cvWaitKey(10);

		//processing for zdf
		//resize the images
		if (scale==1.0){
			//copy yarp image to IPP
			ippiCopy_8u_C3R( img_in_left->getRawImage(),  img_in_left->getRowSize(), l_orig, psb, srcsize);
			ippiCopy_8u_C3R( img_in_right->getRawImage(), img_in_right->getRowSize(), r_orig, psb, srcsize);
      	}else{
			//scale to width,height:
			ippiResize_8u_C3R( img_in_left->getRawImage(), insize, img_in_left->width() * 3, inroi, l_orig, psb, srcsize, scale, scale, IPPI_INTER_CUBIC);
			ippiResize_8u_C3R( img_in_right->getRawImage(), insize, img_in_right->width() * 3, inroi, r_orig, psb, srcsize, scale, scale, IPPI_INTER_CUBIC);
		}
		//copy to grayscale
		ippiRGBToGray_8u_C3C1R( l_orig, psb , rec_im_ly, psb_in, srcsize );
		ippiRGBToGray_8u_C3C1R( r_orig, psb , rec_im_ry, psb_in, srcsize );

		if (acquire){
			ippiCopy_8u_C1R(&rec_im_ly [(( srcsize.height - tsize.height ) / 2 ) * psb_in + ( srcsize.width - tsize.width ) /2 ], psb_in, temp_l, psb_t, tsize);			
			ippiCopy_8u_C1R(&rec_im_ry [(( srcsize.height - tsize.height ) / 2 ) * psb_in + ( srcsize.width - tsize.width ) /2 ], psb_in, temp_r, psb_t, tsize);
		}

		//MAKE LEFT FOVEA
		//find left template in left image:
		ippiCrossCorrValid_NormLevel_8u32f_C1R(&rec_im_ly[(( srcsize.height - tisize.height )/2)*psb_in + ( srcsize.width - tisize.width )/2],
				       psb_in, tisize,
				       temp_l,
				       psb_t, tsize,
				       res_t, psb_rest);

		ippiMaxIndx_32f_C1R( res_t, psb_rest, trsize, &max_t, &sx, &sy); 

		//cout << "X Y " <<  sx << " " << sy << endl;

		if (max_t < 0.60){  //shift_sim_t
	  		//this also prevents tracking motion:
	  		tl_x = 0;
	  		tl_y = 0;
		}
		else{ 
	  	//this initiates tracking motion if non-zero:
	  		tl_x = sx - trsize.width /2;
	  		tl_y = sy - trsize.height /2;
		}
		//ippiCopy_8u_C1R( rec_im_ly, psb_in, fov_l, psb_m, srcsize);
		ippiCopy_8u_C1R( &rec_im_ly [ ( mid_y + tl_y ) * psb_in + mid_x + tl_x], psb_in, fov_l, psb_m, msize ); //original

		//**************************
		//MAKE RIGHT FOVEA
		//find right template in right image:
		ippiCrossCorrValid_NormLevel_8u32f_C1R(&rec_im_ry[((srcsize.height-tisize.height)/2 + dpix_y )*psb_in + (srcsize.width-tisize.width)/2],
					psb_in,tisize,
					temp_r,
					psb_t,tsize,
					res_t, psb_rest);
		ippiMaxIndx_32f_C1R(res_t,psb_rest,trsize,&max_t,&sx,&sy);
		if (max_t < 0.50){ //shift_sim_t
		//this also prevents tracking motion:
			tr_x = 0;
			tr_y = 0;
		}
		else{ 
			//this initiates tracking motion if non-zero:
			tr_x = sx - trsize.width/2;
			tr_y = sy - trsize.height/2;
		}
		//ippiCopy_8u_C1R( rec_im_ry, psb_in, fov_r, psb_m, srcsize);
		ippiCopy_8u_C1R(&rec_im_ry[(mid_y+tr_y+dpix_y)*psb_in + mid_x+tr_x],psb_in,fov_r,psb_m,msize); // original


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
	    		if (dl->get_dog_onoff()[i + j*dl->get_psb()] >= bland_dog_thresh || dr->get_dog_onoff()[i + j*dr->get_psb()] >= bland_dog_thresh ){      
	      			if (RANK0_NDT1==0){
						//use RANK:
						get_rank(c,fov_l,psb_m,rank1);
						get_rank(c,fov_r,psb_m,rank2);
						//get_rank(c,dl->get_dog_onoff(),dl->get_psb(),rank1);
						//get_rank(c,dr->get_dog_onoff(),dr->get_psb(),rank2);
						cmp_res = cmp_rank(rank1,rank2);
	      			}
	    			else{ 
						//use NDT:
						get_ndt(c,fov_l,psb_m,ndt1);
						get_ndt(c,fov_r,psb_m,ndt2);
						//get_ndt(c,dl->get_dog_onoff(),dl->get_psb(),ndt1);
						//get_ndt(c,dr->get_dog_onoff(),dr->get_psb(),ndt2);
						cmp_res = cmp_ndt( ndt1, ndt2 );
	      			}
	      			zd_prob_8u[ j * psb_m + i] = (int)(cmp_res * 255.0);
	    		}
	    		else{
	      				//untextured in both l & r, so set to bland prob (ZD):
	      			zd_prob_8u[j*psb_m+i] = (int)(bland_prob * 255.0);//bland_prob
	    		} 

	    		//RADIAL PENALTY:
	    		//The further from the origin, less likely it's ZD, so reduce zd_prob radially:
	    		//current radius:
	    		r = sqrt((c.x-msize.width/2.0)*(c.x-msize.width/2.0)+(c.y-msize.height/2.0)*(c.y-msize.height/2.0));
	    		rad_pen =  (int) ( (r/rmax)* radial_penalty );//radial_penalty
	    		max_rad_pen = zd_prob_8u[j*psb_m+i];
	    		if(max_rad_pen < rad_pen) {
	      			rad_pen=max_rad_pen;
	    		}
	    		//apply radial penalty
	    		zd_prob_8u[j*psb_m+i]-= rad_pen;
	    
	    		//OTHER:
	    		//manufacture NZD prob (other):
	    		o_prob_8u[psb_m*j+i] = 255 - zd_prob_8u[psb_m*j+i];
	  		}
		}

		//DO MRF OPTIMISATION!!:
		m->proc( fov_r, p_prob ); //provide edge map and probability map PROBLEMMMMMMM!
		//cache for distribution:
		ippiCopy_8u_C1R( m->get_class(), m->get_psb(), out, psb_m, msize);
		
		//evaluate result:
		getAreaCoGSpread(out, psb_m , msize, &area, &cog_x, &cog_y, &spread); 	
	
		cog_x_send = cog_x;
		cog_y_send = cog_y;

		//cout << area << " " << cog_x << " " << cog_y << " " << spread << endl;
		
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
/**/
			
		//If nice segmentation:
		if (area >= min_area && area <= max_area && spread<= max_spread){ 
  			//don't update templates to image centre any more as we have a nice target
   			acquire = false;
  			//update templates to(wards) segmentation CoG:
  			printf("area:%d spread:%f cogx:%f cogy:%f - UPDATING TEMPLATE\n",area,spread,cog_x,cog_y);
  			//Bring cog of target towards centre of fovea:
  			//SNAP GAZE TO OBJECT:
  			cog_x*= cog_snap;
  			cog_y*= cog_snap;
  			ippiCopy_8u_C1R(&fov_l[( mid_x_m + ( (int) round ( cog_x ) ) ) + ( mid_y_m + ( ( int ) round ( cog_y) ) ) * psb_m], psb_m, temp_l, psb_t, tsize );
  			ippiCopy_8u_C1R(&fov_r[( mid_x_m + ( (int) round ( cog_x ) ) ) + ( mid_y_m + ( ( int ) round ( cog_y) ) ) * psb_m], psb_m, temp_r, psb_t, tsize );
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
  			printf("Acquiring new target until nice seg (waiting:%d >= acquire_wait:%d)\n",waiting, acquire_wait );//acquire_wait
  			acquire = true;
		}

//--------------------------------------------------------------------------------------------------------------------------------------------------------------
		//ippiThreshold_LTVal_8u_C1IR( seg_im, psb_m, msize, (Ipp8u)1, (Ipp8u)4);			
		//ippiSubC_8u_C1IRSfs(1,seg_im,psb_m,msize,0);
		//ippiMulC_8u_C1IRSfs(40,seg_im,psb_m,msize,0);

		//revert to yarp image
		ippiCopy_8u_C1R( zd_prob_8u, psb_m, img_out_prob->getRawImage(), img_out_prob->getRowSize(), msize );
		ippiCopy_8u_C1R( seg_im, psb_m, img_out_seg->getRawImage(), img_out_seg->getRowSize(), msize );
		ippiCopy_8u_C1R( seg_dog, psb_m, img_out_dog->getRawImage(), img_out_dog->getRowSize(), msize );

		//send it all
		if (imageOutProb->getOutputCount()>0){ 
			imageOutProb->prepare() = *img_out_prob;	
           	imageOutProb->write();
        }

		if (imageOutSeg->getOutputCount()>0){
           	imageOutSeg->prepare() = *img_out_seg;	
           	imageOutSeg->write();
        }
		if (imageOutDog->getOutputCount()>0){
           	imageOutDog->prepare() = *img_out_dog;	
           	imageOutDog->write();
        }

		}
	}
}

void ZDFThread::threadRelease() 
{
    /* for example, delete dynamically created data-structures */
}

void ZDFThread::initAll()
{

	//struct MultiClass::Parameters params;

	params.iter_max                  = 1; // INI!!!
	params.randomize_every_iteration = 0;
   	params.smoothness_penalty_base   = 50;
    params.smoothness_penalty        = 600;
    params.data_penalty              = 106;
    params.smoothness_3sigmaon2      = 13;
	
	bland_dog_thresh = 1;
	radial_penalty = 50;
	acquire_wait = 25;
	min_area  = 100;
	max_area  = 2800;
	max_spread  = 25;
	cog_snap = 0.3;
	bland_prob = 0.3;
	
	cvNamedWindow( "Settings", 0) ; 
  	cvCreateTrackbar( "DATA_PENALTY", "Settings", &params.data_penalty, 255, 0 );
  	cvCreateTrackbar( "SMOOTHNESS_PENALTY_BASE", "Settings", &params.smoothness_penalty_base, 255, 0 );
  	cvCreateTrackbar( "SMOOTHNESS_PENALTY", "Settings", &params.smoothness_penalty, 1000, 0 );
    cvCreateTrackbar( "RADIAL_PENALTY", "Settings", &radial_penalty, 255, 0 );
    cvCreateTrackbar( "SMOOTHNESS_3SIGMAON2", "Settings", &params.smoothness_3sigmaon2, 255, 0 );
    cvCreateTrackbar( "BLAND_DOG_THRESH", "Settings", &bland_dog_thresh, 255, 0 );

    cout << "Received left input image dimensions: " << img_in_left->width() << " " << img_in_left->height() << endl;
    cout << "Received right input image dimensions: " << img_in_right->width() << " " << img_in_right->height() << endl;

	width = 320; //was 320
	height = 240;//128;//was 240
	scale = 1.0;
	scale  = ((double)width)/img_in_left->width();

	insize.width = img_in_left->width();
  	insize.height = img_in_left->height();

	printf("Scaling to image dimensions: (%d,%d). Scale factor %f\n", width, height,scale);

	inroi.x=0;
  	inroi.y=0;
  	inroi.width  =  img_in_left->width();
  	inroi.height = img_in_left->height();

	srcsize.width = width;//img_in_left->width();
  	srcsize.height = height;//img_in_left->height();

	//psb_in = img_in_left->psb; //Need to figure this one out....may just be initialized later

	msize.width  = 128;//should be taken from ini file // was 128
  	msize.height = 128;//should be taken from ini file // was 128
	tsize.width  = 32;//same 
  	tsize.height = 32;//same
 
	t_lock_lr = 32;//same
	t_lock_ud = 32;//same

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

	l_orig      = ippiMalloc_8u_C3( srcsize.width, srcsize.height, &psb);// to separate Y channel
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
	
	m = new MultiClass( msize, psb_m, nclasses, &params );

	tl_x = 0;
	tl_y = 0;
  	tr_x = 0;
	tr_y = 0;
  	waiting = 0;
    track = false;
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
	init = false;
	area = 0;
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
  
  tau = (is) / (sqrt(n1) * sqrt(n2));
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
  
  *parea = naccum;
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
