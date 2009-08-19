/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: David Vernon
 * email:   david@vernon.eu
 * website: www.robotcub.org & www.vernon.eu
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
 
/*
Audit Trail
-----------

18/07/07  Started work on the development of a YARP version of this module   DV
30/07/09  Migrated to the RFModule class to allow use of the resource finder DV
17/08/09  Amended to comply with iCub Software Development Guidelines        DV
*/ 
 
// crossPowerSpectrum includes
// ------------------------------------------------------------------------

#include "iCub/crossPowerSpectrumVergence.h"

 
CrossPowerSpectrumVergence::CrossPowerSpectrumVergence()
//--------------------------
{
    // intialize private class variables

    eyecub_image   *image1  = NULL;
    eyecub_image   *image2  = NULL;
    eyecub_image   *image_a = NULL;
    eyecub_image   *image_b = NULL;
    eyecub_image   *image_c = NULL;
    eyecub_image   *image_d = NULL;

    threshold                       = 10;    // % of maximum value
    filter_radius                   = 2;     // pixels
    number_of_maxima                = 2;     // cardinal number
    non_maxima_suppression_radius   = 10;    // pixels
    std_dev                         = 20;    // % of image width

    width = 0;
    height = 0;
    depth  = 0;
    image_size  = 0;

    debug = FALSE;
}

CrossPowerSpectrumVergence::~CrossPowerSpectrumVergence(){}
//--------------------------

 
double  CrossPowerSpectrumVergence::getPeriod()
//----------------
{
    return 0.1; //module periodicity (seconds)
}
 
 
// Message handler. 
// This allows other modules or a user to send commands to the module (in bottles)
// This functionality is not yet used in crossPowerSpectrumVergence but it may come in useful later on
// if/when we wish to change the parameters of the module at run time
// For now, just echo all received messages.

bool  CrossPowerSpectrumVergence::respond(const Bottle& command, Bottle& reply) 
{
    printf("Got something, echo is on\n");
    if (command.get(0).asString()=="quit")
        return false;     
    else
        // do something and then reply
        reply=command;
    return true;
}
  
// Configure function. This handles all the module initialization
// It takes as a parameter a previously initialized resource finder object. 
// This object is used to configure the module.

bool  CrossPowerSpectrumVergence::configure(yarp::os::ResourceFinder &rf)
{
    // Open a port and attach it to message handler or terminal.
    // This allows other modules or a user to send commands to the module (in bottles)
    // This functionality is not yet used in crossPowerSpectrumVergence but it may come in useful later on
    // if/when we wish to change the parameters of the module at run time
   
    handlerPort.open(getName());   // use getName() rather than a literal as the configuration might have changed the name
    attach(handlerPort);           // attach to port
    attachTerminal();              // attach to terminal
 
    // Process crossPowerSpectrumVergence module arguments
 
    threshold                          = rf.check("threshold", 
                                                  10, 
                                                  "Threshold for detection of maxima: integer % of global maximum").asInt();
        
    filter_radius                      = rf.check("filter_radius",  
                                                  2, 
                                                  "Radius of filter used to amplify local maxima: pixels").asInt();
        
    number_of_maxima                   = rf.check("number_of_maxima", 
                                                  2, 
                                                  "Number of local maxima (i.e. image regions a given disparity or depth) to consider in the final selection").asInt();
     
    non_maxima_suppression_radius      = rf.check("non_maxima_suppression_radius", 
                                                  10, 
                                                  "Radius in pixels of the non-maxima suppression filter").asInt();

    std_dev                            = rf.check("std_dev", 
                                                  20, 
                                                  "Standard deviation of Gaussian used for centre weighting: % of image width").asInt();

    leftCameraPortName                 = getName(
                                         rf.check("left_camera", 
                                         Value("left_camera:i"), 
                                         "left camera input (string)").asString()
                                         );

    rightCameraPortName                = getName(
                                         rf.check("right_camera",
                                         Value("right_camera:i"), 
                                         "left camera input (string)").asString()
                                         );

    leftImagePortName                  = getName(
                                         rf.check("left_output", 
                                         Value("left_image:o"), 
                                         "left image output (string)").asString()
                                         );

    rightImagePortName                 = getName(
                                         rf.check("right_output", 
                                         Value("right_image:o"), 
                                         "right image output (string)").asString()
                                         );

    crossPowerSpectrumPortName         = getName(
                                         rf.check("cross-power_spectrum", 
                                         Value("cross-power_spectrum:o"), 
                                         "cross-power spectrum output (string)").asString()
                                         );

    filteredCrossPowerSpectrumPortName = getName(
                                         rf.check("filtered_cross-power_spectrum", 
                                         Value("filtered_cross-power_spectrum:o"), 
                                         "filtered cross-power spectrum output (string)").asString()
                                         );

    vergenceDisparityPortName          = getName(
                                         rf.check("vergence_disparity", 
                                         Value("vergence_disparity:o"), 
                                         "vergence disparity output (string)").asString()
                                         );
 
    if (debug) {
       printf("crossPowerSpectrumVergence: parameter values are:\n%d\n%d\n%d\n%d\n%d\n\n",threshold,filter_radius,number_of_maxima,non_maxima_suppression_radius,std_dev);
       printf("crossPowerSpectrumVergence: port names are:\n%s\n%s\n%s\n%s\n%s\n\n",leftCameraPortName.c_str(),
                                                                                    rightCameraPortName.c_str(),
                                                                                    leftImagePortName.c_str(),
                                                                                    rightImagePortName.c_str(),
                                                                                    crossPowerSpectrumPortName.c_str(),
                                                                                    filteredCrossPowerSpectrumPortName.c_str(),
                                                                                    vergenceDisparityPortName.c_str() );
    }
  
    // do all initialization here
        
    Network::init();

    portIn1.open(leftCameraPortName);
    portIn2.open(rightCameraPortName);

    portOut1.open(leftImagePortName);
    portOut2.open(rightImagePortName);
    portOut3.open(crossPowerSpectrumPortName );
    portOut4.open(filteredCrossPowerSpectrumPortName);
    portOut5.open(vergenceDisparityPortName);

    if (debug) printf("crossPowerSpectrum running ... \n");
  
    // grab an image to set the image size

    if (debug) printf("getting image size\n");

    do {
       imgIn1 = portIn1.read(true);
    } while (imgIn1 == NULL);

    width  = imgIn1->width();
    height = imgIn1->height();
    depth = 3;
 
    if (debug) printf("width = %d, height = %d, depth = %d\n",width, height, depth);

    // create the input images of the correct resolution

    image1 = new eyecub_image(width, height, depth);
    image2 = new eyecub_image(width, height, depth);

    if (width == 1024 && height == 768) {
        image_size = 256; // 512
    }
    else if (width == 640 && height == 480) {
        image_size = 256;  
    }
    else {  // width == 320 && height == 240
        image_size = 256; // need to pad
    }
 
    if (debug) printf("image_size = %d\n",image_size);

    // set up standard deviation for apodization (centre weighting)
   
    sigma = (float) image_size * (float) std_dev / 100; 

    image_a = new eyecub_image(image_size, image_size, GREYSCALE_IMAGE, NULL, NULL, EYECUB_INT);
    image_b = new eyecub_image(image_size, image_size, GREYSCALE_IMAGE, NULL, NULL, EYECUB_INT);
    image_c = new eyecub_image(image_size, image_size, GREYSCALE_IMAGE, NULL, NULL, EYECUB_FLOAT);
    image_d = new eyecub_image(image_size, image_size, GREYSCALE_IMAGE, NULL, NULL, EYECUB_FLOAT);

    return true;
}


bool  CrossPowerSpectrumVergence::close()
//------------------
{
    // close ports

    portIn1.close();
    portIn2.close();
    portOut1.close();
    portOut2.close();
    portOut3.close();
    portOut4.close();
    portOut5.close();

    handlerPort.close();

    // delete dynamically created images
   
    if (image1  != NULL) delete image1;
    if (image2  != NULL) delete image2;
    if (image_a != NULL) delete image_a;
    if (image_b != NULL) delete image_b;
    if (image_c != NULL) delete image_c;
    if (image_d != NULL) delete image_d;

    Network::fini();

    return true;
}
 

bool  CrossPowerSpectrumVergence::interruptModule()
//----------------------------
{
    // interrupt ports gracefully
    portIn1.interrupt();
    portIn2.interrupt();

    portOut1.interrupt();
    portOut2.interrupt();
    portOut3.interrupt();
    portOut4.interrupt();
    portOut5.interrupt();

    return true;
}

bool CrossPowerSpectrumVergence::updateModule()
//-------------------------
{
    // crossPowerSpectrumVergence module main process; repeatedly called by the Module 'event loop'

    // grab images ---------------------------------------------------	

    do {
        imgIn1 = portIn1.read(true);
    } while (imgIn1 == NULL);

    do {
       imgIn2 = portIn2.read(true);
    } while (imgIn2 == NULL);


    // copy to local images ... 
    // NB this can be combined with the extraction of square image later

    for (x=0; x<width; x++) {
        for (y=0; y<height; y++) {
            rgb_pixel = imgIn1->safePixel(x,y);
            image1->put_pixel(x, y, rgb_pixel.r, 0);
            image1->put_pixel(x, y, rgb_pixel.g, 1);
            image1->put_pixel(x, y, rgb_pixel.b, 2);
        }
    } 

    for (x=0; x<width; x++) {
        for (y=0; y<height; y++) {
            rgb_pixel = imgIn2->safePixel(x,y);
            image2->put_pixel(x, y, rgb_pixel.r, 0);
            image2->put_pixel(x, y, rgb_pixel.g, 1);
            image2->put_pixel(x, y, rgb_pixel.b, 2);
        }
    }
 
    // now compute the cross-power spectrum
    // ------------------------------------

    // step 1: extract an image of size 2^n x 2^n, converting from RGB to grey-scale if necessary
    // creating two input (visual) and one output (cross-power specturm) images, all square in size, and greyscale
 		

    // copy images ---------------------------------------------------	
	
    image_a->initialize();
    image_b->initialize();
 

    // THIS LOOP NEEDS TO BE OPTIMIZED 

    for (i = 0; i<image_size; i++) {
        for (j = 0; j<image_size; j++) {

            p = ((width - image_size) /  2 + width + i)%width;  // pad by wrap-around if necessary !! pad by reflection would be better !!
            q = ((height - image_size) / 2 + height + j)%height;// ibid.

            if ((p>=0) && (p<width) && (q>=0) && (q<height)) { // defensive ... always be the case

                // first image

                image1->get_pixel(p,q,&pixel_value, 0);
                temp = (int) pixel_value;
                image1->get_pixel(p,q,&pixel_value, 1);
                temp += (int) pixel_value;
                image1->get_pixel(p,q,&pixel_value, 2);
                temp += (int) pixel_value;
                temp = temp / 3;
                pixel_value = (unsigned char) temp;	
 
                image_a->put_pixel(i,j,pixel_value);

                // second image
				
                image2->get_pixel(p,q,&pixel_value, 0);
                temp = (int) pixel_value;
                image2->get_pixel(p,q,&pixel_value, 1);
                temp += (int) pixel_value;
                image2->get_pixel(p,q,&pixel_value, 2);
                temp += (int) pixel_value;
                temp = temp / 3;
                pixel_value = (unsigned char) temp;	
               
                image_b->put_pixel(i,j,pixel_value);
            }
        }
    }
 		 
    // step 2: apodize the image by multiplying by a Gaussian 
    // =============================================

    gaussianApodization (image_a, sigma, image_a);
    gaussianApodization (image_b, sigma, image_b);  


    // step 3: compute the cross_power_spectrum 
    // =============================================
		   
    cross_power_spectrum (image_b, image_a, image_c); // image_c must exist, type FLOAT
			 	   

    // step 4: filter the cross_power_spectrum to enhance local maxima
    // ===============================================================
    
    enhance_local_maxima (image_c, filter_radius, image_d); 	 


    // step 5: locate the local maxima
    // ===============================================================

    find_maxima (image_d, number_of_maxima, non_maxima_suppression_radius, maxima);  
 
    // display -------------------------------------------------------------	  

    // the CPS images are float so we need to contrast stretch them before displaying ... a pity really as it takes time!

    image_c->contrast_stretch();
    image_d->contrast_stretch();


    ImageOf<PixelRgb> &imgOut1 = portOut1.prepare();
    ImageOf<PixelRgb> &imgOut2 = portOut2.prepare();
    ImageOf<PixelRgb> &imgOut3 = portOut3.prepare();
    ImageOf<PixelRgb> &imgOut4 = portOut4.prepare();
 
    imgOut1.resize(image_size,image_size);
    imgOut2.resize(image_size,image_size);
    imgOut3.resize(image_size,image_size);
    imgOut4.resize(image_size,image_size);
 
    // copy image data

    for (x=0; x<image_size; x++) {
        for (y=0; y<image_size; y++) {
            image_a->get_pixel(x, y, &pixel_value, 0); rgb_pixel.r=pixel_value;  // square left image
            image_a->get_pixel(x, y, &pixel_value, 1); rgb_pixel.g=pixel_value;
            image_a->get_pixel(x, y, &pixel_value, 2); rgb_pixel.b=pixel_value;
            imgOut1(x,y) = rgb_pixel;

            image_b->get_pixel(x, y, &pixel_value, 0); rgb_pixel.r=pixel_value; // square right image
            image_b->get_pixel(x, y, &pixel_value, 1); rgb_pixel.g=pixel_value;
            image_b->get_pixel(x, y, &pixel_value, 2); rgb_pixel.b=pixel_value;
            imgOut2(x,y) = rgb_pixel;

            image_c->get_pixel(x, y, &float_pixel_value, 0); rgb_pixel.r=(unsigned char) float_pixel_value; // cross-power spectrum
            image_c->get_pixel(x, y, &float_pixel_value, 1); rgb_pixel.g=(unsigned char) float_pixel_value;
            image_c->get_pixel(x, y, &float_pixel_value, 2); rgb_pixel.b=(unsigned char) float_pixel_value;
            imgOut3(x,y) = rgb_pixel;

            image_d->get_pixel(x, y, &float_pixel_value, 0); rgb_pixel.r=(unsigned char) float_pixel_value; // cross-power spectrum filtered
            image_d->get_pixel(x, y, &float_pixel_value, 1); rgb_pixel.g=(unsigned char) float_pixel_value;
            image_d->get_pixel(x, y, &float_pixel_value, 2); rgb_pixel.b=(unsigned char) float_pixel_value;
            imgOut4(x,y) = rgb_pixel;
        }
    } 

    // draw cross-hairs

    rgb_pixel.r=(unsigned char) 255;
    rgb_pixel.g=(unsigned char) 255;
    rgb_pixel.b=(unsigned char) 0;

    for (i=0; i<number_of_maxima; i++) {
        if (maxima[i].value  > (maxima[0].value * ((float)threshold / 100.0))) {
           addCrossHair(imgOut4, rgb_pixel, maxima[i].x, maxima[i].y, (int) 10);
        }     
    }

    // write out images

    portOut1.write();
    portOut2.write();
    portOut3.write();
    portOut4.write();

    if (debug) {
        printf("Maxima: \n");
        for (i=0; i<number_of_maxima; i++) {
           printf("%8.0f at (%d, %d)\n", maxima[i].value, maxima[i].x, maxima[i].y);
        }
    }  


    // servo -------------------------------------------------------------	  

    /* send the disparity between the two images to the controlGaze module
       this is equivalent to the offset of the detected maximum from the centre of the CPS image

       we need a strategy to choose one of the maxima to control the fixation/vergence
          
       the possibilities are: 
          
       - choose the largest maximum (number 0); 
         this is probably going to correspond to the 
         object that occupies the largest amount of the field of view (or largest energy in the image)
          
       - choose the maximum that is closest to the centre;
         the corresponds to the object that is closest to the current fixation distance
          
       - choose the maximum that is furthest to the LEFT of the cross-power spectrum; 
         this corresponds to the object that is closest to the cameras
          
      We use option 3 at the moment.
    
    */
     
    if (maxima[1].value  > (maxima[0].value * ((float)threshold / 100.0))) {
        i = min(maxima[0].x, maxima[1].x);
        j = min(maxima[0].y, maxima[1].y);
  
    }
    else {
        i = maxima[0].x;
        j = maxima[0].y;
    }

    // normalize the disparity (controlGaze uses normalized image coordinates -1 <= x, y <= +1

    controlGaze_x = (float)(2*i)/(float)image_size - 1 ;
    controlGaze_y = (float)(2*j)/(float)image_size - 1 ;

    Vector& vec = portOut5.prepare();
  
    vec.resize(3,0);

    vec[0] = controlGaze_x;  
    vec[1] = controlGaze_y;
    vec[2] = 0;              // flag to indicate the type of image

    portOut5.write();

    if (debug) printf("controlGaze_x = %f, controlGaze_y = %f\n",controlGaze_x, controlGaze_y);

    return true;
}

