
 /**
 *
 * @ingroup icub_module
 * \defgroup icub_crossPowerSpectrumVergence crossPowerSpectrumVergence
 *
 * Compute histogram of disparity values and select the local maximum value which corresponds to the regions closest to the cameras to control vergence
 *
 * \section intro_sec Description
 * Determine the relative shift required to register one or more regions 
 * in two input images using the cross-power spectrum.
 *
 * The cross-power spectrum of two images is defined as
 *
 * F(w_x, w_y) G*(w_x, w_y) 
 * ------------------------
 * |F(w_x, w_y) G(w_x, w_y)|
 *
 * where F(w_x, w_y) and G(w_x, w_y) are the Fourier tranforms 
 * of images f(x, y) and g(x, y), and G*(w_x, w_y) is the 
 * complex conjugate of G(w_x, w_y)
 * 
 * The positions of local maxima in the cross-power spectrum, 
 * specifically the offset of a detected maximum from the centre of the CPS image, 
 * indicates the relative image shift required to register regions in the image.
 * This can be used to control the vergence of the two eyes.
 *
 * Typically, there will be several regions in the image with different disparities 
 * and hence different vergence angles, each with its own corresponding maximum,
 * we need a strategy to choose one of the maxima to control the fixation/vergence
 *
 * The possibilities are: 
 *
 * 1 choose the largest maximum (number 0); 
 *   this is probably going to correspond to the object that occupies 
 *   the largest amount of the field of view (or largest energy in the image)
 *
 * 2 choose the maximum that is closest to the centre;
 *   the corresponds to the object that is closest to the current fixation distance
 *
 * 3 choose the maximum that is furthest to the LEFT of the cross-power spectrum; 
 *   this corresponds to the object that is closest to the cameras
 *
 * Option 3 is the only option currently implemented.
 *
 *
 * \section lib_sec Libraries
 * YARP.
 *
 * \section parameters_sec Parameters
 * --context 	                     The path (under $ICUB_DIR/app/) where the crossPowerSpectrumVergence.ini configuration file is located  
 *                                   e.g. --context crossPowerSpectrumVergence/conf sets the path to be $ICUB_ROOT/app/crossPowerSpectrumVergence/conf              
 * --std_dev                         Standard deviation of Gaussian used for centre weighting: % of image width (typical value 20)
 * --number_of_maxima                Number of local maxima (i.e. image regions a given disparity or depth) to consider in the final selection (typical value 2)
 * --threshold                       Threshold for detection of maxima: integer % of global maximum (typical value 20)
 * --filter_radius                   Radius in pixels of filter used to amplify local maxima (typical value 2)
 * --non_maxima_suppression_radius   Radius in pixels of the non-maxima suppression filter (typical value 2)
 *
 * \section portsa_sec Ports Accessed
 * 
 * - /icub/cam/left
 * - /icub/cam/right
 *                      
 * \section portsc_sec Ports Created
 *
 *  Input ports
 *
 *  - /left_camera:i
 *  - /right_camera:i
 *
 * Output ports
 *
 *  - /left_image:o
 *  - /right_image:o
 *  - /cross-power_spectrum:o
 *  - /filtered_cross-power_spectrum:o
 *  - /vergence_disparity:o
 *
 * \section in_files_sec Input Data Files
 * None
 *
 * \section out_data_sec Output Data Files
 * None
 *
 * \section conf_file_sec Configuration Files
 * None
 * 
 * \section tested_os_sec Tested OS
 * Linux and Windows.
 *
 * \section example_sec Example Instantiation of the Module
 * crossPowerSpectrumVergece --file crossPowerSpectrumVergence.ini
 *
 * \author David Vernon
 * 
 * Copyright (C) 2008 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 *This file can be edited at src/crossPowerSpectrumVergence/src/crossPowerSpectrumVergence.cpp.
**/

/*
Audit Trail
-----------

18/07/07  Started work on the development of a YARP version of this module   DV
30/07/09  Migrated to the RFModule class to allow use of the resource finder DV
*/ 


// System includes
// ------------------------------------------------------------------------

#include <assert.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/timeb.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

  
// YARP includes
// ------------------------------------------------------------------------

#include <ace/config.h>
#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;

// Migrate _ftime function and _timeb structure to portable equivalents
// --paulfitz
// ------------------------------------------------------------------------

struct portable_timeb {
  int time;
  int millitm;
};

void portable_ftime(struct portable_timeb *data) {
  if (data!=NULL) {
    double now = Time::now();
    double start = now;
    data->time = (int)now;
    data->millitm = (int)((now-data->time)*1000);
  }
}


// fourierVision includes
// ------------------------------------------------------------------------

#include "iCub/fourierVision.h"



//class CrossPowerSpectrumVergence: public Module
class CrossPowerSpectrumVergence: public RFModule

{
private:
    // class variables
	
    eyecub_image   *image1;
    eyecub_image   *image2;
    eyecub_image   *image_a;
    eyecub_image   *image_b;
    eyecub_image   *image_c;
    eyecub_image   *image_d;

    int width;
    int height;
    int depth;
    int image_size;
    unsigned char pixel_value;
    float float_pixel_value;
    int temp;
    int i, j, p, q;
    int x, y;
    float sigma;
    float controlGaze_x;
    float controlGaze_y;

    PixelRgb rgb_pixel;

    ImageOf<PixelRgb> *imgIn1;
    ImageOf<PixelRgb> *imgIn2;
  
    maxima_data_type maxima[10];

    //struct _timeb tb;       // time buffer
    struct portable_timeb tb; // portable version --paulfitz
    long int s0, s1, s2;
    long int ms0, ms1, ms2;
    int time;
     	
    int debug;

    //  ports for acquiring images

    BufferedPort<ImageOf<PixelRgb> > portIn1; 
    BufferedPort<ImageOf<PixelRgb> > portIn2; 
    BufferedPort<ImageOf<PixelRgb> > portOut1; 
    BufferedPort<ImageOf<PixelRgb> > portOut2;
    BufferedPort<ImageOf<PixelRgb> > portOut3; 
    BufferedPort<ImageOf<PixelRgb> > portOut4; 

    // port for sending servo data to controlGaze

    BufferedPort<Vector> portOut5;

	//a port to handle messages

    Port handlerPort; 


    // module arguments

    int threshold;                          // % of maximum value
    int filter_radius;                      // pixels
    int number_of_maxima;                   // cardinal number
    int non_maxima_suppression_radius;      // pixles
    int std_dev;                            // % of image width

public:

    // class methods

    CrossPowerSpectrumVergence()
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

        time = FALSE;

        debug = FALSE;
    }

    ~CrossPowerSpectrumVergence(){}
    //--------------------------

 
    double getPeriod()
    //----------------
    {
        return 0.1; //module periodicity (seconds)
    }
 
 
    // Message handler. 
    // This allows other modules or a user to send commands to the module (in bottles)
    // This functionality is not yet used in crossPowerSpectrumVergence but it may come in useful later on
    // if/when we wish to change the parameters of the module at run time
    // For now, just echo all received messages.

    bool respond(const Bottle& command, Bottle& reply) 
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
	// Significantly, it takes as a parameter a previously initialized resource finder object. 
	// This object is used to configure the module.

    bool configure(yarp::os::ResourceFinder &rf)
    {
		// Open a port and attach it to message handler or terminal.
		// This allows other modules or a user to send commands to the module (in bottles)
		// This functionality is not yet used in crossPowerSpectrumVergence but it may come in useful later on
		// if/when we wish to change the parameters of the module at run time
    
        handlerPort.open("/crossPowerSpectrumVergence");
        attach(handlerPort);  //attach to port
        attachTerminal();     //attach to terminal
 
        if (rf.check("help")) {
            printf("To change context, call with --context <local_path>, where <local_path> is a sub-path from $ICUB_ROOT/app to the .ini file (e.g. crossPowerSpectrumVergence/conf)\n");
            return false;
        }
 
        // Process crossPowerSpectrumVergence module arguments
 
        threshold                     = rf.check("threshold", 10, "Threshold for detection of maxima: integer % of global maximum").asInt();
        filter_radius                 = rf.check("filter_radius",  2, "Radius of filter used to amplify local maxima: pixels").asInt();
        number_of_maxima              = rf.check("number_of_maxima", 2, "Number of local maxima (i.e. image regions a given disparity or depth) to consider in the final selection").asInt();
        non_maxima_suppression_radius = rf.check("non_maxima_suppression_radius", 10, "Radius in pixels of the non-maxima suppression filter").asInt();
        std_dev                       = rf.check("std_dev", 20, "Standard deviation of Gaussian used for centre weighting: % of image width").asInt();
          
 		if (debug) printf("crossPowerSpectrumVergence: parameter values are %d %d %d %d %d\n",threshold,filter_radius,number_of_maxima,non_maxima_suppression_radius,std_dev);
 
        // do all initialization here
        
        Network::init();
	
        portIn1.open(getName("left_camera:i"));
        portIn2.open(getName("right_camera:i"));

        portOut1.open(getName("left_image:o"));
        portOut2.open(getName("right_image:o"));
        portOut3.open(getName("cross-power_spectrum:o"));
        portOut4.open(getName("filtered_cross-power_spectrum:o"));
        portOut5.open(getName("vergence_disparity:o"));

        if (debug) printf("vergence running ... \n");
  
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


    bool close()
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
 

    bool interruptModule()
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

    bool updateModule()
    //-------------------------
    {
        // crossPowerSpectrumVergence module main process; repeatedly called by the Module 'event loop'

        if (time) {portable_ftime( &tb ); s0=tb.time; ms0=tb.millitm;}  // global time for loop

        // grab images ---------------------------------------------------	

        if (time) {portable_ftime(&tb); s1=tb.time; ms1=tb.millitm;}
 
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
 
        if (time) {portable_ftime(&tb); s2=tb.time; ms2=tb.millitm; printf("grab     %4d\n",(s2*1000+ms2)-(s1*1000+ms1));}
 

        // now compute the cross-power spectrum
        // ------------------------------------

        // step 1: extract an image of size 2^n x 2^n, converting from RGB to grey-scale if necessary
        // creating two input (visual) and one output (cross-power specturm) images, all square in size, and greyscale
 		

        // copy images ---------------------------------------------------	
	
        if (time) {portable_ftime(&tb); s1=tb.time; ms1=tb.millitm;}
 
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
		 
        if (time) {portable_ftime(&tb); s2=tb.time; ms2=tb.millitm; printf("copy     %4d\n",(s2*1000+ms2)-(s1*1000+ms1));}


        // step 2: apodize the image by multiplying by a Gaussian 
        // =============================================

        if (time) {portable_ftime(&tb); s1=tb.time; ms1=tb.millitm;}

        gaussianApodization (image_a, sigma, image_a);
        gaussianApodization (image_b, sigma, image_b);  

        if (time) {portable_ftime(&tb); s2=tb.time; ms2=tb.millitm; printf("apodizing%4d\n",(s2*1000+ms2)-(s1*1000+ms1));}


        // step 3: compute the cross_power_spectrum 
        // =============================================
		   
        if (time) {portable_ftime(&tb); s1=tb.time; ms1=tb.millitm;}

        cross_power_spectrum (image_b, image_a, image_c); // image_c must exist, type FLOAT
			 	   
        if (time) {portable_ftime(&tb); s2=tb.time; ms2=tb.millitm; printf("cps      %4d\n",(s2*1000+ms2)-(s1*1000+ms1));}


        // step 4: filter the cross_power_spectrum to enhance local maxima
	     // ===============================================================
		      
        if (time) {portable_ftime(&tb); s1=tb.time; ms1=tb.millitm;}
      
        enhance_local_maxima (image_c, filter_radius, image_d); 	 

        if (time) {portable_ftime(&tb); s2=tb.time; ms2=tb.millitm; printf("filter   %4d\n",(s2*1000+ms2)-(s1*1000+ms1));}


        // step 5: locate the local maxima
        // ===============================================================

        if (time) {portable_ftime(&tb); s1=tb.time; ms1=tb.millitm;}

        find_maxima (image_d, number_of_maxima, non_maxima_suppression_radius, maxima);  
 
        if (time) {portable_ftime(&tb); s2=tb.time; ms2=tb.millitm; printf("maxima   %4d\n",(s2*1000+ms2)-(s1*1000+ms1));}

        // display -------------------------------------------------------------	  



        if (time) {portable_ftime(&tb); s1=tb.time; ms1=tb.millitm;}
	   
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

        if (time) {portable_ftime(&tb); s2=tb.time; ms2=tb.millitm; printf("display  %4d\n",(s2*1000+ms2)-(s1*1000+ms1));}

 

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
     

        if (time) {portable_ftime(&tb); s1=tb.time; ms1=tb.millitm;}

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

        if (time) {portable_ftime(&tb); s2=tb.time; ms2=tb.millitm; printf("servo    %4d\n",(s2*1000+ms2)-(s1*1000+ms1));}
 		
        // Total time ---------------------------------------------------------

        if (time) {portable_ftime(&tb); s2=tb.time; ms2=tb.millitm; printf("total    %4d\n\n",(s2*1000+ms2)-(s0*1000+ms0));}
   
        return true;
    }
};

int main(int argc, char *argv[]) {
    //initialize yarp network
	Network yarp;

	//create the module
    CrossPowerSpectrumVergence module;
    module.setName("/crossPowerSpectrumVergence"); // set default name of module

	// prepare and configure the resource finder
    ResourceFinder rf;
    rf.setVerbose(true);
	rf.setDefaultConfigFile("crossPowerSpectrumVergence.ini");  
	rf.setDefaultContext("crossPowerSpectrumVergence/conf");  // the default path will now be $ICUB_ROOT/app/crossPowerSpectrumVergence/conf              
    rf.configure("ICUB_ROOT", argc, argv);

	// configure the module 
	module.configure(rf);

	// run the module
	return module.runModule();
}

 
