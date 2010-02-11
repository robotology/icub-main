// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _IMAGEPROCESSMODULE_H_
#define _IMAGEPROCESSMODULE_H_

#include <ace/config.h>

//YARP include
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

//openCV include
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>


//within Project Include
#include <iCub/ImageProcessor.h>
//#include <iCub/YARPImgRecv.h>
//#include <iCub/YarpImage2Pixbuf.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;


// general command vocab's
#define COMMAND_VOCAB_HELP VOCAB4('h','e','l','p')
#define COMMAND_VOCAB_SET VOCAB3('s','e','t')
#define COMMAND_VOCAB_GET VOCAB3('g','e','t')
#define COMMAND_VOCAB_RUN VOCAB3('r','u','n')
#define COMMAND_VOCAB_IS VOCAB2('i','s')
#define COMMAND_VOCAB_FAILED VOCAB4('f','a','i','l')
#define COMMAND_VOCAB_OK VOCAB2('o','k')
#define COMMAND_VOCAB_CHILD_COUNT VOCAB2('c','c')
#define COMMAND_VOCAB_WEIGHT VOCAB1('w')
#define COMMAND_VOCAB_CHILD_WEIGHT VOCAB2('c','w')
#define COMMAND_VOCAB_CHILD_WEIGHTS VOCAB3('c','w','s')
#define COMMAND_VOCAB_NAME VOCAB2('s','1')
#define COMMAND_VOCAB_CHILD_NAME VOCAB2('c','n')
#define COMMAND_VOCAB_SALIENCE_THRESHOLD VOCAB2('t','h')
#define COMMAND_VOCAB_NUM_BLUR_PASSES VOCAB2('s','2')
#define COMMAND_VOCAB_RGB_PROCESSOR VOCAB3('r','g','b')
#define COMMAND_VOCAB_YUV_PROCESSOR VOCAB3('y','u','v')
// directional saliency filter vocab's
#define COMMAND_VOCAB_DIRECTIONAL_NUM_DIRECTIONS VOCAB3('d','n','d')
#define COMMAND_VOCAB_DIRECTIONAL_NUM_SCALES VOCAB3('d','n','s')
#define COMMAND_VOCAB_DIRECTIONAL_DBG_SCALE_INDEX VOCAB3('d','s','i')
#define COMMAND_VOCAB_DIRECTIONAL_DBG_DIRECTION_INDEX VOCAB3('d','d','i')
#define COMMAND_VOCAB_DIRECTIONAL_DBG_IMAGE_ARRAY_NAMES VOCAB4('d','a','n','s')
#define COMMAND_VOCAB_DIRECTIONAL_DBG_IMAGE_ARRAY_NAME VOCAB3('d','a','n')
//option for the set command
#define COMMAND_VOCAB_FIL VOCAB3('f','i','l')
#define COMMAND_VOCAB_MAX VOCAB3('m','a','x')
#define COMMAND_VOCAB_OCV VOCAB3('o','c','v')
#define COMMAND_VOCAB_IPP VOCAB3('i','p','p')
#define COMMAND_VOCAB_SEQ VOCAB3('s','e','q')

/**
*
@ingroup icub_module
\defgroup icub_imageprocessor imageProcessor

Module that performs image processing on the input image.
The module has been mainly developed to extract the edges from the input image but it can be used for other operations
of image processing

\section intro_sec Description
This module reads from a buffered port the input image. It immediately produces an output image based on the standard
parameters configuration. However these parameters can be modified any time in order to process the inputimage differently.



The module does:
-   process the input image based on the actual configuration
-   allow the user to change the undertaken processing
-   stream the outimage out


\image html imageProcessor.png

\section lib_sec Libraries
YARP
IPP


\section parameters_sec Parameters
--name : name of the module and relative port name

 
\section portsa_sec Ports Accessed
- /colurPU/rg:o: colourOpponency Map 
- /colurPU/gr:o
- /colurPU/by:o


\section portsc_sec Ports Created
Input ports:
- image:i
- red:i
- green:i
- blue:i
- rg:i
- gr:i
- by:i

Outports:
- outImage:o
- outRed:o 
- outGreen:o
- outBlue:o
- outRG:o
- outGR:o
- outBY:o

InOut ports:
-<name>/cmd

Possible commands that this module is responsive to are:
- set fil: select the filter
- set yuv: select the maximum value of different convolutions
- set ocv: applies the sobel filter (opencv algorithm)
- set ipp: applies the sobel filter (ipp algorithm)
- set seq: sequential algorithm for convolution


\section in_files_sec Input Data Files
none

\section out_data_sec Output Data Files
none
 
\section conf_file_sec Configuration Files
--file.

\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example Instantiation of the Module
imageProcessor --file imageProcessor.ini 


\author Francesco Rea

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

**/

class ImageProcessModule : public Module {
private:
    /**
    * a port for the inputImage
    */
    BufferedPort<ImageOf<PixelRgb> > inImagePort; // 
    /**
    * a port for the redPlane
    */
    BufferedPort<ImageOf<PixelMono> > redPlanePort; //
    /**
    * a port for the greenPlane
    */
    BufferedPort<ImageOf<PixelMono> > greenPlanePort; //
    /**
    * a port for the bluePlane
    */
    BufferedPort<ImageOf<PixelMono> > bluePlanePort; //	 
    /**
    * input port for the R+G- colour Opponency Map
    */
    BufferedPort<ImageOf<PixelMono> > rgPort; 
    /**
    * input port for the G+R- colour Opponency Map
    */
    BufferedPort<ImageOf<PixelMono> > grPort; 
    /**
    * input port for the B+Y- colour Opponency Map
    */
    BufferedPort<ImageOf<PixelMono> > byPort; 	
    /**
    *  output port for edges in R+G- colour Opponency Map
    */
    BufferedPort<ImageOf<PixelMono> > rgEdgesPort; 
    /**
    * output port for edges in G+R- colour Opponency Map
    */
    BufferedPort<ImageOf<PixelMono> > grEdgesPort; 
    /**
    * output port for edges in B+Y- colour Opponency Map
    */
    BufferedPort<ImageOf<PixelMono> > byEdgesPort;
    /**
    * overall edges port combination of maximum values of all the colorOpponency edges
    */
    BufferedPort<ImageOf<PixelMono> > edgesPort;
    /**
    * command port of the module
    */
    BufferedPort<Bottle > cmdPort;
    /**
    * counter of the module
    */
    int ct;
    /**
    * options of the connection
    */
    Property options;	//
    /**
    * width of the input image
    */
    int width;
    /**
    * height of the input image
    */
    int height;
    /**
    * flag that indicates when the reinitiazation has already be done
    */
    bool reinit_flag;
    /**
     * semaphore for the respond function
     */
     Semaphore mutex;
     /**
    * input image reference
    */
    ImageOf<PixelRgb> *inputImg;
    /**
    * input image reference
    */
    ImageOf<PixelMono> *tmp;

    /**
    * function that resets all the mode flags
    */
    void resetFlags();
public:
    /**
    *open the ports of the module
    */
    bool open(Searchable& config); //
    /**
    * tryes to interrupt any communications or resource usage
    */
    bool interruptModule(); // 
    /**
    * closes the modules and all its components
    */
    bool close(); //
    /**
    * active control of the Module
    */
    bool updateModule(); //
    /**
    * set the attribute options of class Property
    */
    /**
    * set the attribute options of class Property
    */
    void setOptions(Property options); //
    /**
    * function to reainitialise the attributes of the class
    */
    void reinitialise(int weight, int height);
    /**
    * respond to command coming from the command port
    */
    bool respond(const Bottle &command,Bottle &reply);
    /**
    * creates some objects necessary for the window
    */
    void createObjects();
    /**
    * sets the adjustments in the window
    */
    void setAdjs();
    //gint timeout_CB (gpointer data);
    //bool getImage();
    /**
    * opens all the ports necessary for the module
    */
    bool openPorts();
    /**
    * closes all the ports opened when the module started
    */
    bool closePorts();
    /**
    * streams out data on ports
    */
    bool outPorts();
    /**
    * sets the module up
    */
    void setUp();
    /**
    * function that starts the imageprocessor
    */
    bool startImageProcessor();


    //_____________ ATTRIBUTES __________________
    

    /**
    * processor that controls the processing of the input image
    */
    ImageProcessor *currentProcessor;
    /**
    * flag that controls if the inputImage has been ever read
    */
    bool inputImage_flag;
    /**
    * flag for maximum convolution
    */
    bool CONVMAX;   
    /**
    * flag for convolution filter
    */
    bool CONVFILTER;
    /**
    * flag that allows find edges function to use sobel from IPP library
    */
    bool IPPISOBEL;
    /**
    * flag that allows find edges function to use opencv sobel
    */
    bool OPENCVSOBEL;
    /**
    * flag that allows find edges function to use a sequence of convolutions
    */
    bool CONVSEQ;
};

#endif //_IMAGEPROCESSMODULE_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------
