// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _RGBPROCESSORMODULE_H_
#define _RGBPROCESSORMODULE_H_

//within project includes
#include <iCub/rgbProcessorThread.h>
#include <iCub/yuvProcessorThread.h>

//IPP include
#include <ippi.h>


//YARP include
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

using namespace yarp::os;
using namespace yarp::sig;

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

/**
*
@ingroup icub_module
\defgroup icub_colourProcessor colourProcessor

This module processes the input image and colour information from this.
Starting from RGB channels it can produce the corrispective YUV channels if the user selects the option.
If not selected the YUV channels are not computed and the module does not require extra computation power.
All these channels are made available to other users via ports.
Colour quantization is performed in order to reduce the effect of the input noise. After the yellow channel is 
extracted the four channels are combined together to create three color opponent channels similar to those in retina.
These are normally indicated with the names R+G-,G+R-, B+Y- and they have a center surround receptive field with
spectrally opponent colour responses

\section intro_sec Description

The module does:
-	run different processors for colour processing
-   reads a configuration file (todo)
-   reads commands from rpc
-	always, the RED, GREEN BLUE planes are streamed out on ports.
-	if the yuv processor is on, the Y, U+V  planes are streamed out on ports
-   the yellow channel is extracted and provided
-   the colour opponency maps are calculated and streamed out on ports
-   for the intensity channel and the chrominance channel it applies a gaussian filter (todo)


\image html colourProcessor.png

\section lib_sec Libraries
YARP 
IPP

\section parameters_sec Parameters
Here is a  comprehensive list of the parameters you can pass to the module.
--name (string) name of the module. The name of all the ports will be istantiated starting from this name 

 
\section portsa_sec Ports Accessed
none

\section portsc_sec Ports Created
<name>/cmd:i
<name>/image:i
<name>/red:o
<name>/green:o
<name>/blue:o
<name>/ychannel:o
<name>/uvchannel:o
<name>/uchannel:o
<name>/vchannel:o
<name>/rg:o
<name>/gr:o
<name>/by:o


Output ports:
- <name>/red:o: streams out a yarp::sig::ImageOf<PixelMono> which is the red plane
- <name>/green:o:  streams out a yarp::sig::Image<PixelMono> which is the green plane
- <name>/blue:o:  streams out a yarp::sig::Image<PixelMono> which is the blue plane
- <name>/ychannel:o: streams out a yarp::sig::ImageOf<PixelMono> which is the intensity information
- <name>/uvchannel:o:  streams out a yarp::sig::Image<PixelMono> which is the chrominance information
- <name>/uchannel:o:    streams out a yarp::sig::Image<PixelMono> which corrispond to the u channel
- <name>/vchannel:o:    streams out a yarp::sig::Image<PixelMono> which corrispond to the v channel     
- <name>/rg:o:  streams out a yarp::sig::Image<PixelMono> which corrispond to the colourOpponency Map defined as R+G-
- <name>/gr:o:  streams out a yarp::sig::Image<PixelMono> which corrispond to the colourOpponency Map defined as G+R-
- <name>/by:o:  streams out a yarp::sig::Image<PixelMono> which corrispond to the colourOpponency Map defined as B+Y-


Input ports:
- <name>/image:i: input ports which takes as input a yarp::sig::ImageOf<PixelRgb>
- <name>/cmd : port for the input rpc commands

\section in_files_sec Input Data Files
none

\section out_data_sec Output Data Files
none
 
\section conf_file_sec Configuration Files
none

\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example Instantiation of the Module
colourProcessor --name /colourPU 

\author Francesco Rea

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

**/


class colourProcessorModule : public Module{
private:
    /**
    * port where the input image is read from
    */
    BufferedPort<ImageOf<PixelRgb> > inputPort;
    /**
    * port where the red plane of the image is streamed
    */
    BufferedPort<ImageOf<PixelMono> > redPort;
    /**
    * port where the green plane of the image is streamed
    */
    BufferedPort<ImageOf<PixelMono> > greenPort;
    /**
    * port where the blue plane of the image is streamed
    */
    BufferedPort<ImageOf<PixelMono> > bluePort;
    /**
    * port where the difference of gaussian R+G- is streamed
    */
    BufferedPort<ImageOf<PixelMono> > rgPort;
    /**
    * port where the difference of gaussian G+R- is streamed
    */
    BufferedPort<ImageOf<PixelMono> > grPort;
    /**
    * port where the difference of gaussian B+Y- of the image is streamed
    */
    BufferedPort<ImageOf<PixelMono> > byPort;
    /**
    * port where the yellow plane of the image is streamed
    */
    BufferedPort<ImageOf<PixelMono> > yellowPort;
    /**
    * port where the ychannel of the image is streamed
    */
    BufferedPort<ImageOf<PixelMono> > yPort;
    /**
    * port where the uchannel of the image is streamed
    */
    BufferedPort<ImageOf<PixelMono> > uPort;
    /**
    * port where the vchannel plane of the image is streamed
    */
    BufferedPort<ImageOf<PixelMono> > vPort;
     /**
    * port where the uvchannel of the input image is streamed
    */
    BufferedPort<ImageOf<PixelMono> > uvPort;
    /**
    * flag that indicates the yuv processor should be start
    */
    bool startyuv_flag;
    /**
    * flag that indicates the yuv processor should be start
    */
    bool startrgb_flag;
    /**
    * port necessary for rpc commands
    */
    BufferedPort<Bottle> cmdPort;

    IppiSize srcsize;
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
    * execution step counter
    */
    int ct;
    /**
    * input image
    */
    ImageOf<PixelRgb> *inputImg;
    /**
    * input image
    */
    ImageOf<PixelRgb> *img;
    //_________ private methods ____________
    /**
    * function that starts the RGB Processor
    */
    void startRgbProcessor();
    /**
    * function that starts the YUV Processor
    */
    void startYuvProcessor();
public:
    /**
    * default constructor
    */
    colourProcessorModule();
    /**
    * destructor
    */
    ~colourProcessorModule(){};
    /**
    *opens the port and intialise the module
    * @param config configuration of the module
    */
    bool open(Searchable& config); 
    /**
    * tries to interrupt any communications or resource usage
    */
    bool interruptModule(); // 
    /**
    * function that set the options detected from the command line
    * @param opt options passed to the module
    */
    void setOptions(yarp::os::Property opt);
    /**
    * catches all the commands that have to be executed when the module is closed
    */
    bool close(); 
    /**
    * function that reinitiases some attributes of the class
    */
    void reinitialise(int width,int height);
    /**
    * respond to command coming from the command port
    */
    bool respond(const Bottle &command,Bottle &reply);
    /**
    * updates the module
    */
    bool updateModule();
    /**
    * function that streams the images out on the ports
    */
    void outPorts();
    


    //_________ public attributes _______________
    yuvProcessorThread yuvProcessor;
    rgbProcessorThread rgbProcessor;
   
};

#endif //_RGBPROCESSORMODULE_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------
