// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _SALIENCYBLOBFINDERMODULE_H_
#define _SALIENCYBLOBFINDERMODULE_H_

//within project includes
#include <iCub/blobFinderThread.h>

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
\defgroup icub_saliencyBlobFinder saliencyBlobFinder

This module receives an edge image and the respective red plane, green plane and yellow plane.
It needs the opponency maps composed as R+G-, G+R-, B+Y-.
The module applies the watershed technique in order to extract all the blob and calculates a saliency map based on the mean color of the blob,
its dimension and the the color distance to the target object

\section intro_sec Description

The module does:
-   stream the mean color image of all the blobs
-   stream the image of the fovea blob
-   stream the saliency map as a gray scale image
-	stream the most salient blob


\section lib_sec Libraries
YARP 
IPP

\section parameters_sec Parameters
Here is a  comprehensive list of the parameters you can pass to the module.
--name (string) name of the module. The name of all the ports will be istantiated starting from this name 

 
\section portsa_sec Ports Accessed
none

\section portsc_sec Ports Created
<name>/cmd
<name>/image:i
<name>/red:i
<name>/green:i
<name>/blue:i
<name>/rg:i
<name>/gr:i
<name>/by:i
<name>/saliencyMap:o


Output ports:
- <name>/saliencyMap:o: map of the most salient blobs

Input ports:
- <name>/image:i: input ports which takes as input a yarp::sig::ImageOf<PixelRgb>
- <name>/red:i: reads out a yarp::sig::ImageOf<PixelMono> which is the red plane
- <name>/green:i:  reads out a yarp::sig::Image<PixelMono> which is the green plane
- <name>/blue:i:  reads out a yarp::sig::Image<PixelMono> which is the blue plane
- <name>/rg:i: acquires the input stream of the R+G- opponency map
- <name>/gr:i: acquires the input stream of the G+R- opponency map
- <name>/by:i: acquires the input stream of th B+Y- opponency map

InOut ports
- <name>/cmd : port for the input rpc commands

\section in_files_sec Input Data Files
none

\section out_data_sec Output Data Files
none
 
\section conf_file_sec Configuration Files
saliencyBlobFinder

\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example Instantiation of the Module
saliencyBlobFinderProcessor --name /blobFinder 

\author Francesco Rea

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

**/


class saliencyBlobFinderModule : public Module{
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
    * port that returns the image output
    */
    BufferedPort<ImageOf<PixelRgb> > outputPort;
    /**
    * port necessary for rpc commands
    */
    BufferedPort<Bottle> cmdPort;
    /**
    * ipp reference to the size of the input image
    */
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
    ImageOf<PixelRgb> *img;
    /**
    * main thread responsable to process the input images and produce an output
    */
    blobFinderThread* blobFinder;

    //_________ private methods ____________
    /**
    * function that reads the ports for colour RGB opponency maps
    */
    bool getOpponencies();
    /**
    * function that reads the ports for the RGB planes
    */
    bool getPlanes();
    
public:
    /**
    * default constructor
    */
    saliencyBlobFinderModule();
    /**
    * destructor
    */
    ~saliencyBlobFinderModule(){};
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
    * @param width width of the input image
    * @param height height of the input image
    */
    void reinitialise(int width,int height);
    /**
    * respond to command coming from the command port
    * @param command command received by the module
    * @param reply answer that this module gives to the command (if any)
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
    
   
};

#endif //__SALIENCYBLOBFINDERMODULE_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------
