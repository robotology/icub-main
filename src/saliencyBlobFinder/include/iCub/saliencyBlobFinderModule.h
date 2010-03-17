// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _SALIENCYBLOBFINDERMODULE_H_
#define _SALIENCYBLOBFINDERMODULE_H_

#include <time.h>
#include <iostream>
#include <map>
using namespace std;


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

#define COMMAND_VOCAB_NAME VOCAB2('s','1')
#define COMMAND_VOCAB_CHILD_NAME VOCAB2('c','n')
#define COMMAND_VOCAB_SALIENCE_THRESHOLD VOCAB2('t','h')
#define COMMAND_VOCAB_NUM_BLUR_PASSES VOCAB2('s','2')
#define COMMAND_VOCAB_RGB_PROCESSOR VOCAB3('r','g','b')
#define COMMAND_VOCAB_YUV_PROCESSOR VOCAB3('y','u','v')
// command vocab for the output produced
#define COMMAND_VOCAB_MEANCOLOURS VOCAB3('m','e','a')
#define COMMAND_VOCAB_TAGGED VOCAB3('t','a','g')
#define COMMAND_VOCAB_MAXSALIENCY VOCAB3('m','a','x')
#define COMMAND_VOCAB_CONTRASTLP VOCAB3('c','l','p')
#define COMMAND_VOCAB_FOVEA VOCAB3('f','o','v')
// other commands
#define COMMAND_VOCAB_TCON VOCAB4('t','c','o','n') //time contant for the controlGaze2
#define COMMAND_VOCAB_TCEN VOCAB3('t','c','e','n') //time constant for the iKinGazeCtrl


#define COMMAND_VOCAB_KBU VOCAB3('k','b','u') //weight of the bottom-up algorithm
#define COMMAND_VOCAB_KTD VOCAB3('k','t','d') //weight of top-down algorithm
#define COMMAND_VOCAB_RIN VOCAB3('r','i','n') //red intensity value
#define COMMAND_VOCAB_GIN VOCAB3('g','i','n') //green intensity value
#define COMMAND_VOCAB_BIN VOCAB3('b','i','n') //blue intensity value
#define COMMAND_VOCAB_MAXDB VOCAB3('M','d','b') //Maximum dimension of the blob drawn
#define COMMAND_VOCAB_MINDB VOCAB3('m','d','b') //minimum dimension of the blob drawn


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

This module extracts visual blobs in the scene and gives a saliency value to any of them.
The process of assigning a saliency value is based on a top-down algorithm and bottom-up algorithm.(see <a href="http://">here</a>).
The bottom-up gives high saliency to isolated blobs whereas the top-down gives high saliency to blobs which have mean colour close to the target colour.

This module receives an edge image and the respective red plane, green plane and yellow plane. It needs the opponency maps composed as R+G-, G+R-, B+Y- as well.
Those are maps that made by the difference of gaussians appied on the three colour planes. These rappresent the colour sentive photo receptors present in humans.

Therefore, the module applies the watershed technique in order to extract all the blob and calculates a saliency map based on the mean color of the blob,
its dimension and the the color distance to the target object.

\section intro_sec Description

The module does:
-   stream the mean color image of all the blobs
-   stream the image of the fovea blob
-   stream the saliency map as a gray scale image
-	stream the most salient blob

\image html saliencyBlobFinder.png


\section lib_sec Libraries
YARP 
IPP

\section parameters_sec Parameters
Here is a  comprehensive list of the parameters you can pass to the module.
--name (string) name of the module. The name of all the ports will be istantiated starting from this name 
--file (string) name of the configuration file.
 
\section portsa_sec Ports Accessed
- /blobFinderInterface/command:o : once manually connected to the <name>/cmd the graphical interface is able to control this module (for further information press Help Button in the interface)
- /colourPU/red:o
- /colourPU/green:o
- /colourPU/blue:o
- /colourPU/rg:o
- /colourPU/gr:o
- /colourPU/by:o
- /imagePU/edges:o

\section portsc_sec Ports Created
- <name>/cmd
- <name>/image:i
- <name>/red:i
- <name>/green:i
- <name>/blue:i
- <name>/rg:i
- <name>/gr:i
- <name>/by:i
- <name>/image:o
- <name>/centroid:o


Output ports:
- <name>/image:o: image output
- <name>/centroid:o : position of the cog of the most salient blob


Input ports:
- <name>/image:i: input ports which takes as input a yarp::sig::ImageOf<PixelRgb>
- <name>/red:i: reads out a yarp::sig::ImageOf<PixelMono> which is the red plane
- <name>/green:i:  reads out a yarp::sig::Image<PixelMono> which is the green plane
- <name>/blue:i:  reads out a yarp::sig::Image<PixelMono> which is the blue plane
- <name>/rg:i: acquires the input stream of the R+G- opponency map
- <name>/gr:i: acquires the input stream of the G+R- opponency map
- <name>/by:i: acquires the input stream of th B+Y- opponency map

InOut ports:
- <name>/cmd : port for the input rpc commands (for further command send Help command)

This module is able to respond to the following set of commands:
- set mea: select the output which represents all the blobs in mean colour
- set tag: select the output which represents all the blobs 
- set max: select the output which represents the most salient blob
- set clp: select the output which represents all the blobs intensified by the saliency
- set fov: select the output which represents the fovea blob

- set kbu: weight of the bottom-up algorithm
- set ktd: weight of top-down algorithm
- set rin: red intensity value
- set gin: green intensity value
- set bin: blue intensity value
- set Mdb: Maximum dimension of the blob analysed
- set mdb: minimum dimension of the blob analysed


\section in_files_sec Input Data Files
none

\section out_data_sec Output Data Files
none
 
\section conf_file_sec Configuration Files
saliencyBlobFinder.ini

\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example Instantiation of the Module
saliencyBlobFinder --file saliencyBlobFinder.ini

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
    * port used for centroid position to controlGaze2
    */
    BufferedPort<Bottle> centroidPort;
    /**
    * port used for centroid position to iKinHead
    */
    Port triangulationPort;
    /**
    * port used for sending responses from triangulationPort back into i
    */
    BufferedPort<Bottle> gazeControlPort;
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
    * buffer image for received image
    */
    ImageOf<PixelMono> *tmpImage;
    /**
    * main thread responsable to process the input images and produce an output
    */
    blobFinderThread* blobFinder;
    /**
    * bottle where the command received is saved ready for the respond
    */
    Bottle* command;
    /**
    * bottle where the reply will be stored for further purpose
    */
    Bottle* reply;
    /**
    * time variable
    */
    time_t start;
    /**
    * time variable
    */
    time_t end;
    /**
    * position of the target in the body-centered frame (x coordinate)
    */
    double target_x;
    /**
    * position of the target in the body-centered frame (y coordinate)
    */
    double target_y;
    /**
    * position of the target in the body-centered frame (z coordinate)
    */
    double target_z;
    //_________ private methods ____________
    /**
    * function that reads the ports for colour RGB opponency maps
    */
    bool getOpponencies();
    /**
    * function that reads the ports for the RGB planes
    */
    bool getPlanes();
    /**
    * function that copies flags to the blobFinder thread
    */
    void copyFlags();

    /**
    * map of the occurences of control positions
    */
    std::map<const char*,int> occurencesMap;
    
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
    
    //---------- flags --------------------------
	/**
	* flag for drawing contrastLP
	*/
	bool contrastLP_flag;
	/**
	* flag for drawing meanColourImage
	*/
	bool meanColour_flag;
	/**
	* flag for drawing blobCatalog
	*/
	bool blobCataloged_flag;
	/**
	* flag for drawing foveaBlob
	*/
	bool foveaBlob_flag;
	/**
	* flag for drawing colorVQ
	*/
	bool colorVQ_flag;
	/**
	* flag for drawing maxSaliencyBlob
	*/
	bool maxSaliencyBlob_flag;
	/**
	* flag for drawing blobList
	*/
	bool blobList_flag;
	/**
	* flag for the drawings
	*/
	bool tagged_flag;
	/**
	* flag for drawing watershed image
	*/
	bool watershed_flag;
   
};

#endif //__SALIENCYBLOBFINDERMODULE_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------
