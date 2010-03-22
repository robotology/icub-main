// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _selectiveAttentionModule_H_
#define _selectiveAttentionModule_H_

#include <ace/config.h>

//YARP include
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

//openCV include
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>


//within Project Include
#include <iCub/selectiveAttentionProcessor.h>
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

// directional saliency filter vocab's
#define COMMAND_VOCAB_DIRECTIONAL_NUM_DIRECTIONS VOCAB3('d','n','d')
#define COMMAND_VOCAB_DIRECTIONAL_NUM_SCALES VOCAB3('d','n','s')
#define COMMAND_VOCAB_DIRECTIONAL_DBG_SCALE_INDEX VOCAB3('d','s','i')
#define COMMAND_VOCAB_DIRECTIONAL_DBG_DIRECTION_INDEX VOCAB3('d','d','i')
#define COMMAND_VOCAB_DIRECTIONAL_DBG_IMAGE_ARRAY_NAMES VOCAB4('d','a','n','s')
#define COMMAND_VOCAB_DIRECTIONAL_DBG_IMAGE_ARRAY_NAME VOCAB3('d','a','n')
//option for the set command
#define COMMAND_VOCAB_K1 VOCAB2('k','1')
#define COMMAND_VOCAB_K2 VOCAB2('k','2')
#define COMMAND_VOCAB_K3 VOCAB2('k','3')
#define COMMAND_VOCAB_K4 VOCAB2('k','4')
#define COMMAND_VOCAB_K5 VOCAB2('k','5')
#define COMMAND_VOCAB_K6 VOCAB2('k','6')

/**
*
@ingroup icub_module
\defgroup icub_selectiveAttentionProcessor selectiveAttentionProcessor

Module that combines the saliency map using winner-takes-all algorithm and selects the attentive region of interest.


\section intro_sec Description
Module that combines the saliency map using winner-takes-all algorithm and selected the attentive region of interest.
Moreover it is in charge of closing the first loop of self-reingorcement. In other words, it sends back commands that are used
in order to select the area of interest more precisely.



The module does:
-   process the input images as saliency maps
-   determine the winner-take-all saliency region
-   apply the Inhibition of return (todo)
-   send feedback to the preattentive section of the application in order to reinforce the stimulus coming from selected area


\image html selectiveAttentionProcessor.png

\section lib_sec Libraries
YARP
OPENCV
IPP


\section parameters_sec Parameters
--name : name of the module and relative port name

 
\section portsa_sec Ports Accessed
none


\section portsc_sec Ports Created
Input ports:
- image:i : input image where the colours are extracted.
- map1:i : image coming from the 1st saliency map
- map2:i : image coming from the 2nd saliency map
- map3:i : image coming from the 3rd saliency map
- map4:i : image coming from the 4th saliency map
- map5:i : image coming from the 5th saliency map
- map6:i : image coming from the 6th saliency map

Outports:
- attention:o : graphical output of the selected region of interest (winner-take-all)
- combination:o : graphical output that shows the linear combination of different maps
- centroid:o : port where the coordinate of the attention focus are sent 
- feedback:o : port necessary to send back commands to preattentive processors

InOut ports:
-<name>/cmd

Possible commands that this module is responsive to are:
- set k1: coefficient for the linear combination of map1
- set k2: coefficient for the linear combination of map2
- set k3: coefficient for the linear combination of map3
- set k4: coefficient for the linear combination of map4
- set k5: coefficient for the linear combination of map5
- set k5: coefficient for the linear combination of map6


\section in_files_sec Input Data Files
none

\section out_data_sec Output Data Files
none
 
\section conf_file_sec Configuration Files
--file.

\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example Instantiation of the Module
selectiveAttentionEngine --file selectiveAttentionEngine.ini 


\author Francesco Rea

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

**/

class selectiveAttentionModule : public Module {
private:
    /**
    * a port for the inputImage
    */
    BufferedPort<ImageOf<PixelRgb> > inImagePort; // 
    /**
    * input port for the 1st saliency map
    */
    BufferedPort<ImageOf<PixelMono> > map1Port; //
    /**
    * input port for the 2nd saliency map
    */
    BufferedPort<ImageOf<PixelMono> > map2Port; //
    /**
    * input port for the 3rd saliency map
    */
    BufferedPort<ImageOf<PixelMono> > map3Port; //	 
    /**
    * input port for the 4th saliency map
    */
    BufferedPort<ImageOf<PixelMono> > map4Port; 
    /**
    * input port for the 5th saliency map
    */
    BufferedPort<ImageOf<PixelMono> > map5Port; 
    /**
    * input port for the 6th saliency map
    */
    BufferedPort<ImageOf<PixelMono> > map6Port; 	
    /**
    *  output port that represent the linear combination of different maps
    */
    BufferedPort<ImageOf<PixelMono> > linearCombinationPort; 
    /**
    *  output port that represent the selected attention output
    */
    BufferedPort<ImageOf<PixelMono> > selectedAttentionPort; 
    /**
    *  output port where the centroid coordinate is sent
    */
    BufferedPort<Bottle > centroidPort; 
    /**
    *  port necessary to send back command to the preattentive processors
    */
    Port feedbackPort; 
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
    * temporary mono image
    */
    ImageOf<PixelMono> *tmp;
    /**
    * input image of the 1st map
    */
    ImageOf<PixelMono> *map1Img;
    /**
    * input image of the 2nd map
    */
    ImageOf<PixelMono> *map2Img;
    /**
    * input image of the 3rd map
    */
    ImageOf<PixelMono> *map3Img;
    /**
    * input image of the 4th map
    */
    ImageOf<PixelMono> *map4Img;
    /**
    * input image of the 5th map
    */
    ImageOf<PixelMono> *map5Img;
    /**
    * input image of the 6th map
    */
    ImageOf<PixelMono> *map6Img;
    /**
    * value read from the blobFinder component (red intensity of the target)
    */
    int targetRED;
    /**
    * value read from the blobFinder component (green intensity of the target)
    */
    int targetGREEN;
    /**
    * value read from the blobFinder component (blue intensity of the target)
    */
    int targetBLUE;
    /**
    * value of the weight of top-down approach in the blobFinder
    */
    int salienceTD;
    /**
    * value of the weight of bottom-up approach in the blobFinder
    */
    int salienceBU;

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
    * function that starts the selectiveAttentionProcessor
    */
    bool startselectiveAttentionProcessor();


    //_____________ ATTRIBUTES __________________
    

    /**
    * processor that controls the processing of the input image
    */
    selectiveAttentionProcessor *currentProcessor;
    /**
    * flag that controls if the inputImage has been ever read
    */
    bool inputImage_flag;
    
};

#endif //_selectiveAttentionModule_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------
