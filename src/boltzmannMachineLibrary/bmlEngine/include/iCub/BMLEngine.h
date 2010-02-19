// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef _BMLEngine_H_
#define _BMLEngine_H_

#include <ace/config.h>

//YARP include
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

//openCV include
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>

//IPP include
//#include <ipp.h>

//within Project Include
#include <iCub/MachineBoltzmann.h>
#include <iCub/YARPImgRecv.h>
#include <iCub/imageThread.h>
#include <string>


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;

/**
* This class implements a process able of getting command from a controller 
* interpreting them in term of callings to function of the 
* library BM(BOLZMANN MACHINE LIBRARY)
* the module reads any command on the port /inCmd
* whereas the input image for any clamping is read on /inputImage
*
* \author Rea Francesco
*/
/**
*
@ingroup icub_module
\defgroup icub_bmlEngine bmlEngine

This class implements a process able of getting command from a controller 
interpreting them in term of callings to function of the library BM(BOLZMANN MACHINE LIBRARY)
the module reads any command on the port /inCmd whereas the input image for any clamping is read on /inputImage

\section intro_sec Description
This module receives commands as bottle from the bmlInterface GUI. The command respect a communication stardard based
on bottle composed of vocabols


The module does:
- reads commands from the bmlInterface GUI
- produces images representing the state of every allocated layer
- clamp an input into the selected layer

\image html boltzmannMachineLibrary.png

\section lib_sec Libraries
YARP
OPENCV

\section parameters_sec Parameters
--name:defines the name of the module and the rootname of every port
 
\section portsa_sec Ports Accessed
- /imageProcessor/cmd 


\section portsc_sec Ports Created
Input ports:
- /imageProcessorInterface/img:i
Outports
- /imageProcessorInterface/cmd



\section in_files_sec Input Data Files
none

\section out_data_sec Output Data Files
none
 
\section conf_file_sec Configuration Files
none


\section tested_os_sec Tested OS
Linux and Windows.

\section example_sec Example Instantiation of the Module
imageProcessorInterface


\author Francesco Rea

Copyright (C) 2008 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

**/
class BMLEngine : public Module {
private:
    /** 
     * input port for possible coming images
     */
    BufferedPort<ImageOf<PixelRgb> > port; // input port for possible coming images
    /** 
    * output port n°0 for writing image out
    */
    BufferedPort<ImageOf<PixelRgb> > port0; //port for writing the image out
    /** 
    * output port n°1 for writing image out
    */
    BufferedPort<ImageOf<PixelRgb> > port1; //port for writing the image out
    /** 
    * output port n°2 for writing image out
    */
    BufferedPort<ImageOf<PixelRgb> > port2; //port for writing the image out
    /** 
    * output port n°3 for writing image out
    */
    BufferedPort<ImageOf<PixelRgb> > port3; //port for writing the image out
    /** 
    * port where the commands are vehiculated from controller to engine
    */
    BufferedPort<yarp::os::Bottle> portCmd;
    /**
    * thread that produces the image of the 0 layer
    */
    imageThread* layer0Image;
    /**
    * thread that produces the image of the 1 layer
    */
    imageThread* layer1Image;
    /**
    * thread that produces the image of the 2 layer
    */
    imageThread* layer2Image;
    /**
    * thread that produces the image of the 3 layer
    */
    imageThread* layer3Image;
    /**
    * plane for the port
    */
    BufferedPort<ImageOf<PixelMono> > port_plane; 
    /** 
    * counter for the update step
    */
    int ct;
    /** 
    * options for the connection
    */
    Property options;	//options of the connection
    /** 
    * Object that recalls the Boltzmann Machine Library
    */
    MachineBoltzmann *mb;
    /** 
    * scale factor for the output image representing a layer (X axis)
    */
    int scaleFactorX;
    /** 
    * scale factor for the output image representing a layer (Y axis)
    */
    int scaleFactorY;
    /** 
    * sinchronized with the number of layer active
    */
    int currentLayer;
    /**
    *counter incremented inside the updateModule
    */
    int count;  
    /**
    * iterator for the elements
    */
    map<std::string,Layer>::iterator iterE;
    /**
    * interator for units
    */
    map<std::string,Unit>::iterator iterU;
    /**
    * iterator for the units
    */
    map<std::string,Connection>::iterator iterC;
    ImageOf<PixelRgb> img_tmp; //=new ImageOf<PixelRgb>;
    ImageOf<PixelRgb> img; //=new ImageOf<PixelRgb>;
    ImageOf<PixelRgb> *img0; //=new ImageOf<PixelRgb>;
    ImageOf<PixelRgb> *img2; //=new ImageOf<PixelRgb>;
    /** 
    * flag that enable the drawing of the layer present in the simulation
    */
    bool enableDraw;
    
    /** 
    * flag that regulates the execution of the freely mode
    */
    bool runFreely;
    /** 
    * flag that regulates the execution of the clamped mode
    */
    bool runClamped;
    /**
    * temporary red plane of the image
    */
    //Ipp8u *red_tmp;
    /**
    * temporary blue plane of the image
    */
    //Ipp8u *blue_tmp;
    /**
    * temporary green plane of the image
    */
    //Ipp8u *green_tmp;
    //Ipp8u *im_out;
    /**
    * temporary image composition of planes
    */
    //Ipp8u* im_tmp[3];
    //Ipp8u* im_tmp_tmp;
    int psb;
    /** 
    * value that indicates whether an area can be visually clamped 
    */
    int clampingThreshold;
public:
    /** 
    * open the ports
    */
    bool open(Searchable& config); //open the port
    /** 
    * try to interrupt any communication or resource usage
    */
    bool interruptModule(); // try to interrupt any communications or resource usage
    /** 
    * close all the ports
    */
    bool close(); //closes all the ports
    /** 
    * active control of the module
    */
    bool updateModule(); //active control of the Module
    /**
    * uses a bottle to comunicate a command to the module linked on 
    */
    bool outCommandPort();
    /**
    * closes port image
    */
    bool closePortImage();
    /**
    * opens port Image
    */
    bool openPortImage();
    /**
    * open the port necessary to send commands
    */
    void openCommandPort();
    /**
    * close the port necessary to send commands
    */
    void closeCommandPort();
    /** 
    * set the attribute options of class Property
    */
    void setOptions(Property options); //set the attribute options of class Property
    /** 
    * function that sets the scaleFactorX
    * @param value new value of the scaleFactorX
    */
    void setScaleFactorX(int value);
    /** 
    * function that sets the scaleFactorY
    * @param value new value of the scaleFactorY
    */
    void setScaleFactorY(int value);
    /** 
    * function that set the number of the layer active 
    * @param value number of the layer actually active
    */
    void setCurrentLayer(int value);
    /** 
    * function that loads the configuration file with a specific filename 
    * param filename name of the file where the configuration is loaded
    * 
    */
    void loadConfiguration(string filename);
    /** 
    * function that adds a layer to boltzmann machine
    * @param the number of the already istantiated layers
    * @param the number of the columns in the layer
    * @param the number of rows in the layer
    */
    void addLayer(int number,int colDimension, int rowDimension);
    /** 
    * function that clamp a Layer mapping an image onto it
    * param layerNumber reference to the layer name
    */
    void clampLayer(int LayerNumber);
    /** 
    * function that clamp an image as input of a Layer
    * @param layer reference to the layer name
    */
    void clampLayer(Layer layer);
    /** 
    * input Image which is mapped onto the selected layer
    */
    ImageOf<PixelRgb> *ptr_inputImage;
    /** 
    * input Image which is mapped onto the selected layer
    */
    ImageOf<PixelRgb> *ptr_inputImage2;
    /** 
    * number of layers already istantiated
    */
    int countLayer;
    /**
    * Output Port for commands
    */
    yarp::os::BufferedPort<yarp::os::Bottle> *_pOutPort;
    /**
    * command that is send throgh the command port
    */
    string *outCommand;
    /**
    *bottle containing the option of the command
    */
    Bottle bOptions;
    /**
    * flag that tells if the probability of the freely mode has to be calculated
    */
    bool probFreely_flag;
    /**
    * flag that tells if the probability of the clamped mode has to be calculated
    */
    bool probClamped_flag;
    /**
    * flag that indicates if the inputImage is ready for clamping
    */
    bool inputImage_flag;
};

#endif //_BMLEngine_H_


//----- end-of-file --- ( next line intentionally left blank ) ------------------
