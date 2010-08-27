// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea
 * email:   francesco.rea@iit.it
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
 * @file selectiveAttentionProcessor.cpp
 * @brief Implementation of the thread which performs computation in the selectiveAttentionModule
 */

#ifndef _selectiveAttentionProcessor_H_
#define _selectiveAttentionProcessor_H_

//ipp include
#include <ippi.h>

//openCV includes
#include <cv.h>
//#include <cvaux.h>
#include <highgui.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

/* Log-Polar includes */
#include <iCub/RC_DIST_FB_logpolar_mapper.h>

#include <string>
#include <time.h>

const int THREAD_RATE=30;

/**
 *This code groups together a series of useful functions that can be used for ImageProcessing
 */


class selectiveAttentionProcessor:public yarp::os::RateThread {
    private:
        int psb; //width step of 8u images
        int psb32; //width step of 32f images
        int psb_border; //width step of the image with border for 3x3 operator convolution
        IppiSize srcsize; //IppiSize reference to the dimension of the input image
        yarp::sig::ImageOf<yarp::sig::PixelMono> *tmp; //temporary mono image
        yarp::sig::ImageOf<yarp::sig::PixelRgb> *tmp2; //temporary rgb image
        yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > inImagePort; //a port for the inputImage (colour)
        yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imageCartOut; //cartesian image result of the remapping
        yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > map1Port; // input port for the 1st saliency map
        yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > map2Port; // input port for the 2nd saliency map
        yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > map3Port; // input port for the 3rd saliency map
        yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > map4Port; //input port for the 4th saliency map
        yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > map5Port; //input port for the 5th saliency map
        yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > map6Port; //input port for the 6th saliency map
        yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > linearCombinationPort; //output port that represent the linear combination of different maps
        yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > selectedAttentionPort; //output port that represent the selected attention output
        yarp::os::BufferedPort<yarp::os::Bottle > centroidPort;//output port where the centroid coordinate is sent
        yarp::os::Port feedbackPort; //port necessary to send back command to the preattentive processors
        Ipp8u* map1_ippi; //ippi image of the 1st map
        Ipp8u* map2_ippi; //ippi image of the 2nd map
        Ipp8u* map3_ippi; //ippi image of the 3rd map
        Ipp8u* map4_ippi; //ippi image of the 4th map
        Ipp8u* map5_ippi; //ippi image of the 5th map
        Ipp8u* map6_ippi; //ippi image of the 6th map
      
        //yarp::sig::ImageOf<yarp::sig::PixelMono>* outputImagePlane; //temp variable for plane extraction;
        int cLoop; //counter of the loop
        int camSel;   //select the image plane: left or right ( 0: left, 1: right )
        yarp::sig::ImageOf<yarp::sig::PixelRgb> *image_out; //temp variable for plane extraction;
        yarp::sig::ImageOf<yarp::sig::PixelMono> *image_tmp; //temp variable for plane extraction;
        int targetRED; //value read from the blobFinder component (red intensity of the target)
        int targetGREEN; //value read from the blobFinder component (green intensity of the target)
        int targetBLUE; //value read from the blobFinder component (blue intensity of the target)
        int width; //width of the input image
        int height; //height of the input image
        int xSizeValue; //x dimension of the remapped cartesian image
        int ySizeValue; //y dimension of the remapped cartesian image
        int overlap;     //overlap in the remapping
        int numberOfRings;  //number of rings in the remapping
        int numberOfAngles; //number of angles in the remapping
        double salienceTD; //value of the weight of top-down approach in the blobFinder
        double salienceBU; //value of the weight of bottom-up approach in the blobFinder
        unsigned char targetRed; //colour information passed back for the reinforcement
        unsigned char targetGreen; //colour information passed back for the reinforcement
        unsigned char targetBlue; //colour information passed back for the reinforcement
        std::string name; //name of the module and rootname of the connection
        bool reinit_flag; //flag that is set after the dimension of the images is defined
        bool interrupted; //flag set when the interrputed function has already be called
        bool idle; //flag for the idle state of the processor   
        bool gazePerform; //flag that allows the processor to ask the gazeControl for saccadic movements
        yarp::os::Semaphore mutex; //semaphore for the respond function

        double xm, ym; //position of the most salient object in the combination
        yarp::dev::IGazeControl *igaze; //Ikin controller of the gaze
        yarp::dev::PolyDriver* clientGazeCtrl; //polydriver for the gaze controller
        iCub::logpolar::logpolarTransform trsf;

    public:
        /**
        * constructor
        */
        selectiveAttentionProcessor(int rateThread);//

        /**
        * default destructor
        */
        ~selectiveAttentionProcessor();//

        /**
        * constructor 
        * @param inputImage image where the selectiveAttentionProcessor is started from
        */
        selectiveAttentionProcessor(yarp::sig::ImageOf<yarp::sig::PixelRgb>* inputImage );//

        /**
        *initialization of the thread 
        */
        bool threadInit();

        /**
        * active loop of the thread
        */
        void run();

        /**
        * releases the thread
        */
        void threadRelease();

        /**
        * method that resize images once the processor knows the dimesions of the input
        * @param width width dimension the image is resized to
        * @param height height dimension the image is resized to
        */
        void resizeImages(int width, int height);

        /**
        * function called when the module is poked with an interrupt command
        */
        void interrupt();

        /**
        * function that reinitiases some attributes of the class
        */
        void reinitialise(int width, int height);

        /**
        * function that gives reference to the name of the module
        * @param name of the module
        */
        void setName(std::string name);

        /**
        * function that returns the name of the module
        * @param str string to be added
        * @return name of the module
        */
        std::string getName(const char* str);

        /**
        * opens all the ports necessary for the module
        * @return return whether the operation was successful
        */
        bool openPorts();

        /**
        * closes all the ports opened when the module started
        * @return return whether the operation was successful
        */
        bool closePorts();

        /**
        * streams out data on ports
        * @return return whether the operation was successful
        */
        bool outPorts();

        /**
        * set the flag idle locking the resource
        */
        void setIdle(bool value);
        
        /*
        * function that sets which one the two attentive chain (left or right) drives the gaze
        * @param value is the value of the cam selection ( 0: left, 1: right )
        */
        void setCamSelection(int value);

        /**
        * function that set the dimension of the remapped cartesian image
        * @param xSize dimension in x
        */
        void setXSize(int xSize);

        /**
        * function that set the dimension of the remapped cartesian image
        * @param ySize dimension in y
        */
        void setYSize(int ySize);

        /**
        * function that declare the overlap needed in the reconstruction
        * @param overlap value of the overlapping
        */
        void setOverlap(int overlap);

        /**
        * function that declares the number of rings of the image has to be remapped
        * @param numberOfRings number of rings
        */
        void setNumberOfRings(int numberOfRings);

        /**
        * function that declares the number of angles of the image has to be remapped
        * @param numberOfAngles number of angles
        */
        void setNumberOfAngles(int numberOfAngles);

        /*
        * function that declare whether to perform saccadic movement with gazeControl
        * @param value It is true when the saccadic movement is performed
        */
        void setGazePerform(bool value);

        /**
        * function that extract the contour and the center of gravity
        * @param inputImage input image where the contours are extracted from
        * @param outImage representation of the contours
        * @param inputColourImage image where the colour are extracted
        * @param x x position of the center of mass of the contours
        * @param y y position of the center of mass of the contours
        */
        void extractContour(yarp::sig::ImageOf<yarp::sig::PixelMono>* inputImage,yarp::sig::ImageOf<yarp::sig::PixelRgb>* inputColourImage ,int& x,int& y);

        /**
        * function that extracts the colour of a region around a pixel given the input image
        * @param inputColourImage input image where the region colour is read.
        * @param x position in the image plane
        * @param y position in the image plane
        * @param redIntensity intensity of the red plane of the region colour
        * @param greenIntensity intensity of the red plane of the region colour
        * @param blueIntensity intensity of the red plane of the region colour
        */
        void getPixelColour(yarp::sig::ImageOf<yarp::sig::PixelRgb>* inputColourImage,int x ,int y, unsigned char &redIntensity, unsigned char &greenIntensity, unsigned char &blueIntensity);


        yarp::sig::ImageOf<yarp::sig::PixelRgb> *inImage; //input image  of the processing
        yarp::sig::ImageOf<yarp::sig::PixelRgb> *inColourImage; // input image  of the processing
        yarp::sig::ImageOf<yarp::sig::PixelMono>* map1_yarp; //saliency map coming from the 1st source
        yarp::sig::ImageOf<yarp::sig::PixelMono>* map2_yarp; //saliency map coming from the 2nd source
        yarp::sig::ImageOf<yarp::sig::PixelMono>* map3_yarp; //saliency map coming from the 3rd source
        yarp::sig::ImageOf<yarp::sig::PixelMono>* map4_yarp; //saliency map coming from the 4th source
        yarp::sig::ImageOf<yarp::sig::PixelMono>* map5_yarp; //saliency map coming from the 5th source
        yarp::sig::ImageOf<yarp::sig::PixelMono>* map6_yarp; //saliency map coming from the 6th source
        yarp::sig::ImageOf<yarp::sig::PixelMono>* edges_yarp; //yarp image of the composition of all the edges
        double k1; //coefficient for the linear combination of maps
        double k2; //coefficient for the linear combination of maps
        double k3; //coefficient for the linear combination of maps
        double k4; //coefficient for the linear combination of maps
        double k5; //coefficient for the linear combination of maps
        double k6; //coefficient for the linear combination of maps
        IplImage *cvImage16; // tmp IPLImage necessary for edge detection 16 bit
        IplImage *cvImage8; //tmp IPLImage necessary for edge detection 16 bit
        Ipp8u* im_out;

        int inputImage_flag;  //processor flag
        
        
        static const int CONVMAX_TH=100; //parameter of the findEdges function
        static const int CONVSEQ_TH=500; //parameter of the findEdges function
        yarp::sig::ImageOf<yarp::sig::PixelMono>* outputImage; // result of the selection
        yarp::sig::ImageOf<yarp::sig::PixelMono>* outputImage2; //result of the selection
        yarp::sig::ImageOf<yarp::sig::PixelMono>* linearCombinationImage; //result of the combination
        int centroid_x; //center of gravity of the selective attention (x position)
        int centroid_y; //center of gravity of the selective attention (y position)
        time_t start2; //time variable
        time_t end2; //time variable
        yarp::sig::ImageOf<yarp::sig::PixelRgb> *inputImg; //input image reference
        
};

#endif // _selectiveAttentionModule_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

