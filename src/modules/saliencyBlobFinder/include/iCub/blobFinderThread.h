// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * @file blobFinderThread.h
 * @brief module class definition for the blob finder thread (this is the module actual processing engine).
 */

#ifndef _BLOBFINDERTHREAD_H_
#define _BLOBFINDERTHREAD_H_

#include <ippi.h>
#include <ippcore.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <iCub/WatershedOperator.h>
#include <iCub/SalienceOperator.h>

using namespace yarp::os;
using namespace yarp::sig;

/*
 * LATER: add proper Doxygen documentation here.
 */
class blobFinderThread : public yarp::os::RateThread {
private:
    BufferedPort<ImageOf<PixelRgb> > inputPort;         // port where the input image is read from
    BufferedPort<ImageOf<PixelMono> > edgesPort;        // port where the edges image is read
    BufferedPort<ImageOf<PixelMono> > saliencePort;     // port that returns the salience map
    BufferedPort<ImageOf<PixelRgb> > outputPort3;       // port that returns the image output 3channels
    BufferedPort<ImageOf<PixelMono> > rgPort;           // port where the difference of gaussian R+G- is streamed
    BufferedPort<ImageOf<PixelMono> > grPort;           // port where the difference of gaussian G+R- is streamed
    BufferedPort<ImageOf<PixelMono> > byPort;           // port where the difference of gaussian B+Y- of the image is streamed
    BufferedPort<ImageOf<PixelMono> > yellowPort;       // port where the yellow plane of the image is streamed

    ImageOf<PixelMono> *outContrastLP;                  // image result of the function outContrastLP
    ImageOf<PixelMono> *tmpImage;                       // buffer image for received image
    ImageOf<PixelBgr> *outMeanColourLP;                 // image result of the function meanColourLP;
    
    IppiSize srcsize;                                   // ipp reference to the size of the input image
    int width;                                          // width of the input image
    int height;                                         // height of the input image
    
    std::string name;                                   // name of the module and rootname of the connection
    bool reinit_flag;                                   // flag that indicates when the reinitiazation has already be done
    bool resized_flag;                                  // flag that indicates if the images have been resized
    
    ImageOf<PixelRgb> *img;                             // input image
    ImageOf<PixelMono> *edges;                          // edges image
   
    WatershedOperator *wOperator;                       // pointer to the watershed operator
    char* blobList;                                     // vector of boolean which tells whether there is a blob or not
    
    ImageOf<PixelRgb>  *_outputImage3;                  // pointer to the 3 channels output image of the watershed algorithm
    ImageOf<PixelMono> *_inputImgRGS;                   // input image of the opponency R+G-
    ImageOf<PixelMono> *_inputImgGRS;                   // input image of the opponency G+R-
    ImageOf<PixelMono> *_inputImgBYS;                   // input image of the opponency B+Y-
    ImageOf<PixelInt>  *ptr_tagged;                     // pointer to the image of tags

private:
    SalienceOperator *salience;                             // reference to the salience operator
    YARPBox* max_boxes;                                     // pointer to the most salient blob

    ImageOf<PixelMono> *image_out;                          // image which is plotted in the drawing area
    ImageOf<PixelRgb> *image_out2;                          // image which is plotted in the drawing area

    ImageOf<yarp::sig::PixelRgb> *ptr_inputImg;             // pointer to the input image
    ImageOf<PixelMono> *ptr_inputImgRed;                    // pointer to the red plane input image
    ImageOf<PixelMono> *ptr_inputImgGreen;                  // pointer to the green plane input image
    ImageOf<PixelMono> *ptr_inputImgBlue;                   // pointer to the input blue plane image
    ImageOf<PixelMono> *ptr_inputImgRG;                     // pointer to the input image R+G-
    ImageOf<PixelMono> *ptr_inputImgGR;                     // pointer to the input image G+R-
    ImageOf<PixelMono> *ptr_inputImgBY;                     // pointer to the input image B+Y-
    ImageOf<PixelRgb>* _procImage;                          // pointer to the output image of the watershed algorithm

    int maxBLOB;                                            // maxBLOB dimension
    int minBLOB;                                            // minBLOB dimension
    int max_tag;                                            // number of blobs
    int minBoundingArea;                                    // dimension of the bounding area in saliency BU algorithm
    int saddleThreshold;                                    // threshold necessary to determine the saddle point of rain falling watershed

private: 
    /**
    * resizes all the needed images
    * @param width width of the input image
    * @param height height of the input image
    */
    void resizeImages(int width, int height);

    /**
    * function that extracts characteristics of all the blobs in the catalogue and save them
    * @param stable parameters that enable some lines of code for the stable version
    */
    void drawAllBlobs(bool stable);
    
    /**
    * function that free memory allocated for the look-up table
    */
    bool freeLookupTables();

public:
    /**
    * constructor
    * param rateThread period of the processing thread
    */
    blobFinderThread(int rateThread);

    /**
    * destructor
    */
    ~blobFinderThread();

    /**
    *initialization of the thread 
    */
    bool threadInit();

    /**
    * active loop of the thread
    */
    void run();

    /**
    *releases the thread
    */
    void threadRelease();

    /**
    * function that reinitiases some attributes of the class
    * @param height height of the input image
    * @param width width of the input image
    */
    void reinitialise(int width,int height);

    /**
    * function called when the module is poked with an interrupt command
    */
    void interrupt();
    
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
    * function the applies the watershed (rain falling) algorithm
    * @param edgesImage image representing the edges extracted from colourOpponency maps
    */
    void rain(ImageOf<PixelMono>* edgesImage);
    
    /**
    * function that resets all the flags for the desired output
    */
    void resetFlags();
    
    /**
    * function that reads the ports for colour RGB opponency maps
    */
    bool getOpponencies();
    
    /**
    * function that reads the ports for the RGB planes
    * @param inputImage image where we can extract planes from
    */
    bool getPlanes(ImageOf<PixelRgb>* inputImage);

    /**
     *
     */
    inline bool setMinBoundingArea(int w) { minBoundingArea = w; return true; }

    /**
     *
     */
    inline int getMinBoundingArea() const { return minBoundingArea; }

    /**
     *
     */
    inline bool setSaliencyPercentageArea(double p) { return salience->setPercentageArea(p); }

    /**
     *
     */
    inline double getSaliencyPercentageArea() const { return salience->getPercentageArea(); }

    /**
     *
     */
    inline bool setMaxBlobSize(int s) { maxBLOB = s; return true; }

    /**
     *
     */
    inline int getMaxBlobSize() const { return maxBLOB; }

    /**
     *
     */
    inline bool setMinBlobSize(int s) { minBLOB = s; return true; }

    /**
     *
     */
    inline int getMinBlobSize() const { return minBLOB; }
};

#endif //__BLOBFINDERTHREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------
