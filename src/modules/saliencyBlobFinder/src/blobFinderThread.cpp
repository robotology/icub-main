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
 * @file blobFinderThread.cpp
 * @brief module class implementation of the blob finder thread (this is the module actual processing engine).
 */

#include <cstring>
#include <iostream>

#include <iCub/blobFinderThread.h>

using namespace std;
using namespace yarp::os;

const int DEFAULT_THREAD_RATE = 100;

blobFinderThread::blobFinderThread(int rateThread = DEFAULT_THREAD_RATE) : RateThread(rateThread)
{
    saddleThreshold = 10;
    reinit_flag = false;
    resized_flag = false;

    outContrastLP=new ImageOf<PixelMono>;
    outMeanColourLP=new ImageOf<PixelBgr>;

    _procImage = new ImageOf<PixelRgb>;
    _outputImage3 = new ImageOf<PixelRgb>;

    ptr_inputImg = new ImageOf<yarp::sig::PixelRgb>; 
    ptr_inputImgRed = new ImageOf<yarp::sig::PixelMono>; 
    ptr_inputImgGreen = new ImageOf<yarp::sig::PixelMono>; 
    ptr_inputImgBlue = new ImageOf<yarp::sig::PixelMono>; 
    ptr_inputImgRG = new ImageOf<yarp::sig::PixelMono>; 
    ptr_inputImgGR = new ImageOf<yarp::sig::PixelMono>; 
    ptr_inputImgBY = new ImageOf<yarp::sig::PixelMono>; 
    edges = new ImageOf<yarp::sig::PixelMono>; 
    img = new ImageOf<PixelRgb>;
    tmpImage = new ImageOf<PixelMono>;
    image_out = new ImageOf<PixelMono>;
    
    _inputImgRGS = new ImageOf<PixelMono>;
    _inputImgGRS = new ImageOf<PixelMono>;
    _inputImgBYS = new ImageOf<PixelMono>;

    // some standard parameters on the blob search.
    maxBLOB = 4096;
    minBLOB = 100;
    minBoundingArea = 225;
}

blobFinderThread::~blobFinderThread() {
    delete outContrastLP;
    delete outMeanColourLP;
    delete _procImage;
    delete _outputImage3;
    delete ptr_inputImg;        // pointer to the input image
    delete edges;               // pointer to the edges image

    delete ptr_inputImgRed;     // pointer to the input image of the red plane
    delete ptr_inputImgGreen;   // pointer to the input image of the green plane
    delete ptr_inputImgBlue;    // pointer to the input image of the blue plane
    delete ptr_inputImgRG;      // pointer to the input image of the R+G- colour opponency
    delete ptr_inputImgGR;      // pointer to the input image of the G+R- colour opponency
    delete ptr_inputImgBY;      // pointer to the input image of the B+Y- colour opponency

    delete _inputImgRGS;
    delete _inputImgGRS;
    delete _inputImgBYS;

    delete img;
    delete tmpImage;
    delete image_out;
    delete wOperator;
    delete salience;
    delete ptr_tagged;
}

void blobFinderThread::setName(std::string str) {
    this->name=str; 
}

std::string blobFinderThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void blobFinderThread::reinitialise(int width, int height) {
    this->width = width;
    this->height = height;

    img->resize(width, height);
    edges->resize(width, height);
    tmpImage->resize(width, height);
    image_out->resize(width, height);
    
    resizeImages(width, height);

    reinit_flag = true;
}

void blobFinderThread::resizeImages(int width, int height) {
    srcsize.height = height;
    srcsize.width = width;

    const int widthstep = int(ceil(width/32.0)*32);

    ptr_tagged = new yarp::sig::ImageOf<yarp::sig::PixelInt>;
    ptr_tagged->resize(widthstep, height);
    
    this->wOperator=new WatershedOperator(true, width, height, widthstep, saddleThreshold);
    this->salience=new SalienceOperator(width, height);

    outMeanColourLP->resize(width, height);
    outContrastLP->resize(width, height);

    _procImage->resize(width, height);
    _outputImage3->resize(width, height);
    _inputImgRGS->resize(width, height);
    _inputImgGRS->resize(width, height);
    _inputImgBYS->resize(width, height);

    blobList = new char [width*height+1];

    ptr_inputImg->resize(width, height);
    ptr_inputImgRed->resize(width, height);
    ptr_inputImgGreen->resize(width, height);
    ptr_inputImgBlue->resize(width, height);
    ptr_inputImgRG->resize(width, height);
    ptr_inputImgGR->resize(width, height);
    ptr_inputImgBY->resize(width, height);

    resized_flag = true;
}

/**
 * initialization of the thread 
 */
bool blobFinderThread::threadInit() {
    inputPort.open(getName("/image:i").c_str());
    edgesPort.open(getName("/edges:i").c_str());
    rgPort.open(getName("/rg:i").c_str());
    grPort.open(getName("/gr:i").c_str());
    byPort.open(getName("/by:i").c_str());

    saliencePort.open(getName("/salienceMap:o").c_str());
    outputPort3.open(getName("/imageC3:o").c_str());
    return true;
}

/**
* function called when the module is poked with an interrupt command
*/
void blobFinderThread::interrupt() {
    inputPort.interrupt();          // getName("image:i");
    edgesPort.interrupt();          // getName(edges:i);
    rgPort.interrupt();             // open(getName("rg:i"));
    grPort.interrupt();             // open(getName("gr:i"));
    byPort.interrupt();             // open(getName("by:i"));
    saliencePort.interrupt();
    outputPort3.interrupt();
}

/**
 * active loop of the thread
 */
void blobFinderThread::run() {
    //
    img = inputPort.read(false);
    if (0 != img) {
        if (!reinit_flag) {

            srcsize.height=img->height();
            srcsize.width=img->width();
            height=img->height();
            width=img->width();

            reinitialise(img->width(), img->height());
        }

        ippiCopy_8u_C3R(img->getRawImage(), img->getRowSize(), ptr_inputImg->getRawImage(), ptr_inputImg->getRowSize(), srcsize);

        bool ret1=true, ret2=true;
        ret1 = getOpponencies();
        ret2 = getPlanes(img);
        if (!ret1 || !ret2)
            return;

        tmpImage=edgesPort.read(false);
        if (tmpImage != 0)
            ippiCopy_8u_C1R(tmpImage->getRawImage(), tmpImage->getRowSize(), edges->getRawImage(), edges->getRowSize(), srcsize);

        rain(edges);
        drawAllBlobs(false);

        //salience->DrawMaxSaliencyBlob(*salience->maxSalienceBlob_img, max_tag, *tagged);
        //ippiCopy_8u_C1R(salience->maxSalienceBlob_img->getRawImage(), salience->maxSalienceBlob_img->getRowSize(), _outputImage->getRawImage(), _outputImage->getRowSize(), srcsize);

        salience->ComputeMeanColors(max_tag);
        salience->DrawMeanColorsLP(*outMeanColourLP, *ptr_tagged);
        ippiCopy_8u_C3R(outMeanColourLP->getRawImage(), outMeanColourLP->getRowSize(), _outputImage3->getRawImage(), _outputImage3->getRowSize(), srcsize);	

        if((0 != _outputImage3) && (outputPort3.getOutputCount())) { 
            outputPort3.prepare() = *(_outputImage3);
            outputPort3.write();
        }
        if((0 != outContrastLP) && (saliencePort.getOutputCount())) { 
            saliencePort.prepare() = *(outContrastLP);
            saliencePort.write();
        }        
    }
    else
        Time::delay(0.5);
}

/**
 *	releases the thread
 */
void blobFinderThread::threadRelease() {
    rgPort.close();
    grPort.close();
    byPort.close();

    outputPort3.close();
    saliencePort.close();
    inputPort.close();
}

/**
 * function that reads the ports for colour RGB opponency maps
 */
bool blobFinderThread::getOpponencies() {

    tmpImage=rgPort.read(false);
    if (tmpImage != NULL)
        ippiCopy_8u_C1R(tmpImage->getRawImage(), tmpImage->getRowSize(), ptr_inputImgRG->getRawImage(), ptr_inputImgRG->getRowSize(), srcsize);
    
    tmpImage=grPort.read(false);
    if (tmpImage != NULL)
        ippiCopy_8u_C1R(tmpImage->getRawImage(), tmpImage->getRowSize(), ptr_inputImgGR->getRawImage(), ptr_inputImgGR->getRowSize(), srcsize);
    
    tmpImage=byPort.read(false);
    if (tmpImage != NULL)
        ippiCopy_8u_C1R(tmpImage->getRawImage(), tmpImage->getRowSize(), ptr_inputImgBY->getRawImage(), ptr_inputImgBY->getRowSize(), srcsize);

    return true;
}

/**
 * LATER: proper Doxygen documentation here!
 * function that reads the ports for the RGB planes
 */
bool blobFinderThread::getPlanes(ImageOf<PixelRgb>* inputImage) {
    Ipp8u* shift[3];
    shift[0] = ptr_inputImgRed->getRawImage(); 
    shift[1] = ptr_inputImgGreen->getRawImage();
    shift[2] = ptr_inputImgBlue->getRawImage();
    ippiCopy_8u_C3P3R(inputImage->getRawImage(), inputImage->getRowSize(), shift, ptr_inputImgRed->getRowSize(), srcsize);
    return true;
}

/**
 * applies the watershed (rain falling) algorithm
 */
void blobFinderThread::rain(ImageOf<PixelMono>* edgesImage) {
    max_tag = wOperator->apply(*edgesImage, *ptr_tagged);
}

void blobFinderThread::drawAllBlobs(bool stable)
{
    // initialization of all blobs (bounding boxes of the blobs). Store blob list internally (to SalienceOperator class).
    // starts from a tagged image and builds a list of boxes.
    salience->blobCatalog(*ptr_tagged, *ptr_inputImgRG, *ptr_inputImgGR, *ptr_inputImgBY,
                          *ptr_inputImgBlue, *ptr_inputImgGreen, *ptr_inputImgRed, max_tag);

    // computes the salience of each blob (fills the rg, gr, by average values into the boxes). 
    salience->ComputeSalienceAll(max_tag, max_tag);

    // extracts the PixelBgr color of tag=1. Assuming this is the fovea?
    //PixelBgr varFoveaBlob = salience->varBlob(*ptr_tagged, *ptr_inputImgRG, *ptr_inputImgGR, *ptr_inputImgBY, 1 /* (*ptr_tagged)(0,0) */);

    // draw the fovea blob into the blobFov image? Also assuming the tag=1.
    //salience->drawFoveaBlob(*blobFov, *tagged);
    
    //memset(blobList, 0, sizeof(char)*(max_tag+1));

    // finds all pixels that have != tags from the (0,0) which is the fovea but are connected according
    // to a certain geometric neighborhood relation. Returns a list of blobtags of the connected pixels.
    //wOperator->findNeighborhood(*ptr_tagged, 0, 0, blobList);

    // it uses the varFoveaBlob (color) to compare similar pixels and the list of blobs searched earlier.
    // sets all pixels that are fovea-like to tag=1.
    //salience->fuseFoveaBlob3(*ptr_tagged, blobList, varFoveaBlob, max_tag);

    // Comment the following line to disable the elimination of invalid blob
    // salience->RemoveNonValidNoRange(max_tag, BLOB_MAXSIZE, BLOB_MINSIZE);
    salience->RemoveNonValidNoRange(max_tag, maxBLOB, minBLOB);

    PixelMono pixelRG = 0;
    PixelMono pixelGR = 0;
    PixelMono pixelBY = 0;

    // draws the saliency of all blobs into the tagged image and returns a mono image.
    int nBlobs=salience->DrawContrastLP2(*ptr_inputImgRG, *ptr_inputImgGR, *ptr_inputImgBY,
        *outContrastLP, *ptr_tagged, max_tag,
        1.0, 0.0,
        pixelRG, pixelGR, pixelBY, 255); 
}
