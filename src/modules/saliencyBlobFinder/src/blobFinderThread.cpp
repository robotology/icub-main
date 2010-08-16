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

const int defaultSize = 240;
const int SPIKE_COUNTS = 10;
const int THREAD_RATE = 33;

#define _inputImgRed (*(ptr_inputImgRed))
#define _inputImgGreen (*(ptr_inputImgGreen))
#define _inputImgBlue (*(ptr_inputImgBlue))
#define _inputImgRG (*(ptr_inputImgRG))
#define _inputImgGR (*(ptr_inputImgGR))
#define _inputImgBY (*(ptr_inputImgBY))
#define _tagged (*(ptr_tagged))


bool logpolarToCart(yarp::sig::ImageOf<yarp::sig::PixelRgb>& cart,const yarp::sig::ImageOf<yarp::sig::PixelRgb>& lp,lp2CartPixel *l2cTable) {
    // adjust padding.
    if (lp.getPadding() != 0) {
        int i;
        const int byte = lp.width() * sizeof(PixelRgb);
        unsigned char *d = lp.getRawImage() + byte;
        for (i = 1; i < lp.height(); i ++) {
            unsigned char *s = (unsigned char *)lp.getRow(i);
            memmove(d, s, byte);
            d += byte;
        }
    }

    RCgetCartImg (cart.getRawImage(), lp.getRawImage(), l2cTable, cart.width() * cart.height());

    // adjust padding.
    if (cart.getPadding() != 0) {
        const int byte = cart.width() * sizeof(PixelRgb);
        int i;
        for (i = cart.height()-1; i >= 1; i--) {
            unsigned char *d = cart.getRow(i);
            unsigned char *s = cart.getRawImage() + i*byte;
            memmove(d, s, byte);
        }
    }
    return true;
}

blobFinderThread::blobFinderThread(int rateThread = THREAD_RATE) : RateThread(rateThread)
{
    /* important, makes IPP single threaded! */
    ippSetNumThreads(1);
    saddleThreshold=10;
    reinit_flag=false;
    interrupted_flag=false;
    filterSpikes_flag=false;
    freetorun=false;
    resized_flag=false;
    ct=0;
    count=0;

    /* logpolar table, to be removed later */
    l2cTable=0;

    outContrastLP=new ImageOf<PixelMono>;
    outMeanColourLP=new ImageOf<PixelBgr>;

    max_boxes = new YARPBox[3];
    _procImage=new ImageOf<PixelRgb>;
    _outputImage3=new ImageOf<PixelRgb>;
    _outputImage3Merged=new ImageOf<PixelRgb>;
    _outputImage3Cart=new ImageOf<PixelRgb>;
    _outputImage=new ImageOf<PixelMono>;

    ptr_inputImg=new ImageOf<yarp::sig::PixelRgb>; 
    ptr_inputImgRed=new ImageOf<yarp::sig::PixelMono>; 
    ptr_inputImgGreen= new ImageOf<yarp::sig::PixelMono>; 
    ptr_inputImgBlue= new ImageOf<yarp::sig::PixelMono>; 
    ptr_inputImgRG= new ImageOf<yarp::sig::PixelMono>; 
    ptr_inputImgGR= new ImageOf<yarp::sig::PixelMono>; 
    ptr_inputImgBY= new ImageOf<yarp::sig::PixelMono>; 
    edges=new ImageOf<yarp::sig::PixelMono>; 
    img=new ImageOf<PixelRgb>;
    tmpImage=new ImageOf<PixelMono>;
    image_out=new ImageOf<PixelMono>;
    
    _inputImgRGS=new ImageOf<PixelMono>;
    _inputImgGRS=new ImageOf<PixelMono>;
    _inputImgBYS=new ImageOf<PixelMono>;
    
    blobFov=new ImageOf<PixelMono>;
    
    salienceBU=0.5;
    salienceTD=0.5;
    maxBLOB=4096;
    minBLOB=100;

    constantTimeGazeControl=1;
    constantTimeCentroid=1;

    targetRED=1;
    targetGREEN=1;
    targetBLUE=1;
    searchRG=0;
    searchGR=0;
    searchBY=0;
    minBoundingArea=225;
}

blobFinderThread::~blobFinderThread() {
    freeLookupTables();         // redundant

    delete[] max_boxes;
    
    delete outContrastLP;
    delete outMeanColourLP;
    delete _procImage;
    delete _outputImage3;
    delete _outputImage3Merged;
    delete _outputImage3Cart;
    delete _outputImage;
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
    delete blobFov;
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

bool blobFinderThread::freeLookupTables() {
        if (l2cTable)
            RCdeAllocateL2CTable(l2cTable);
        l2cTable = 0;
        return true;
    }

void blobFinderThread::reinitialise(int width, int height) {
    this->width=width;
    this->height=height;

    img->resize(width,height);
    edges->resize(width,height);
    tmpImage->resize(width,height);
    image_out->resize(width,height);
    
    resizeImages(width,height);

    //save look-up table to file
    int nEcc,nAng;
    nEcc=height;
    nAng=width;

    if (l2cTable == 0) {
        l2cTable = new lp2CartPixel[defaultSize*defaultSize];
        if (l2cTable == 0) {
            cerr << "logPolarLibrary: can't allocate l2c lookup tables, wrong size?" << endl;
        }
    }

    const float scaleFactor = (const float) RCcomputeScaleFactor (nEcc,nAng,defaultSize,defaultSize, 1.0);
    RCbuildL2CMap (nEcc, nAng, defaultSize, defaultSize, 1.0, scaleFactor, 0, 0, ELLIPTICAL, "./");
    RCallocateL2CTable (l2cTable, defaultSize,defaultSize, "./");
}

void blobFinderThread::resizeImages(int width, int height) {
    srcsize.height=height;
    srcsize.width=width;

    double divider = ceil(width/32.0);
    int widthstep = int(divider*32);

    ptr_tagged = new yarp::sig::ImageOf<yarp::sig::PixelInt>;
    ptr_tagged->resize(widthstep,height);
    
    this->wOperator=new WatershedOperator(true,width,height,widthstep,saddleThreshold);
    this->salience=new SalienceOperator(width,height);

    outMeanColourLP->resize(width,height);
    outContrastLP->resize(width,height);

    _procImage->resize(width,height);
    _outputImage->resize(width,height);
    _outputImage3->resize(width,height);
    _outputImage3Merged->resize(width,height);
    _outputImage3Cart->resize(defaultSize,defaultSize);

    _inputImgRGS->resize(width,height);
    _inputImgGRS->resize(width,height);
    _inputImgBYS->resize(width,height);
    blobFov->resize(width,height);

    blobList = new char [width*height+1];

    ptr_inputImg->resize(width,height);
    _inputImgRed.resize(width,height);
    _inputImgGreen.resize(width,height);
    _inputImgBlue.resize(width,height);
    _inputImgRG.resize(width,height);
    _inputImgGR.resize(width,height);
    _inputImgBY.resize(width,height);

    resized_flag=true;
}

/**
 * initialization of the thread 
 */
bool blobFinderThread::threadInit() {
    bool ret = false;
    bool ok=true;
  
    inputPort.open(getName("/image:i").c_str());
    edgesPort.open(getName("/edges:i").c_str());
    checkPort.open(getName("/check:o").c_str());

    rgPort.open(getName("/rg:i").c_str());
    grPort.open(getName("/gr:i").c_str());
    byPort.open(getName("/by:i").c_str());

    outputPort.open(getName("/image:o").c_str());
    saliencePort.open(getName("/salienceMap:o").c_str());
    outputPort3.open(getName("/imageC3:o").c_str());
    centroidPort.open(getName("/centroid:o").c_str());
    triangulationPort.open(getName("/triangulation:o").c_str());
    gazeControlPort.open(getName("/gazeControl:o").c_str());

    return true;
}

void blobFinderThread::resetFlags() {
    contrastLP_flag=false;
    meanColour_flag=false;
    blobCataloged_flag=false;
    foveaBlob_flag=false;
    colorVQ_flag=false;
    maxSaliencyBlob_flag=false;
    blobList_flag=false;
    tagged_flag=false;
    watershed_flag=false;
}

/**
* function called when the module is poked with an interrupt command
*/
void blobFinderThread::interrupt() {
    interrupted_flag=true;          // this flag must be switched before the unlock of every input port

    inputPort.interrupt();          // getName("image:i");
    edgesPort.interrupt();          // getName(edges:i);
    checkPort.interrupt();

    rgPort.interrupt();             // open(getName("rg:i"));
    grPort.interrupt();             // open(getName("gr:i"));
    byPort.interrupt();             // open(getName("by:i"));

    outputPort.interrupt();         // open(getName("image:o"));
    saliencePort.interrupt();
    outputPort3.interrupt();
    centroidPort.interrupt();       // open(getName("centroid:o"));
    triangulationPort.interrupt();  // open(getName("triangulation:o"));
    gazeControlPort.interrupt();    // open(getName("gazeControl:o"));
}

/**
 * active loop of the thread
 */
void blobFinderThread::run() {
    //
    if (!interrupted_flag) {
        ct++;
        // reading input port and extracting colour planes
        img = inputPort.read(false);
        if (0 != img) {
            if (!reinit_flag) {
                srcsize.height=img->height();
                srcsize.width=img->width();
                this->height=img->height();
                this->width=img->width();
                reinitialise(img->width(), img->height());
                reinit_flag=true;
                img->resize(this->width,this->height);    
            }

            ippiCopy_8u_C3R(img->getRawImage(), img->getRowSize(),ptr_inputImg->getRawImage(), ptr_inputImg->getRowSize(),srcsize);

            bool ret1=true,ret2=true;
            ret1=getOpponencies();
            ret1=true;
            ret2=getPlanes(img);
            if (ret1&&ret2)
                freetorun=true;
            if (!freetorun)
                return;

            bool redPlane_flag=false;
            bool greenPlane_flag=false;
            bool bluePlane_flag=false;
            bool RG_flag=false;
            bool GR_flag=false;
            bool BY_flag=false;

            bool conversion=true;
            tmpImage=edgesPort.read(false);
            if (tmpImage != 0)
                ippiCopy_8u_C1R(tmpImage->getRawImage(),tmpImage->getRowSize(),edges->getRawImage(), edges->getRowSize(),srcsize);
            rain(edges);

            this->drawAllBlobs(true);
            this->salience->DrawMaxSaliencyBlob(*salience->maxSalienceBlob_img,max_tag,*tagged);

            ippiCopy_8u_C1R(salience->maxSalienceBlob_img->getRawImage(),salience->maxSalienceBlob_img->getRowSize(),_outputImage->getRawImage(),_outputImage->getRowSize(),srcsize);
            salience->ComputeMeanColors(max_tag);
            salience->DrawMeanColorsLP(*outMeanColourLP,*tagged);
            ippiCopy_8u_C3R(this->outMeanColourLP->getRawImage(),this->outMeanColourLP->getRowSize(),_outputImage3->getRawImage(),_outputImage3->getRowSize(),srcsize);	

            outPorts();
            
            if(checkPort.getOutputCount()) {
                if (meanColour_flag) {
                    ippiAdd_8u_C3RSfs(ptr_inputImg->getRawImage(),ptr_inputImg->getRowSize(),_outputImage3->getRawImage(),_outputImage3->getRowSize(),_outputImage3Merged->getRawImage(),_outputImage3Merged->getRowSize(),srcsize,1);
                }
                else if(contrastLP_flag) {
                    ippiCopy_8u_C1R(this->outContrastLP->getRawImage(),this->outContrastLP->getRowSize(),_outputImage->getRawImage(),_outputImage->getRowSize(),srcsize);
                    Ipp8u* im_tmp[3]={_outputImage->getRawImage(),_outputImage->getRawImage(),_outputImage->getRawImage()};
                    ippiCopy_8u_P3C3R(im_tmp,_outputImage->getRowSize(),img->getRawImage(),img->getRowSize(),srcsize);
                    ippiAdd_8u_C3RSfs(ptr_inputImg->getRawImage(),ptr_inputImg->getRowSize(),img->getRawImage(),img->getRowSize(),_outputImage3Merged->getRawImage(),_outputImage3Merged->getRowSize(),srcsize,1);
                }
                else {
                    Ipp8u* im_tmp[3]={_outputImage->getRawImage(),_outputImage->getRawImage(),_outputImage->getRawImage()};
                    ippiCopy_8u_P3C3R(im_tmp,_outputImage->getRowSize(),img->getRawImage(),img->getRowSize(),srcsize);
                    ippiAdd_8u_C3RSfs(ptr_inputImg->getRawImage(),ptr_inputImg->getRowSize(),img->getRawImage(),img->getRowSize(),_outputImage3Merged->getRawImage(),_outputImage3Merged->getRowSize(),srcsize,1);
                }
                    
                logpolarToCart(*_outputImage3Cart,*_outputImage3Merged,l2cTable);
                checkPort.prepare() = *(_outputImage3Cart);
                checkPort.write();
            }
        }
    } // if (!interrupted)
}

void blobFinderThread::stop() {
    rgPort.close();
    grPort.close();
    byPort.close();

    outputPort.close();
    outputPort3.close();
    saliencePort.close();
    checkPort.close();

    inputPort.close();
    centroidPort.close();
    gazeControlPort.close();
    triangulationPort.close();
    //threadRelease();
}

/**
 *	releases the thread
 */
void blobFinderThread::threadRelease() {
    // ??
    freeLookupTables();
}

/**
 * function that reads the ports for colour RGB opponency maps
 */
bool blobFinderThread::getOpponencies() {

    tmpImage=rgPort.read(false);
    if(tmpImage!=NULL)
        ippiCopy_8u_C1R(tmpImage->getRawImage(),tmpImage->getRowSize(),ptr_inputImgRG->getRawImage(), ptr_inputImgRG->getRowSize(),srcsize);
    
    tmpImage=grPort.read(false);
    if(tmpImage!=NULL)
        ippiCopy_8u_C1R(tmpImage->getRawImage(),tmpImage->getRowSize(),ptr_inputImgGR->getRawImage(), ptr_inputImgGR->getRowSize(),srcsize);
    
    tmpImage=byPort.read(false);
    if(tmpImage!=NULL)
        ippiCopy_8u_C1R(tmpImage->getRawImage(),tmpImage->getRowSize(),ptr_inputImgBY->getRawImage(), ptr_inputImgBY->getRowSize(),srcsize);

    return true;
}

/**
 * LATER: proper Doxygen documentation here!
 * function that reads the ports for the RGB planes
 */
bool blobFinderThread::getPlanes(ImageOf<PixelRgb>* inputImage) {
    Ipp8u* shift[3];
    shift[0]=ptr_inputImgRed->getRawImage(); 
    shift[1]=ptr_inputImgGreen->getRawImage();
    shift[2]=ptr_inputImgBlue->getRawImage();
    ippiCopy_8u_C3P3R(inputImage->getRawImage(),inputImage->getRowSize(),shift,ptr_inputImgRed->getRowSize(),srcsize);
    return true;
}

bool blobFinderThread::outPorts(){ 
    if((0!=_outputImage)&&(outputPort.getOutputCount())){ 
        outputPort.prepare() = *(_outputImage);
        outputPort.write();
    }
    if((0!=_outputImage3)&&(outputPort3.getOutputCount())){ 
        outputPort3.prepare() = *(_outputImage3);
        outputPort3.write();
    }
    if((0!=outContrastLP)&&(saliencePort.getOutputCount())){ 
        saliencePort.prepare() = *(outContrastLP);
        saliencePort.write();
    }
    return true;
}

/**
 * applies the watershed (rain falling) algorithm
 */
void blobFinderThread::rain(ImageOf<PixelMono>* edgesImage) {
    max_tag=wOperator->apply(*edgesImage,_tagged);
    salience->blobCatalog(_tagged, _inputImgRG, _inputImgGR, _inputImgBY,
        _inputImgBlue, _inputImgGreen, _inputImgRed, max_tag);
        
    blobCataloged_flag=true;
    // istruction to set the ptr_tagged in the Watershed Module with the static variable _tagged
    tagged=ptr_tagged; //ptr_tagged is the pointer to _tagged
}

void blobFinderThread::drawAllBlobs(bool stable)
{
    salience->ComputeSalienceAll(this->max_tag,this->max_tag);
    // extracts the PixelBgr color of a fovea blob
    PixelBgr varFoveaBlob = salience->varBlob(*tagged, *ptr_inputImgRG, *ptr_inputImgGR, *ptr_inputImgBY, 1);

    salience->drawFoveaBlob(*blobFov, *tagged);
    //__OLD//salience.drawBlobList(blobFov, tagged, blobList, max_tag, 127);
    
    memset(blobList, 0, sizeof(char)*(max_tag+1));

    // - faster
    // - it considers also "lateral" pixels
    // - it doesn't add pixels iteratively
    wOperator->findNeighborhood(*tagged, 0, 0, blobList);
    salience->fuseFoveaBlob3(*tagged, blobList, varFoveaBlob,max_tag);

    // alternative method
    //__OLD//rain.fuseFoveaBlob(tagged, blobList, max_tag);
    //__OLD//blobList[1]=2; // so the fovea blob is eliminated by the removeBlobList
    //__OLD//salience.statBlobList(tagged, blobList, max_tag, fovBox);

    YARPBox fovBox;
    fovBox=salience->getBlobNum(1);
    //__OLD//salience.removeBlobList(blobList, max_tag);
    salience->removeFoveaBlob(*tagged);
    //__OLD//salience.updateFoveaBlob(tagged, blobList, max_tag);

    if (stable) {
        for (int i=0; i<2; i++) {
            memset(blobList, 0, sizeof(char)*(max_tag+1));
            wOperator->findNeighborhood(*tagged, 0, 0, blobList);
            //const int minBoundingArea=15*15;
            int count=salience->countSmallBlobs(*tagged, blobList, max_tag, minBoundingArea);
            //printf("Count of small blobs: %d \n",count);
            blobList[1]=0;
            salience->mergeBlobs(*tagged, blobList, max_tag, 1);
        }

        /*__OLD//while (num!=0) {
            blobList[1]=0;
            salience.mergeBlobs(tagged, blobList, max_tag, 1);
            memset(blobList, 0, sizeof(char)*max_tag);
            rain.findNeighborhood(tagged, 0, 0, blobList);
            num = salience.checkSmallBlobs(tagged, blobList, max_tag, minBoundingArea);
        }*/
    }
        
    //__OLD//salience.drawFoveaBlob(blobFov, tagged);
    //__OLD//salience.drawBlobList(blobFov, tagged, blobList, max_tag, 127);
    
    // Comment the following line to disable the elimination of invalid blob
    // salience->RemoveNonValidNoRange(max_tag, BLOB_MAXSIZE, BLOB_MINSIZE);
    salience->RemoveNonValidNoRange(max_tag,maxBLOB,minBLOB);
    
    //__OLD//salience.DrawContrastLP(rg, gr, by, tmp1, tagged, max_tag, 0, 1, 30, 42, 45); // somma coeff pos=3 somma coeff neg=-3
    //__OLD//salience.checkIOR(tagged, IORBoxes, num_IORBoxes);
    //__OLD//salience.doIOR(tagged, IORBoxes, num_IORBoxes);

    PixelMono searchTD=0;
    PixelMono pixelRG=0,pixelGR=0,pixelBY=0;
    searchRG=((targetRED-targetGREEN+255)/510)*255;
    pixelRG=searchRG;
    searchGR=((targetGREEN-targetRED+255)/510)*255;
    pixelGR=searchGR;
    PixelMono addRG=((targetRED+targetGREEN)/510)*255;
    searchBY=((targetBLUE-addRG+255)/510)*255; 
    pixelBY=searchBY;

    if(salienceBU<=0.01) {
        salienceBU=0;
    }
    if(salienceTD<=0.01) {
        salienceTD=0;
    }
    int nBlobs=salience->DrawContrastLP2(_inputImgRG, _inputImgGR, _inputImgBY,
        *outContrastLP, *tagged, max_tag,
        salienceBU, salienceTD,
        pixelRG, pixelGR, pixelBY, 255); // somma coeff pos=3 somma coeff neg=-3

    //__OLD//meanOppCol.Zero();
    //__OLD//salience.DrawMeanOpponentColorsLP(meanOppCol, tagged);

    /*__OLD//blobFinder.DrawGrayLP(tmp1, tagged, 200);
    //__OLD//ACE_OS::sprintf(savename, "./rain.ppm");
    //__OLD//YARPImageFile::Write(savename, tmp1);*/
    //__OLD//rain.tags2Watershed(tagged, oldWshed);
}
