// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco
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
 * @file indLogMotionThread.cpp
 * @brief Implementation of the independent log-motion thread (see indLogMotionThread.h).
 */

//#include <iCub/RC_DIST_FB_logpolar_mapper.h>
#include <iCub/indLogMotionThread.h>
#include <iCub/RC_DIST_FB_logpolar_mapper.h>
#include <cstring>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;

const int sizeExtention = 10;
const int sizeVertCutting = 50;

indLogMotionThread::indLogMotionThread(){
    resized=false;
}

indLogMotionThread::~indLogMotionThread() {
}

bool indLogMotionThread::threadInit() {
    /* open ports */ 
    inPort.open(getName("/image:i").c_str());
    interInPort.open(getName("/interm:i").c_str());
    interOutPort.open(getName("/interm:o").c_str());
    outPort.open(getName("/image:o").c_str());
    return true;
}

void indLogMotionThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string indLogMotionThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void indLogMotionThread::resize(int widthp, int heightp) {
    height_orig=heightp;
    width_orig=widthp;
    height=heightp-sizeVertCutting;
    width=widthp+2*sizeExtention;
    inputImage=new ImageOf<PixelRgb>;
    inputImage->resize(width,height);
}

void indLogMotionThread::run() {
    while (isStopping() != true) {
        tmp=inPort.read(true);
        
        resize(tmp->width(),tmp->height());
        //extend the image
        ImageOf<PixelRgb>& inputImage= interOutPort.prepare();
        inputImage.resize(width,height);
        const int pxsize = inputImage.getPixelSize();
        unsigned char *d = inputImage.getPixelAddress(sizeExtention,0);
        unsigned char *s = tmp->getPixelAddress(0,sizeVertCutting);
        int tmpWidth=tmp->width();
        const int bytes = tmpWidth * pxsize;
        int dRowSize=inputImage.getRowSize();
        int sRowSize=tmp->getRowSize();
        for (int i = 0; i < height; i++) {
            memcpy(d, s, bytes);
            d += dRowSize;
            s += sRowSize;
        }

        const int px = sizeExtention * pxsize;
        //const int width = inputImage->width();
        for (int row = 0; row < height; row++) {
            memcpy (inputImage.getPixelAddress(width-sizeExtention, row),
                    inputImage.getPixelAddress(sizeExtention, row),
                    px);
            memcpy (inputImage.getPixelAddress(0, row),
                    inputImage.getPixelAddress(width-sizeExtention-sizeExtention, row),
                    px);
        }
        interOutPort.write();
        ImageOf<PixelRgb>* intermImage=interInPort.read(false);
        if(intermImage!=0) {
            ImageOf<PixelRgb>& outImage=outPort.prepare();
            outImage.resize(width_orig,height_orig);
            //send the image to the independent motion detector and gets the result
            //process the image for the output
            
            unsigned char* inputp=intermImage->getPixelAddress(sizeExtention,0);
            unsigned char* outputp=outImage.getRawImage();
            const int bytesExt = width_orig * pxsize;
            int destRowSize=outImage.getRowSize();
            int srcRowSize=inputImage.getRowSize();
            //creating the first row
            for(int x = 0;x < width_orig; x++) {
                *outputp++=0;
                *outputp++=0;
                *outputp++=0;
            }
            outputp-=destRowSize;
            //recreating what has been cut copying the fisrt line
            for(int i = 1; i < sizeVertCutting; i++) {
                memcpy(outputp+destRowSize, outputp, bytesExt);
                outputp += destRowSize;
            }
            //coping the result of the independet Motion Detector
            for (int i = 0; i < height; i++) {
                memcpy(outputp, inputp, bytesExt);
                outputp += destRowSize;
                inputp += srcRowSize;
            }
            outPort.write();
        }
    }
}

void indLogMotionThread::onStop() {
    //interrupt ports
    inPort.interrupt();
    interInPort.interrupt();
    interOutPort.interrupt();
    outPort.interrupt();
    //close ports
    inPort.close();
    interInPort.close();
    interOutPort.close();
    outPort.close();
}

void indLogMotionThread::threadRelease() {
}

