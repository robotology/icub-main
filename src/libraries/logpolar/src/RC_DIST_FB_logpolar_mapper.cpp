/*
 *  logpolar mapper library. subsamples rectangular images into logpolar.
 *
 *  Copyright (C) 2005-2010 The RobotCub Consortium
 *  Author: Fabio Berton & Giorgio Metta
 *  RobotCub Consortium, European Commission FP6 Project IST-004370
 *  email:   fberton@dist.unige.it, giorgio.metta@iit.it
 *  website: www.robotcub.org
 *
 *  Permission is granted to copy, distribute, and/or modify this program 
 *  under the terms of the GNU General Public License, version 2 or any later
 *  version published by the Free Software Foundation. A copy of the license can be 
 *  found at http://www.robotcub.org/icub/license/gpl.txt
 *
 *  This program is distributed in the hope that it will be useful, but WITHOUT ANY
 *  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
 *  PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 */

/**
 * \file rc_dist_fb_logpolar_mapper.cpp 
 * \brief Implementation of the logpolar class.
 */

#include <iCub/RC_DIST_FB_logpolar_mapper.h>
#include <yarp/sig/IplImage.h>

#include <iostream>
#include <math.h>
#include <string.h>

using namespace iCub::logpolar;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

//
bool iCub::logpolar::subsampleFovea(yarp::sig::ImageOf<yarp::sig::PixelRgb>& dst, const yarp::sig::ImageOf<yarp::sig::PixelRgb>& src) {
    //
    if (dst.width()!=dst.height() || dst.width()>src.width() || dst.height()>src.height()) {
        cerr << "subsampleFovea: can't perform the requested operation" << endl;
        return false;
    }

    const int fov = dst.width();
    const int offset = src.height()/2-fov/2;
    const int col = src.width()/2-fov/2;
    const int bytes = fov*sizeof(PixelRgb);

    for (int i = 0; i < fov; i++) {
        unsigned char *s = (unsigned char *)src.getRow(i+offset)+col*sizeof(PixelRgb);
        unsigned char *d = dst.getRow(i);
        memcpy(d, s, bytes);
    }
    return true;
}

//
bool iCub::logpolar::replicateBorderLogpolar(yarp::sig::Image& dest, const yarp::sig::Image& src, int maxkernelsize) {
    //
    if (src.width()+2*maxkernelsize != dest.width() || src.height()+maxkernelsize != dest.height()) {
        std::cerr << "ReplicateBorderLogpolar: images aren't correctly sized for the operation" << std::endl;
        return false;
    }

    if (src.width()%2) {
        std::cerr << "ReplicateBorderLogpolar: image width must be an even number" << std::endl;
        return false;
    }

    // copy of the image
    const int pxsize = dest.getPixelSize();
    unsigned char *d = dest.getPixelAddress (maxkernelsize, maxkernelsize);
    unsigned char *s = src.getRawImage();
    const int bytes = src.width() * pxsize;

    for (int i = 0; i < src.height(); i++) {
        memcpy(d, s, bytes);
        d += dest.getRowSize();
        s += src.getRowSize();
    }

    // memcpy of the horizontal fovea lines (rows) 
    const int sizeBlock = src.width() / 2;
    for(int i = 0; i < maxkernelsize; i++) {
        memcpy(dest.getPixelAddress(sizeBlock+maxkernelsize, maxkernelsize-1-i),
                dest.getPixelAddress(maxkernelsize, maxkernelsize+i),
                sizeBlock * pxsize);
        memcpy(dest.getPixelAddress(maxkernelsize, maxkernelsize-1-i),
                dest.getPixelAddress(sizeBlock+maxkernelsize, maxkernelsize+i),
                sizeBlock * pxsize);
    }

    // copy of the block adjacent angular positions (columns)
    const int px = maxkernelsize * pxsize;
    const int width = dest.width();

    for (int row = 0; row < dest.height(); row++) {
        memcpy (dest.getPixelAddress(width-maxkernelsize, row),
                dest.getPixelAddress(maxkernelsize, row),
                px);
        memcpy (dest.getPixelAddress(0, row),
                dest.getPixelAddress(width-maxkernelsize-maxkernelsize, row),
                px);
    }

    return true;
}


// implementation of the ILogpolarAPI interface.
bool logpolarTransform::allocLookupTables(int mode, int necc, int nang, int w, int h, double overlap) {
    //
    if (allocated()) {
        // check, return false in case size has changed. need to manually free and recompute maps.
        if (mode != mode_ || necc != necc_ || nang != nang_ || w != width_ || h != height_ || overlap != overlap_) {
            cerr << "logpolarTransform: new size differ from previously allocated maps" << endl;
            return false;
        }

        cerr << "logpolarTransform: tried a reallocation of already configured maps, no action taken" << endl;
        return true;
    }

    necc_ = necc;
    nang_ = nang;
    width_ = w;
    height_ = h;
    overlap_ = overlap;
    mode_ = mode;
    const double scaleFact = RCcomputeScaleFactor ();    
    
    if (c2lTable == 0 && (mode & C2L)) {
        c2lTable = new cart2LpPixel[necc*nang];
        if (c2lTable == 0) {
            cerr << "logpolarTransform: can't allocate c2l lookup tables, wrong size?" << endl;
            return false;
        }

        RCbuildC2LMap (scaleFact, ELLIPTICAL, PAD_BYTES(w*3, YARP_IMAGE_ALIGN));
    }

    if (l2cTable == 0 && (mode & L2C)) {
        l2cTable = new lp2CartPixel[w*h];
        if (l2cTable == 0) {
            cerr << "logPolarLibrary: can't allocate l2c lookup tables, wrong size?" << endl;
            return false;
        }

        RCbuildL2CMap (scaleFact, 0, 0, ELLIPTICAL, PAD_BYTES(nang*3, YARP_IMAGE_ALIGN));
    }
    return true;
}

bool logpolarTransform::freeLookupTables() {
    if (c2lTable)
        RCdeAllocateC2LTable ();
    if (l2cTable)
        RCdeAllocateL2CTable ();
    return true;
}

bool logpolarTransform::cartToLogpolar(yarp::sig::ImageOf<yarp::sig::PixelRgb>& lp, 
                                       const yarp::sig::ImageOf<yarp::sig::PixelRgb>& cart) {
    if (!(mode_ & C2L)) {
        cerr << "logPolarLibrary: conversion to logpolar called with wrong mode set" << endl;
        return false;
    }

    // LATER: assert whether lp & cart are effectively nang * necc as the c2lTable requires.
    RCgetLpImg (lp.getRawImage(), (unsigned char *)cart.getRawImage(), c2lTable, lp.getPadding());
    return true;
}

bool logpolarTransform::logpolarToCart(yarp::sig::ImageOf<yarp::sig::PixelRgb>& cart,
                            const yarp::sig::ImageOf<yarp::sig::PixelRgb>& lp) {
    if (!(mode_ & L2C)) {
        cerr << "logPolarLibrary: conversion to cartesian called with wrong mode set" << endl;
        return false;
    }

    // LATER: assert whether lp & cart are effectively of the correct size.
    RCgetCartImg (cart.getRawImage(), lp.getRawImage(), l2cTable, cart.getPadding());

    return true;
}

// internal implementation of the logpolarTransform class.

inline double __max64f (double x, double y) {
    return (x > y) ? x : y;
}

void logpolarTransform::RCdeAllocateC2LTable ()
{
    if (c2lTable) {
        delete[] c2lTable[0].position; // iweight is contiguous to position.
        delete[] c2lTable;
    }
    c2lTable = 0;
}

void logpolarTransform::RCdeAllocateL2CTable ()
{
    if (l2cTable) {
        delete[] l2cTable[0].position;
        delete[] l2cTable;
    }
    l2cTable = 0;
}

double logpolarTransform::RCgetLogIndex ()
{
    double logIndex;
    logIndex = (1.0 + sin (PI / nang_)) / (1.0 - sin (PI / nang_));
    return logIndex;
}

double logpolarTransform::RCcomputeScaleFactor ()
{
    double maxRad;
    double receptFieldRadius;
    double r0;
    double lambda;
    int fov;
	int cSize = width_;
	
	if (width_ > height_)
		cSize = height_;

    double totalRadius;
    double angle = (2.0 * PI / nang_);   // Angular size of one pixel
    double sinus = sin (angle / 2.0);

    lambda = (1.0 + sinus) / (1.0 - sinus);
    fov = (int) (lambda / (lambda - 1));
    r0 = 1.0 / (pow (lambda, fov) * (lambda - 1));

    maxRad = pow (lambda, necc_ - 1) * (r0 + 0.5 / pow (lambda, fov));

    receptFieldRadius = maxRad * 2.0 * sinus * (overlap_ + 1.0) / (2.0);
    totalRadius = maxRad + receptFieldRadius;
    totalRadius = (cSize / 2) / totalRadius;
    return totalRadius;
}

int logpolarTransform::RCbuildC2LMap (double scaleFact, int mode, int padding)
{
    // store map in c2lTable which is supposedly already allocated (while the internal arrays are allocated on the fly).

    if (overlap_ <= -1.0) {
        cerr << "logpolarTransform: overlap must be greater than -1" << endl;
        return 1;
    }

    int cSize = width_;

    if (width_ > height_)
        cSize = height_;

    const double precision = 10.0;
    double angle = (2.0 * PI / nang_);   // Angular size of one pixel
    double sinus = sin (angle / 2.0);
    double tangent = sinus / cos (angle / 2.0);

    int fov;
    int lim;

    double lambda;              // Log Index
    double firstRing;           // Diameter of the receptive fields in the first ring when overlap is 0
    double *currRad;            // Distance of the center of the current ring RF's from the center of the mapping
    double *nextRad;            // Distance of the center of the next ring's RF's from the center of the mapping
    double r0;                  // lower limit of RF0 in the "pure" log polar mapping

    double x0, y0;
    double locX, locY, locRad;
    double *radii;
    double *tangaxis;
    double *radialaxis;
    double *focus;
    double *radii2;
    double A;
    double L;
    double step;
    double F0x, F0y, F1x, F1y;
    double maxaxis;

    int rho, theta, j, sz;

    double *sintable;
    double *costable;
    int intx, inty;
    bool found;
    int mapsize;
    float *weight;

    // intiialization starts more or less here.
    lambda = (1.0 + sinus) / (1.0 - sinus);
    fov = (int) (lambda / (lambda - 1));
    firstRing = (1.0 / (lambda - 1)) - (int) (1.0 / (lambda - 1));
    r0 = 1.0 / (pow (lambda, fov) * (lambda - 1));

    // main table pointer (temporary).
    cart2LpPixel *table = c2lTable;

    // temporary.
    tangaxis = new double[necc_];
    radialaxis = new double[necc_];
    focus = new double[necc_];
    radii2 = new double[necc_];
    currRad = new double[necc_];
    nextRad = new double[necc_];

    if ((tangaxis == 0) || (radialaxis == 0) || (focus == 0) || (radii2 == 0) || (currRad == 0) || (nextRad == 0))
        goto C2LAllocError;

    /************************
     * RF's size Computation *
     ************************/

    for (rho = 0; rho < necc_; rho++) {
        if (rho < fov)
            if (rho == 0) {
                currRad[rho] = firstRing * 0.5;
                nextRad[rho] = firstRing;
            }
            else {
                currRad[rho] = rho + firstRing - 0.5;
                nextRad[rho] = currRad[rho] + 1.0;
            }
        else {
            currRad[rho] = pow (lambda, rho) * (r0 + 0.5 / pow (lambda, fov));
            nextRad[rho] = lambda * currRad[rho];
        }
        tangaxis[rho] =
            scaleFact * 2.0 * currRad[rho] * sinus * (overlap_ + 1.0) / (2.0);

        radialaxis[rho] =
            scaleFact * (nextRad[rho] - currRad[rho]) * (overlap_ + 1.0);

        if (rho < fov)
            radialaxis[rho] /= 2.0;
        else
            radialaxis[rho] /= (1.0 + lambda);

        if ((rho < fov) && (mode == ELLIPTICAL)) {
            A = radialaxis[rho] * radialaxis[rho];
            L = scaleFact * currRad[rho] * (overlap_ + 1.0);
            L = L * L;
            tangaxis[rho] = tangent * sqrt (L - A);
        }
    }
    radialaxis[0] = 0.5 * scaleFact * (firstRing) * (overlap_ + 1.0);

    if (mode == RADIAL)
        radii = radialaxis;
    else
        radii = tangaxis;

    for (rho = 0; rho < necc_; rho++) {
        if (mode != ELLIPTICAL)
            focus[rho] = 0;
        else {
            focus[rho] =
                -sqrt (fabs (radialaxis[rho] * radialaxis[rho] -
                             tangaxis[rho] * tangaxis[rho]));
        }

        if (tangaxis[rho] >= radialaxis[rho])
            focus[rho] = -focus[rho];

        radii2[rho] = radii[rho] * radii[rho];
    }

    sintable = new double[nang_];
    costable = new double[nang_];

    if ((sintable == 0) || (costable == 0))
        goto C2LAllocError;

    for (j = 0; j < nang_; j++) {
        sintable[j] = sin (angle * (j + 0.5));
        costable[j] = cos (angle * (j + 0.5));
    }

    // compute overall table size for contiguous allocation (position & weigth).
    sz = 0;
    for (rho = 0; rho < necc_; rho++) {
        if ((mode == RADIAL) || (mode == TANGENTIAL))
            lim = (int) (radii[rho] + 1.5);
        else
            lim = (int) (__max64f (tangaxis[rho], radialaxis[rho]) + 1.5);

        step = lim / precision;
        if (step > 1) step = 1;

        mapsize = (int) (precision * lim * precision * lim + 1);
        sz += (mapsize * nang_);
    }

    table->position = new int[sz * 2];
    if (table->position == 0)
        goto C2LAllocError;
    table->iweight = table->position + sz;

    for (rho = 0; rho < necc_; rho++) {
        if ((mode == RADIAL) || (mode == TANGENTIAL))
            lim = (int) (radii[rho] + 1.5);
        else
            lim = (int) (__max64f (tangaxis[rho], radialaxis[rho]) + 1.5);

        step = lim / precision;

        if (step > 1)
            step = 1;

        mapsize = (int) (precision * lim * precision * lim + 1);
        weight = new float[mapsize];
        if (weight == 0)
            goto C2LAllocError;

        for (theta = 0; theta < nang_; theta++) {
            //
            memset (table->position, 0, mapsize);
            memset (weight, 0, mapsize);

            x0 = scaleFact * currRad[rho] * costable[theta];
            y0 = scaleFact * currRad[rho] * sintable[theta];

            if (focus[rho] >= 0) {
                F0x = x0 - focus[rho] * sintable[theta];
                F0y = y0 + focus[rho] * costable[theta];
                F1x = x0 + focus[rho] * sintable[theta];
                F1y = y0 - focus[rho] * costable[theta];
                maxaxis = tangaxis[rho];
            }
            else {
                F0x = x0 - focus[rho] * costable[theta];
                F0y = y0 - focus[rho] * sintable[theta];
                F1x = x0 + focus[rho] * costable[theta];
                F1y = y0 + focus[rho] * sintable[theta];
                maxaxis = radialaxis[rho];
            }

            if ((mode == RADIAL) || (mode == TANGENTIAL))
                maxaxis = radii[rho];

            for (locX = (x0 - lim); locX <= (x0 + lim); locX += step)
                for (locY = (y0 - lim); locY <= (y0 + lim); locY += step) {

                    intx = (int) (locX + width_ / 2);
                    inty = (int) (locY + height_ / 2);

                    locRad =
                        sqrt ((locX - F0x) * (locX - F0x) +
                              (locY - F0y) * (locY - F0y));
                    locRad +=
                        sqrt ((locX - F1x) * (locX - F1x) +
                              (locY - F1y) * (locY - F1y));

                    if (locRad < 2 * maxaxis) {
                        if ((inty < height_) && (inty >= 0)) {
                            if ((intx < width_) && (intx >= 0)) {
                                found = false;
                                j = 0;
                                while ((j < mapsize) && (table->position[j] != 0)) {
                                    //
                                    if (table->position[j] == (3 * (inty * (width_ + padding) + intx))) {
                                        weight[j]++;
                                        found = true;
                                        j = mapsize;
                                    }
                                    j++;
                                }

                                if (!found)
                                    for (j = 0; j < mapsize; j++) {
                                        if (table->position[j] == 0) {
                                            table->position[j] = 3 * (inty * (width_ + padding) + intx);
                                            weight[j]++;
                                            break;
                                        }
                                    }
                            }
                        }
                    }
                }

            for (j = 0; j < mapsize; j++)
                if (weight[j] == 0)
                    break;

            table->divisor = j;

            float sum = 0.0;
            int k;
            for (k = 0; k < j; k++)
                sum += weight[k];

            for (k = 0; k < j; k++) {
                weight[k] = weight[k] / sum;
                table->iweight[k] = (int) (weight[k] * 65536.0);
            } 

            if (theta != nang_-1 || rho != necc_-1) {
                table[1].position = table->position + mapsize;
                table[1].iweight = table->iweight + mapsize;
            }
            table++;
        }

        delete[] weight;    // :(
    }

    // clean up temporaries.
    if (tangaxis) delete[] tangaxis;
    if (radialaxis) delete[] radialaxis;
    if (focus) delete[] focus;
    if (radii2) delete[] radii2;
    if (currRad) delete[] currRad;
    if (nextRad) delete[] nextRad;
    if (sintable) delete[] sintable;
    if (costable) delete[] costable;
    return 0;

C2LAllocError:
    // clean up temporaries.
    if (tangaxis) delete[] tangaxis;
    if (radialaxis) delete[] radialaxis;
    if (focus) delete[] focus;
    if (radii2) delete[] radii2;
    if (currRad) delete[] currRad;
    if (nextRad) delete[] nextRad;
    if (sintable) delete[] sintable;
    if (costable) delete[] costable;
    cerr << "logpolarTransform: memory allocation issue, no tables generated" << endl;
    return 2;
}

void logpolarTransform::RCgetLpImg (unsigned char *lpImg, unsigned char *cartImg, cart2LpPixel * Table, int padding)
{
    int r[3];
    int t = 0;

    unsigned char *img = lpImg;

    for (int i = 0; i < necc_; i++, img+=padding) {
        for (int j = 0; j < nang_; j++) {
            r[0] = r[1] = r[2] = 0;
            t = 0;

            const int div = Table->divisor;
            int *pos = Table->position;
            int *w = Table->iweight;
            for (int i = 0; i < div; i++, pos++, w++) {
                int *d = r;
                unsigned char *in = &cartImg[*pos];
                *d++ += *in++ * *w;
                *d++ += *in++ * *w;
                *d += *in * *w;
                t += *w;
            }

            *img++ = (unsigned char)(r[0] / t);
            *img++ = (unsigned char)(r[1] / t);
            *img++ = (unsigned char)(r[2] / t);

            Table++;
        }
    }
}

void logpolarTransform::RCgetCartImg (unsigned char *cartImg, unsigned char *lpImg, lp2CartPixel * Table, int padding)
{
    int k, i, j;
    int tempPixel[3];
    unsigned char *img = cartImg;

    for (k = 0; k < height_; k++, img += padding) {
        for (j = 0; j < width_; j++) {
            tempPixel[0] = 0;
            tempPixel[1] = 0;
            tempPixel[2] = 0;

            if (Table->iweight != 0) {
                for (i = 0; i < Table->iweight; i++) {
                    int *d = tempPixel;
                    unsigned char *lp = &lpImg[Table->position[i]];
                    *d++ += *lp++;
                    *d++ += *lp++;
                    *d += *lp;
                }

                *img++ = tempPixel[0] / Table->iweight;
                *img++ = tempPixel[1] / Table->iweight;
                *img++ = tempPixel[2] / Table->iweight;
            }
            else {
                *img++ = 0;
                *img++ = 0;
                *img++ = 0;
            }

            Table++;
        }
    }
}

// inverse logpolar.
int logpolarTransform::RCbuildL2CMap (double scaleFact, int hOffset, int vOffset, int mode, int padding)
{
    if (overlap_ <= -1.0) {
        cerr << "logpolarTransform: overlap must be greater than -1" << endl;
        return 1;
    }

	int cSize = width_;
	
	if (width_ > height_)
		cSize = height_;

    double angle = (2.0 * PI / nang_);   // Angular size of one pixel
    double sinus = sin (angle / 2.0);
    double tangent = sinus / cos (angle / 2.0);
    int fov;                    // Number of rings in fovea
    int lim;

    double lambda;              // Log Index
    double firstRing;           // Diameter of the receptive fields in the first ring when overlap is 0
    double *currRad;            // Distance of the center of the current ring RF's from the center of the mapping
    double *nextRad;            // Distance of the center of the next ring's RF's from the center of the mapping
    double r0;                  // lower limit of RF0 in the "pure" log polar mapping

    double x0, y0;              // Cartesian Coordinates
    double locX, locY, locRad;
    double *radii;
    double *tangaxis;
    double *radialaxis;
    double *focus;
    double *radii2;
    double A;
    double L;
    int *partCtr;
    double step;
    double F0x, F0y, F1x, F1y;
    double maxaxis;

    int rho, theta, j, memSize;
    bool found;
    const double precision = 10.0;

    // main table pointer (temporary).
    lp2CartPixel *table = l2cTable;

    // temporary counter (per pixel).
    partCtr = new int[width_ * height_];
    if (partCtr == 0)
        goto L2CAllocError;

    memset (partCtr, 0, width_ * height_ * sizeof (int));

    lambda = (1.0 + sinus) / (1.0 - sinus);
    fov = (int) (lambda / (lambda - 1));
    firstRing = (1.0 / (lambda - 1)) - (int) (1.0 / (lambda - 1));
    r0 = 1.0 / (pow (lambda, fov) * (lambda - 1));

    tangaxis = new double[necc_];
    radialaxis = new double[necc_];
    focus = new double[necc_];
    radii2 = new double[necc_];
    currRad = new double[necc_];
    nextRad = new double[necc_];

    if ((tangaxis == 0) || (radialaxis == 0) || (focus == 0) || (radii2 == 0) || (currRad == 0) || (nextRad == 0))
        goto L2CAllocError;

    /************************
     * RF's size Computation *
     ************************/

    for (rho = 0; rho < necc_; rho++) {
        if (rho < fov)
            if (rho == 0) {
                currRad[rho] = firstRing * 0.5;
                nextRad[rho] = firstRing;
            }
            else {
                currRad[rho] = rho + firstRing - 0.5;
                nextRad[rho] = currRad[rho] + 1.0;
            }
        else {
            currRad[rho] = pow (lambda, rho) * (r0 + 0.5 / pow (lambda, fov));
            nextRad[rho] = lambda * currRad[rho];
        }
        
        tangaxis[rho] =
            scaleFact * 2.0 * currRad[rho] * sinus * (overlap_ + 1.0) / (2.0);

        radialaxis[rho] =
            scaleFact * (nextRad[rho] - currRad[rho]) * (overlap_ + 1.0);

        if (rho < fov)
            radialaxis[rho] /= 2.0;
        else
            radialaxis[rho] /= (1.0 + lambda);

        if ((rho < fov) && (mode == ELLIPTICAL))
        {
            A = radialaxis[rho] * radialaxis[rho];
            L = scaleFact * currRad[rho] * (overlap_ + 1.0);
            L = L * L;
            tangaxis[rho] = tangent * sqrt (L - A);
        }
    }

    radialaxis[0] = 0.5 * scaleFact * (firstRing) * (overlap_ + 1.0);

    if (mode == RADIAL)
        radii = radialaxis;
    else
        radii = tangaxis;

    for (rho = 0; rho < necc_; rho++) {
        if (mode != ELLIPTICAL)
            focus[rho] = 0;
        else {
            focus[rho] =
                -sqrt (fabs (radialaxis[rho] * radialaxis[rho] -
                             tangaxis[rho] * tangaxis[rho]));
        }
        if (tangaxis[rho] >= radialaxis[rho])
            focus[rho] = -focus[rho];

        radii2[rho] = radii[rho] * radii[rho];
    }
    
    double *sintable;
    double *costable;
    int intx, inty;

    sintable = new double[nang_];
    costable = new double[nang_];
    if ((sintable == 0) || (costable == 0))
        goto L2CAllocError;

    for (j = 0; j < nang_; j++) {
        sintable[j] = sin (angle * (j + 0.5));  // Angular positions of the centers of the RF's
        costable[j] = cos (angle * (j + 0.5));
    }

    memSize = 0;
    for (rho = 0; rho < necc_; rho++) {
        //
        if ((mode == RADIAL) || (mode == TANGENTIAL))
            lim = (int) (radii[rho] + 1.5);
        else
            lim = (int) (__max64f (tangaxis[rho], radialaxis[rho]) + 1.5);

        step = lim / precision;
        if (step > 1)
            step = 1;

        for (theta = 0; theta < nang_; theta++) {
            //
            x0 = scaleFact * currRad[rho] * costable[theta];
            y0 = scaleFact * currRad[rho] * sintable[theta];

            if (focus[rho] >= 0) {
                F0x = x0 - focus[rho] * sintable[theta];
                F1x = x0 + focus[rho] * sintable[theta];
                F0y = y0 + focus[rho] * costable[theta];
                F1y = y0 - focus[rho] * costable[theta];
                maxaxis = tangaxis[rho];
            }
            else {
                F0x = x0 - focus[rho] * costable[theta];
                F1x = x0 + focus[rho] * costable[theta];
                F0y = y0 - focus[rho] * sintable[theta];
                F1y = y0 + focus[rho] * sintable[theta];
                maxaxis = radialaxis[rho];
            }
            if ((mode == RADIAL) || (mode == TANGENTIAL))
                maxaxis = radii[rho];

            for (locX = (x0 - lim); locX <= (x0 + lim); locX += step)
                for (locY = (y0 - lim); locY <= (y0 + lim); locY += step) {
                    //
                    intx = (int) (locX + width_ / 2);
                    inty = (int) (locY + height_ / 2);

                    locRad =
                        sqrt ((locX - F0x) * (locX - F0x) +
                              (locY - F0y) * (locY - F0y));
                    locRad +=
                        sqrt ((locX - F1x) * (locX - F1x) +
                              (locY - F1y) * (locY - F1y));

                    if (locRad < 2 * maxaxis)
                        if ((inty + vOffset < height_) && (inty + vOffset >= 0))
                            if ((intx + hOffset < width_)
                                && (intx + hOffset >= 0)) {
                                //
                                partCtr[(inty + vOffset) * width_ + intx + hOffset]++;
                                memSize ++;
                            }
                }
        }
    }

    table->position = new int[memSize]; // contiguous allocation.
    if (table->position == 0)
        goto L2CAllocError;
    memset(table->position, -1, sizeof(int) * memSize);
    table->iweight = 0;

    for (j = 1; j < width_ * height_; j++) {
        table[j].position = table[j-1].position + partCtr[j-1];
        table[j].iweight = 0;
    }

    for (rho = 0; rho < necc_; rho++) {
        if ((mode == RADIAL) || (mode == TANGENTIAL))
            lim = (int) (radii[rho] + 1.5);
        else
            lim = (int) (__max64f (tangaxis[rho], radialaxis[rho]) + 1.5);

        step = lim / precision;

        if (step > 1)
            step = 1;

        for (theta = 0; theta < nang_; theta++) {
            //
            x0 = scaleFact * currRad[rho] * costable[theta];
            y0 = scaleFact * currRad[rho] * sintable[theta];

            if (focus[rho] >= 0) {
                F0x = x0 - focus[rho] * sintable[theta];
                F0y = y0 + focus[rho] * costable[theta];
                F1x = x0 + focus[rho] * sintable[theta];
                F1y = y0 - focus[rho] * costable[theta];
                maxaxis = tangaxis[rho];
            }
            else {
                F0x = x0 - focus[rho] * costable[theta];
                F0y = y0 - focus[rho] * sintable[theta];
                F1x = x0 + focus[rho] * costable[theta];
                F1y = y0 + focus[rho] * sintable[theta];
                maxaxis = radialaxis[rho];
            }
            if ((mode == RADIAL) || (mode == TANGENTIAL))
                maxaxis = radii[rho];

            for (locX = (x0 - lim); locX <= (x0 + lim); locX += step)
                for (locY = (y0 - lim); locY <= (y0 + lim); locY += step) {
                    intx = (int) (locX + width_ / 2);
                    inty = (int) (locY + height_ / 2);

                    locRad =
                        sqrt ((locX - F0x) * (locX - F0x) +
                              (locY - F0y) * (locY - F0y));
                    locRad +=
                        sqrt ((locX - F1x) * (locX - F1x) +
                              (locY - F1y) * (locY - F1y));

                    if (locRad < 2 * maxaxis)
                        if ((inty + vOffset < height_) && (inty + vOffset >= 0))
                            if ((intx + hOffset < width_) && (intx + hOffset >= 0)) {
                                found = false;

                                for (j = 0; j < partCtr[(inty + vOffset) * width_ + intx + hOffset]; j++) {
                                    if (table [(inty + vOffset) * width_ + intx + hOffset].position[j] == 3 * (rho * nang_ + theta) + (padding * rho)) {
                                        found = true;
                                        break;
                                    }
                                }

                                if (!found) {
                                    table[(inty + vOffset) * width_ + intx + hOffset].position[table [(inty + vOffset) * width_ + intx + hOffset].iweight] = 3 * (rho * nang_ + theta) + (padding * rho);
                                    table[(inty + vOffset) * width_ + intx + hOffset].iweight++;
                                }
                            }
                }
        }
    }

    if (tangaxis) delete[] tangaxis;
    if (radialaxis) delete[] radialaxis;
    if (focus) delete[] focus;
    if (radii2) delete[] radii2;
    if (currRad) delete[] currRad;
    if (nextRad) delete[] nextRad;
    if (sintable) delete[] sintable;
    if (costable) delete[] costable;
    if (partCtr) delete [] partCtr;
    return 0;

L2CAllocError:
    if (tangaxis) delete[] tangaxis;
    if (radialaxis) delete[] radialaxis;
    if (focus) delete[] focus;
    if (radii2) delete[] radii2;
    if (currRad) delete[] currRad;
    if (nextRad) delete[] nextRad;
    if (sintable) delete[] sintable;
    if (costable) delete[] costable;
    if (partCtr) delete [] partCtr;
    cerr << "logpolarTransform: memory allocation issue, no tables generated" << endl;
    return 2;
}


//
// leftovers beyond this point
// LATER: remove!
//
#if 0
void RCreconstructColor (unsigned char *color, unsigned char *grey, int width, int height)
{
    int x, y;

    for (y = 1; y < height - 1; y += 2) //green1
        for (x = 2; x < width; x += 2)
        {
            color[3 * (y * width + x) + 0] =
                (grey[(y - 1) * width + x] + grey[(y + 1) * width + x]) / 2;
            color[3 * (y * width + x) + 1] = grey[y * width + x];
            color[3 * (y * width + x) + 2] =
                (grey[(y) * width + x - 1] + grey[(y) * width + x + 1]) / 2;
        }
    for (y = 2; y < height; y += 2) //green2
        for (x = 1; x < width - 1; x += 2)
        {
            color[3 * (y * width + x) + 0] =
                (grey[(y) * width + x - 1] + grey[(y) * width + x + 1]) / 2;
            color[3 * (y * width + x) + 1] = grey[y * width + x];
            color[3 * (y * width + x) + 2] =
                (grey[(y - 1) * width + x] + grey[(y + 1) * width + x]) / 2;
        }

    for (y = 2; y < height; y += 2) //red
        for (x = 2; x < width; x += 2)
        {
            color[3 * (y * width + x) + 0] = grey[y * width + x];
            color[3 * (y * width + x) + 1] =
                (grey[(y - 1) * width + x] + grey[(y + 1) * width + x] +
                 grey[y * width + x - 1] + grey[y * width + x + 1]) / 4;
            color[3 * (y * width + x) + 2] =
                (grey[(y - 1) * width + x - 1] +
                 grey[(y - 1) * width + x + 1] + grey[(y + 1) * width + x -
                                                      1] + grey[(y +
                                                                 1) * width +
                                                                x + 1]) / 4;
        }
    for (y = 1; y < height - 1; y += 2) //blue
        for (x = 1; x < width - 1; x += 2)
        {
            color[3 * (y * width + x) + 0] =
                (grey[(y - 1) * width + x - 1] +
                 grey[(y - 1) * width + x + 1] + grey[(y + 1) * width + x -
                                                      1] + grey[(y +
                                                                 1) * width +
                                                                x + 1]) / 4;
            color[3 * (y * width + x) + 1] =
                (grey[(y - 1) * width + x] + grey[(y + 1) * width + x] +
                 grey[y * width + x - 1] + grey[y * width + x + 1]) / 4;
            color[3 * (y * width + x) + 2] = grey[y * width + x];
        }

    //Borders
    for (x = 1; x < width - 1; x += 2)
    {
        color[3 * (x) + 0] = (grey[x - 1] + grey[x + 1]) / 2;
        color[3 * (x) + 1] = grey[x];
        color[3 * (x) + 2] = grey[width + x];

        color[3 * ((height - 1) * width + x) + 0] =
            (grey[(height - 2) * width + x - 1] +
             grey[(height - 2) * width + x + 1]) / 2;
        color[3 * ((height - 1) * width + x) + 1] =
            (grey[(height - 2) * width + x] +
             grey[(height - 1) * width + x - 1] + grey[(height - 1) * width +
                                                       x + 1]) / 3;
        color[3 * ((height - 1) * width + x) + 2] =
            grey[(height - 1) * width + x];
    }
    for (x = 2; x < width; x += 2)
    {
        color[3 * (x) + 0] = grey[x];
        color[3 * (x) + 1] =
            (grey[width + x] + grey[x - 1] + grey[x + 1]) / 3;
        color[3 * (x) + 2] = (grey[width + x - 1] + grey[width + x + 1]) / 2;

        color[3 * ((height - 1) * width + x) + 0] =
            grey[(height - 2) * width + x];
        color[3 * ((height - 1) * width + x) + 1] =
            grey[(height - 1) * width + x];
        color[3 * ((height - 1) * width + x) + 2] =
            (grey[(height - 1) * width + x - 1] +
             grey[(height - 1) * width + x + 1]) / 2;
    }
    for (y = 1; y < height - 1; y += 2)
    {
        color[3 * (y * width) + 0] =
            (grey[(y - 1) * width] + grey[(y + 1) * width]) / 2;
        color[3 * (y * width) + 1] = grey[y * width];
        color[3 * (y * width) + 2] =
            (grey[(y) * width + width - 1] + grey[(y) * width + 1]) / 2;

        color[3 * (y * width + width - 1) + 0] =
            (grey[(y - 1) * width + width - 2] + grey[(y - 1) * width] +
             grey[(y + 1) * width + width - 2] + grey[(y + 1) * width]) / 4;
        color[3 * (y * width + width - 1) + 1] =
            (grey[(y - 1) * width + width - 1] +
             grey[(y + 1) * width + width - 1] + grey[y * width + width - 2] +
             grey[y * width]) / 4;
        color[3 * (y * width + width - 1) + 2] = grey[y * width + width - 1];
    }
    for (y = 2; y < height; y += 2)
    {
        color[3 * (y * width) + 0] = grey[y * width];
        color[3 * (y * width) + 1] =
            (grey[(y - 1) * width] + grey[(y + 1) * width] +
             grey[y * width + width - 1] + grey[y * width + 1]) / 4;
        color[3 * (y * width) + 2] =
            (grey[(y - 1) * width + width - 1] + grey[(y - 1) * width + 1] +
             grey[(y + 1) * width + width - 1] + grey[(y + 1) * width +
                                                      1]) / 4;

        color[3 * (y * width + width - 1) + 0] =
            (grey[(y) * width + width - 2] + grey[(y) * width]) / 2;
        color[3 * (y * width + width - 1) + 1] = grey[y * width + width - 1];
        color[3 * (y * width + width - 1) + 2] =
            (grey[(y - 1) * width + width - 1] +
             grey[(y + 1) * width + width - 1]) / 2;
    }

    color[3 * (width * height - 1) + 0] =
        (grey[(height - 2) * width + width - 2] +
         grey[(height - 2) * width]) / 2;
    color[3 * (width * height - 1) + 1] =
        (grey[(height - 2) * width + width - 1] +
         grey[(height - 1) * width + width - 2] +
         grey[(height - 1) * width]) / 3;
    color[3 * (width * height - 1) + 2] = grey[width * height - 1];

    color[3 * ((height - 1) * width) + 0] = grey[(height - 2) * width];
    color[3 * ((height - 1) * width) + 1] = grey[(height - 1) * width];
    color[3 * ((height - 1) * width) + 2] =
        (grey[(height - 1) * width + width - 1] +
         grey[(height - 1) * width + 1]) / 2;

    color[0] = grey[0];
    color[1] = (grey[width] + grey[width - 1] + grey[1]) / 3;
    color[2] = (grey[width + width - 1] + grey[width + 1]) / 2;

    color[3 * (width - 1) + 0] = (grey[width - 2] + grey[0]) / 2;
    color[3 * (width - 1) + 1] = grey[width - 1];
    color[3 * (width - 1) + 2] = (grey[width + width - 1]);
}

int RCbuildC2LMapBayer (int nEcc, int nAng, int xSize, int ySize, double overlap, double scaleFact, int mode, char *path)
{
    if (overlap <= -1.0)
        return 1;

	int cSize = xSize;
	
	if(xSize>ySize)
		cSize = ySize;

    double angle = (2.0 * PI / nAng);   //Angular size of one pixel
    double sinus = sin (angle / 2.0);
    double tangent = sinus / cos (angle / 2.0);

    int fov;
    int lim;

    double lambda;              //Log Index
    double firstRing;           //Diameter of the receptive fields in the first ring when overlap is 0
    double *currRad;            //Distance of the center of the current ring RF's from the center of the mapping
    double *nextRad;            //Distance of the center of the next ring's RF's from the center of the mapping
    double r0;                  //lower limit of RF0 in the "pure" log polar mapping

    double x0, y0;
    double locX, locY, locRad;
    double *radii;
    double *tangaxis;
    double *radialaxis;
    double *focus;
    double *radii2;
    double A;
    double L;
    double step;
    double F0x, F0y, F1x, F1y;
    double maxaxis;

    int rho, theta, j;

    cart2LpPixel c2LpMap;

    lambda = (1.0 + sinus) / (1.0 - sinus);
    fov = (int) (lambda / (lambda - 1));
    firstRing = (1.0 / (lambda - 1)) - (int) (1.0 / (lambda - 1));
    r0 = 1.0 / (pow (lambda, fov) * (lambda - 1));

    tangaxis = new double[nEcc];
    radialaxis = new double[nEcc];
    focus = new double[nEcc];
    radii2 = new double[nEcc];

    currRad = new double[nEcc];
    nextRad = new double[nEcc];

    if ((tangaxis == 0) || (radialaxis == 0) || (focus == 0)
        || (radii2 == 0))
        return 2;

    if ((currRad == 0) || (nextRad == 0))
        return 2;

    /************************
     * RF's size Computation *
     ************************/

    for (rho = 0; rho < nEcc; rho++)
    {
        if (rho < fov)
            if (rho == 0)
            {
                currRad[rho] = firstRing * 0.5;
                nextRad[rho] = firstRing;
            }
            else
            {
                currRad[rho] = rho + firstRing - 0.5;
                nextRad[rho] = currRad[rho] + 1.0;
            }
        else
        {
            currRad[rho] = pow (lambda, rho) * (r0 + 0.5 / pow (lambda, fov));
            nextRad[rho] = lambda * currRad[rho];
        }
        tangaxis[rho] =
            scaleFact * 2.0 * currRad[rho] * sinus * (overlap + 1.0) / (2.0);

        radialaxis[rho] =
            scaleFact * (nextRad[rho] - currRad[rho]) * (overlap + 1.0);

        if (rho < fov)
            radialaxis[rho] /= 2.0;
        else
            radialaxis[rho] /= (1.0 + lambda);

        if ((rho < fov) && (mode == ELLIPTICAL))
        {
            A = radialaxis[rho] * radialaxis[rho];
            L = scaleFact * currRad[rho] * (overlap + 1.0);
            L = L * L;
            tangaxis[rho] = tangent * sqrt (L - A);
        }
    }
    radialaxis[0] = 0.5 * scaleFact * (firstRing) * (overlap + 1.0);

    if (mode == RADIAL)
        radii = radialaxis;
    else
        radii = tangaxis;

    for (rho = 0; rho < nEcc; rho++)
    {
        if ((mode == RADIAL) || (mode == TANGENTIAL))
            focus[rho] = 0;
        else
	{
            focus[rho] =
                -sqrt (fabs (radialaxis[rho] * radialaxis[rho] -
                             tangaxis[rho] * tangaxis[rho]));
	}		
        if (tangaxis[rho] >= radialaxis[rho])
            focus[rho] = -focus[rho];

        radii2[rho] = radii[rho] * radii[rho];
    }

    int totalcounter = 0;

    FILE *fout;
    char filename[256];

    sprintf (filename, "%sC2LPBayer.gio", path);

    fout = fopen (filename, "wb");

    fwrite (&totalcounter, sizeof (int), 1, fout);

    double *sintable;
    double *costable;

    int intx, inty;

    sintable = new double[nAng];
    costable = new double[nAng];

    if ((sintable == 0) || (costable == 0))
        return 2;

    for (j = 0; j < nAng; j++)
    {
        sintable[j] = sin (angle * (j + 0.5));
        costable[j] = cos (angle * (j + 0.5));
    }

    bool found;
    int mapsize;
    float *weight;
    double precision = 10.0;

    for (rho = 0; rho < nEcc; rho++)
    {
        if ((mode == RADIAL) || (mode == TANGENTIAL))
            lim = (int) (radii[rho] + 1.5);
        else
            lim = (int) (__max64f (tangaxis[rho], radialaxis[rho]) + 1.5);

        step = lim / precision;

        if (step > 1)
            step = 1;

        mapsize = (int) (precision * lim * precision * lim + 1);

        c2LpMap.position = new int[mapsize];
        weight = new float[mapsize];
        c2LpMap.iweight = new int[mapsize];

        if ((c2LpMap.position == 0) || (weight == 0)
            || (c2LpMap.iweight == 0))
            return 2;

        for (theta = 0; theta < nAng; theta++)
        {
            memset (c2LpMap.position, 0, mapsize);
            memset (weight, 0, mapsize);

            x0 = scaleFact * currRad[rho] * costable[theta];
            y0 = scaleFact * currRad[rho] * sintable[theta];

            if (focus[rho] >= 0)
            {
                F0x = x0 - focus[rho] * sintable[theta];
                F0y = y0 + focus[rho] * costable[theta];
                F1x = x0 + focus[rho] * sintable[theta];
                F1y = y0 - focus[rho] * costable[theta];
                maxaxis = tangaxis[rho];
            }
            else
            {
                F0x = x0 - focus[rho] * costable[theta];
                F0y = y0 - focus[rho] * sintable[theta];
                F1x = x0 + focus[rho] * costable[theta];
                F1y = y0 + focus[rho] * sintable[theta];
                maxaxis = radialaxis[rho];
            }
            if ((mode == RADIAL) || (mode == TANGENTIAL))
                maxaxis = radii[rho];

            for (locX = (x0 - lim); locX <= (x0 + lim); locX += step)
                for (locY = (y0 - lim); locY <= (y0 + lim); locY += step)
                {
                    intx = (int) (locX + xSize / 2);
                    inty = (int) (locY + ySize / 2);

                    locRad =
                        sqrt ((locX - F0x) * (locX - F0x) +
                              (locY - F0y) * (locY - F0y));
                    locRad +=
                        sqrt ((locX - F1x) * (locX - F1x) +
                              (locY - F1y) * (locY - F1y));

                    if (locRad < 2 * maxaxis)
                    {
                        if ((inty < ySize) && (inty >= 0))
                        {
                            if ((intx < xSize) && (intx >= 0))
                            {
                                if ((((intx + inty) % 2 == 1)
                                     && ((rho + theta) % 2 == 1))
                                    || ((intx % 2 == 0) && (inty % 2 == 0)
                                        && (rho % 2 == 0) && (theta % 2 == 0))
                                    || ((intx % 2 == 1) && (inty % 2 == 1)
                                        && (rho % 2 == 1)
                                        && (theta % 2 == 1)))
                                {
                                    found = false;
                                    j = 0;
                                    while ((j < mapsize)
                                           && (c2LpMap.position[j] != 0))
                                    {
                                        if (c2LpMap.position[j] ==
                                            (inty * xSize + intx))
                                        {
                                            weight[j]++;
                                            found = true;
                                            j = mapsize;
                                        }
                                        j++;
                                    }

                                    if (!found)
                                        for (j = 0; j < mapsize; j++)
                                        {
                                            if (c2LpMap.position[j] == 0)
                                            {
                                                c2LpMap.position[j] =
                                                    (inty * xSize + intx);
                                                weight[j]++;
                                                break;
                                            }
                                        }
                                }
                            }
                        }
                    }
                }

            for (j = 0; j < mapsize; j++)
                if (weight[j] == 0)
                    break;

            c2LpMap.divisor = j;

            if (j == 0)
            {
                intx = (int) (x0 + 0.0 + xSize / 2);
                inty = (int) (y0 + 0.0 + ySize / 2);
                if ((rho + theta) % 2 == 1)
                {
                    //find closest green pixel
                    if ((intx + inty) % 2 == 1) //I'm on green
                    {
                        c2LpMap.position[0] = (inty * xSize + intx);
                    }
                    else        //I'm not
                    {
                        if (intx > inty)
                            c2LpMap.position[0] = ((inty - 1) * xSize + intx);
                        else
                            c2LpMap.position[0] = ((inty) * xSize + intx - 1);
                    }
                    c2LpMap.divisor = 1;
                }
                else if ((rho % 2 == 0) && (theta % 2 == 0))
                {
                    if ((inty % 2 == 0) && (intx % 2 == 0))
                    {
                        c2LpMap.position[0] = (inty * xSize + intx);
                    }
                    else if ((inty % 2 == 0) && (intx % 2 == 1))
                    {
                        c2LpMap.position[0] = ((inty) * xSize + intx - 1);
                    }
                    else if ((inty % 2 == 1) && (intx % 2 == 0))
                    {
                        c2LpMap.position[0] = ((inty - 1) * xSize + intx);
                    }
                    else
                    {
                        c2LpMap.position[0] = ((inty - 1) * xSize + intx - 1);
                    }
                    c2LpMap.divisor = 1;

                }
                else
                {
                    if ((inty % 2 == 1) && (intx % 2 == 1))
                    {
                        c2LpMap.position[0] = (inty * xSize + intx);
                    }
                    else if ((inty % 2 == 0) && (intx % 2 == 1))
                    {
                        c2LpMap.position[0] = ((inty - 1) * xSize + intx);
                    }
                    else if ((inty % 2 == 1) && (intx % 2 == 0))
                    {
                        c2LpMap.position[0] = ((inty) * xSize + intx - 1);
                    }
                    else
                    {
                        c2LpMap.position[0] = ((inty - 1) * xSize + intx - 1);
                    }
                    c2LpMap.divisor = 1;

                }
                weight[0] = 1;
                j = 1;
            }

            totalcounter += c2LpMap.divisor;

            float sum = 0.0;
            int k;
            for (k = 0; k < j; k++)
                sum += weight[k];
            for (k = 0; k < j; k++)
            {
                weight[k] = weight[k] / sum;
                c2LpMap.iweight[k] = (int) (weight[k] * 65536.0);
            }

            fwrite (&c2LpMap.divisor, sizeof (int), 1, fout);
            fwrite (c2LpMap.position, sizeof (int), c2LpMap.divisor, fout);
            fwrite (c2LpMap.iweight, sizeof (int), c2LpMap.divisor, fout);

        }
        delete[]c2LpMap.position;
        delete[]weight;
        delete[]c2LpMap.iweight;
    }
    delete[]tangaxis;
    delete[]radialaxis;
    delete[]focus;
    delete[]radii2;
    delete[]currRad;
    delete[]nextRad;
    delete[]sintable;
    delete[]costable;

    fseek (fout, 0, SEEK_SET);
    fwrite (&totalcounter, sizeof (int), 1, fout);
    fclose (fout);

    return 0;

}

#endif


