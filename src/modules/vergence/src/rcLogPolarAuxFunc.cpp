// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea, Vadim Tikhanoff
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
#include <iCub/rcLogPolarAuxFunc.h>

bool isWithin(int v,int max, int min) {

    if ( (v <= max) && (v >= min) )
        return true;
    else
        return false;
    }

double find_max_value(double *v, int size) {
    double tmpMax = v[0];
    for (int i = 0; i < size; i++) {
        if (v[i] >= tmpMax)
            tmpMax = v[i];
        }
    return tmpMax;
}

int find_max_index(double *v, int size) {
    int ind = 0;
    double tmpMax = v[0];
    for (int i = 0; i < size; i++) {
        if (v[i] >= tmpMax) {
            tmpMax = v[i];
            ind = i;
        }
    }
    return ind;
}

double find_min_value(double *v, int size) {
    double tmpMin = v[size-1];
    for (int i = size - 1; i >= 0; i--) {
        if (v[i] <= tmpMin)
            tmpMin = v[i];
    }
    return tmpMin;
}

int find_min_index(double *v, int size) {
    int ind = size - 1;
    double tmpMin = v[0];
    for (int i = size - 1; i >= 0; i--) {
        if (v[i] <= tmpMin) {
            tmpMin = v[i];
            ind = i;
        }
    }
    return ind;
}


Image_Data SetParam(int rho, int theta, int mode, double overlap, int xo, int yo, int xr, int yr) {
    Image_Data par;

    par.Size_Rho = rho;
    par.Size_Theta = theta;
    par.Size_LP = rho*theta;
    par.Fovea_Mode = mode;
    par.overlap = overlap;

    // Remapped Cartesian Metrics
    par.Size_X_Remap = xr;
    par.Size_Y_Remap = yr;
    par.Size_Img_Remap = xr*yr;

    // Original Cartesian Metrics
    par.Size_X_Orig = xo;
    par.Size_Y_Orig = yo;
    par.Size_Img_Orig = xo*yo;

    //Computed Parameters
    par.scale_Orig = RCcomputeScaleFactor(rho, theta, xo, yo, overlap);
    par.scale_Remap = RCcomputeScaleFactor(rho, theta, xr, yr, overlap);
    
    double ang = PI/(double)theta;
    double sinus = sin(ang);
    par.LogIndex = (1.0 + sinus)/(1.0 - sinus);
    par.firstRing = (1.0/(par.LogIndex - 1.0)) - (int)(1.0/(par.LogIndex - 1.0));
    par.Size_Fovea = (int) (par.LogIndex / (par.LogIndex - 1.0));
    par.r0 = 1.0 / (pow (par.LogIndex, par.Size_Fovea) * (par.LogIndex - 1.0));

    return par;
}


double getX(Image_Data par, int rho, int theta) {
    double radius, angle, x;

    if (rho < 0)
        rho = 0;
    if (rho >= par.Size_Rho)
        rho = par.Size_Rho - 1;

    if (rho < par.Size_Fovea) {	
        if (rho == 0)
            radius = par.firstRing*0.5;
        else	
            radius = (double)rho + par.firstRing - 0.5;
    }
    else
        radius = pow (par.LogIndex, rho) * (par.r0 + 0.5 / pow (par.LogIndex, par.Size_Fovea));

    if (theta < 0)
        theta = par.Size_Theta - abs(theta)%par.Size_Theta;
    if (theta >= par.Size_Theta)
        theta %= par.Size_Theta;
    
    angle = (2*PI*(double)theta)/(double)par.Size_Theta;
    
    x = par.scale_Orig*radius*cos(angle);

    return x;
}

double getY(Image_Data par, int rho, int theta) {
    double radius, angle, y;

    if (rho < 0)
        rho = 0;
    if (rho >= par.Size_Rho)
        rho = par.Size_Rho - 1;

    if (rho < par.Size_Fovea) {	
        if (rho == 0)
           radius = par.firstRing*0.5;
        else	
           radius = (double)rho + par.firstRing - 0.5;
    }
    else
        radius = pow (par.LogIndex, rho) * (par.r0 + 0.5 / pow (par.LogIndex, par.Size_Fovea));

    if (theta < 0)
        theta = par.Size_Theta - abs(theta)%par.Size_Theta;

    if (theta >= par.Size_Theta)
        theta %= par.Size_Theta;

    angle = (2.0*PI*(double)theta)/(double)par.Size_Theta;

    y = par.scale_Orig*radius*sin(angle);

    return y;
}


int getRho(Image_Data par, double x, double y) {
    double arg, radius;
    int r;

    radius = sqrt(pow(x,2) + pow(y,2))/par.scale_Orig;
    if (radius >= (double)par.Size_Fovea + par.firstRing - 0.5) {

        arg = (2.0*radius*pow (par.LogIndex, par.Size_Fovea))/(2.0*par.r0* pow (par.LogIndex, par.Size_Fovea) + 1);
        r = (int)(log(arg)/log(par.LogIndex));
    }
    else
        r = (int)(radius - par.firstRing + 0.5);

    return r;
}


int getTheta(Image_Data par, double x, double y) {
    double angle;
    int t;

    angle = atan2(y,x);
    if (angle < 0)
        angle += 2*PI;
    
    t = (int)(angle*par.Size_Theta/(2.0*PI));

    return t;
}


void Build_Shift_Table(Image_Data par, char *path) {

    cout << "building table " << endl;
    int n_shifts;
    double *shifts;
    int * ShiftMap;
    char File_Name [256];
    FILE * fout;
    int mult = 1;
    int cSize = (int)_min((double)par.Size_X_Orig, (double)par.Size_Y_Orig);

    //Creating StepList
    n_shifts = 1+6*cSize/(4*mult);
    shifts = new double[n_shifts];
    for (int j = 0; j < mult*n_shifts; j += mult)
    shifts[j/mult] = j-mult*(n_shifts/2);

    //Creating ShiftTable 
    ShiftMap = new int[n_shifts*par.Size_LP];
    double tmpX, tmpY;
    int newRho, newTheta;

    for (int k = 0; k < n_shifts; k++) {
        for (int r = 0; r < par.Size_Rho; r++) {
            for ( int t = 0; t < par.Size_Theta; t++) {
                tmpX = getX(par, r, t) + shifts[k];
                tmpY = getY(par, r, t);
                newRho = getRho(par, tmpX, tmpY);
                newTheta = getTheta(par, tmpX, tmpY);
                if ((newRho >= 0) && (newRho < par.Size_Rho) && (newTheta >= 0) && (newTheta < par.Size_Theta))
                    ShiftMap[k*par.Size_LP + par.Size_Theta*r + t] = 3*newRho*par.Size_Theta + 3*newTheta;
                else
                    ShiftMap[k*par.Size_LP + par.Size_Theta*r + t] = 0;
            }
        }		
    }

    //Writing on file
    sprintf(File_Name,"%s%dx%d_%dx%d_ShiftMap.gio",path, cSize, cSize, par.Size_Theta, par.Size_Rho);
    fout = fopen(File_Name,"wb");
    fwrite(&n_shifts,sizeof(int),1,fout);
    fwrite(shifts,sizeof(double),n_shifts,fout);
    fwrite(ShiftMap,sizeof(int),n_shifts*par.Size_LP,fout);
    fclose(fout);

    cout << "finished building table " << endl;
    delete [] shifts;
    delete [] ShiftMap;
}


int Load_Shift_Table(Image_Data par, int *shiftTab, double *stepList, char *path) {
    char File_Name [256];
    FILE * fin;
    int n;
    int cSize = (int)_min((double)par.Size_X_Orig, (double)par.Size_Y_Orig);
    sprintf(File_Name,"%s%dx%d_%dx%d_ShiftMap.gio",path, cSize, cSize, par.Size_Theta, par.Size_Rho);

    if ((fin = fopen(File_Name,"rb")) == NULL) {
        Build_Shift_Table(par, path);
        fin = fopen(File_Name,"rb");
    }

    fread(&n,sizeof(int),1,fin);

    fread(stepList,sizeof(double),n,fin);
    fread(shiftTab,sizeof(int),n*par.Size_LP,fin);
    fclose(fin);
    return n;
}


void img2unpaddedVectMultiple(unsigned char *vl, Image imgl, unsigned char *vr, Image imgr ) {

    for (int i = 0; i < imgl.height(); i++) {
        for (int j = 0; j < imgl.width()*imgl.getPixelSize(); j++) {
            vl[i*imgl.width()*imgl.getPixelSize()+j] = *(imgl.getRow(i)+j);	
            vr[i*imgr.width()*imgr.getPixelSize()+j] = *(imgr.getRow(i)+j);			
        }
    }
}

void img2unpaddedVect( unsigned char *v, Image img ) {

    for (int i = 0; i < img.height(); i++) {
        for (int j = 0; j < img.width()*img.getPixelSize(); j++) {
            v[i*img.width()*img.getPixelSize()+j] = *(img.getRow(i)+j);	
        }
    }
}


void unpaddedVect2img(unsigned char *v, Image &img) {
    for (int i = 0; i < img.height(); i++) {
        for (int j = 0; j < img.width()*img.getPixelSize(); j++) {
            *(img.getRow(i)+j) = v[i*img.width()*img.getPixelSize()+j];			
        }
    }
}

