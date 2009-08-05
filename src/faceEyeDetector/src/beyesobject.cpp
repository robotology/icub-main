/* 
 *  BEYESOBJECT.cpp
 *
 *  Created by Bret Fortenberry on Aug 27, 2003.
 *  Fixes: 
 * 
 *  Copyright (c) 2003 Machine Perception Laboratory 
 *  University of California San Diego.
 * 
 * Please read the disclaimer and notes about redistribution 
 * at the end of this file.
 *  
 */

#include <iCub/beyesobject.h>
#include <cmath>
#include <math.h>


#define xfactor 1.56f//1.56
#define yfactor 0.78f

bEyesObject::bEyesObject() : xLeft(0), yLeft(0), xRight(0), yRight(0), 
		   leftScale(0), rightScale(0), leftEye(false), rightEye(false){}
		 
bEyesObject::bEyesObject(float xLeft_in, float yLeft_in, float xRight_in, float yRight_in){
	xLeft = xLeft_in;
	yLeft = yLeft_in;
	xRight = xRight_in; 
	yRight = yRight_in;
}

bEyesObject::bEyesObject(const bEyesObject &thelist)
{
	x = thelist.x;
	y = thelist.y;
	xSize = thelist.xSize;
	ySize = thelist.ySize;
	xLeft = thelist.xLeft;
	yLeft = thelist.yLeft;
	xRight = thelist.xRight;
	yRight = thelist.yRight;
	degrees = thelist.degrees;
	leftEye = thelist.leftEye;
	rightEye = thelist.rightEye;
	scale = thelist.scale;
}


bEyesObject::~bEyesObject() {}

int bEyesObject::findBox()
{
	if(!leftEye || !rightEye)
		return 0;
	xSize = static_cast<float>(floor(findEucDis()*xfactor));
	ySize = static_cast<float>(floor(xSize/2));
	degrees = atan2((yRight-yLeft),(xLeft-xRight));
    float width = xLeft-xRight;
	float midpointx = xRight + static_cast<float>(floor(width/2));
	float midpointy = yRight + static_cast<float>(floor((yLeft-yRight)/2)); 
	x = midpointx - static_cast<float>(floor(cos(degrees)*(xSize*.5) + sin(degrees)*(ySize*.5)));
	y = midpointy + static_cast<float>(floor(sin(degrees)*(xSize*.5) - cos(degrees)*(ySize*.5)));
	return 1;
}

float bEyesObject::getAbs(float x){
	float rtn = x;
	if(x < 0){
		float temp = x * 2;
		rtn = x - temp;
	}
	return rtn;
}		

double bEyesObject::findEucDis()
{
	double xSquare = getAbs(xLeft-xRight) * getAbs(xLeft-xRight);
	double ySquare = getAbs(yLeft-yRight) * getAbs(yLeft-yRight);
	double EucDis = sqrt(xSquare + ySquare);
	return EucDis;
}



/*
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * 
 *    1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *    3. The name of the author may not be used to endorse or promote products derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
