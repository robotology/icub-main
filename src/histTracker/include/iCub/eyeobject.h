/* 
 *  EYEOBJECT.h
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

#ifndef _EYEOBJECT_H_
#define _EYEOBJECT_H_

#include "visualobject.h"
#include <algorithm>
#include <list>
#include <math.h>

using namespace std;


class EyeObject : public VisualObject
{
 public:
	//double activation;
	double cascade_level;
	int flag;


	EyeObject(feature_type eye);//eye = righteye || lefteye
	EyeObject(feature_type eye, float x_in, float y_in, float xSize_in, float ySize_in, float scale_in);
	EyeObject(feature_type eye, float x_in, float y_in, float scale_in, double activation_in, double cascade_in);
	//EyeObject(const EyeObject &thelist);
	EyeObject(const TSquare<float> &square, feature_type eye);
	~EyeObject();

	virtual void clear();

	bool operator>(const EyeObject &b ) const {
		return (this->activation > b.activation);
	}
	bool operator<(const EyeObject &b) const {
		return (this->activation < b.activation);
	}

};





#endif

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
