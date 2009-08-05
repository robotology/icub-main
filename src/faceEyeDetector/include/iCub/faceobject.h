/*
 *  FACEOBJECT.h
 *
 *  Created by Bret Fortenberry on Aug 27, 2003.
 *  Fixes: 
 *		To compile properly,  the copy constructor FaceObject(const FaceObject &thelist);
 *		is commented.
 *		by Plinio Moreno on Oct 06, 2007
 * 
 *  Copyright (c) 2003 Machine Perception Laboratory 
 *  University of California San Diego.
 * 
 * Please read the disclaimer and notes about redistribution 
 * at the end of this file.
 *  
 */

#ifndef _FACEOBJECT_H_
#define _FACEOBJECT_H_

#include "visualobject.h"
#include "eyeobject.h"
#include "beyesobject.h"
#include <algorithm>
#include <list>
#include <vector>
#include <math.h>


#define EYEMEMSIZE 10000

enum combine_mode{none=0, average, wt_avg, wt_max, mean_shift, mpi_search, maximum, face_only};
const int num_modes = 8;
const char mode_strings[num_modes][25] = {"none", "average", "wt_avg", "wt_max", "mean_shift", "mpi_search", "maximum", "face_only"};
#define DEFAULT_COMBINE_MODE none

using namespace std;


class FaceObject : public VisualObject
{
 public:

  bEyesObject eyes;
	vector< EyeObject > leftEyes;
	vector< EyeObject > rightEyes;

	FaceObject();
	FaceObject(float x_in, float y_in, float xSize_in, float ySize_in, float scale_in);
	//FaceObject(const FaceObject &thelist);
	FaceObject(TSquare<float> &square);
	FaceObject(list<Square>::iterator face);
	~FaceObject();

	void findMax();
	void posterior(combine_mode mode);

	virtual void clear();

private:
	float det3(vector< vector< float > > &matrix);
  void TransCof3(vector< vector< float > > &inMtx, vector< vector< float > > &rtnMtx, float det);
	bool findOutliers(vector< vector< float > > &meanSub, vector< vector< float > > &invMtx, vector< EyeObject > &Eyes);
	int mtxMult(vector< vector< float > > &matrix1, vector< vector< float > > &matrix2, vector< vector< float > > &rtn_matrix);
	int mtxMult_2T(vector< vector< float > > &matrix1, vector< vector< float > > &matrix2, vector< vector< float > > &rtn_matrix, float mult = 1.0f);
#ifndef WIN32
	int prtMtx(vector< vector< float > > &matrix);
#endif

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
