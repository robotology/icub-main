/*
 *  eyefinder.h
 *
 *  Author:Ian Fasel
 *  Changes:Bret Fortenberry
 *	Fixes:
 *		In the findEyes method, the previous parameter VisualObject &mFaces is
 *		changed to list<FaceObject *> &mFaces.
 *		by Plinio Moreno Oct 6, 2007
 *  Copyright (c) 2003 Machine Perception Laboratory
 *  University of California San Diego.
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

#ifndef __EYEFINDER_H__
#define __EYEFINDER_H__
#ifndef Xcode
#include "mpisearchFaceDetector.h"
#else
#include <mpisearch/mpisearchFaceDetector.h>
#endif
#include "faceobject.h"
#include <vector>
#include "cv.h"

#define checkAll 1
#define MPISEARCH_OBJECT_TYPE float

enum patch_rez{eye_dist=0, eye_only=1, half_dist, smallest, largest, largest_2}; // resolution of patch
#define DEFAULT_PATCH_REZ eye_dist

enum centering_condition{eye_centered=0,face_centered};
#define DEFAULT_CENTERING eye_centered

struct EYESHIFTARRAY{
	int x;
	int y;
};

// sufficient statistics for Gaussian prior
// note: 
struct GPrior{
  vector< float > mean;
  vector< vector< float > > cov;
  vector< vector< float > > invCov;
  GPrior();
  void Release();
  GPrior(const vector< float > &mean_, const vector< vector< float > > &cov_, const vector< vector< float > > &invCov_);
  void SetPrior(const vector< float > &mean_, const vector< vector< float > > &cov_, const vector< vector< float > > &invCov_);
}; 

struct ROIdata{
  int numscales;
  vector< float > mean;
  vector< vector< float > > cov;
  vector< vector< float > > invCov;
  vector< float > scales;
  vector< vector< float > > bounds;
  void Release();

};

//class MPEyeFinder : public MPISearchBinary {
class MPEyeFinder : public MPISearchFaceDetector {
 public:

  MPEyeFinder ( int use_ada=0 ); // 0=gentleboost, faster, not quite as accurate, 1=adaboost, slower, a bit more accurate
  ~MPEyeFinder( );

  int findEyes(const RImage<MPISEARCH_OBJECT_TYPE> &pixels, list<FaceObject *> &mFaces,
               float WINSHIFT=1.25, float overlap=0.0, combine_mode mode=none);
  int findEyesUsingOCVFaceDetect(const RImage<MPISEARCH_OBJECT_TYPE> &pixels, list<FaceObject *> &mFaces, CvSeq* cvFaces,
			  float WINSHIFT=1.25, float overlap=0.0, combine_mode mode=none);
  void initStream(const int width, const int height, double WINSHIFT=1.25);
  void resetStream(const int width, const int height, double WINSHIFT=1.25);
  void releaseStream();
  void SetCentering(const centering_condition &c);
  void SetRez(const patch_rez &r);
 protected:
  void getPatchWidth(patch_rez p, float &patchWidthPct);
  //GPrior MPEyeFinder::setROI(FaceObject* &face, feature_type eye_type);
  GPrior setROI(FaceObject* &face, feature_type eye_type);
  int eyeSearch(MPISearchStream<MPISEARCH_OBJECT_TYPE> &thestream, FeatureData &thedata, vector< EyeObject > &eyelist, 
		FaceObject *current_face, GPrior &gp, feature_type eyetype, combine_mode mode=DEFAULT_COMBINE_MODE);
  void getEyeOffsets(patch_rez p, centering_condition centering, float &xoff, float &yoff);
  void initROIdata();
  int matrixMult(int row1,int col1, float *matrix1, int row2, int col2, float *matrix2, float *rtn_matrix);
  int matrixMult(int row1,int col1, float *matrix1, int row2, int col2, vector< vector< float > > matrix2, float *rtn_matrix);
  // patch specific things
  patch_rez m_rez;
  centering_condition m_centering;
  float mean_dist;

  FeatureData left_eye_data;
  MPISearchStream<MPISEARCH_OBJECT_TYPE> left_eye_stream;
  ROIdata leftROIdata;

  FeatureData right_eye_data;
  MPISearchStream<MPISEARCH_OBJECT_TYPE> right_eye_stream;
  ROIdata rightROIdata;
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

