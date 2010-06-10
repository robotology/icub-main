/*
 *  eyefinder.cc
 *
 *  Author:Ian Fasel
 *  Fixes:
 *
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

#include <iCub/vis/eyefinder.h>
#include <iCub/vis/eye_finder_ROI.h>
#include <math.h>
#include <string>
#include <iostream>

#ifdef WIN32
#include <windows.h>
#endif

#define ROI_namespace eyefinder_ROI_low_thresh

GPrior::GPrior(){};
GPrior::GPrior(const vector< float > &mean_, const vector< vector< float > > &cov_, const vector< vector< float > > &invCov_){
  SetPrior(mean_, cov_, invCov_);
}
void GPrior::Release(){
  mean.clear();
  cov.clear();
  invCov.clear();
}
void GPrior::SetPrior(const vector< float > &mean_, const vector< vector< float > > &cov_, const vector< vector< float > > &invCov_){
  mean=mean_;
  cov=cov_;
  invCov=invCov_;
}

void ROIdata::Release(){
  mean.clear();
  cov.clear();
  invCov.clear();
  scales.clear();
  bounds.clear();
}


MPEyeFinder::MPEyeFinder ( int use_ada ) : MPISearchFaceDetector(use_ada),
m_rez(DEFAULT_PATCH_REZ), m_centering(DEFAULT_CENTERING){ 
  initROIdata();
}

MPEyeFinder::~MPEyeFinder ( ){ 
  releaseStream();
  leftROIdata.Release();
  rightROIdata.Release();

}

void MPEyeFinder::SetCentering(const centering_condition &c){m_centering = c;}
void MPEyeFinder::SetRez(const patch_rez &r){m_rez = r;}

void MPEyeFinder::initStream(const int width, const int height, double WINSHIFT){
  stream.init(width, height, data, WINSHIFT);
  //printData(data,data.numfeatures-1);
  //printf("Stream initialized\n");
  left_eye_stream.init(stream, left_eye_data, 1);
  //printData(left_eye_data,left_eye_data.numfeatures-1);
  right_eye_stream.init(stream, right_eye_data, 1);
  //printData(right_eye_data,left_eye_data.numfeatures-1);
}

void MPEyeFinder::resetStream(const int width, const int height, double WINSHIFT){
  // Note: order matters! eye_streams are not owners of all their data, so they must be released first
  releaseStream();
  stream.init(width, height, data, WINSHIFT);
  left_eye_stream.init(stream, left_eye_data, 1);
  right_eye_stream.init(stream, right_eye_data, 1);
}
void MPEyeFinder::releaseStream(){
  // Note: order matters! eye_streams are not owners of all their data, so they must be released first
  left_eye_stream.release();
  right_eye_stream.release();
  stream.release();
}

void MPEyeFinder::initROIdata(){
  const int dim = 3; // 3-dimensional data
  int i;

  for(i = 0; i < dim; ++i)
    leftROIdata.mean.push_back(ROI_namespace::leftmean[i]);
  for(i = 0; i < dim; ++i){
    vector< float > v;
    vector< float > iv;
    for(int j = 0; j < dim; ++j){
      v.push_back(ROI_namespace::leftcov[i][j]);
      iv.push_back(ROI_namespace::leftInvCov[i][j]);
    }
    leftROIdata.cov.push_back(v);
    leftROIdata.invCov.push_back(iv);
  }
  leftROIdata.numscales = ROI_namespace::numleftscales;
  for(i = 0; i < leftROIdata.numscales; ++i)
    leftROIdata.scales.push_back(ROI_namespace::leftscales[i]);
  for(i = 0; i < leftROIdata.numscales; ++i){
    vector< float > v;
    for(int j = 0; j < 6; ++j)
      v.push_back(ROI_namespace::leftbounds[i][j]);
    leftROIdata.bounds.push_back(v);
  }

  for(i = 0; i < dim; ++i)
    rightROIdata.mean.push_back(ROI_namespace::rightmean[i]);
  for(i = 0; i < dim; ++i){
    vector< float > v;
    vector< float > iv;
    for(int j = 0; j < dim; ++j){
      v.push_back(ROI_namespace::rightcov[i][j]);
      iv.push_back(ROI_namespace::rightInvCov[i][j]);
    }
    rightROIdata.cov.push_back(v);
    rightROIdata.invCov.push_back(iv);
  }
  rightROIdata.numscales = ROI_namespace::numrightscales;
  for(i = 0; i < rightROIdata.numscales; ++i)
    rightROIdata.scales.push_back(ROI_namespace::rightscales[i]);
  for(i = 0; i < rightROIdata.numscales; ++i){
    vector< float > v;
    for(int j = 0; j < 6; ++j)
      v.push_back(ROI_namespace::rightbounds[i][j]);
    rightROIdata.bounds.push_back(v);
  }
}

GPrior MPEyeFinder::setROI(FaceObject* &face, feature_type eye_type){
  ROIdata *d = NULL;
  MPIImagePyramid<MPISEARCH_OBJECT_TYPE> *mpi = NULL;
  float xoff=0.0, yoff=0.0, patchWidthPct=0.0, patch_width, patch_height;
  int i;

  getEyeOffsets(m_rez, m_centering, xoff, yoff);
  getPatchWidth(m_rez, patchWidthPct);

  switch(eye_type){
  case lefteye:
    d = &leftROIdata;
    mpi = left_eye_stream.mpi;
    xoff = -xoff;
    patch_width = (float)left_eye_stream.m_data->patch_width;
    patch_height = (float)left_eye_stream.m_data->patch_width;
    break;
  case righteye:
    d = &rightROIdata;
    mpi = right_eye_stream.mpi;
    patch_width = (float)right_eye_stream.m_data->patch_width;
    patch_height = (float)right_eye_stream.m_data->patch_width;
    break;
	default:
		patch_width = 1;
		break;
  }

  // First, clear out the ROI in the current stream
  // mpi->InitROI();
  ROI roi = mpi->getROI();
  for(i = 0; i < (int)roi.vmin_x.size(); ++i){
    roi.vmin_x[i] = 0; roi.vmax_x[i] = 0; roi.vmin_y[i] = 0; roi.vmax_y[i] = 0; 
  }
  
  // Get the multiplier for finding the scale_factor of an eyepatch given an eye distance
  float scale_factor_ratio = patchWidthPct * face->xSize * 2.0f / patch_width;

  // Now, loop through all the scales we would ideally search, and set
  // the bounds according to the actual scales we have access to, given
  // the size of the expected eye patch and the fact that we can only
  // access integer scales

  float scale_factor;
  int current_scale_ind;
  int last_scale_ind = -1;
  for(i = 0; i < d->numscales; ++i){
    scale_factor = scale_factor_ratio * (d->mean[2]+d->scales[i]);
    current_scale_ind = mpi->getClosestScale(scale_factor);
    //scale_factor = mpi->scale_factors[current_scale_ind];
    if(current_scale_ind != last_scale_ind){
      scale_factor = static_cast<int>(max(1.0f,scale_factor) * 2.0f + 1.0f)/ 2.0f; // grows tiny scales a bit
      float half_window_width = patch_width * scale_factor / (2.0f-max(0.0f,1.0f-2.0f*patchWidthPct)); // widens small scale detectors
      // Commentary:
      // (bounds[i][1] + d->mean[0] + xoff):  the ROI w.r.t the center of the search patch
      //             * face->xSize: convert to pixels
      //             +-  half_window_width: adjust so this ROI bounds searchable patches on left and right edge
      //             + face->x,y: make it with respect to the face location

      // POSSIBLE BUG! ERROR! Only uses window width, not height.  Check here if problems occur
      //cout << "d->bounds[i][1] + d->mean[0] + xoff" << d->bounds[i][1] + d->mean[0] + xoff << endl;
      roi.vmin_x[current_scale_ind] = (int)((d->bounds[i][1] + d->mean[0] + xoff) * face->xSize - half_window_width + face->x);
      roi.vmax_x[current_scale_ind] = (int)((d->bounds[i][2] + d->mean[0] + xoff) * face->xSize + half_window_width + face->x);
      roi.vmin_y[current_scale_ind] = (int)((d->bounds[i][4] + d->mean[1] + yoff) * face->ySize - half_window_width + face->y);
      roi.vmax_y[current_scale_ind] = (int)((d->bounds[i][5] + d->mean[1] + yoff) * face->ySize + half_window_width + face->y);
      last_scale_ind = current_scale_ind;
    }
  }
  // Now, set the current stream to use this ROI
  mpi->SetROI(roi);
  return GPrior(d->mean, d->cov, d->invCov);
}


int MPEyeFinder::findEyes(const RImage<MPISEARCH_OBJECT_TYPE> &pixels, list<FaceObject *> &mFaces,
			  float WINSHIFT, float overlap , combine_mode mode){

  // First, find the faces using built-in face detector weights and search algorithm
				  /*int nBoxes = mFaces.size();
			int myCont = 0;
			if(nBoxes != 0) {
				while(!mFaces.empty( ))
				//while ( myCont< nBoxes )
				{
					//Square face = faces.front();  
					mFaces.pop_front();
				}
			}*/
	list<FaceObject *>::iterator currentFace = mFaces.begin();
    list<FaceObject *>::iterator lastFace = mFaces.end();
    for(; currentFace != lastFace; ++currentFace){
	    //printf("i: %d\n",i);
	    FaceObject *hTracker = *currentFace;
        delete hTracker;
    }
  mFaces.clear();
  FaceBoxList faces;
  int numwindows = search(pixels, faces, 1, WINSHIFT);
  if ( overlap != 0 )
	faces.simplify( overlap );
  //std::cout << "Found " << faces.size() << " faces after simplification." << endl;
  //setDebug(true);
  //cout << "debug: " << debug << endl;

  //if(faces.size()==0){
  //  pixels.print(20);
  //  setDebug(true);
  //  numwindows = search(pixels, faces, 1, WINSHIFT);
  //}
  int totalRightEyes, totalLeftEyes;
  if(faces.size() != 0) {
    // for each found face, find eyes
    list<Square>::iterator face = faces.begin();
    list<Square>::iterator last_face = faces.end();
    for( ; face != last_face; ++face){
      //cout << "Found face at (x=" << face->x<<", y=" << face->y << ", size= "<< face->size << ")" << endl;
      FaceObject *current_face = new FaceObject(face);
      if (mode != face_only){

	// search for left and right eyes
    GPrior rightPrior = setROI(current_face,righteye);
	totalRightEyes = eyeSearch(right_eye_stream, right_eye_data, current_face->rightEyes, current_face, rightPrior, righteye, mode);
	rightPrior.Release();
	//std::cout << "totalRightEyes: " << totalRightEyes << ", rightEyes.size(): "<< current_face->rightEyes.size() << endl;
	GPrior leftPrior = setROI(current_face, lefteye);
	totalLeftEyes = eyeSearch(left_eye_stream, left_eye_data, current_face->leftEyes, current_face, leftPrior, lefteye, mode);
	leftPrior.Release();
	//std::cout << "totalLeftEyes: " << totalLeftEyes << ", leftEyes.size(): "<< current_face->leftEyes.size() << endl;
      }
	//printf("Memory leak\n");
      // make final hypothesis of best eyes and add face to list
      current_face->posterior(mode);
      mFaces.push_front(current_face);
	  //delete current_face;
    }
  }
  //faces.objects.~list();
  /*nBoxes = faces.size();
			myCont = 0;
			if(nBoxes != 0) {
				while(!faces.empty( ))
				//while ( myCont< nBoxes )
				{
					//Square face = faces.front();  
					faces.pop_front();
				}
			}*/
  faces.clear();
  //delete faces;
  return numwindows;
}

int MPEyeFinder::findEyesUsingOCVFaceDetect(const RImage<MPISEARCH_OBJECT_TYPE> &pixels, list<FaceObject *> &mFaces, CvSeq* cvFaces,
			  float WINSHIFT, float overlap , combine_mode mode){

  // First, find the faces using built-in face detector weights and search algorithm
	list<FaceObject *>::iterator currentFace = mFaces.begin();
    list<FaceObject *>::iterator lastFace = mFaces.end();
    for(; currentFace != lastFace; ++currentFace){
	    //printf("i: %d\n",i);
	    FaceObject *hTracker = *currentFace;
        delete hTracker;
    }
  mFaces.clear();
  FaceBoxList faces;
  int faceSize=0;
  int i;
  for( i = 0; i < (cvFaces ? cvFaces->total : 0); i++ )
		{
			CvRect* r = (CvRect*)cvGetSeqElem( cvFaces, i );
			if (r->width > r->height)
				faceSize = r->width;
			else
				faceSize = r->height;
			/*pt1.x = r->x*scale+rect.x;
			pt2.x = (r->x+r->width)*scale+rect.x;
			pt1.y = r->y*scale+rect.y;
			pt2.y = (r->y+r->height)*scale+rect.y;*/
			//printf("faceSize: %d\n",faceSize);
			faces.push_front(Square (faceSize, r->x,r->y,0));
			//delete r;
  }
			
  //int numwindows = search(pixels, faces, 1, WINSHIFT);
  stream.mpi->m_stride = WINSHIFT;
  this->integrateImages(pixels, stream);
  int numwindows = cvFaces->total;
  //printf("numwindows: %d\n",numwindows);
  if ( overlap != 0 )
	  faces.simplify( overlap );
  //faces.simplify(0.05f);
  //std::cout << "Found " << faces.size() << " faces after simplification." << endl;
  //setDebug(true);
  //cout << "debug: " << debug << endl;

  //if(faces.size()==0){
  //  pixels.print(20);
  //  setDebug(true);
  //  numwindows = search(pixels, faces, 1, WINSHIFT);
  //}
  int totalRightEyes, totalLeftEyes;
  if(faces.size() != 0) {
    // for each found face, find eyes
    list<Square>::iterator face = faces.begin();
    list<Square>::iterator last_face = faces.end();
    for( ; face != last_face; ++face){
      //cout << "Found face at (x=" << face->x<<", y=" << face->y << ", size= "<< face->size << ")" << endl;
      FaceObject *current_face = new FaceObject(face);
      if (mode != face_only){

	// search for left and right eyes
    GPrior rightPrior = setROI(current_face,righteye);
	//integrateImages(pixels, right_eye_stream);
	//printf("Entering to look for eyes\n");
	totalRightEyes = eyeSearch(right_eye_stream, right_eye_data, current_face->rightEyes, current_face, rightPrior, righteye, mode);
	//std::cout << "totalRightEyes: " << totalRightEyes << ", rightEyes.size(): "<< current_face->rightEyes.size() << endl;
	GPrior leftPrior = setROI(current_face, lefteye);
	totalLeftEyes = eyeSearch(left_eye_stream, left_eye_data, current_face->leftEyes, current_face, leftPrior, lefteye, mode);
	//std::cout << "totalLeftEyes: " << totalLeftEyes << ", leftEyes.size(): "<< current_face->leftEyes.size() << endl;
      }

      // make final hypothesis of best eyes and add face to list
      current_face->posterior(mode);
      mFaces.push_front(current_face);
	  //delete current_face;
    }
  }
  faces.clear();
  return numwindows;
}


// EyeSearch imitates the built-in search, but it doesn't need to re-integrate images,
// and doesn't need to worry about so many variable options, such as block flags or
// returning real valued activations.
int MPEyeFinder::eyeSearch(MPISearchStream<MPISEARCH_OBJECT_TYPE> &thestream, FeatureData &thedata, vector< EyeObject > &eyelist, FaceObject *current_face, GPrior &gp, feature_type eye_type, combine_mode mode){
  int scale_index;
  float scale_factor;
  int x=0,y=0;
  int totalEyesFound = 0;
  double activation;
  double *activation_ptr = &activation;
  float xoff=0.0, yoff=0.0;
  float meanSub[3], rtnVector[3];
  float lp[1];

  // Compute the offset of the eye location from the ul corner of the patch
  // NOTE: I think this is correct, but it bears going over
  // It should reduce to an offset of exactly halfpatch in the eye_centered condition
  float patchWidthPct=0.0;
  //printf("Before eye offsets\n");
  getEyeOffsets(m_rez, m_centering, xoff, yoff);
  //printf("after eye offsets\n");
  getPatchWidth(m_rez, patchWidthPct);
  //printf("after patch width\n");
  //printf("patch width: %f\n",thedata.patch_width);
  float patch_ratio = patchWidthPct * leftROIdata.mean[2] * 2.0f; // get halfpatch size as proportion of face size
  float my_yoff = yoff / patch_ratio; // convert offset from face size ratio to halfpatch ratio
  float my_xoff = xoff / patch_ratio; // convert offset from face size ratio to halfpatch ratio
  if(eye_type == lefteye) my_xoff = -my_xoff; // note the mirror imaging for right eyes
  my_yoff = (0.5f-my_yoff) * (thedata.patch_width-1); // eye's offset from ul corner =  halfpatch-offset
  my_xoff = (0.5f-my_xoff) * (thedata.patch_width-1); // eye's offset from ul corner =  halfpatch-offset
  //printf("Entered to eyeSearch\n");
  MPIImagePyramid<MPISEARCH_OBJECT_TYPE>::const_iterator scale = thestream.mpi->begin(), last_scale = thestream.mpi->end();
  for( ; scale != last_scale; ++scale){ 
    scale_index = scale.getScale(scale_factor);
    int current_xoff = static_cast<int>(scale_factor * my_xoff);
    int current_yoff = static_cast<int>(scale_factor * my_yoff);
	//std::cout << "scale_factor: " << scale_factor << ", current_xoff: " << current_xoff << ", current_yoff: " << current_yoff << endl;
    // get pointers to cached values for this scale
	//printf("Scale cyle entered\n");
    MPISEARCH_OBJECT_TYPE sf2 = scale_factor * scale_factor;
    CornerCache<MPISEARCH_OBJECT_TYPE> **corners = thestream.corners[scale_index];
    MPISEARCH_OBJECT_TYPE *fns = thestream.fns[scale_index];
    CornerCache<MPISEARCH_OBJECT_TYPE> *nw_c = thestream.nw_c[scale_index];
    MPISEARCH_OBJECT_TYPE nw_fn = thestream.nw_fn[scale_index];
    MPIScaledImage<MPISEARCH_OBJECT_TYPE>::const_iterator window = (*scale).begin(), last_window = (*scale).end();
    for( ; window != last_window; ++window){
      // check the window.  If it passes, cascade_level will be 1.  Otherwise, it will be  <= 0.
      double cascade_level = classifyWindow(window, thedata, corners, fns, thestream.norm_window, nw_c,
					    nw_fn, scale_factor, sf2, activation_ptr);
      switch(mode){
      case none:

	window.getCoords(x,y);
	eyelist.push_back(EyeObject(eye_type, (float)(x+current_xoff), (float)(y+current_yoff), scale_factor, activation, cascade_level));
	totalEyesFound++;
	//cout << "("<< x+current_xoff << "," << y+current_yoff << "): " << activation << endl; 
	break;
			
      case wt_max:
      case wt_avg:
	if(cascade_level > 0){
	  window.getCoords(x,y);
	  x += current_xoff;
	  y += current_yoff;
					
	  // gaussian section
	  meanSub[0] = ((x-current_face->x)/current_face->xSize)-gp.mean[0]; 
	  meanSub[1] = ((y-current_face->y)/current_face->ySize)-gp.mean[1]; 
	  int scale_size = static_cast<int>(scale_factor*(thedata.patch_width-1));
	  meanSub[2] = ((scale_size*gp.mean[2])-(current_face->xSize*gp.mean[2]))/scale_size;
	  matrixMult(1, 3, meanSub, 3, 3, gp.invCov, rtnVector);
	  matrixMult(1, 3, rtnVector, 3, 1, meanSub, lp);
	  activation = (2*activation) - (.5*(*lp));

	  if(mode == wt_max){
	    eyelist.push_back(EyeObject(eye_type, (float)x, (float)y, scale_factor, activation, cascade_level));
	    totalEyesFound++;
	  }
	  else if(mode == wt_avg){
	    if(eyelist.size() < 10){
	      eyelist.push_back(EyeObject(eye_type, (float)x, (float)y, scale_factor, activation, cascade_level));
	      totalEyesFound++;
	      if (eyelist.size() == 10)
		sort(eyelist.begin(), eyelist.end());
	    }
	    else {
	      if (activation > eyelist[0].activation){
		eyelist[0] = EyeObject(eye_type, (float)x, (float)y, scale_factor, activation, cascade_level);
		sort(eyelist.begin(), eyelist.end());
	      }
	    }
	  }
	}
	break;
      case average:
	window.getCoords(x,y);
	x += current_xoff;
	y += current_yoff;
	if(eyelist.size() < 10){
	  eyelist.push_back(EyeObject(eye_type, (float)x, (float)y, scale_factor, activation, cascade_level));
	  totalEyesFound++;
	  if (eyelist.size() == 10)
	    sort(eyelist.begin(), eyelist.end());
	}
	else {
	  if (activation > eyelist[0].activation){
	    eyelist[0] = EyeObject(eye_type, (float)x, (float)y, scale_factor, activation, cascade_level);
	    sort(eyelist.begin(), eyelist.end());
	  }
	}
	break;
      default:
	eyelist.push_back(EyeObject(eye_type, (float)x, (float)y, scale_factor, activation, cascade_level));
	break;
      };
    } // end window loop
  }//end scale loop
  //cout << endl;
  return totalEyesFound;
}




void MPEyeFinder::getPatchWidth(patch_rez p, float &patchWidthPct){
  switch(p){ // note: in near future, change this to look up p from FeatureData
  case largest_2:
    patchWidthPct = 3.4f; break;
  case largest:
    patchWidthPct = 1.5f; break;
  case eye_dist:
    patchWidthPct = 1; break;
  case half_dist:
    patchWidthPct = .5f; break;
  case eye_only:
    patchWidthPct = .22f; break;
  case smallest:
    patchWidthPct = .11f; break;
  }
}

void MPEyeFinder::getEyeOffsets(patch_rez p, centering_condition centering, float &xoff, float &yoff){
  switch(centering) {
  case eye_centered:
    xoff = 0;
    yoff = 0;
    break;
  case face_centered:
    switch(p) {
    case largest_2:
      xoff = 0;
      yoff = 0;
      break;
    case largest:
      xoff = 0.10451f;
      yoff = 0.09642f;
      break;
    case eye_dist:
      xoff = 0.090967f;
      yoff = 0.082883f;
      break;
    case half_dist:
      xoff = 0;
      yoff = 0;
      break;
    case eye_only:
      xoff = 0;
      yoff = 0;
      break;
    case smallest:
      xoff = 0;
      yoff = 0;
      break;
    }
    break;
  }
}

int MPEyeFinder::matrixMult(int row1,int col1, float *matrix1, int row2, int col2, float *matrix2, float *rtn_matrix){
  if(col1 != row2)
    return 0;
  float comb;
  for(int cur_row = 0; cur_row < row1; ++cur_row){
    for(int cur_col = 0; cur_col < col2; ++cur_col){
      comb = 0;
      for(int cur_pos = 0; cur_pos < col1; ++cur_pos){
	comb += matrix1[(cur_row*col1)+cur_pos] * matrix2[cur_col+(cur_pos*col2)];
      }
      int pos = cur_row*col2+cur_col;
      rtn_matrix[pos] = comb;
    }
  }
  return 1;
}

int MPEyeFinder::matrixMult(int row1,int col1, float *matrix1, int row2, int col2, vector< vector< float > > matrix2, float *rtn_matrix){
  if(col1 != row2)
    return 0;
  float comb;
  for(int cur_row = 0; cur_row < row1; ++cur_row){
    for(int cur_col = 0; cur_col < col2; ++cur_col){
      comb = 0;
      for(int cur_pos = 0; cur_pos < col1; ++cur_pos){
	comb += matrix1[(cur_row*col1)+cur_pos] * matrix2[cur_pos][cur_col];//[(cur_col*row2)+cur_pos];
      }
      rtn_matrix[(cur_row*col2)+cur_col] = comb;
    }
  }
  return 1;
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

