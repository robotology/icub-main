/*  
 *  mpisearch.h
 *
 *  Created by Ian Fasel on Feb 02, 2003. (mpisearch)
 *    Based on code written by Ryan Dahl, Apr 2, 2002, (viola++)
 *      which was based on matlab code by Ian Fasel August 2001, (violaSearch.m) 
 *      which was based on matlab code written by John Hershey, May 2001, (viola.m)
 *      which was based on a talk and paper by Paul Viola and Michael Jones, 2001.
 *  Fixes: 
 * 
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
#ifndef _MPISEARCH_H_
#define _MPISEARCH_H_

#include "mpiimage.h"
#include "featuredata.h"
#include "faceboxlist.h"
#include <string>
#include <list>
#include <string>
#include <iostream>
#include <fstream>

extern "C" {
#include <math.h>
#include <string.h>
#include <stdlib.h>
}


#define sqr(x) ((x)*(x))
#define cube(x) ((x)*(x)*(x))
#ifndef PI
#define PI 3.141592654f
#endif

using namespace std;

#define TRUE 1
#define FALSE 0

const float SCALE_FACTOR = 1.2f;
const float MIN_VARIANCE_F = 0.000729f;

template < class T > 
class CornerCache{
public:
  int scaledCornerX;
  int scaledCornerY;
  int scaledIndex;
  T value;
};

/***  Provides an architecture for performing operations on every
*  sub-window of an image at multiple scales.
*/
template < class T > class MPISearchObjectDetector; // forward declaration

// MPISearchStream: holds all the memory and caches for the ObjectDetector  
template < class T > 
class MPISearchStream {
  // friend class MPISearchObjectDetector;  // C++ classes don't inherit friends -- so how can I do data hiding?
    // Probably the only real way is to make Stream a class that only has scope within ObjectDetector
public:
  // Member Functions
  MPISearchStream();
  ~MPISearchStream();
  void init(const int width, const int height, FeatureData &data, double WINSHIFT=1.25);
  void reset(const int width, const int height, FeatureData &data, double WINSHIFT=1.25);
  void init(const MPISearchStream &s, FeatureData &data, double WINSHIFT=1.25);
  void reset(const MPISearchStream &s, FeatureData &data, double WINSHIFT=1.25);
  void release();
  ROI getROI() const;
  void SetROI(ROI &theroi);
  void SetROI(const int &minx,const int &maxx,const int &miny,const int &maxy,const int &minsc,const int &macsc);
//private:
  void makeNormWindow_MPI_CacheCorners(FeatureData &thedata, double WINSHIFT_);
  CornerCache< T >** cacheCorners (float &scale_factor, FeatureData &thedata,  T * &fns, int &image_width,
                              Corner norm_window[4], CornerCache< T > nwc[4],  T  &nw_fn);
  void deleteCacheCorners ( CornerCache< T >** &pcc,  T * &fns );
  void makeNormWindow( Corner norm_window[4], FeatureData &thedata);
  
  // Member data
  FeatureData *m_data;
  RImage< T > *pixels2;
  vector< RImage< T > *> images;
  Corner norm_window[4];
  int height, width, block_height, block_width;
  vector< typename MPIScaledImage< T >::const_iterator > block_windows;
  int FailedCycles;
  MPIImagePyramid< T > *mpi;
  vector< CornerCache< T > *> nw_c;     // normalization window corner cache (for scales);
  vector< T > nw_fn;
  int numfeatures;
  vector< CornerCache< T > **> corners; // feature corner cache (for scales);
  vector< T* >  fns;
  bool do_integral;
  // my_memory keeps track of use of the image. The destructor will only erase the image if 
  //   my_memory == 0. The proper way to do this of course is reference counting, but we're 
  //   getting by as is.  Still, I'd like to migrate to Boost pointers in the future.
  bool my_memory; 
  bool allocated;
};

// virtual class, requires that a base class be derived that handles loading FeatureData
template <class T>
class MPISearchObjectDetector {
 public:
  MPISearchObjectDetector ( );
  virtual ~MPISearchObjectDetector ( );

  // pixels -- 			the image to be scanned
  // faces  -- 			structure to store the detected objects (faces)
  // flags  -- 			an array with a value corresponding to each window at all scales
  // output_values -- 		Stick the output of the classifier up to this point here
  // box -- 			flag to return the found face boxes.  Set to 0 to get activation only.
  // Note: flags and output_values are being used during learning, only used from Matlab right now
  int search (const RImage< T > &pixels, FaceBoxList &faces, int USE_BLOCK_FLAGS = 1,
            float WINSHIFT = 1.25, double* index_flags = NULL,
            double* output_values = NULL, int box = 1);
  void initStream(const int width, const int height, double WINSHIFT=1.25);
  void resetStream(const int width, const int height, double WINSHIFT=1.25);
  void releaseStream();

  bool DataLoaded();
  void setDebug(const bool val);
  void setDebug2(const bool val);

  void AdjSearchWindow(Square &Front); //note only works if FailedCycles are updated
  void AdjSearchWindow(TSquare<float> &Front); //note only works if FailedCycles are updated
  void setPixelMax(T maxpixval);
  int FailedCycles();
  bool allocated();
  RIntegral<T>* getIntegralPtr(void); // to get intergral image pointer
 protected:

  double classifyWindow(typename MPIScaledImage< T >::const_iterator &window,  FeatureData &thedata, 
	   CornerCache< T >** &corners,  T * &fns, Corner norm_window[4], CornerCache< T > nw_c[4],
        T  nw_fn, float scale_factor,  T  sf2, double* &activation);
  //void AdjSearchWindow(Square &Front, int Width, int Height);//note only works if FailedCycles are updated
  void printData (FeatureData &thedata, int ind);
  virtual void processFace(typename MPIScaledImage< T >::const_iterator &window,
                           const  T  &one_over__sf2_times_std,
                           const  T  & mean_over_std, const int &numFaces);

  void integrateImages(const RImage<  T >& p, MPISearchStream< T > &stream);
  //void integrateImages(const RImage<  T > &p, vector< RImage<  T >* > &i, const int D);
  bool debug, debug2;
  int last_width, last_height;
  MPISearchStream< T > stream;
  FeatureData data;
	T minvariance;
};

///////////////////////
// These are the virtual functions that derived classes MUST modify

// default to no debugging, and minimum variance appropriate for gray pixels in [0,1]
// Note that you should call setPixelMax if your calling function will scale pixels to
// be between e.g., 0 and 255.  It should work the same, just be sure to call setPixelMax
template <class T>
MPISearchObjectDetector<T>::MPISearchObjectDetector():debug(false),debug2(false),
	minvariance( (T)MIN_VARIANCE_F ) {}

template < class T >
MPISearchObjectDetector<T>::~MPISearchObjectDetector () {  }
////////////////////////////////

template < class T >
void MPISearchObjectDetector<T>::processFace(typename MPIScaledImage< T >::const_iterator &window,
                                          const  T  &a, const  T  &b, const int &numFaces){}

template < class T >
void MPISearchObjectDetector<T>::initStream(const int width, const int height, double WINSHIFT){
  stream.init(width, height, data, WINSHIFT);
}
template < class T >
void MPISearchObjectDetector<T>::resetStream(const int width, const int height, double WINSHIFT){
  stream.reset(width, height, data, WINSHIFT);
}
template < class T >
void MPISearchObjectDetector<T>::releaseStream(){
  if(stream.allocated)
    stream.release();
}

template < class T >
void MPISearchObjectDetector<T>::setPixelMax(T maxpixval){
	minvariance = sqr(maxpixval * static_cast<T>(0.027));
}

// This search function is somewhat specific to faces, but most of the internals are portable
template < class T >
int MPISearchObjectDetector<T>::search (const RImage< T > &pixels, FaceBoxList &faces, int USE_BLOCK_FLAGS, float WINSHIFT, double * index_flags, double * output_values, int box)
{
  // The commented code below (checking if size changed) breaks the eyefinder. So don't do it!
  if(!stream.allocated){// || (pixels.width != last_width) || (pixels.height != last_height) ){
    // std::cout << "MPISearchObjectDetector<T>::search: stream not allocated. Allocating data" << endl;
    stream.reset(pixels.width, pixels.height, data,  WINSHIFT);
  //  last_width = pixels.width; last_height = pixels.height;
  } else
    stream.mpi->m_stride = WINSHIFT;
  //cout << "mpisearch::search: " << stream.mpi->m_roi << endl;
  //cout << "MPISearchObjectDetector<T>::search: "<< stream.roi << endl;
  //cout << "About to enter integrateImages" << endl;
  integrateImages(pixels, stream);
  //if(debug){
    //cout << "num_cascades: " << data.numcascades << endl; 
    //printData(data, 1);
    //  cout << "thedata.real_fun: " << thedata.real_fun << endl;
  //}

  int numWindows = 0, scale_index;
  float scale_factor;
  bool check_it;
  double default_cascade = 1;
  double default_activation = 0;
  double *cascade_level = &default_cascade;
  double *activation = &default_activation;
  MPIImagePyramid<float>::const_iterator scale = stream.mpi->begin();
  MPIImagePyramid<float>::const_iterator last_scale = stream.mpi->end();
  //if(debug)
  //  stream.images[2]->print(20);
  for( ; scale != last_scale; ++scale){
    // get pointers to cached values for this scale
    scale_index = scale.getScale(scale_factor);
    //cout << "Scale number: " << scale_index << ", scale_factor = " << scale_factor << endl;
     T  sf2 = scale_factor * scale_factor;
    CornerCache< T > **corners = stream.corners[scale_index];
     T  *fns = stream.fns[scale_index];
    CornerCache< T > *nw_c = stream.nw_c[scale_index];
     T  nw_fn = stream.nw_fn[scale_index];
    typename MPIScaledImage< T >::const_iterator window = (*scale).begin(), last_window = (*scale).end();
    for( ; window != last_window; ++window, ++numWindows){
      //if(debug)
      //if( numWindows == (66052 - 1) ){
      //  cout << "Setting debug"<< endl;
      //  setDebug(true);
      //}
      //else
      //  setDebug(false);
      // A little logic here to skip a window for various reasons
      check_it = true;
      if(index_flags){
	//if(debug2)
	//  cout << "index_flags IS NOT NULL!!! " << endl;
        cascade_level = &(index_flags[numWindows]);
        if(*cascade_level != 1)
          check_it = false;
      }
      //if(debug2)
      //cout << "USE_BLOCK_FLAGS="<< USE_BLOCK_FLAGS << endl;
      if(USE_BLOCK_FLAGS){
	//if(debug2)
	//cout << "the BLOCK_FLAG ="<< window.getPixel(2,0) << endl;
	if( window.getPixel(2,0) )
          check_it = false;
      }
      int x, y;
      window.getCoords(x,y);
      //cout << "(" << x << ","<< y<< ") ";
      //if(debug2)
      //cout << "check_it = " << check_it << endl;
      if(check_it){
        if(output_values)
          activation = output_values + numWindows;
        *cascade_level = classifyWindow(window, data, corners, fns, stream.norm_window, nw_c, nw_fn, scale_factor, sf2, activation);
        //if(debug)
        //  cout << "activation: " << *activation << endl; 
        if(*cascade_level > 0) {
	  //if(debug){
	  //cout<< "Found a face" << endl;
	  // cout << "numWindows = " << numWindows << endl;
	  //}
          if(box){
            faces.push_front(window.getSquare());
            //cout << "Face at: (" << faces.front().x << ", " << faces.front().y << ", " << faces.front().scale << ")" << endl;
            if(USE_BLOCK_FLAGS){
              for(int by = 0; by < stream.block_height; ++by)
                for(int bx = -stream.block_width; bx < stream.block_width; ++bx)
                  window.setShiftPixel(2, bx, by, 1);
              stream.block_windows.push_back(window);
            }
          }
        }
      }
    } // end window loop
    //cout << endl;
    if(USE_BLOCK_FLAGS){
      while(stream.block_windows.size() > 0){
        typename MPIScaledImage< T >::const_iterator window = stream.block_windows[0];
        for(int by = 0; by < stream.block_height; ++by)
          for(int bx = -stream.block_width; bx < stream.block_width; ++bx)
            window.setShiftPixel(2, bx, by, 0);
        stream.block_windows.pop_back();
      }
    }

  }  // end scale loop

  /**************added*****************/
  if (!faces.empty())
    stream.FailedCycles = 0;
  else{
    //cout << "Failed" << endl;
    stream.FailedCycles++;
  }
  /**************end*****************/
  return numWindows;
}


// This function uses the weights for an additive classifier such as AdaBoost or GentleBoost.  It currently
// is somewhat specific to the face-processing task, but you can make a similar function that doesn't have these
// assumptions.
// First, it calculates the mean and variance of the window.
template < class T >
double MPISearchObjectDetector<T>::classifyWindow(typename MPIScaledImage< T >::const_iterator &window, FeatureData &thedata,
                                                  CornerCache< T >** &corners,  T* &fns, Corner norm_window[4], CornerCache< T > nw_c[4],
                                                  T nw_fn, float scale_factor, T sf2, double* &activation){

  T mean = 0.0, mean2 = 0.0, standard_deviation, one_over__sf2_times_std, mean_over_std, f, std2;
  
  //if(debug)
  //  cout << "Classifying window" << endl;
  // calculate the statistics for this sub window
  for(int corner = 0; corner < 4; corner++) {
    mean  += window.getPixel0( nw_c[corner].scaledIndex) * norm_window[corner].value;
    mean2 += window.getPixel1( nw_c[corner].scaledIndex) * norm_window[corner].value;
  }
  mean /= nw_fn;
  mean2 /= nw_fn;
  std2 = mean2 - mean*mean;  

  // if the std deviation is below some point then ignore
  //if(standard_deviation < 0.027)
  if(std2 < minvariance)
    return -99999; // special flag for std bailout
  standard_deviation = sqrt(std2)*thedata.stdAdjusts[static_cast<int>(scale_factor+0.5f)-1];
  //if(debug){
  //  cout << "mean2 - mean*mean: " << (mean2 - mean*mean);
  //  cout << ", mean: " << mean << ", mean2: " << mean2 << ", std: " << standard_deviation << endl;
  //}
  // cache a few values used in normalization
  // one_over__sf2_times_std = 1/(sf2*standard_deviation);
  one_over__sf2_times_std = (float) 1.0/(sf2*standard_deviation);
  mean_over_std = mean/standard_deviation;
  T aH = 0.0f, a,b,c,d;
  
  ///////// Begin Cascade (Strong Classifier) /////////////////////////////
  // run through each stage in the cascade
  for (int cascade_iter = 0; cascade_iter < thedata.numcascades; cascade_iter++){
    Cascade &cascade = thedata.cascades[cascade_iter];
    if(!thedata.preserve_aH)
      aH = 0.0;

    if(!thedata.real_fun){
      // run through the features of each cascade stage
      for (int feature_iter = cascade.start; feature_iter <= cascade.end; feature_iter++){
        Feature &feature = thedata.features[feature_iter];
        f = 0.0f;
        // for each corner in the feature
        CornerCache< T > *feature_corners = corners[feature_iter];
        //for(int corner = 0; corner < feature.numcorners; corner++) {
        //  f+= window.getPixel0( feature_corners[corner].scaledIndex ) * feature.corners[corner].value;
        //}
        for(int corner = 0; corner < feature.numcorners; corner+=4) {
              a = window.getPixel0( feature_corners[corner].scaledIndex ) * feature_corners[corner].value;
              b = window.getPixel0( feature_corners[corner+1].scaledIndex ) * feature_corners[corner+1].value;
              c = window.getPixel0( feature_corners[corner+2].scaledIndex ) * feature_corners[corner+2].value;
              d = window.getPixel0( feature_corners[corner+3].scaledIndex ) * feature_corners[corner+3].value;
          f+= a + b + c + d;
        }
        // Note: if T is float, there is floating point roundoff error here, but that should be ok.
        
        // normalize..
        //f = (f/sf2 - fns[feature_iter] * mean) / standard_deviation;
        f = f*one_over__sf2_times_std - fns[feature_iter]*mean_over_std;
        
        //////////  Begin simple classifier (Weak Learner) /////////////
        // Threshold classifier, using AdaBoost
        // 1) maybe take absolute value if indicated
        // 2) Threshold (weight is in {-1, +1}, so its above *or below* thresh)
        if(f < 0) if(feature.abs) f = -f;
        if(thedata.plus_minus_one)
          if(feature.weight * f > feature.bias) aH += feature.alpha; else aH -= feature.alpha;
        else
          if(feature.weight * f > -feature.bias) aH += feature.alpha;
        //////////  End simple classifier (Weak Learner)  //////////////
        //if(debug2){
        // cout << "f = " << f << endl;
        // cout << "aH = " << aH << endl;
        //}
      }
    } else {
      // run through the features of each cascade stage
      for (int feature_iter = cascade.start; feature_iter <= cascade.end; feature_iter++){
        Feature &feature = thedata.features[feature_iter];
        f = 0.0f;
        // for each corner in the feature
        CornerCache< T > *feature_corners = corners[feature_iter];
        //for(int corner = 0; corner < feature.numcorners; corner++) {
        //  f+= window.getPixel0( feature_corners[corner].scaledIndex ) * feature.corners[corner].value;
        //}
        for(int corner = 0; corner < feature.numcorners; corner+=4) {
              a = window.getPixel0( feature_corners[corner].scaledIndex ) * feature_corners[corner].value;
              b = window.getPixel0( feature_corners[corner+1].scaledIndex ) * feature_corners[corner+1].value;
              c = window.getPixel0( feature_corners[corner+2].scaledIndex ) * feature_corners[corner+2].value;
              d = window.getPixel0( feature_corners[corner+3].scaledIndex ) * feature_corners[corner+3].value;
          f+= a + b + c + d;
        }
        // Note: I think there is numerical error here because of floating point roundoff error.  But that realy should be ok.
        
        // normalize..
        //f = (f/sf2 - fns[feature_iter] * mean) / standard_deviation;
        //if(debug){
        //  cout << "f (before normalization): " << f << endl;
        //}
        f = f*one_over__sf2_times_std - fns[feature_iter]*mean_over_std;
        //if(debug){
        //  cout << "f (after normalization): " << f << endl;
        //}
      
        //////////  Begin simple classifier (Weak Learner) /////////////
        // Using some real-valued WeakLearner, perhaps learned via GentleBoost
        // 1) scale feature output to 1:nl
        // 2) round and use as index into tuning curve
        // WARNING!!!! WEIRD PROBLEMS WITH 0-1 INDEXING (MATLAB VS C) ETC.  
        //     PLEASE MAKE SURE THIS IS FIXED!
        f = (float)((f - feature.bias) * feature.nl_over_range);
        //if(debug){
        //  cout << "feature.nl_over_range: " << feature.nl_over_range << endl;
        //  cout << "f: " << f << endl;
        //  cout << "feature.tuning_curve: ";
        //  for(int tc =0; tc <= thedata.maxval; ++tc)
        //    cout << feature.tuning_curve[tc] << " " ;
        //  cout << endl;
              
        //  cout << "thedata.minval: " << thedata.minval << ", feature.tuning_curve[thedata.minval]" 
        //       << feature.tuning_curve[thedata.minval] << endl;
        // cout << "thedata.maxval: " << thedata.maxval << ", feature.tuning_curve[thedata.maxval]" 
        //       << feature.tuning_curve[thedata.maxval] << endl;
        //}
        if(f<thedata.minval)  // was 1
          aH += (float)feature.tuning_curve[thedata.minval]; // was 1
        else if(f>thedata.maxval) // was nl
          aH += (float)feature.tuning_curve[thedata.maxval]; // was nl
        else
          aH += (float)feature.tuning_curve[static_cast<int>(f+0.5f)];
        if(debug){
					std::cout << "f: " << f << ", aH: " << aH << std::endl;
        }
      }
    }
    
    ///// Cascade threshold -- the "strong classifier"'s final nonlinearity ////
    if(aH < cascade.thresh){
      //cout << "exit the cascade, there is no face after cascade " << cascade_iter << endl;
      *activation = static_cast<double>(aH);
      //if(debug){
      // cout << "aH = " << aH << endl;
      // cout << "*activation = " << *activation << endl;
      //}
      return static_cast<double>(-cascade_iter);
    }
  }
  *activation = static_cast<double>(aH);
  //if(debug){
  //  cout << "aH = " << aH << endl;
  //  cout << "*activation = " << *activation << endl;
  //}
  return 1.0;
}

template < class T >
void MPISearchObjectDetector<T>::integrateImages(const RImage<  T  >& pixels, MPISearchStream<T> &thestream){
   T  temp;
   T  *p, *q;
  ROI roi = thestream.mpi->getROI();
  // The ROI in the MPIImage Pyramid is too large by 1 because it is looking at integral images.
  roi.m_max_x -= 1;  roi.m_max_y -= 1; // sorry about this, I know its confusing
  for(int y = roi.m_min_y; y < roi.m_max_y; y++) {
    p = pixels.array + (pixels.width*y + roi.m_min_x);
    q = thestream.pixels2->array + (thestream.pixels2->width*y + roi.m_min_x);
    for(int x = roi.m_min_x; x < roi.m_max_x; x++) {
      //temp = pixels.getPixel(x,y);
      temp = *p++;
      //thestream.pixels2->setPixel(x, y, temp * temp);
      *q++ = temp*temp; 
    }
  }
  // create integral images
  static_cast<RIntegral< T  >* >(thestream.images[0])->integrate(pixels, roi);
  static_cast<RIntegral< T  >* >(thestream.images[1])->integrate(*(thestream.pixels2), roi);
}

template < class T >
void MPISearchObjectDetector<T>::printData (FeatureData &thedata, int ind) {
	std::cerr << "patchsize = " << thedata.patchsize << endl;
  std::cerr << "patch_width = " << thedata.patch_width << endl;
  std::cerr << "patch_height = " << thedata.patch_height << endl;
  std::cerr << "numfeatures = " << thedata.numfeatures << endl;
  std::cerr << "numcascades = " << thedata.numcascades << endl;
  std::cerr << "normOffset.top = " << thedata.normOffset.top << endl;
  std::cerr << "normOffset.left = " << thedata.normOffset.left << endl;
  std::cerr << "normOffset.right = " << thedata.normOffset.right << endl;
  std::cerr << "normOffset.bottom = " << thedata.normOffset.bottom << endl;
  std::cerr << "numStdAdjusts = " << thedata.numStdAdjusts << endl;

  std::cerr << "feature[" << ind << "]:" << endl;
  std::cerr << "numcorners = " << thedata.features[ind].numcorners << endl;
  std::cerr << "real_fun" << thedata.real_fun << endl;
  if(!thedata.real_fun){
    std::cerr << "alpha = " << thedata.features[ind].alpha << endl;
    std::cerr << "bias = " << thedata.features[ind].bias << endl;
    std::cerr << "weight = " << thedata.features[ind].weight << endl;
    std::cerr << "abs = " << thedata.features[ind].abs << endl;
  } else {
    std::cerr << "bias = " << thedata.features[ind].bias << endl;
    std::cout << "Weights:";
    for(int j = 0; j <= thedata.nl; ++j)
      std::cout << " " << thedata.features[ind].tuning_curve[j];
    std::cout << endl;
  }
  //cerr << "range = " << thedata.features[ind].range << endl;
  for (int i = 0; i<thedata.features[ind].numcorners; ++i) {
    std::cerr << "corners_x[" << i << "] = " << thedata.features[ind].corners[i].x << endl;
    std::cerr << "corners_y[" << i << "] = " << thedata.features[ind].corners[i].y << endl;
    std::cerr << "corners_value[" << i << "] = " << thedata.features[ind].corners[i].value << endl;
  }

  if(thedata.numStdAdjusts){
    std::cerr << "stdAdjusts:" << endl;
    for (int j = 0; j < thedata.numStdAdjusts; ++j)
      std::cerr << thedata.stdAdjusts[j] << " ";
    std::cerr << endl;
  }

  std::cerr << "Cascades:" << endl;
  for (int k = 0; k < thedata.numcascades; ++k) 
    std::cerr << thedata.cascades[k].start << "\t" << thedata.cascades[k].end << "\t" << thedata.cascades[k].thresh << endl; 
}

template < class T >
void MPISearchObjectDetector<T>::setDebug(const bool val){ debug = val; }

template < class T >
void MPISearchObjectDetector<T>::setDebug2(const bool val){ debug2 = val; }

template < class T >
int MPISearchObjectDetector<T>::FailedCycles(){ return stream.FailedCycles; };

template < class T >
bool MPISearchObjectDetector<T>::DataLoaded(){ return ( data.numfeatures > 0 );}

template < class T >
bool MPISearchObjectDetector<T>::allocated(){ return stream.allocated;}

template< class T >
RIntegral<T>* MPISearchObjectDetector<T>::getIntegralPtr( void ){
  // to get intergral image pointer
  return static_cast<RIntegral<T>*>(stream.images[0]);
}

/*adjusts window for faster search.  Note if FailedCycles are not used it will not readjust to 
span a larger window to search for a face*/
template < class T >
void MPISearchObjectDetector<T>::AdjSearchWindow(TSquare< float > &F) // Width and Height are not needed as arguments
{
  Square S(static_cast<int>(F.size), static_cast<int>(F.x), static_cast<int>(F.y), static_cast<int>(F.scale));
  AdjSearchWindow(S);
}
template < class T >
void MPISearchObjectDetector<T>::AdjSearchWindow(Square &Front)
{
  //std::cout << "MPISearchObjectDetector<T>::AdjSearchWindow" << stream.mpi->getROI() << endl;
	int midSize;
	int minX, maxX, minY, maxY, minscale, maxscale;
         static Square oldFront;
	 
	 //std::cout << Front << endl;

     // IMPORTANT NOTE:
     // stream.SetROI keeps itself consistent: i.e., it keeps the ROI within the bounds of the image pyramid.
     // Therefore, it is not necessary to do bounds checking yourself.

     if (!Front.size) //set up roi for next image, makes it smaller to speed up processing
         Front = oldFront;
     else
         oldFront = Front;
     

     if (stream.FailedCycles > 5)
     {
       ROI theroi;
       stream.SetROI(theroi);
     }
     else
     {
		//each failed cycle roi increases by size of facebox in X and Y direction
		//until it reaches size of image
		midSize = static_cast<int>(ceil((float)Front.size/2.0f));
		minX = Front.x - (midSize * (stream.FailedCycles+1));
		maxX = Front.x + Front.size + (midSize * (stream.FailedCycles+1));
		minY = Front.y - (midSize * (stream.FailedCycles+1));
		maxY = Front.y + Front.size + (midSize * (stream.FailedCycles+1));
                minscale = Front.scale-stream.FailedCycles;
                maxscale = Front.scale+stream.FailedCycles+1;
           	stream.SetROI(minX,maxX,minY,maxY,minscale,maxscale);
     }
}


template< class T >
MPISearchStream< T >::MPISearchStream() : 
  FailedCycles(0), 
  mpi(0), 
  my_memory(false), 
  allocated(false)
{

}

template< class T >
MPISearchStream< T >::~MPISearchStream(){ if(allocated) release(); }
template< class T >
void MPISearchStream< T >::init(const int width_, const int height_, FeatureData &thedata, double WINSHIFT_){
  if(allocated){
    //cout << "MPISearchStream< T >::init:  Tried to init stream, but it thinks its already allocated" << endl;
    release();
  }
  //cout << "MPISearchStream< T >::init: Allocating data for width_ = " << width_ << ", height_ = " << height_ << endl;
  m_data = &thedata;
  width = width_; height = height_;
  do_integral = true;
  allocated = true;
  pixels2 = new RImage< T >(width, height);
  // create integral images
  my_memory = true;
  RIntegral< T > *ii = new RIntegral< T >(width, height);
  RIntegral< T > *ii2 = new RIntegral< T >(width, height);
  RImage< T > * BlockFlag = new RImage< T >(width+1, height+1, 0);
  images.push_back(ii);
  images.push_back(ii2);
  images.push_back(BlockFlag);
  block_height = static_cast<int>(m_data->patch_height/3);
  block_width = static_cast<int>(m_data->patch_width/3);
  makeNormWindow_MPI_CacheCorners(thedata, WINSHIFT_);
}

template< class T >
void MPISearchStream< T >::init(const MPISearchStream &s, FeatureData &thedata, double WINSHIFT_){
  my_memory = false;
  m_data = &thedata;
  width = s.width; height = s.height;
  //printf("stream width: %d\n",width);
  //printf("stream height: %d\n",height);
  do_integral = false;
  pixels2 = s.pixels2;
  images = s.images;
  block_height = static_cast<int>(m_data->patch_height/3);
  block_width = static_cast<int>(m_data->patch_width/3);
  //printf("block width: %d\n",block_width);
  //printf("block height: %d\n",block_height);
  makeNormWindow_MPI_CacheCorners(thedata, WINSHIFT_);
  allocated = true;  
}

template< class T >
void MPISearchStream< T >::makeNormWindow_MPI_CacheCorners(FeatureData &thedata, double WINSHIFT_){
  // Create the normalization window feature
  makeNormWindow( norm_window, thedata);

  // create image pyramid object
  mpi = new MPIImagePyramid< T >(images, 1.2f, m_data->patch_width-1, m_data->patch_height-1, (float)WINSHIFT_);
  typename MPIImagePyramid< T >::const_iterator scale = mpi->begin(), last_scale = mpi->end();
  int scale_index; 
  float scale_factor;
  numfeatures = thedata.numfeatures;
  // Cache all the corner calculations for each scale
  //printf("Cache all the corner calculations for each scale\n");
  for( ; scale != last_scale; ++scale){
    scale_index = scale.getScale(scale_factor);
     T  *fns_;
     T  nw_fn_;
    CornerCache< T > *nw_c_ = new CornerCache< T >[4];
    CornerCache< T > **corners_ = cacheCorners(scale_factor, thedata, fns_, images[0]->width, norm_window, nw_c_, nw_fn_);
    nw_fn.push_back(nw_fn_);
    nw_c.push_back(nw_c_);
    fns.push_back(fns_);
    corners.push_back(corners_);
  }
}

template< class T >
void MPISearchStream< T >::release(){
  if(allocated){
    unsigned int i;
    allocated = false;
    if(my_memory){
      delete pixels2;
      for(i = 0; i < images.size(); ++i){
        delete images[i];
      }
      images.clear();
    }
    for(i = 0 ; i < corners.size(); ++i){
      //cout << "MPISearchStream< T >::release(): calling deleteCacheCorners()" << endl;
      deleteCacheCorners(corners[i], fns[i]);
      delete [] nw_c[i];
    }
    nw_c.clear(); // normalization window corner cache (for scales);
    nw_fn.clear();
    corners.clear(); // feature corner cache (for scales);
    fns.clear();
    delete mpi;
    mpi = 0;
  }
  return;
}
template< class T >
void MPISearchStream< T >::reset(const int width_, const int height_, FeatureData &thedata, double WINSHIFT_){
  if(allocated) release();
  init(width_, height_, thedata, WINSHIFT_);
}

template< class T >
void MPISearchStream< T >::reset(const MPISearchStream &s, FeatureData &thedata, double WINSHIFT_){
  if(allocated) release();
  init(s, thedata, WINSHIFT_);
}
template< class T >
ROI MPISearchStream< T >::getROI() const { return mpi->getROI(); }
template< class T >
void MPISearchStream< T >::SetROI(ROI &theroi){
  mpi->SetROI(theroi);
}
template< class T >
void MPISearchStream< T >::SetROI(const int &minx,const int &maxx,const int &miny,const int &maxy,const int &minsc,const int &maxsc){
  ROI theroi(minx, maxx, miny, maxy, minsc, maxsc);
  mpi->SetROI(theroi);
  //cout << "MPISearchStream< T >::SetROI " << theroi << endl;
}

template< class T >
CornerCache< T >** MPISearchStream< T >::cacheCorners (float &scale_factor, FeatureData &thedata,  T * &fns,
                                                     int &image_width, Corner norm_window[4], CornerCache< T > nwc[4],  T  &nw_fn)
{
  //if(debug){
  //cout << "scale_factor = " << scale_factor << endl;
  //for(int j = 0; j < thedata.features[0].numcorners; ++j) {
  //  cout << "features[0].corners x y = (" << scale_factor * thedata.features[0].corners[j].x;
  //  cout << ", " << scale_factor * thedata.features[0].corners[j].y << ")" << endl;
  //}
  //}
  bool make16bytes = true;

  //CornerCache< T > **pcc = (CornerCache< T >**)malloc(thedata.numfeatures*sizeof(CornerCache< T >*));
  CornerCache< T > **pcc = new CornerCache< T >* [thedata.numfeatures];
  //fns = ( T  *)malloc(thedata.numfeatures*sizeof( T ));
  fns = new  T  [thedata.numfeatures];
  int i, newnc = 0;
  for(i = 0; i < thedata.numfeatures; ++i) {
    CornerCache< T > *cc;
    if(make16bytes){
        newnc = thedata.features[i].numcorners + (((4 - (thedata.features[i].numcorners % 4)) % 4));
        cc = new CornerCache< T > [newnc];
    } else {
        cc = new CornerCache< T > [thedata.features[i].numcorners];
    }

    int fn = 0;
    float scaledCornerFloatX;
    float scaledCornerFloatY;
    float correction;
    for(int j = 0; j < thedata.features[i].numcorners; ++j) {
      scaledCornerFloatX = scale_factor * thedata.features[i].corners[j].x;
      scaledCornerFloatY = scale_factor * thedata.features[i].corners[j].y;
      cc[j].scaledCornerX = static_cast<int>(scaledCornerFloatX+0.5);
      cc[j].scaledCornerY = static_cast<int>(scaledCornerFloatY+0.5);
      //if(cc[j].scaledCornerX * cc[j].scaledCornerY > 0.000001)
      //  correction = (scaledCornerFloatX * scaledCornerFloatY) /(cc[j].scaledCornerX * cc[j].scaledCornerY);
      //else
        correction = 1;  
      cc[j].scaledIndex = cc[j].scaledCornerY * image_width + cc[j].scaledCornerX;
      cc[j].value = thedata.features[i].corners[j].value * correction;
      //cout << "(" <<  scaledCornerFloatX << ", " << scaledCornerFloatY << ") --> (";
      //cout <<  cc[j].scaledCornerX << ", " << cc[j].scaledCornerY << "). ";
      //cout << "correction: "<< correction << endl;
      fn += thedata.features[i].corners[j].value *
        thedata.features[i].corners[j].x *
        thedata.features[i].corners[j].y;
    }
    if(make16bytes && (thedata.features[i].numcorners < newnc)){
        for(int j = thedata.features[i].numcorners; j < newnc; ++j) {
            cc[j].scaledCornerX = 0;
            cc[j].scaledCornerY = 0;
            cc[j].scaledIndex = 0;
            cc[j].value = 0;
        }
    }
    
    pcc[i] = cc;
    fns[i] = static_cast< T >(fn);
  }
  //printf("Last part\n");
  nw_fn = 0;
  for(i = 0; i < 4; ++i){
    nwc[i].scaledCornerX = static_cast<int>(scale_factor * norm_window[i].x);
    nwc[i].scaledCornerY = static_cast<int>(scale_factor * norm_window[i].y);
    nwc[i].scaledIndex = nwc[i].scaledCornerY * image_width + nwc[i].scaledCornerX;
    nw_fn += norm_window[i].value * nwc[i].scaledCornerX * nwc[i].scaledCornerY;
  }

  return pcc;
}
template< class T >
void MPISearchStream< T >::deleteCacheCorners ( CornerCache< T > ** &pcc,  T * &fns )
{
  for(int i = 0; i < numfeatures; i++) {
    //free(pcc[i]);
    delete [] pcc[i];
  }
  //free(pcc);
  delete [] pcc;
  // free(fns);
  delete [] fns;
  // Note: someone replaced with xmlFree for windows changes, which makes no sense to me
}
template< class T >
void MPISearchStream< T >::makeNormWindow( Corner norm_window[4], FeatureData &thedata){
  // Create the normalization window feature
  norm_window[0].x = thedata.normOffset.top; norm_window[0].y = thedata.normOffset.left; norm_window[0].value = 1;
  norm_window[1].x = thedata.normOffset.top; norm_window[1].y = thedata.patch_height-1 - thedata.normOffset.right; norm_window[1].value = -1;
  norm_window[2].x = thedata.patch_width-1 - thedata.normOffset.bottom; norm_window[2].y = thedata.normOffset.left; norm_window[2].value = -1;
  norm_window[3].x = thedata.patch_width-1 - thedata.normOffset.bottom; norm_window[3].y = thedata.patch_height-1 - thedata.normOffset.right;
  norm_window[3].value = 1;
  //printf("After normalized window\n");
}

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

