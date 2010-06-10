/*
 *  featuredata.h
 *
 *  Created by Rhan Dahl on Thu Apr 18 2002.
 *  Fixes: Added matlab ci struct support  -EC, 08/18/2002
 * 
 *  Copyright (c) 2002 Machine Perception Laboratory 
 *  University of California San Diego.
 * 
 * Please read the disclaimer and notes about redistribution 
 * at the end of this file.
 *
 */
#ifndef _FEATURE_DATA_H_
#define _FEATURE_DATA_H_

typedef struct {
  int start;
  int end;
  float thresh;
} Cascade;

typedef struct {
  int x, y, value;
} Corner;

typedef struct {
  int id;
  int numcorners;
  float alpha;
  float bias;
  int weight;
  int abs;

  // additional fields for real-valued features
  double nl_over_range;
  double * tuning_curve;

  Corner *corners;
} Feature;

typedef struct {
  int top, left, right, bottom;
} OffsetWindow;

typedef struct {
  Feature *features;
  Cascade *cascades;
  int numfeatures;
  int numcascades;
  int numStdAdjusts;
  int patchsize;
  int patch_height;
  int patch_width;
  int plus_minus_one;
  int preserve_aH;
  float * stdAdjusts;
  OffsetWindow normOffset;
  bool real_fun;
  int nl;
  int minval; // there was a MATLAB vs C 0-1 indexing problem, so this helps fix it
  int maxval; // there was a MATLAB vs C 0-1 indexing problem, so this helps fix it
} FeatureData;


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










