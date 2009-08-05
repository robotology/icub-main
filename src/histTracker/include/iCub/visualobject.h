/* 
 *  VISUALOBJECT.h
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

#ifndef _VISUALOBJECT_H_
#define _VISUALOBJECT_H_

#include <algorithm>
#include <math.h>
#include <list>
#ifndef Xcode
#include "square.h"
#else
#include <mpisearch/square.h>
#endif

using namespace std;

enum feature_type{e_face = 1, lefteye, righteye, botheyes};

class VisualObject{
 public:
  float x;
  float y;
  float xSize;
  float ySize;
  float scale;
	double activation;
  feature_type feature;
  bool listFlag;

  //list< VisualObject *> objects;
  //typedef list< VisualObject *>::iterator VisualObject_iter;

  VisualObject(); 
  VisualObject(feature_type feature_in);
  VisualObject(float x_in, float y_in, float xSize_in, float scale_in, feature_type feature_in, float ySize_in = 0.0);
  //VisualObject(const VisualObject &thelist);
  VisualObject(TSquare<float> &square, feature_type feature_in);
  virtual ~VisualObject();

  /*virtual void clear();
  inline int size() const {return objects.size();} 
  inline bool empty() const {return objects.empty();}
  inline void sort() {objects.sort();}
  inline void erase(VisualObject_iter &object){objects.erase(object);}
  inline void push_back( VisualObject  *const &object){objects.push_back(object);}
  inline void push_front( VisualObject  *const &object){objects.push_front(object);}
  inline void pop_front(){objects.pop_front();}
  inline void pop_back(){objects.pop_back();}
  inline VisualObject_iter begin() { return objects.begin(); }
  inline VisualObject_iter end() { return objects.end(); }
  inline VisualObject  *front() const { return objects.front(); }
  inline VisualObject  *back() const { return objects.back(); }
  inline void insert(VisualObject_iter list1, VisualObject_iter list2, 
	  VisualObject_iter list3){objects.insert(list1, list2, list3);}
*/
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
