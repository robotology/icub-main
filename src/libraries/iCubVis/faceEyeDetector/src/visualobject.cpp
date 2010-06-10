/* 
 *  VISUALOBJECT.cpp
 *
 *  Created by Bret Fortenberry on Aug 27, 2003.
 *  Fixes:
 *		To remove a memory leak, the attribute list< VisualObject *> objects;
 *		is commented, and all its related methods are also commented.
 *		by Plinio Moreno on Oct 06, 2007
 * 
 *  Copyright (c) 2003 Machine Perception Laboratory 
 *  University of California San Diego.
 * 
 * Please read the disclaimer and notes about redistribution 
 * at the end of this file.
 *  
 */

#include <iCub/vis/visualobject.h>



VisualObject::VisualObject():listFlag(true) {}
VisualObject::VisualObject(feature_type feature_in)
:x(0), y(0), xSize(0), ySize(0), scale(0), activation(0.0), 
feature(feature_in), listFlag(true) {}

VisualObject::VisualObject(float x_in, float y_in, float xSize_in, float scale_in, feature_type feature_in, float ySize_in)
:x(x_in), y(y_in), xSize(xSize_in), ySize(ySize_in),  
scale(scale_in), feature(feature_in), listFlag(true) {}


/*VisualObject::VisualObject(const VisualObject &thelist)
:objects(thelist.objects){}*/

VisualObject::VisualObject(TSquare<float> &square, feature_type feature_in)
:x(square.x), y(square.y), xSize(square.size), ySize(square.size), scale(square.scale), 
feature(feature_in) {}

VisualObject::~VisualObject() {
//  clear();
};

/*void VisualObject::clear(){
  list< VisualObject* >::iterator it = objects.begin();
  if(objects.size())
    {
      for(; it != objects.end(); ++it)
	{
	  (*it)->clear();
	}
    }
  objects.clear();
}*/

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

