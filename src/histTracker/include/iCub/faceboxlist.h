/* 
 *  FACEBOXLIST.h
 *
 *  Created by Ian Fasel on Feb 02, 2003.
 *  Fixes: 
 * 
 *  Copyright (c) 2003 Machine Perception Laboratory 
 *  University of California San Diego.
 * 
 * Please read the disclaimer and notes about redistribution 
 * at the end of this file.
 *  
 */

#ifndef _FACEBOXLIST_H_
#define _FACEBOXLIST_H_

#include "square.h"
#include <algorithm>
#include <list>
#include <math.h>
#include <iostream>

#ifdef WIN32
#include <windows.h>
#endif

using namespace std;

class avg_object {
 public:
  float  l, r, t, b; // left, right, top, bottom
  float  scale;
  int  nL, nR, nT, nB, nScale; // number of contributing boxes
  bool normalized;

  avg_object();
  avg_object(const avg_object &v);
  ~avg_object();
  avg_object operator+(const avg_object& v) const;
  avg_object & operator+=(const avg_object &v);
  void normalize();
  void unnormalize();  
};

template< class T >
class ObjectList{
  typedef typename list<TSquare< T > >::iterator TSquare_iter;
 public:
  ObjectList() {};
  ObjectList(const ObjectList &thelist):objects(thelist.objects){};
  ~ObjectList() {};

  void simplify(float tol);
  inline int size() const {return objects.size();}
  inline bool empty() const {return objects.empty(); }
  inline void clear() {objects.clear();}
  inline void sort() {objects.sort();}
  inline void erase(TSquare_iter &object){objects.erase(object);}
  inline void push_back(const TSquare< T >  &object){objects.push_back(object);}
  inline void push_front(const TSquare< T >  &object){objects.push_front(object);}
  inline void pop_front(){objects.pop_front();}
  inline TSquare_iter begin() { return objects.begin(); }
  inline TSquare_iter end() { return objects.end(); }
  inline TSquare< T >  front() const { return objects.front(); }
  inline TSquare< T >  back() const { return objects.back(); }

  list< TSquare< T > > objects;
  float  overlap(avg_object &a, avg_object &b);
/*	ObjectList& operator=(const ObjectList &ob){
		for( list< TSquare< T > >::const_iterator it = ob.objects.begin(); 
				it != ob.objects.end(); ++it){
					objects.push_back(*it);
		}
		return *this;
	}
*/
};

typedef ObjectList<int> FaceBoxList;

template< class T >
float ObjectList<T>::overlap(avg_object &a, avg_object &b) {
  float  x = max(0.0f,min(a.r,b.r)-max(a.l,b.l));
  float  y = max(0.0f,min(a.b,b.b)-max(a.t,b.t));
  float  intersection = x*y;
  float  total = (a.r-a.l)*(a.b-a.t) + (b.r-b.l)*(b.b-b.t);
  float  unin = total-intersection; // note: 'union' is reserved
  float  p = intersection/unin;
  return(p);
}

template< class T >
void ObjectList<T>::simplify (float tol)
{
  list<avg_object>* origObjects;
  avg_object tmp;

  if(objects.size() > 1){
    //
    // First, put objects into convenient form
    //
    TSquare<T> f;
    origObjects = new list<avg_object>;
    while(objects.size() > 0) {
      f = objects.front();
      //cout << "Simplifying from (x,y,scale,size): (" << f.x << ", " << f.y << ", " << f.scale<<", " << f.size << ")" << endl;
      tmp.l = f.x;
      tmp.r = f.x + f.size;
      tmp.t = f.y;
      tmp.b = f.y + f.size;
      //tmp.n = 1;
      tmp.scale = f.scale;
      origObjects->push_back(tmp);  // append the copy
      objects.pop_front();
    }

    //
    // Now, iterate through boxes and combine those that overlap with first
    // box.  Make a new list containing originals and combined boxes.  Now
    // repeat, using this new list in place of the original, until all boxes
    // have been checked with all other boxes.
    //
    list<avg_object> *newObjects = 0;
    list<avg_object> *overlappingObjects = 0;
    //float  collapsed;
		if((int)origObjects->size() > 0){
     for(int i = 0; i < (int)origObjects->size(); i++){
      newObjects = new list<avg_object>;
      overlappingObjects = new list<avg_object>;

      //
      // store the first object in the overlapping list.
      //
      overlappingObjects->push_back(origObjects->front());
      origObjects->pop_front();
      //
      //	Loop through all boxes in list, and add them either to the new list,
      //  or the overlapping boxes list.
      //
      while ( origObjects->size() > 0 ) {
        float  o = overlap(overlappingObjects->front(),origObjects->front());
        if(o > tol)
          overlappingObjects->push_back(origObjects->front());
        else
          newObjects->push_back(origObjects->front());
        origObjects->pop_front();
      }


      //
      // Now average all the overlapping boxes, and add them to the new list.
      //
      list<avg_object>::iterator over_iter = overlappingObjects->begin(),
        over_iter_end = overlappingObjects->end();
      tmp = *over_iter;
      for ( over_iter++; over_iter != over_iter_end; over_iter++){
        tmp += *over_iter;
      }
      tmp.normalize();
      //cout << "tmp after normallizing: (" << tmp.t << ","<< tmp.b << ","<< tmp.l << ","<< tmp.r << ")"<<endl;
      newObjects->push_back(tmp);

      //
      //  Make the new list pose as the original list, and free memory so we
      //  can make a new new list and overlapping boxes list.
      //
      delete(origObjects);        // get rid of the old original list
      origObjects = newObjects;	// make the new list pose as the original
      delete(overlappingObjects); // get rid of the overlapping list
     }

     // finally, write them back into object
     while( !newObjects->empty() ) {
      tmp = *(newObjects->begin());
      TSquare< T > square (static_cast<int>(tmp.r-tmp.l), static_cast<int>(tmp.l), static_cast<int>(tmp.t), static_cast<int>(floor(tmp.scale+.5)));
      objects.push_front(square);
      f = objects.front();
      newObjects->pop_front ( );
     }
     delete newObjects;
	  }
  }
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
