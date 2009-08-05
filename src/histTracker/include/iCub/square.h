/*
 *  square.h
 *
 *  Copyright (c) 2002 Machine Perception Laboratory 
 *  University of California San Diego.
 *
 * Please read the disclaimer and notes about redistribution 
 * at the end of this file.
 *
 */
#ifndef _SQUARE_H_
#define _SQUARE_H_

//#include <iostream.h>


template <class T>
class TSquare {
	//friend ostream& operator<<(ostream& s, const TSquare<T>& tsquare);

  public: 
    TSquare (T size_, T x_, T y_, T scale_);
    TSquare  ():x(0),y(0),size(0),scale(0){};
	TSquare (const TSquare<double> &other) {
		x = (int)other.x;
		y = (int)other.y;
		size = (int)other.size;
		scale = (int)other.scale;
	}
	inline void set(T x_, T y_, T size_, T scale_) {
      x = x_;
      y = y_;
      size = size_;
      scale = scale_;
	}
	inline TSquare operator+ (T val) const {
      TSquare<T> c = *this;
      c.size += val; c.x += val; c.y += val;
	  c.scale += val;
      return c;
	}
	inline TSquare operator- (T val) const {
      TSquare<T> c = *this;
      c.size -= val; c.x -= val; c.y -= val;
	  c.scale -= val;
      return c;
	}
    inline TSquare operator/ (T val) const {
      TSquare<T> c = *this;
      c.size /= val; c.x /= val; c.y /= val;
	  c.scale /= val;
      return c;
    }
    inline TSquare & operator*=(T val){
      x *= val;
      y *= val;
      size *= val;
	  scale *= val;
      return *this;
    }
    inline TSquare & operator+=(TSquare<double> &other){
		x += other.x;
		y += other.y;
		size += other.size;
		scale += other.scale;
		return *this;
	}
    bool intersect (TSquare<T>*);
    inline bool isValid() const { return (x || y || size || scale);}
    T x;
    T y;
    T size;
	T scale;

};

typedef TSquare<int> Square;

template<class T>
bool operator< (const TSquare<T> &Lft,const TSquare<T> &Rht){ return Lft.size < Rht.size; }
template<class T>
bool operator> (const TSquare<T> &Lft,const TSquare<T> &Rht){ return Lft.size > Rht.size; }
template<class T>
bool decreasing (const TSquare<T> &Lft,const TSquare<T> &Rht){ return Lft.size > Rht.size; }


template <class T> 
TSquare<T>::TSquare (T size_, T x_, T y_, T scale_=0)
{
  size = size_;
  x = x_;
  y = y_;
  scale = scale_;
}

template <class T>
bool TSquare<T>::intersect (TSquare<T> *other)
{
  Square *smaller, *bigger; 
  
  if( size >= other->size) {
    bigger = this;
    smaller = other;
  } else {
    bigger = other;
    smaller = this;
  }

  if(smaller->x + smaller->size >= bigger->x && smaller->x <= bigger->x + bigger->size)
    if(smaller->y + smaller->size >= bigger->y && smaller->y <= bigger->y + bigger->size)
      return 1;

  return 0;
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

