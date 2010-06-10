/* 
 *  faceboxlist.cc
 *
 *  Created by Ian Fasel on Feb 02, 2003.
 *  Fixes: 
 * 
 *  Copyright (c) 2003 Machine Perception Laboratory 
 *  University of California San Diego.
 * 
 * Please read the disclaimer and notes about redistribution 
 * at the end of this file.
 */

#include <iCub/vis/faceboxlist.h>
#include <list>
#include <math.h>
#include <iostream>


avg_object::avg_object():l(0),r(0),t(0),b(0),scale(0),
nL(1),nR(1),nT(1),nB(1),nScale(1),normalized(false){}

avg_object::avg_object(const avg_object &v):l(v.l),r(v.r),t(v.t),b(v.b),scale(v.scale),
nL(v.nL),nR(v.nR),nT(v.nT),nB(v.nB),nScale(v.nScale),normalized(v.normalized){}

avg_object::~avg_object(){}

avg_object avg_object::operator+(const avg_object& v) const{
  avg_object tmp = *this;
  if(tmp.normalized) tmp.unnormalize();
  if(!v.normalized){
	  if(tmp.l > 0){
		  tmp.l += v.l; tmp.nL += v.nL;}
	  if(tmp.r > 0){
		  tmp.r += v.r; tmp.nR += v.nR;}
	  if(tmp.t > 0){
		  tmp.t += v.t; tmp.nT += v.nT;}
	  if(tmp.b > 0){
		  tmp.b += v.b; tmp.nB += v.nB;}
	  if(tmp.scale > 0){
		  tmp.scale += v.scale; tmp.nScale += v.nScale;}
  }
  else{
	  if(tmp.l > 0){
		  tmp.l += v.l*v.nL; tmp.nL += v.nL;}
	  if(tmp.r > 0){
		  tmp.r += v.r*v.nR; tmp.nR += v.nR;}
	  if(tmp.t > 0){
		  tmp.t += v.t*v.nT; tmp.nT += v.nT;}
	  if(tmp.b > 0){
		  tmp.b += v.b*v.nB; tmp.nB += v.nB;}
	  if(tmp.scale > 0){
		  tmp.scale += v.scale*v.nScale; tmp.nScale += v.nScale;}
  }
  return tmp;
}

avg_object & avg_object::operator+=(const avg_object &v){
  if(normalized) unnormalize();
  if(!v.normalized){
	  if(l > 0){
		  l += v.l; nL += v.nL;}
	  if(r > 0){
		  r += v.r; nR += v.nR;}
	  if(t > 0){
		  t += v.t; nT += v.nT;}
	  if(b > 0){
		  b += v.b; nB += v.nB;}
	  if(scale > 0){
		  scale += v.scale; nScale += v.nScale;}
  }
  else{
	  if(l > 0){
		  l += v.l*v.nL; nL += v.nL;}
	  if(r > 0){
		  r += v.r*v.nR; nR += v.nR;}
	  if(t > 0){
		  t += v.t*v.nT; nT += v.nT;}
	  if(b > 0){
		  b += v.b*v.nB; nB += v.nB;}
	  if(scale > 0){
		  scale += v.scale*v.nScale; nScale += v.nScale;}
  }
  return *this;
}

void avg_object::normalize(){
  t*=1.0f/nT; b*=1.0f/nB; l*=1.0f/nL; r*=1.0f/nR; scale*=1.0f/nScale;
  normalized = true;
}

void avg_object::unnormalize(){
  t*=nT; b*=nB; l*=nL; r*=nR; scale*=nScale;
  normalized = false;
}


/*avg_object::avg_object():l(0),r(0),t(0),b(0),n(1),scale(0),normalized(false){}

avg_object::avg_object(const avg_object &v):l(v.l),r(v.r),t(v.t),b(v.b),n(v.n),scale(v.scale),normalized(v.normalized){}

avg_object::~avg_object(){}

avg_object avg_object::operator+(const avg_object& v) const{
  avg_object tmp = *this;
  if(tmp.normalized) tmp.unnormalize();
  if(!v.normalized){
    tmp.l+=v.l; tmp.r+=v.r; tmp.t+=v.t; tmp.b+=v.b; tmp.scale+=v.scale; tmp.n+=v.n;}
  else{
    tmp.l+=v.l*v.n; tmp.r+=v.r*v.n; tmp.t+=v.t*v.n; tmp.b+=v.b*v.n; tmp.scale+=v.scale*v.n; tmp.n+=v.n;
  }
  return tmp;
}

avg_object & avg_object::operator+=(const avg_object &v){
  if(normalized) unnormalize();
  if(!v.normalized){
    l += v.l; r += v.r; t += v.t; b += v.b; n += v.n; scale += v.scale;}
  else{
    l += v.l*v.n; r += v.r*v.n; t += v.t*v.n; b += v.b*v.n; n += v.n; scale += v.scale*v.n;
  }
  return *this;
}

void avg_object::normalize(){
  float v = 1.0f/n;
  t*=1.0f/n; b*=1.0f/n; l*=1.0f/n; r*=1.0f/n; scale*=1.0f/n;
  normalized = true;
}

void avg_object::unnormalize(){
  t*=n; b*=n; l*=n; r*=n; scale*=n;
  normalized = false;
}


*/


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

