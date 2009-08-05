/*
 * roi.cc
 *
 * Copyright (c) 2002 Machine Perception Laboratory
 * University of California San Diego.
 * Please read the disclaimer and notes about redistribution
 * at the end of this file.
 *
 * Authors: Ian Fasel
 */

#include <iCub/roi.h>

ROI::ROI(int min_x, int max_x, int min_y, int max_y, int min_scale, int max_scale) :
          m_min_x(min_x), m_max_x(max_x), m_min_y(min_y), m_max_y(max_y),
	   m_min_scale(min_scale), m_max_scale(max_scale){}

void ROI::reset(){
  m_min_x=MIN_RANGE; m_max_x=MAX_RANGE; m_min_y=MIN_RANGE; m_max_y=MAX_RANGE; m_min_scale=MIN_RANGE; m_max_scale=MAX_RANGE;}

ROI& ROI::operator=(const ROI &roi){
  if(&roi != this){
    m_min_x = roi.m_min_x; m_max_x = roi.m_max_x; m_min_y = roi.m_min_y; m_max_y = roi.m_max_y;
    m_min_scale = roi.m_min_scale; m_max_scale = roi.m_max_scale; m_limit_scale = roi.m_limit_scale;
    vmin_x = roi.vmin_x; vmin_y = roi.vmin_y; vmax_x = roi.vmax_x; vmax_y=roi.vmax_y;
  }
  return *this;
}


//#ifndef WIN32
std::ostream& operator<< (std::ostream& s, const ROI& roi){
  s << "ROI: (";
  s << "minx=" << roi.m_min_x << ", maxx=" << roi.m_max_x << ", ";
  s << "miny=" << roi.m_min_y << ", maxy=" << roi.m_max_y << ", ";
  s << "minscale=" << roi.m_min_scale << ", maxscale=" << roi.m_max_scale;
  s << ")";
  //  for(unsigned int i=0; i< roi.vmin_x.size(); ++ i){
  //  s << endl << "At scale " << i << ": ";
  //  s << "minx=" << roi.vmin_x[i] << ", maxx=" << roi.vmax_x[i] << ", ";
  //  s << "miny=" << roi.vmin_y[i] << ", maxy=" << roi.vmax_y[i] << ", ";
  // }
  return s;
}
//#endif

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
