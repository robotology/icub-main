/*
 * roi.h
 *
 * Copyright (c) 2002 Machine Perception Laboratory
 * University of California San Diego.
 * Please read the disclaimer and notes about redistribution
 * at the end of this file.
 *
 * Authors: Ian Fasel
 */
#ifndef _ROI_H_
#define _ROI_H_

#include <vector>

class ROI;

#include <iostream>
std::ostream& operator<< (std::ostream& s, const ROI& roi);

#define MAX_RANGE 32767
#define MIN_RANGE -32767

 
class ROI{
  friend std::ostream& operator<<(std::ostream& s, const ROI& roi); 
 public:
  ROI(int min_x=MIN_RANGE, int max_x=MAX_RANGE, int min_y=MIN_RANGE, int max_y=MAX_RANGE,
      int min_scale=MIN_RANGE, int max_scale=MAX_RANGE);
  void reset();
  ROI& operator=(const ROI &roi);

  int m_min_x, m_max_x, m_min_y, m_max_y, m_min_scale, m_max_scale, m_limit_scale;
  std::vector< int > vmin_x;
  std::vector< int > vmin_y;
  std::vector< int > vmax_x;
  std::vector< int > vmax_y;
};



#endif

/*
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 *    1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 *    3. The name of the author may not be used to endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

