/*
 *  eyefinder.cc
 *
 *  Author:Ian Fasel
 *
 */

#include <iCub/eyefinderBinary.h>
#include <iCub/lefteye_eye_dist_eye_centered.h>
/*#include <iCub/lefteye_eye_dist_face_centered.h>
#include <iCub/lefteye_eye_only_eye_centered.h>
#include <iCub/lefteye_half_dist_eye_centered.h>
#include <iCub/lefteye_largest_2_eye_centered.h>
#include <iCub/lefteye_largest_eye_centered.h>
#include <iCub/lefteye_largest_face_centered.h>
#include <iCub/lefteye_smallest_eye_centered.h>*/

#include <iCub/righteye_eye_dist_eye_centered.h>
/*#include <iCub/righteye_eye_dist_face_centered.h>
#include <iCub/righteye_eye_only_eye_centered.h>
#include <iCub/righteye_half_dist_eye_centered.h>
#include <iCub/righteye_largest_2_eye_centered.h>
#include <iCub/righteye_largest_eye_centered.h>
#include <iCub/righteye_largest_face_centered.h>
#include <iCub/righteye_smallest_eye_centered.h>*/


MPEyeFinderBinary::MPEyeFinderBinary ( int use_ada_face_detector) : MPEyeFinder( use_ada_face_detector ) {
  lefteye_eye_dist_eye_centered::assignData(this, left_eye_data);
  righteye_eye_dist_eye_centered::assignData(this, right_eye_data);
}

MPEyeFinderBinary::~MPEyeFinderBinary ( ){ }

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
