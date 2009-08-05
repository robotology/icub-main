/////////////////////////////////////////////////////////////////////////
///                                                                   ///
///                                                                   ///
/// This Academic Free License applies to any software and associated ///
/// documentation (the "Software") whose owner (the "Licensor") has   ///
/// placed the statement "Licensed under the Academic Free License    ///
/// Version 1.0" immediately after the copyright notice that applies  ///
/// to the Software.                                                  ///
/// Permission is hereby granted, free of charge, to any person       ///
/// obtaining a copy of the Software (1) to use, copy, modify, merge, ///
/// publish, perform, distribute, sublicense, and/or sell copies of   ///
/// the Software, and to permit persons to whom the Software is       ///
/// furnished to do so, and (2) under patent claims owned or          ///
/// controlled by the Licensor that are embodied in the Software as   ///
/// furnished by the Licensor, to make, use, sell and offer for sale  ///
/// the Software and derivative works thereof, subject to the         ///
/// following conditions:                                             ///
/// Redistributions of the Software in source code form must retain   ///
/// all copyright notices in the Software as furnished by the         ///
/// Licensor, this list of conditions, and the following disclaimers. ///
/// Redistributions of the Software in executable form must reproduce ///
/// all copyright notices in the Software as furnished by the         ///
/// Licensor, this list of conditions, and the following disclaimers  ///
/// in the documentation and/or other materials provided with the     ///
/// distribution.                                                     ///
///                                                                   ///
/// Neither the names of Licensor, nor the names of any contributors  ///
/// to the Software, nor any of their trademarks or service marks,    ///
/// may be used to endorse or promote products derived from this      ///
/// Software without express prior written permission of the Licensor.///
///                                                                   ///
/// DISCLAIMERS: LICENSOR WARRANTS THAT THE COPYRIGHT IN AND TO THE   ///
/// SOFTWARE IS OWNED BY THE LICENSOR OR THAT THE SOFTWARE IS         ///
/// DISTRIBUTED BY LICENSOR UNDER A VALID CURRENT LICENSE. EXCEPT AS  ///
/// EXPRESSLY STATED IN THE IMMEDIATELY PRECEDING SENTENCE, THE       ///
/// SOFTWARE IS PROVIDED BY THE LICENSOR, CONTRIBUTORS AND COPYRIGHT  ///
/// OWNERS "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, ///
/// INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   ///
/// FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO      ///
/// EVENT SHALL THE LICENSOR, CONTRIBUTORS OR COPYRIGHT OWNERS BE     ///
/// LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN   ///
/// ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN ///
/// CONNECTION WITH THE SOFTWARE.                                     ///
///                                                                   ///
/// This license is Copyright (C) 2002 Lawrence E. Rosen. All rights  ///
/// reserved. Permission is hereby granted to copy and distribute     ///
/// this license without modification. This license may not be        ///
/// modified without the express written permission of its copyright  ///
/// owner.                                                            ///
///                                                                   ///
///                                                                   ///
/////////////////////////////////////////////////////////////////////////

///
///
///       YARP - Yet Another Robotic Platform (c) 2001-2003 
///
///                    #pasa#
///
///     "Licensed under the Academic Free License Version 1.0"
///

///
/// $Id: YARPFft.h,v 1.1 2007/03/02 03:07:23 vislab Exp $
///
///


/*--------------------------------*-C-*---------------------------------*
 * File:
 *	fftn.h
 * ---------------------------------------------------------------------*
 * Re[]:	real value array
 * Im[]:	imaginary value array
 * nTotal:	total number of complex values
 * nPass:	number of elements involved in this pass of transform
 * nSpan:	nspan/nPass = number of bytes to increment pointer
 *		in Re[] and Im[]
 * isign:	exponent: +1 = forward  -1 = reverse
 * scaling:	normalizing constant by which the final result is *divided*
 *	scaling == -1, normalize by total dimension of the transform
 *	scaling <  -1, normalize by the square-root of the total dimension
 *
 * ----------------------------------------------------------------------
 * See the comments in the code for correct usage!
 */

#ifndef __YARPFfth__
#define __YARPFfth__

//#include <yarp/YARPConfig.h>

/* double precision routine */
extern int fftn (int ndim, const int dims[], double Re[], double Im[],
		 int isign, double scaling);

extern void fft_free (void);
extern void fft_alloc (int max_factors, int max_perm);

//
// FAST PORTING July 2001: so far this class can be only instantiated once...
// because it uses static memory allocation.
//
// LATER: complete porting to C++.
//
class YARPFft
{
public:
	YARPFft (int max_factors, int max_perm) 
	{ 
		fft_alloc(max_factors, max_perm);
	}

	~YARPFft() 
	{ 
		fft_free(); 
	}

	void Fft (int ndim, const int dims[], double Re[], double Im[], int isign, double scaling)
	{
		fftn (ndim, dims, Re, Im, isign, scaling);
	}


	// helper functions.
	static void Convert2DoubleScaled (const unsigned char *in, double *out, int w, int h);
	static void Convert2Double (const unsigned char *in, double *out, int w, int h);
	static void Fft2DShift (const double *in, double *out, int w, int h);
};

//
// See the YARPFft.cpp for implementation.
//	- the whole class is really weird as it is now but with little work
//		I imagine I could clean it.
//
#endif
