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
///                    #nat#
///
///     "Licensed under the Academic Free License Version 1.0"
///

///
/// $Id: YARPIntegralImage.cpp,v 1.1 2009/06/23 15:23:31 rea Exp $
///
///

#include <iCub/YARPIntegralImage.h>
//#include <yarp/YARPLogpolar.h>

YARPIntegralImage::YARPIntegralImage()
{
	_nRows = 0;
	_nCols = 0;
	_nfovea = 0;
	_max = 1.0;
}

YARPIntegralImage::YARPIntegralImage(int nC, int nR, int sf)
{
	_resize(nC, nR, sf);
}

YARPIntegralImage::~YARPIntegralImage()
{

}

void YARPIntegralImage::resize(int nC, int nR, int sf)
{
	_resize(nC, nR, sf);
}

void YARPIntegralImage::_resize(int nC, int nR, int sf)
{
	//ACE_ASSERT ((nR>0) && (nC>0));

	_nCols = nC;
	_nRows = nR;
	_nfovea = sf;

	_max = (float) (255*(nR-sf)*nC);
	
	_integralImg.resize(nC, nR);
	_rowSum.resize(nC,nR);
}

int YARPIntegralImage::computeCartesian(ImageOf<PixelMono> &input)
{
	int r;
	int c;

	// first row
	_rowSum(0,0) = (float) input(0,0);
	_integralImg(0, 0) = _rowSum(0,0);
		
	for(c = 1; c < _nCols; c++)
	{
		_rowSum(c,0) = _rowSum(c-1,0) + (float) input(c,0);
		_integralImg(c,0) = _rowSum(c,0);
	}
		
	for(r = 1; r < _nRows; r++)
	{
		// first col
		_rowSum(0,r) = (float) input(0,r);
		_integralImg(0, r) = _integralImg(0, r-1) + _rowSum(0,r);
		
		for(c = 1; c < _nCols; c++)
		{
			_rowSum(c,r) = _rowSum(c-1,r) + (float) input(c,r);
			_integralImg(c,r) = _integralImg(c, r-1) + _rowSum(c,r);
		}
	}

	return true;
}

int YARPIntegralImage::computeCartesian(ImageOf<PixelMonoSigned> &input)
{
	int r;
	int c;

	// first row
	_rowSum(0,0) = (float) input(0,0);
	_integralImg(0, 0) = _rowSum(0,0);
		
	for(c = 1; c < _nCols; c++)
	{
		_rowSum(c,0) = _rowSum(c-1,0) + (float) input(c,0);
		_integralImg(c,0) = _rowSum(c,0);
	}
		
	for(r = 1; r < _nRows; r++)
	{
		// first col
		_rowSum(0,r) = (float) input(0,r);
		_integralImg(0, r) = _integralImg(0, r-1) + _rowSum(0,r);
		
		for(c = 1; c < _nCols; c++)
		{
			_rowSum(c,r) = _rowSum(c-1,r) + (float) input(c,r);
			_integralImg(c,r) = _integralImg(c, r-1) + _rowSum(c,r);
		}
	}

	return true;
}

int YARPIntegralImage::computeLp(ImageOf<PixelMono> &input)
{
	int r;
	int c;

	// first pixel, init
	_rowSum(0,0) = 0; //input(0,0)/255.0;
	_integralImg(0,0) = _rowSum(0,0);
	
	// fovea, zero it for now
	for(r = 0; r<_nfovea; r++)
		for(c = 0; c < _nCols; c++)
		{
			_rowSum(c,r) = 0;
			_integralImg(c,r) = 0;
		}
	
	
	for(r = _nfovea; r < _nRows; r++)
	{
		// first col
		//_rowSum(0,r) = (float) (input(0,r)*pSize(c,r,_nfovea));
		_rowSum(0,r) = (float) (input(0,r)*sizeof(c*r*_nfovea));
		_integralImg(0, r) = _integralImg(0, r-1) + _rowSum(0,r);
		
		for(c = 1; c < _nCols; c++)
		{
			
			//_rowSum(c,r) = _rowSum(c-1,r) + (float) (input(c,r)*pSize(c,r,_nfovea));
			_rowSum(c,r) = _rowSum(c-1,r) + (float) (input(c,r)*sizeof(c*r*_nfovea));
			_integralImg(c,r) = _integralImg(c, r-1) + _rowSum(c,r);
		}
	}

	return true;
}

int YARPIntegralImage::computeLp(ImageOf<PixelMonoSigned> &input)
{
	int r;
	int c;

	// first pixel, init
	_rowSum(0,0) = 0; //input(0,0)/255.0;
	_integralImg(0,0) = _rowSum(0,0);
	
	// fovea, zero it for now
	for(r = 0; r<_nfovea; r++)
		for(c = 0; c < _nCols; c++)
		{
			_rowSum(c,r) = 0;
			_integralImg(c,r) = 0;
		}
	
	
	for(r = _nfovea; r < _nRows; r++)
	{
		// first col
		//_rowSum(0,r) = (float) (input(0,r)*pSize(c,r,_nfovea));
		_rowSum(0,r) = (float) (input(0,r)*sizeof(c*r*_nfovea));
		_integralImg(0, r) = _integralImg(0, r-1) + _rowSum(0,r);
		
		for(c = 1; c < _nCols; c++)
		{
			//_rowSum(c,r) = _rowSum(c-1,r) + (float) (input(c,r)*pSize(c,r,_nfovea));
			_rowSum(c,r) = _rowSum(c-1,r) + (float) (input(c,r)*sizeof(c*r*_nfovea));
			_integralImg(c,r) = _integralImg(c, r-1) + _rowSum(c,r);
		}
	}

	return true;
}
