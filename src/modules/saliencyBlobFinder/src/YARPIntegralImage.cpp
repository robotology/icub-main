// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea, Giorgio Metta and Francesco Orabona and Lorenzo Natale
 * email:   francesco.rea@iit.it
 * website: www.robotcub.org 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

/**
 * @file YARPIntegralImage.cpp
 * @brief implementation of the integral image class (very old code from 2001 or so).
 */

#include <iCub/YARPIntegralImage.h>

using namespace yarp::sig;

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
