
#include "DisparityTool.h"


DisparityTool::DisparityTool()
{
	//ACE_OS::sprintf(_path,"%s","data/Tables/");

	_shiftFunction	  = NULL;
	_shiftFunctionInv = NULL;
	_shiftMap		  = NULL;
	_corrFunct		  = NULL;
	_ssdFunct         = NULL;
	_count			  = NULL;

	_maxCount = 0.0;

	init(_srho, _stheta, _fmode, _overlap, _xsize, _ysize, _xsizeR, _ysizeR, _srho);
}


DisparityTool::~DisparityTool()
{
	if (_shiftFunction != NULL)
		delete [] _shiftFunction;

	if (_shiftMap != NULL)
		delete [] _shiftMap;

	if (_corrFunct != NULL)
		delete [] _corrFunct;

	if (_ssdFunct != NULL)
		delete [] _ssdFunct;

	if (_count != NULL)
		delete [] _count;

	if (_shiftFunctionInv == NULL)
		delete [] _shiftFunctionInv;
}


void DisparityTool::init(int rho, int theta, int mode, double overlap, int xo, int yo, int xr, int yr, int actR)
{
	_img = SetParam(rho, theta, mode, overlap, xo, yo, xr, yr);
	_actRings = actR;
	AllocateVectors();	
	LoadShiftMap();
	computeCountVector(_count);
}


void DisparityTool::AllocateVectors()
{	
	if (_shiftFunction != NULL)
		delete [] _shiftFunction;
	if (_shiftMap != NULL)
		delete [] _shiftMap;
	if (_corrFunct != NULL)
		delete [] _corrFunct;
	if (_ssdFunct != NULL)
		delete [] _ssdFunct;
	if (_count != NULL)
		delete [] _count;

	char File_Name [256];
	FILE * fin;
	int n;
	int cSize = (int)__min((double)_img.Size_X_Orig, (double)_img.Size_Y_Orig);
	sprintf(File_Name,"%s%dx%d_%dx%d_ShiftMap.gio",_path, cSize, cSize, _img.Size_Theta, _img.Size_Rho);

	if ((fin = fopen(File_Name,"rb")) == NULL)
	{
		Build_Shift_Table(_img, _path);
		fin = fopen(File_Name,"rb");
	}

	fread(&n,sizeof(int),1,fin);
	fclose(fin);

	_shiftFunction = new double[n];
	_shiftMap = new int[n*_img.Size_LP];
	_count = new int[n];
	_corrFunct = new double[n];
	_ssdFunct = new double[n];
}


void DisparityTool::LoadShiftMap()
{
	_shiftLevels = Load_Shift_Table(_img, _shiftMap, _shiftFunction, _path);
	_shiftMax = _shiftLevels - 1;
	_shiftMin = 0;


	if (_shiftFunctionInv != NULL)
		delete [] _shiftFunctionInv;
	int tmpIndex;
	double dmax, dmin;
	dmax = find_max_value(_shiftFunction, _shiftLevels);
	dmin = find_min_value(_shiftFunction, _shiftLevels);

	_shiftFunctionInv = new int[(int)(dmax-dmin)+1];
	for (int i = 0; i <= dmax-dmin; i++)
	{
		for (int j = 0; j < _shiftLevels; j++)
		{
			tmpIndex = j;
			if (_shiftFunction[j] >= i+dmin)
				break;
		}
		_shiftFunctionInv[i] = tmpIndex;
	}	
}


void DisparityTool::makeHistogram(ImageOf<PixelMono>& hImg)
{
	int i,j;
	int height = hImg.height();
	int width = hImg.width();

	unsigned char bgColor = 0;
	unsigned char hColor = 190;
	unsigned char lineColor = 255;

	unsigned char * hist = new unsigned char [height*width*hImg.getPixelSize()];
	img2unpaddedVect(hist, hImg);

	int offset = (width - _shiftLevels + 1)/2;

	for (j = 0; j < height*width; j++)
		hist[j] = bgColor;

	for (i = 0; i < _shiftLevels-1; i++)
	{
		if ((i+offset >=0)&&(i+offset<width))
		{
			for (j = height-(int)(height*_corrFunct[i]); j < height; j++)
					hist[(j*width+i+offset)] = hColor;
		}
	}


	for (i = 0; i < 3; i++)
	{
		if ((_maxShifts[i].index+offset >=0)&&(_maxShifts[i].index+offset<width))
		{
			for (j = height-(int)(height*_corrFunct[_maxShifts[i].index]); j < height; j++)
					hist[(j*width+_maxShifts[i].index+offset)] = lineColor;
		}
	}

	//Drawing Zero Reference
	for (j = 0; j < height; j += height/9)
	{
		for (int k = 0; k < height/18; k++)
		{
			hist[((j+k)*width + _shiftLevels/2 + offset)] = lineColor;
		}
	}

	//Drawing Limits (dark gray sides)
/*	for (int y = 0; y < height; y++)
	{
		for (int x1 = 0; x1 < _shiftMin; x1++)
			hist[(y*width+x1+offset)] = 120;
		for (int x2 = width - 1; x2 > _shiftMax; x2--)
			hist[(y*width+x2+offset)] = 120;
	}/***/

	//Partially inverted color
	for (int y = 0; y < height; y++)
	{
		for (int x1 = 0; x1 < _shiftMin; x1++)
			hist[(y*width+x1+offset)] = hist[(y*width+x1+offset)]+100;
		for (int x2 = width - 1; x2 > _shiftMax; x2--)
			hist[(y*width+x2+offset)] = hist[(y*width+x2+offset)]+100;
	}/***/


	//Drawing Limits (inverted colors sides)
/*	for (int y = 0; y < height; y++)
	{
		for (int x1 = 0; x1 < _shiftMin; x1++)
			hist[(y*width+x1+offset)] = -hist[(y*width+x1+offset)]+255;
		for (int x2 = width - 1; x2 > _shiftMax; x2--)
			hist[(y*width+x2+offset)] = -hist[(y*width+x2+offset)]+255;
	}/***/

	unpaddedVect2img(hist, hImg);

}


void DisparityTool::makeNormSSDHistogram(ImageOf<PixelMono>& hImg)
{
	int i,j;
	int height = hImg.height();
	int width = hImg.width();

	unsigned char bgColor = 0;
	unsigned char hColor = 190;
	unsigned char lineColor = 255;

	unsigned char * hist = new unsigned char [height*width*hImg.getPixelSize()];
	img2unpaddedVect(hist, hImg);

	int offset = (width - _shiftLevels + 1)/2;

	for (j = 0; j < height*width; j++)
		hist[j] = bgColor;

	for (i = 0; i < _shiftLevels-1; i++)
	{
		if ((i+offset >=0)&&(i+offset<width))
		{
			for (j = height-(int)(height*_ssdFunct[i]); j < height; j++)
					hist[(j*width+i+offset)] = hColor;
		}
	}


	for (i = 0; i < 3; i++)
	{
		if ((_maxShifts[i].index+offset >=0)&&(_maxShifts[i].index+offset<width))
		{
			for (j = height-(int)(height*_ssdFunct[_maxShifts[i].index]); j < height; j++)
					hist[(j*width+_maxShifts[i].index+offset)] = lineColor;
		}
	}

	//Drawing Zero Reference
	for (j = 0; j < height; j += height/9)
	{
		for (int k = 0; k < height/18; k++)
		{
			hist[((j+k)*width + _shiftLevels/2 + offset)] = lineColor;
		}
	}

	//Drawing Limits (dark gray sides)
	for (int y = 0; y < height; y++)
	{
		for (int x1 = 0; x1 < _shiftMin; x1++)
			hist[(y*width+x1+offset)] = 120;
		for (int x2 = width - 1; x2 > _shiftMax; x2--)
			hist[(y*width+x2+offset)] = 120;
	}/***/

	//Partially inverted color
/*	for (int y = 0; y < height; y++)
	{
		for (int x1 = 0; x1 < _shiftMin; x1++)
			hist[(y*width+x1+offset)] = hist[(y*width+x1+offset)]+100;
		for (int x2 = width - 1; x2 > _shiftMax; x2--)
			hist[(y*width+x2+offset)] = hist[(y*width+x2+offset)]+100;
	}/***/


	//Drawing Limits (inverted colors sides)
/*	for (int y = 0; y < height; y++)
	{
		for (int x1 = 0; x1 < _shiftMin; x1++)
			hist[(y*width+x1+offset)] = -hist[(y*width+x1+offset)]+255;
		for (int x2 = width - 1; x2 > _shiftMax; x2--)
			hist[(y*width+x2+offset)] = -hist[(y*width+x2+offset)]+255;
	}/***/

	unpaddedVect2img(hist, hImg);

}


void DisparityTool::Remap(ImageOf<PixelRgb>  & lpIn, ImageOf<PixelRgb>  & cartOut)
{	
	unsigned char *tmpLp, *tmpCart;
    lp2CartPixel *l2cTable;
    l2cTable = new lp2CartPixel[_img.Size_X_Remap * _img.Size_Y_Remap];
    if (l2cTable == NULL)
    {
        exit (-1);
    }
	int test;

	cartOut.resize(_img.Size_X_Remap, _img.Size_Y_Remap);
	cartOut.zero();
    test = RCallocateL2CTable (l2cTable, _img.Size_X_Remap, _img.Size_Y_Remap, _path);
	if (test == 1)
	{
    	RCbuildL2CMap (_img.Size_Rho, _img.Size_Theta, _img.Size_X_Remap, _img.Size_Y_Remap, _img.overlap, _img.scale_Remap, 0, 0, _img.Fovea_Mode, _path);
    	RCallocateL2CTable (l2cTable, _img.Size_X_Remap, _img.Size_Y_Remap, _path);
	}

	tmpCart = new unsigned char [_img.Size_Img_Remap*cartOut.getPixelSize()];
	tmpLp = new unsigned char [_img.Size_LP*lpIn.getPixelSize()];
	img2unpaddedVect(tmpLp, lpIn);
    RCgetCartImg (tmpCart, tmpLp, l2cTable, _img.Size_Img_Remap);
	unpaddedVect2img(tmpCart, cartOut);
}


void DisparityTool::computeCountVector(int *count)
{	
	int tIndex;
	int k,i,j,k1, i2;
	if (count == NULL)
		count = new int [_shiftLevels];
	for (k = 0; k < _shiftLevels; k++)
	{
		k1 = k * _img.Size_LP;
		count[k] = 0;
		for (j=0; j<_actRings; j++)
		{
			tIndex = j*_img.Size_Theta;
			for (i=0; i<_img.Size_Theta; i++)
			{
				i2 = _shiftMap[k1 + tIndex+i];
				if (i2 > 0)
					count[k]++;
			}
		}
	}

	// find max
	_maxCount = 0;
	for (k = 0; k < _shiftLevels; k++)
	{
		if (count[k] > _maxCount)
			_maxCount = count[k];
	}

}


void DisparityTool::findShiftMax(const double *corr)
{
	// search max
	double corrMax = 0;
	int ret = 0;
	int l;


	for(l = _shiftMin; l <= _shiftMax; l++)
	{
		if (corr[l] > corrMax)
		{
			corrMax = corr[l];
			ret = l;
		}
	}
	
	_maxShifts[0].index = ret;
	_maxShifts[0].corr = corrMax;
	_maxShifts[0].disp = shiftToDisparity(ret);
}


void DisparityTool::findSecondMaxes(const double *corr, int posMax)
{
	// search max
	double corrMax;
	double threshold = 0.005;
	int max1;
	int max2;

	int ret = 0;
	int l;
	
	//Searching in left direction
	l = posMax-1;
	while( (l > _shiftMin) && (corr[l] <= corr[l+1]) )
		l--;	
	corrMax = corr[l];
	max1 = l;
	for(; l >= _shiftMin; l--)
	{
		if (corr[l] > (corrMax + threshold))
		{
			corrMax = corr[l];
			max1 = l;
		}
	}

	//Searching in right direction
	l = posMax+1;
	while( (l < _shiftMax) && (corr[l-1] >= corr[l]) )
		l++;
	corrMax = corr[l];
	max2 = l;
	for(; l <= _shiftMax; l++)
	{
		if (corr[l] > (corrMax + threshold))
		{
			corrMax = corr[l];
			max2 = l;
		}
	}

	if (corr[max1]>corr[max2])
	{
		_maxShifts[1].index = max1;
		_maxShifts[2].index = max2;
	}
	else
	{
		_maxShifts[2].index = max1;
		_maxShifts[1].index = max2;
	}

	_maxShifts[3].index = zeroShift();

	for (int k = 1; k < 4; k++)
	{
		_maxShifts[k].corr = corr[_maxShifts[k].index];
		_maxShifts[k].disp = shiftToDisparity(_maxShifts[k].index);
	}
}


shift_Struct DisparityTool::filterMaxes()
{
	int maxIndex;
	int closest;
	
	float threshold = 0.005f;
	float threshold2 = 0.05f;
	float vals[__nMaxes];
	for (int i = 0; i < __nMaxes - 1; i++)
	{
		vals[i] = _maxShifts[i].corr;
		if ( !isWithin(_maxShifts[i].index, _shiftMax, _shiftMin) || (vals[i] < threshold) )
			vals[i] = -1;
	}
	vals[__nMaxes - 1] = 0;

	// find closest
	closest = 0;
	int zs = zeroShift();
	for (int ii = 0; ii < (__nMaxes - 1); ii++)
	{
		if (abs(_maxShifts[ii].index-zs) < (_maxShifts[closest].index-zs))
			closest = ii;
	}

	// find higher, starting from closest, so that if no best matches are found we go 
	// to the closest peak
	maxIndex = closest;
	for (int iii = 0; iii < __nMaxes; iii++)
	{
		if (vals[iii] > (vals[maxIndex] + threshold2) )
			maxIndex = iii;
	}

	return _maxShifts[maxIndex];
}


int DisparityTool::computeDisparityCorrRGBsum(ImageOf<PixelRgb> & inRImg, ImageOf<PixelRgb> & inLImg, int step)
{

	int i,j,k,k1;
	int iL,iR;
	unsigned char *rPtr, *lPtr;
	shift_Struct ret;

	double average_Lr;
	double average_Lg;
	double average_Lb;
	double average_Rr;
	double average_Rg;
	double average_Rb;

	double numr;
	double den_Lr;
	double den_Rr;
	double numg;
	double den_Lg;
	double den_Rg;
	double numb;
	double den_Lb;
	double den_Rb;
	double pixelL;
	double pixelR;

	double R_corr;
	double G_corr;
	double B_corr;


	lPtr = new unsigned char [_img.Size_LP*inLImg.getPixelSize()];
	rPtr = new unsigned char [_img.Size_LP*inRImg.getPixelSize()];
	img2unpaddedVect(lPtr, inLImg);
	img2unpaddedVect(rPtr, inRImg);


	//Correlation Function Computation
	/*********************************/
	for (k = 0; k < _shiftLevels; k++)
	{
		average_Lr = 0;
		average_Lg = 0;
		average_Lb = 0;
		average_Rr = 0;
		average_Rg = 0;
		average_Rb = 0;

		R_corr = 0;
		G_corr = 0;
		B_corr = 0;

		pixelL = 0;
		pixelR = 0;

		k1 = k *_img.Size_LP;

		for (j = 0; j < _actRings; j++)
		{
			for (i = 0; i < _img.Size_Theta; i++)
			{
				iR = _shiftMap[k1 + j*_img.Size_Theta + i];
				iL = 3 * (j*_img.Size_Theta + i);
				if (iR > 0)
				{
					average_Lr += lPtr[iL];
					average_Rr += rPtr[iR];
					average_Lg += lPtr[iL+1];
					average_Rg += rPtr[iR+1];
					average_Lb += lPtr[iL+2];
					average_Rb += rPtr[iR+2];
				}
			}
		}

		if (_count[k] != 0)
		{
			average_Lr /= _count[k];
			average_Rr /= _count[k];
			average_Lg /= _count[k];
			average_Rg /= _count[k];
			average_Lb /= _count[k];
			average_Rb /= _count[k];
		}

		numr   = 0;
		den_Lr = 0;
		den_Rr = 0;
		numg   = 0;
		den_Lg = 0;
		den_Rg = 0;
		numb   = 0;
		den_Lb = 0;
		den_Rb = 0;

		for (j = _actRings - 1; j >= 0; j-=step)
		{

			for (i = _img.Size_Theta - 1; i >= 0; i-=step)
			{
				iR = _shiftMap[k1 + j*_img.Size_Theta + i];
				iL = 3 * (j*_img.Size_Theta + i);
				if (iR > 0)
				{
					pixelL = lPtr[iL] - average_Lr;
					pixelR = rPtr[iR] - average_Rr;
					numr   += (pixelL * pixelR);
					den_Lr += (pixelL * pixelL);
					den_Rr += (pixelR * pixelR);

					pixelL = lPtr[iL+1] - average_Lg;
					pixelR = rPtr[iR+1] - average_Rg;
					numg   += (pixelL * pixelR);
					den_Lg += (pixelL * pixelL);
					den_Rg += (pixelR * pixelR);

					pixelL = lPtr[iL+2] - average_Lb;
					pixelR = rPtr[iR+2] - average_Rb;
					numb   += (pixelL * pixelR);
					den_Lb += (pixelL * pixelL);
					den_Rb += (pixelR * pixelR);

				}
			}
		}

		R_corr = numr / sqrt(den_Lr * den_Rr + 0.00001);
		G_corr = numg / sqrt(den_Lg * den_Rg + 0.00001);
		B_corr = numb / sqrt(den_Lb * den_Rb + 0.00001);
		_corrFunct[k] = (R_corr + G_corr + B_corr) / 3.0;
		_corrFunct[k] *= _count[k];
		_corrFunct[k] /= _maxCount;

	}
	/*********************************/

	findShiftMax(_corrFunct);
	findSecondMaxes(_corrFunct, _maxShifts[0].index);
	ret = filterMaxes();
	
	return (int)ret.disp;
}


int DisparityTool::computeDisparityCorrRGBprod(ImageOf<PixelRgb> & inRImg, ImageOf<PixelRgb> & inLImg, int step)
{

	int i,j,k,k1;
	int iL,iR;
	unsigned char *rPtr, *lPtr;
	shift_Struct ret;

	double average_Lr;
	double average_Lg;
	double average_Lb;
	double average_Rr;
	double average_Rg;
	double average_Rb;

	double numr;
	double den_Lr;
	double den_Rr;
	double numg;
	double den_Lg;
	double den_Rg;
	double numb;
	double den_Lb;
	double den_Rb;
	double pixelL;
	double pixelR;

	double R_corr;
	double G_corr;
	double B_corr;


	lPtr = new unsigned char [_img.Size_LP*inLImg.getPixelSize()];
	rPtr = new unsigned char [_img.Size_LP*inRImg.getPixelSize()];
	img2unpaddedVect(lPtr, inLImg);
	img2unpaddedVect(rPtr, inRImg);


	//Correlation Function Computation
	/*********************************/
	for (k = 0; k < _shiftLevels; k++)
	{
		average_Lr = 0;
		average_Lg = 0;
		average_Lb = 0;
		average_Rr = 0;
		average_Rg = 0;
		average_Rb = 0;

		R_corr = 0;
		G_corr = 0;
		B_corr = 0;

		pixelL = 0;
		pixelR = 0;

		k1 = k *_img.Size_LP;

		for (j = 0; j < _actRings; j++)
		{
			for (i = 0; i < _img.Size_Theta; i++)
			{
				iR = _shiftMap[k1 + j*_img.Size_Theta + i];
				iL = 3 * (j*_img.Size_Theta + i);
				if (iR > 0)
				{
					average_Lr += lPtr[iL];
					average_Rr += rPtr[iR];
					average_Lg += lPtr[iL+1];
					average_Rg += rPtr[iR+1];
					average_Lb += lPtr[iL+2];
					average_Rb += rPtr[iR+2];
				}
			}
		}

		if (_count[k] != 0)
		{
			average_Lr /= _count[k];
			average_Rr /= _count[k];
			average_Lg /= _count[k];
			average_Rg /= _count[k];
			average_Lb /= _count[k];
			average_Rb /= _count[k];
		}

		numr   = 0;
		den_Lr = 0;
		den_Rr = 0;
		numg   = 0;
		den_Lg = 0;
		den_Rg = 0;
		numb   = 0;
		den_Lb = 0;
		den_Rb = 0;

		for (j = _actRings - 1; j >= 0; j-=step)
		{

			for (i = _img.Size_Theta - 1; i >= 0; i-=step)
			{
				iR = _shiftMap[k1 + j*_img.Size_Theta + i];
				iL = 3 * (j*_img.Size_Theta + i);
				if (iR > 0)
				{
					pixelL = lPtr[iL] - average_Lr;
					pixelR = rPtr[iR] - average_Rr;
					numr   += (pixelL * pixelR);
					den_Lr += (pixelL * pixelL);
					den_Rr += (pixelR * pixelR);

					pixelL = lPtr[iL+1] - average_Lg;
					pixelR = rPtr[iR+1] - average_Rg;
					numg   += (pixelL * pixelR);
					den_Lg += (pixelL * pixelL);
					den_Rg += (pixelR * pixelR);

					pixelL = lPtr[iL+2] - average_Lb;
					pixelR = rPtr[iR+2] - average_Rb;
					numb   += (pixelL * pixelR);
					den_Lb += (pixelL * pixelL);
					den_Rb += (pixelR * pixelR);

				}
			}
		}

		R_corr = numr / sqrt(den_Lr * den_Rr + 0.00001);
		G_corr = numg / sqrt(den_Lg * den_Rg + 0.00001);
		B_corr = numb / sqrt(den_Lb * den_Rb + 0.00001);
		_corrFunct[k] = R_corr * G_corr * B_corr;
		_corrFunct[k] *= _count[k];
		_corrFunct[k] /= _maxCount;

	}
	/*********************************/

	findShiftMax(_corrFunct);
	findSecondMaxes(_corrFunct, _maxShifts[0].index);
	ret = filterMaxes();
	
	return (int)ret.disp;
}


int DisparityTool::computeDisparitySSDAvg_RGBsum(ImageOf<PixelRgb> & inRImg, ImageOf<PixelRgb> & inLImg, int step)
{

	int i,j,k,k1;
	int iL,iR;
	unsigned char *rPtr, *lPtr;
	shift_Struct ret;
	double tmpMax;

	double average_Lr;
	double average_Lg;
	double average_Lb;
	double average_Rr;
	double average_Rg;
	double average_Rb;

	double numr;
	double numg;
	double numb;
	double pixelL;
	double pixelR;

	double R_ssd;
	double G_ssd;
	double B_ssd;


	lPtr = new unsigned char [_img.Size_LP*inLImg.getPixelSize()];
	rPtr = new unsigned char [_img.Size_LP*inRImg.getPixelSize()];
	img2unpaddedVect(lPtr, inLImg);
	img2unpaddedVect(rPtr, inRImg);


	//SSD Function Computation
	/*********************************/
	for (k = 0; k < _shiftLevels; k++)
	{
		average_Lr = 0;
		average_Lg = 0;
		average_Lb = 0;
		average_Rr = 0;
		average_Rg = 0;
		average_Rb = 0;

		R_ssd = 0;
		G_ssd = 0;
		B_ssd = 0;

		pixelL = 0;
		pixelR = 0;

		k1 = k *_img.Size_LP;

		for (j = 0; j < _actRings; j++)
		{
			for (i = 0; i < _img.Size_Theta; i++)
			{
				iR = _shiftMap[k1 + j*_img.Size_Theta + i];
				iL = 3 * (j*_img.Size_Theta + i);
				if (iR > 0)
				{
					average_Lr += lPtr[iL];
					average_Rr += rPtr[iR];
					average_Lg += lPtr[iL+1];
					average_Rg += rPtr[iR+1];
					average_Lb += lPtr[iL+2];
					average_Rb += rPtr[iR+2];
				}
			}
		}

		if (_count[k] != 0)
		{
			average_Lr /= _count[k];
			average_Rr /= _count[k];
			average_Lg /= _count[k];
			average_Rg /= _count[k];
			average_Lb /= _count[k];
			average_Rb /= _count[k];
		}

		numr   = 0;
		numg   = 0;
		numb   = 0;

		for (j = _actRings - 1; j >= 0; j-=step)
		{

			for (i = _img.Size_Theta - 1; i >= 0; i-=step)
			{
				iR = _shiftMap[k1 + j*_img.Size_Theta + i];
				iL = 3 * (j*_img.Size_Theta + i);
				if (iR > 0)
				{
					pixelL = lPtr[iL] - average_Lr;
					pixelR = rPtr[iR] - average_Rr;
					numr   += (pixelL - pixelR)*(pixelL - pixelR);

					pixelL = lPtr[iL+1] - average_Lg;
					pixelR = rPtr[iR+1] - average_Rg;
					numg   += (pixelL - pixelR)*(pixelL - pixelR);

					pixelL = lPtr[iL+2] - average_Lb;
					pixelR = rPtr[iR+2] - average_Rb;
					numb   += (pixelL - pixelR)*(pixelL - pixelR);

				}
			}
		}
		R_ssd = numr;
		G_ssd = numg;
		B_ssd = numb;
		_ssdFunct[k] = (R_ssd + G_ssd + B_ssd) / 3.0;

	}
	/*********************************/

	for (int s = 0; s < _shiftLevels; s++)
	{	
		tmpMax = find_max_value(_ssdFunct, _shiftLevels);
		if (tmpMax > 0.0)
			_ssdFunct[s] = 1.0 - _ssdFunct[s]/tmpMax;
		_ssdFunct[s] *= _count[s];
		_ssdFunct[s] /= _maxCount;
	}

	findShiftMax(_ssdFunct);
	findSecondMaxes(_ssdFunct, _maxShifts[0].index);
	ret = filterMaxes();
	
	return (int)ret.disp;
}


int DisparityTool::computeDisparitySSDAvg_RGBprod(ImageOf<PixelRgb> & inRImg, ImageOf<PixelRgb> & inLImg, int step)
{

	int i,j,k,k1;
	int iL,iR;
	unsigned char *rPtr, *lPtr;
	shift_Struct ret;
	double tmpMax;

	double average_Lr;
	double average_Lg;
	double average_Lb;
	double average_Rr;
	double average_Rg;
	double average_Rb;

	double numr;
	double numg;
	double numb;
	double pixelL;
	double pixelR;

	double R_ssd;
	double G_ssd;
	double B_ssd;


	lPtr = new unsigned char [_img.Size_LP*inLImg.getPixelSize()];
	rPtr = new unsigned char [_img.Size_LP*inRImg.getPixelSize()];
	img2unpaddedVect(lPtr, inLImg);
	img2unpaddedVect(rPtr, inRImg);


	//SSD Function Computation
	/*********************************/
	for (k = 0; k < _shiftLevels; k++)
	{
		average_Lr = 0;
		average_Lg = 0;
		average_Lb = 0;
		average_Rr = 0;
		average_Rg = 0;
		average_Rb = 0;

		R_ssd = 0;
		G_ssd = 0;
		B_ssd = 0;

		pixelL = 0;
		pixelR = 0;

		k1 = k *_img.Size_LP;

		for (j = 0; j < _actRings; j++)
		{
			for (i = 0; i < _img.Size_Theta; i++)
			{
				iR = _shiftMap[k1 + j*_img.Size_Theta + i];
				iL = 3 * (j*_img.Size_Theta + i);
				if (iR > 0)
				{
					average_Lr += lPtr[iL];
					average_Rr += rPtr[iR];
					average_Lg += lPtr[iL+1];
					average_Rg += rPtr[iR+1];
					average_Lb += lPtr[iL+2];
					average_Rb += rPtr[iR+2];
				}
			}
		}

		if (_count[k] != 0)
		{
			average_Lr /= _count[k];
			average_Rr /= _count[k];
			average_Lg /= _count[k];
			average_Rg /= _count[k];
			average_Lb /= _count[k];
			average_Rb /= _count[k];
		}

		numr   = 0;
		numg   = 0;
		numb   = 0;

		for (j = _actRings - 1; j >= 0; j-=step)
		{

			for (i = _img.Size_Theta - 1; i >= 0; i-=step)
			{
				iR = _shiftMap[k1 + j*_img.Size_Theta + i];
				iL = 3 * (j*_img.Size_Theta + i);
				if (iR > 0)
				{
					pixelL = lPtr[iL] - average_Lr;
					pixelR = rPtr[iR] - average_Rr;
					numr   += (pixelL - pixelR)*(pixelL - pixelR);

					pixelL = lPtr[iL+1] - average_Lg;
					pixelR = rPtr[iR+1] - average_Rg;
					numg   += (pixelL - pixelR)*(pixelL - pixelR);

					pixelL = lPtr[iL+2] - average_Lb;
					pixelR = rPtr[iR+2] - average_Rb;
					numb   += (pixelL - pixelR)*(pixelL - pixelR);

				}
			}
		}
		R_ssd = numr;
		G_ssd = numg;
		B_ssd = numb;
		_ssdFunct[k] = R_ssd * G_ssd * B_ssd;

	}

	
	/*********************************/

	for (int s = 0; s < _shiftLevels; s++)
	{	
		tmpMax = find_max_value(_ssdFunct, _shiftLevels);
		if (tmpMax > 0.0)
			_ssdFunct[s] = 1.0 - _ssdFunct[s]/tmpMax;
		_ssdFunct[s] *= _count[s];
		_ssdFunct[s] /= _maxCount;
	}

	findShiftMax(_ssdFunct);
	findSecondMaxes(_ssdFunct, _maxShifts[0].index);
	ret = filterMaxes();
	
	return (int)ret.disp;
}


int DisparityTool::computeDisparitySSD_RGBsum(ImageOf<PixelRgb> & inRImg, ImageOf<PixelRgb> & inLImg, int step)
{

	int i,j,k,k1;
	int iL,iR;
	unsigned char *rPtr, *lPtr;
	shift_Struct ret;
	double tmpMax;

	double numr;
	double numg;
	double numb;
	double pixelL;
	double pixelR;

	double R_ssd;
	double G_ssd;
	double B_ssd;


	lPtr = new unsigned char [_img.Size_LP*inLImg.getPixelSize()];
	rPtr = new unsigned char [_img.Size_LP*inRImg.getPixelSize()];
	img2unpaddedVect(lPtr, inLImg);
	img2unpaddedVect(rPtr, inRImg);


	//SSD Function Computation
	/*********************************/
	for (k = 0; k < _shiftLevels; k++)
	{
		R_ssd = 0;
		G_ssd = 0;
		B_ssd = 0;

		pixelL = 0;
		pixelR = 0;

		k1 = k *_img.Size_LP;

		numr   = 0;
		numg   = 0;
		numb   = 0;

		for (j = _actRings - 1; j >= 0; j-=step)
		{

			for (i = _img.Size_Theta - 1; i >= 0; i-=step)
			{
				iR = _shiftMap[k1 + j*_img.Size_Theta + i];
				iL = 3 * (j*_img.Size_Theta + i);
				if (iR > 0)
				{
					pixelL = lPtr[iL];
					pixelR = rPtr[iR];
					numr   += (pixelL - pixelR)*(pixelL - pixelR);

					pixelL = lPtr[iL+1];
					pixelR = rPtr[iR+1];
					numg   += (pixelL - pixelR)*(pixelL - pixelR);

					pixelL = lPtr[iL+2];
					pixelR = rPtr[iR+2];
					numb   += (pixelL - pixelR)*(pixelL - pixelR);

				}
			}
		}
		R_ssd = numr;
		G_ssd = numg;
		B_ssd = numb;
		_ssdFunct[k] = (R_ssd + G_ssd + B_ssd) / 3.0;

	}
	/*********************************/

	for (int s = 0; s < _shiftLevels; s++)
	{	
		tmpMax = find_max_value(_ssdFunct, _shiftLevels);
		if (tmpMax > 0.0)
			_ssdFunct[s] = 1.0 - _ssdFunct[s]/tmpMax;
		_ssdFunct[s] *= _count[s];
		_ssdFunct[s] /= _maxCount;
	}

	findShiftMax(_ssdFunct);
	findSecondMaxes(_ssdFunct, _maxShifts[0].index);
	ret = filterMaxes();
	return (int)ret.disp;
}


int DisparityTool::computeDisparitySSD_RGBprod(ImageOf<PixelRgb> & inRImg, ImageOf<PixelRgb> & inLImg, int step)
{

	int i,j,k,k1;
	int iL,iR;
	unsigned char *rPtr, *lPtr;
	shift_Struct ret;
	double tmpMax;

	double numr;
	double numg;
	double numb;
	double pixelL;
	double pixelR;

	double R_ssd;
	double G_ssd;
	double B_ssd;


	lPtr = new unsigned char [_img.Size_LP*inLImg.getPixelSize()];
	rPtr = new unsigned char [_img.Size_LP*inRImg.getPixelSize()];
	img2unpaddedVect(lPtr, inLImg);
	img2unpaddedVect(rPtr, inRImg);


	//SSD Function Computation
	/*********************************/
	for (k = 0; k < _shiftLevels; k++)
	{
		R_ssd = 0;
		G_ssd = 0;
		B_ssd = 0;

		pixelL = 0;
		pixelR = 0;

		k1 = k *_img.Size_LP;

		numr   = 0;
		numg   = 0;
		numb   = 0;

		for (j = _actRings - 1; j >= 0; j-=step)
		{

			for (i = _img.Size_Theta - 1; i >= 0; i-=step)
			{
				iR = _shiftMap[k1 + j*_img.Size_Theta + i];
				iL = 3 * (j*_img.Size_Theta + i);
				if (iR > 0)
				{
					pixelL = lPtr[iL];
					pixelR = rPtr[iR];
					numr   += (pixelL - pixelR)*(pixelL - pixelR);

					pixelL = lPtr[iL+1];
					pixelR = rPtr[iR+1];
					numg   += (pixelL - pixelR)*(pixelL - pixelR);

					pixelL = lPtr[iL+2];
					pixelR = rPtr[iR+2];
					numb   += (pixelL - pixelR)*(pixelL - pixelR);

				}
			}
		}
		R_ssd = numr;
		G_ssd = numg;
		B_ssd = numb;
		_ssdFunct[k] = R_ssd * G_ssd * B_ssd;

	}
	/*********************************/

	for (int s = 0; s < _shiftLevels; s++)
	{	
		tmpMax = find_max_value(_ssdFunct, _shiftLevels);
		if (tmpMax > 0.0)
			_ssdFunct[s] = 1.0 - _ssdFunct[s]/tmpMax;
		_ssdFunct[s] *= _count[s];
		_ssdFunct[s] /= _maxCount;
	}

	findShiftMax(_ssdFunct);
	findSecondMaxes(_ssdFunct, _maxShifts[0].index);
	ret = filterMaxes();
	
	return (int)ret.disp;
}


void DisparityTool::print_Data(char *filename)
{
	FILE *fout;

	fout = fopen(filename, "wa");
	fprintf(fout, "\nMax Shift:\n\tdisp index: %d\tdisp value: %d\tcorr value: %f\n", _maxShifts[0].index, (int)_maxShifts[0].disp, _maxShifts[0].corr);
	fprintf(fout, "\nSecond Max:\n\tdisp index: %d\tdisp value: %d\tcorr value: %f\n", _maxShifts[1].index, (int)_maxShifts[1].disp, _maxShifts[1].corr);
	fprintf(fout, "\nThird Max:\n\tdisp index: %d\tdisp value: %d\tcorr value: %f\n", _maxShifts[2].index, (int)_maxShifts[2].disp, _maxShifts[2].corr);
	fprintf(fout, "\nZero Shift:\n\tdisp index: %d\tdisp value: %d\tcorr value: %f\n\n", _maxShifts[3].index, (int)_maxShifts[3].disp, _maxShifts[3].corr);

	for (int k = 0; k < _shiftLevels; k++)
	{
		fprintf(fout,"Disp: %d\t", (int)shiftToDisparity(k));
		fprintf(fout,"Corr: %f\n", getCorrValue(k));
	}
	fclose (fout);
}


void DisparityTool::print_dataBlock(FILE *fout, shift_Struct *shifts, int numb)
{
	fprintf(fout, "\nImage Nr %d\n", numb);
	fprintf(fout, "\n\tMax Shift:\n\t\tdisp index: %d\tdisp value: %d\tcorr value: %f\n", shifts[0].index, (int)shifts[0].disp, shifts[0].corr);
	fprintf(fout, "\n\tSecond Max:\n\t\tdisp index: %d\tdisp value: %d\tcorr value: %f\n", shifts[1].index, (int)shifts[1].disp, shifts[1].corr);
	fprintf(fout, "\n\tThird Max:\n\t\tdisp index: %d\tdisp value: %d\tcorr value: %f\n", shifts[2].index, (int)shifts[2].disp, shifts[2].corr);
	fprintf(fout, "\n\tZero Shift:\n\t\tdisp index: %d\tdisp value: %d\tcorr value: %f\n\n\n", shifts[3].index, (int)shifts[3].disp, shifts[3].corr);

}


void DisparityTool::print_Data_SSD(char *filename)
{
	FILE *fout;

	fout = fopen(filename, "wa");
	fprintf(fout, "\nMin SSD:\n\tdisp index: %d\tdisp value: %d\tSSD value: %f\n", _maxShifts[0].index, (int)_maxShifts[0].disp, _maxShifts[0].corr);
	fprintf(fout, "\nSecond Min:\n\tdisp index: %d\tdisp value: %d\tSSD value: %f\n", _maxShifts[1].index, (int)_maxShifts[1].disp, _maxShifts[1].corr);
	fprintf(fout, "\nThird Min:\n\tdisp index: %d\tdisp value: %d\tSSD value: %f\n", _maxShifts[2].index, (int)_maxShifts[2].disp, _maxShifts[2].corr);
	fprintf(fout, "\nZero Shift:\n\tdisp index: %d\tdisp value: %d\tSSD value: %f\n\n", _maxShifts[3].index, (int)_maxShifts[3].disp, _maxShifts[3].corr);

	for (int k = 0; k < _shiftLevels; k++)
	{
		fprintf(fout,"Disp: %d\t", (int)shiftToDisparity(k));
		fprintf(fout,"SSD: %f\n", getSSDValue(k));
	}
	fclose (fout);
}


void DisparityTool::print_dataBlock_SSD(FILE *fout, shift_Struct *shifts, int numb)
{
	fprintf(fout, "\nImage Nr %d\n", numb);
	fprintf(fout, "\n\tMin SSD:\n\t\tdisp index: %d\tdisp value: %d\tSSD value: %f\n", shifts[0].index, (int)shifts[0].disp, shifts[0].corr);
	fprintf(fout, "\n\tSecond Min:\n\t\tdisp index: %d\tdisp value: %d\tSSD value: %f\n", shifts[1].index, (int)shifts[1].disp, shifts[1].corr);
	fprintf(fout, "\n\tThird Min:\n\t\tdisp index: %d\tdisp value: %d\tSSD value: %f\n", shifts[2].index, (int)shifts[2].disp, shifts[2].corr);
	fprintf(fout, "\n\tZero Shift:\n\t\tdisp index: %d\tdisp value: %d\tSSD value: %f\n\n\n", shifts[3].index, (int)shifts[3].disp, shifts[3].corr);

}


int DisparityTool::computeDisparityCorrRGBsum2(ImageOf<PixelRgb> & inRImg, ImageOf<PixelRgb> & inLImg, int step)
{
	int i,j,k,k1;
	int iL,iR;
	unsigned char *rPtr, *lPtr;
	shift_Struct ret;

	double average_Lr;
	double average_Lg;
	double average_Lb;
	double average_Rr;
	double average_Rg;
	double average_Rb;

	double numr;
	double den_Lr;
	double den_Rr;
	double numg;
	double den_Lg;
	double den_Rg;
	double numb;
	double den_Lb;
	double den_Rb;
	double pixelL;
	double pixelR;

	double R_corr;
	double G_corr;
	double B_corr;


	lPtr = new unsigned char [_img.Size_LP*inLImg.getPixelSize()];
	rPtr = new unsigned char [_img.Size_LP*inRImg.getPixelSize()];
	img2unpaddedVect(lPtr, inLImg);
	img2unpaddedVect(rPtr, inRImg);


	//Correlation Function Computation
	/*********************************/
	for (k = 0; k < _shiftLevels; k++)
	{
		average_Lr = 0;
		average_Lg = 0;
		average_Lb = 0;
		average_Rr = 0;
		average_Rg = 0;
		average_Rb = 0;

		R_corr = 0;
		G_corr = 0;
		B_corr = 0;

		pixelL = 0;
		pixelR = 0;

		k1 = k *_img.Size_LP;

		for (j = 0; j < _actRings; j++)
		{
			for (i = 0; i < _img.Size_Theta; i++)
			{
				iR = _shiftMap[k1 + j*_img.Size_Theta + i];
				iL = 3 * (j*_img.Size_Theta + i);
				if (iR > 0)
				{
					average_Lr += lPtr[iL];
					average_Rr += rPtr[iR];
					average_Lg += lPtr[iL+1];
					average_Rg += rPtr[iR+1];
					average_Lb += lPtr[iL+2];
					average_Rb += rPtr[iR+2];
				}
			}
		}

		if (_count[k] != 0)
		{
			average_Lr /= _count[k];
			average_Rr /= _count[k];
			average_Lg /= _count[k];
			average_Rg /= _count[k];
			average_Lb /= _count[k];
			average_Rb /= _count[k];
		}

		numr   = 0;
		den_Lr = 0;
		den_Rr = 0;
		numg   = 0;
		den_Lg = 0;
		den_Rg = 0;
		numb   = 0;
		den_Lb = 0;
		den_Rb = 0;

		for (j = _actRings - 1; j >= 0; j-=step)
		{

			for (i = _img.Size_Theta - 1; i >= 0; i-=step)
			{
				iR = _shiftMap[k1 + j*_img.Size_Theta + i];
				iL = 3 * (j*_img.Size_Theta + i);
				if (iR > 0)
				{
					pixelL = lPtr[iL] - average_Lr;
					pixelR = rPtr[iR] - average_Rr;
					numr   += (pixelL * pixelR);
					den_Lr += (pixelL * pixelL);
					den_Rr += (pixelR * pixelR);

					pixelL = lPtr[iL+1] - average_Lg;
					pixelR = rPtr[iR+1] - average_Rg;
					numg   += (pixelL * pixelR);
					den_Lg += (pixelL * pixelL);
					den_Rg += (pixelR * pixelR);

					pixelL = lPtr[iL+2] - average_Lb;
					pixelR = rPtr[iR+2] - average_Rb;
					numb   += (pixelL * pixelR);
					den_Lb += (pixelL * pixelL);
					den_Rb += (pixelR * pixelR);

				}
			}
		}

		R_corr = (numr * numr) / (den_Lr * den_Rr + 0.00001);
		G_corr = (numg * numg) / (den_Lg * den_Rg + 0.00001);
		B_corr = (numb * numb) / (den_Lb * den_Rb + 0.00001);
		_corrFunct[k] = (R_corr + G_corr + B_corr) / 3.0;
		_corrFunct[k] *= _count[k];
		_corrFunct[k] /= _maxCount;

	}
	/*********************************/

	findShiftMax(_corrFunct);
	findSecondMaxes(_corrFunct, _maxShifts[0].index);
	ret = filterMaxes();
	
	return (int)ret.disp;
}


int DisparityTool::computeDisparityCorrRGBsum3(ImageOf<PixelRgb> & inRImg, ImageOf<PixelRgb> & inLImg, int step)
{
	int i,j,k,k1;
	int iL,iR;
	unsigned char *rPtr, *lPtr;
	shift_Struct ret;

	double average_Lr;
	double average_Lg;
	double average_Lb;
	double average_Rr;
	double average_Rg;
	double average_Rb;

	double numr;
	double den_Lr;
	double den_Rr;
	double numg;
	double den_Lg;
	double den_Rg;
	double numb;
	double den_Lb;
	double den_Rb;
	double pixelL;
	double pixelR;

	double R_corr;
	double G_corr;
	double B_corr;


	lPtr = new unsigned char [_img.Size_LP*inLImg.getPixelSize()];
	rPtr = new unsigned char [_img.Size_LP*inRImg.getPixelSize()];
	img2unpaddedVect(lPtr, inLImg);
	img2unpaddedVect(rPtr, inRImg);


	//Correlation Function Computation
	/*********************************/
	for (k = 0; k < _shiftLevels; k++)
	{
		average_Lr = 0;
		average_Lg = 0;
		average_Lb = 0;
		average_Rr = 0;
		average_Rg = 0;
		average_Rb = 0;

		R_corr = 0;
		G_corr = 0;
		B_corr = 0;

		pixelL = 0;
		pixelR = 0;

		k1 = k *_img.Size_LP;

		for (j = 0; j < _actRings; j++)
		{
			for (i = 0; i < _img.Size_Theta; i++)
			{
				iR = 3 * (j*_img.Size_Theta + i);
				iL = 3 * (j*_img.Size_Theta + i);

				average_Lr += lPtr[iL];
				average_Rr += rPtr[iR];
				average_Lg += lPtr[iL+1];
				average_Rg += rPtr[iR+1];
				average_Lb += lPtr[iL+2];
				average_Rb += rPtr[iR+2];
			}
		}

		average_Lr /= _img.Size_LP;
		average_Rr /= _img.Size_LP;
		average_Lg /= _img.Size_LP;
		average_Rg /= _img.Size_LP;
		average_Lb /= _img.Size_LP;
		average_Rb /= _img.Size_LP;

		numr   = 0;
		den_Lr = 0;
		den_Rr = 0;
		numg   = 0;
		den_Lg = 0;
		den_Rg = 0;
		numb   = 0;
		den_Lb = 0;
		den_Rb = 0;

		for (j = _actRings - 1; j >= 0; j-=step)
		{

			for (i = _img.Size_Theta - 1; i >= 0; i-=step)
			{
				iR = _shiftMap[k1 + j*_img.Size_Theta + i];
				iL = 3 * (j*_img.Size_Theta + i);
				if (iR > 0)
				{
					pixelL = lPtr[iL] - average_Lr;
					pixelR = rPtr[iR] - average_Rr;
					numr   += (pixelL * pixelR);
					den_Lr += (pixelL * pixelL);
					den_Rr += (pixelR * pixelR);

					pixelL = lPtr[iL+1] - average_Lg;
					pixelR = rPtr[iR+1] - average_Rg;
					numg   += (pixelL * pixelR);
					den_Lg += (pixelL * pixelL);
					den_Rg += (pixelR * pixelR);

					pixelL = lPtr[iL+2] - average_Lb;
					pixelR = rPtr[iR+2] - average_Rb;
					numb   += (pixelL * pixelR);
					den_Lb += (pixelL * pixelL);
					den_Rb += (pixelR * pixelR);

				}
			}
		}

		R_corr = numr / sqrt(den_Lr * den_Rr + 0.00001);
		G_corr = numg / sqrt(den_Lg * den_Rg + 0.00001);
		B_corr = numb / sqrt(den_Lb * den_Rb + 0.00001);
		_corrFunct[k] = (sqrt(R_corr) + sqrt(G_corr) + sqrt(B_corr)) / 3.0;
		_corrFunct[k] *= _count[k];
		_corrFunct[k] /= _maxCount;

	}
	/*********************************/

	findShiftMax(_corrFunct);
	findSecondMaxes(_corrFunct, _maxShifts[0].index);
	ret = filterMaxes();
	
	return (int)ret.disp;
}

