
#if !defined(AFX_DISPARITY_H__E3019D2A_4BC3_4AD0_8355_AAA601391249__INCLUDED_)
#define AFX_DISPARITY_H__E3019D2A_4BC3_4AD0_8355_AAA601391249__INCLUDED_

//#include <ace/config.h>
//#include <ace/OS.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <iCub/RC_Logpolar_AuxFunct.h>

#ifdef YARP_HAS_PRAGMA_ONCE
#	pragma once
#endif

const int __nMaxes = 4;



using namespace yarp::os;
using namespace yarp::sig;
using namespace _logpolarParams;


class DisparityTool
{
public:
	DisparityTool();
	virtual ~DisparityTool();

	inline int getShiftLevels()
		{ return _shiftLevels; }
	inline int zeroShift()
		{ return (_shiftLevels/2); }
	inline double getCorrValue(int n)
		{ return _corrFunct[n]; }
	inline double getSSDValue(int n)
		{ return _ssdFunct[n]; }
	inline void setInhibition(int max, int min)
		{ _shiftMax = disparityToShift((double)max); _shiftMin = disparityToShift((double)min); }
	inline int getLimitsMax()
		{ return _shiftMax; }
	inline int getLimitsMin()
		{ return _shiftMin; }	
	inline void setRings(int r) 
		{ _actRings = r; computeCountVector(_count); }
	inline shift_Struct getMax(int n = 0)
		{ return _maxShifts[n]; }
	inline double shiftToDisparity(int shift)
		{ return _shiftFunction[shift]; }
	inline int disparityToShift(double disp)
		{ double min = find_min_value(_shiftFunction, _shiftLevels);
		  double max = find_max_value(_shiftFunction, _shiftLevels);
		  int index = (int)(disp - min);
		  if (index < 0)
		  	  index = 0;
		  else if (index > (max-min))
			  index = max-min;			  
		  return _shiftFunctionInv[index]; }

	int computeDisparityCorrRGBsum (ImageOf<PixelRgb> & inRImg, ImageOf<PixelRgb> & inLImg, int step);
	int computeDisparityCorrRGBsum2 (ImageOf<PixelRgb> & inRImg, ImageOf<PixelRgb> & inLImg, int step);
	int computeDisparityCorrRGBsum3 (ImageOf<PixelRgb> & inRImg, ImageOf<PixelRgb> & inLImg, int step);
	int computeDisparityCorrRGBprod (ImageOf<PixelRgb> & inRImg, ImageOf<PixelRgb> & inLImg, int step);
	int computeDisparitySSD_RGBsum (ImageOf<PixelRgb> & inRImg, ImageOf<PixelRgb> & inLImg, int step);
	int computeDisparitySSD_RGBprod (ImageOf<PixelRgb> & inRImg, ImageOf<PixelRgb> & inLImg, int step);
	int computeDisparitySSDAvg_RGBsum (ImageOf<PixelRgb> & inRImg, ImageOf<PixelRgb> & inLImg, int step);
	int computeDisparitySSDAvg_RGBprod (ImageOf<PixelRgb> & inRImg, ImageOf<PixelRgb> & inLImg, int step);
	void makeHistogram(ImageOf<PixelMono> & hImg);
	void makeNormSSDHistogram(ImageOf<PixelMono> & hImg);
	void Remap(ImageOf<PixelRgb>  & lpIn, ImageOf<PixelRgb>  & cartOut);
	void init(int rho, int theta, int mode, double overlap, int xo, int yo, int xr, int yr, int actR);
	void print_Data(char *filename);
	void print_dataBlock(FILE *fout, shift_Struct *shifts, int numb);
	void print_Data_SSD(char *filename);
	void print_dataBlock_SSD(FILE *fout, shift_Struct *shifts, int numb);

	shift_Struct filterMaxes();
	void findShiftMax(const double *ssd);
	void findSecondMaxes(const double *ssd, int posMin);
	void computeCountVector(int *count);
	void LoadShiftMap();
	void AllocateVectors();


protected:

	Image_Data _img;

	int   _shiftLevels;
	int * _shiftMap;
	double * _shiftFunction;
	int * _shiftFunctionInv;
	double * _corrFunct;
	double *_ssdFunct;
	shift_Struct _maxShifts[__nMaxes];

	int *_count;
	int _maxCount;
		
	int _actRings;

	int _shiftMax;
	int _shiftMin;

	char _path[256];
};

#endif // !defined(AFX_DISPARITY_H__E3019D2A_4BC3_4AD0_8355_AAA601391249__INCLUDED_)
