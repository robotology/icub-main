#include <stdio.h>
#include <ipl.h>
#include <math.h>
#include <float.h>

typedef IplImage* pIplImage;

class YARPHMax
{
	int subDim;
	int maxDim;
	int _numFil;
	int _numPatch;
	int _idxPatch;
	IplConvKernelFP **_filters;
	pIplImage *_subFeature; // float
	IplConvKernelFP ***_patch;
	pIplImage *srcSqA; //float
	int *_x;
	int *_y;
	IplConvKernelFP *_sumFil1row;
	IplConvKernelFP *_sumFil1col;
	IplConvKernelFP *_sumFil2;

	/////////////////////////////////
	IplImage *srcFP;
	IplImage *srcSq;
	IplImage *tmp;
	IplImage *tmp2;
	IplImage *srcNorm;
	
	IplImage *tmp1S;
	IplImage *tmp2S;
	IplImage *imgNorm;
	IplImage *sum;
	////////////////////////////////

	void _freeFilters();
	int _readFiltersFromFile(const char *src);
	float _sumSqFilter(float *values, int dim);
	void _normFilter(float *values, int dim);
	void _convertFP(IplImage* src, IplImage* dst);
	void _absDiv(IplImage* src1, IplImage* src2, IplImage* dst);
	void _maxFilter(IplImage* src, IplImage* dst, const int dim);
	void _maxFilterAndSubsample(IplImage* src, IplImage* dst, const int dim, const int fact);
	void _sqrtImage(IplImage* src, IplImage* dst);
	void _subsample(IplImage* src, IplImage* dst, int fact);
	IplConvKernelFP** _randomPatch(IplImage** src, int dim, int idx);
	float _findMax(IplImage* src);
	float _findMin(IplImage* src);
	int _firstStep(IplImage* src);
	int _maxResponse(double* resp);

	IplConvKernelFP* _createConvKernelFP(int rows, int cols, int anchorX, int anchorY, float *values);
	IplImage* _createImageFloat(const int width, const int height);

	IplConvKernelFP* _createGaborFilter(double rot, int size, double div);

	int _ran1N(int min, int max);

	void _saveImgData(char* filename, IplImage* src);
public:
	YARPHMax::YARPHMax();
	YARPHMax::~YARPHMax();

	int gatherPatch(IplImage* src, int num);
	void endGather();
	void init(const int numP, const int xmax, const int ymax);
	int apply(IplImage* src, double *resp);
	int savePatches(const char* name);
	int loadPatches(const char* name);
};

/* get pointer to pixel at (col,row),
   for multi-channel images (col) should be multiplied by number of channels */
#define IPL_IMAGE_ELEM( image, elemtype, row, col )       \
	( ( (elemtype*) ((image)->imageData + (image)->widthStep*(row)) )[(col)] )

#define IPL_IMAGE_ELEM_ADDRESS( rowAddress, elemtype, col )       \
	( ( (elemtype*) (rowAddress) )[(col)] )

#define UABS(X) (x>0?x:-x)