#include <ipl.h>

#include "YARPHMax.h"


YARPHMax::YARPHMax()
{
	_numFil=0;
	_filters=NULL;
	_subFeature=NULL;
	_patch=NULL;

	//if (_readFiltersFromFile("d:\\yarp2\\zgarbage\\hmax\\filters.hmx")==-1) {
		_freeFilters();
		_numFil=4;
		_filters = new IplConvKernelFP* [_numFil];
		_filters[0]=_createGaborFilter(90, 11, 3.9);
		_filters[1]=_createGaborFilter(-45, 11, 3.9);
		_filters[2]=_createGaborFilter(0, 11, 3.9);
		_filters[3]=_createGaborFilter(45, 11, 3.9);
	//}

	_subFeature = new pIplImage [_numFil];

	maxDim=4;
	subDim=maxDim+1;
}


YARPHMax::~YARPHMax()
{
	if (_subFeature!=NULL) delete [] _subFeature;
}


void YARPHMax::init(const int numP, const int xmax, const int ymax)
{
	_numPatch=numP;
	_idxPatch=0;

	_patch= new IplConvKernelFP** [_numPatch];

	float values[25];
	for (int i=0; i<25; i++)
		values[i]=1;

	_sumFil1row=iplCreateConvKernelFP(11, 1, 5, 0, values);
	_sumFil1col=iplCreateConvKernelFP(1, 11, 0, 5, values);
	_sumFil2=iplCreateConvKernelFP(3, 3, 1, 1, values);

	_x = new int [_numPatch];
	_y = new int [_numPatch];

	srcFP=_createImageFloat(xmax,ymax);
	srcSq=_createImageFloat(xmax,ymax);
	tmp=_createImageFloat(xmax,ymax);
	tmp2=_createImageFloat(xmax,ymax);
	srcNorm=_createImageFloat(xmax,ymax);

	for (i=0; i<_numFil; i++)
		_subFeature[i]=_createImageFloat(int(ceil(double(xmax-2*maxDim)/subDim)), int(ceil(double(ymax-2*maxDim)/subDim)));

	const int xmax_s=_subFeature[0]->width;
	const int ymax_s=_subFeature[0]->height;

	tmp1S=_createImageFloat(xmax_s,ymax_s);
	tmp2S=_createImageFloat(xmax_s,ymax_s);
	sum=_createImageFloat(xmax_s,ymax_s);
	imgNorm=_createImageFloat(xmax_s,ymax_s);
}

IplImage* YARPHMax::_createImageFloat(const int width, const int height)
{
    IplImage* pImage = iplCreateImageHeader(
						          1,
								  0,
                                  IPL_DEPTH_32F,
                                  "GRAY",
                                  "GRAY",
                                  IPL_DATA_ORDER_PIXEL,
                                  IPL_ORIGIN_TL,
                                  IPL_ALIGN_QWORD,
                                  width,
                                  height,
                                  NULL,
                                  NULL,
                                  NULL,
                                  NULL);
	iplAllocateImageFP(pImage, 1, 0);
	return pImage;
}


void _destroyImage(pIplImage& img)
{
	if (img!=NULL) {
		iplDeallocateImage(img);
		img=NULL;
	}
}


void _zeroImage(IplImage* img)
{
	memset(img->imageData,0,img->imageSize);
}


void YARPHMax::endGather()
{
	srcSqA = new pIplImage [_numFil];
	//printf("End of collecting patches\n");
}

int YARPHMax::_firstStep(IplImage* src)
{
	//printf("First step...");

	const int xmax=src->width;
	const int ymax=src->height;

	//iplConvert(src, srcFP);
	//iplScaleFP(src, srcFP);
	_convertFP(src, srcFP);

	iplSquare(srcFP, srcSq);
	iplConvolveSep2DFP(srcSq, srcNorm, _sumFil1row, _sumFil1col);
	_sqrtImage(srcNorm, srcNorm);
			
	for (int i=0; i<_numFil; i++) {
		iplConvolve2DFP(srcFP, tmp, _filters+i, 1, IPL_SUM);
		_absDiv(tmp, srcNorm, tmp2);

		//_saveImgData("d:\\yarp2\\zgarbage\\prova.m", tmp2);

		//tmp=tmp2; //Problem with the border due to _maxFilter
		//memcpy(tmp->imageData, tmp2->imageData, tmp2->imageSize); //Problem with the border due to _maxFilter
		//_maxFilter(tmp2, tmp, 4);
		//_subsample(tmp, _subFeature[i], fact);
		_maxFilterAndSubsample(tmp2, _subFeature[i], maxDim, subDim);
		//_maxFilterAndSubsample(tmp, _subFeature[i], 4, fact);
		if (_findMax(_subFeature[i])>1) 
			printf("Errore!\n");

		//_saveImgData("d:\\yarp2\\zgarbage\\prova.m", _subFeature[i]);
	}

	//printf("done!\n");

	return -1;
}


int YARPHMax::_maxResponse(double* resp)
{
	_zeroImage(imgNorm);
	for (int i=0; i<_numFil; i++) {
		//srcSqA[i]=_createImageFloat(xmax,ymax);
		//iplSquare(_subFeature[i], srcSqA[i]);
		iplSquare(_subFeature[i], tmp1S);
		iplConvolveSep2DFP(tmp1S, tmp2S, _sumFil1row, _sumFil1col);
		iplAdd(imgNorm,tmp2S,imgNorm);
	}
	
	for (i=0; i<_numPatch; i++) {
		_zeroImage(sum);
		IplConvKernelFP** p=_patch[i];
		float psq=0;
		for (int n=0; n<_numFil; n++) {
			psq+=_sumSqFilter(p[n]->values, p[n]->nCols*p[n]->nRows);
			//iplAddSFP(sum,sum,psq);

			//iplConvolveSep2DFP(srcSqA[n], tmpNorm, _sumFil1row, _sumFil1col);
			//iplAdd(tmpNorm,sum,sum);
			//iplAdd(srcSqA[n],sum,sum);
			
			iplConvolve2DFP(_subFeature[n], tmp1S, p+n, 1, IPL_SUM);
			iplMultiplySFP(tmp1S,tmp1S,-2);
			iplAdd(tmp1S,sum,sum);
		}
		iplAdd(imgNorm,sum,sum);
		resp[i]=_findMin(sum)+psq+1e-10f;
	}

	return -1;
}


int YARPHMax::gatherPatch(IplImage* src, int num)
{
	_firstStep(src);

	for (int i=0; i<num; i++) {
		_patch[_idxPatch]=_randomPatch(_subFeature, 2, _idxPatch);
		if (_idxPatch<_numPatch) _idxPatch++;
	}
	printf("%d random patches collected\n",_idxPatch);

	return (_idxPatch-_numPatch);
}


int YARPHMax::savePatches(const char* name)
{
	FILE  *fp = fopen(name, "wb");
	
	if (!fp) {
		fprintf(stderr, "Error - cannot open file for writing\n");
		return -1;
	}

	fwrite(&_numPatch, sizeof(int), 1, fp);
	fwrite(&_numFil, sizeof(int), 1, fp);

	for (int i=0; i<_numPatch; i++) {
		IplConvKernelFP** p=_patch[i];
		for (int n=0; n<_numFil; n++) {
			int cols, rows, anchorX, anchorY;
			float *values;
			cols=p[n]->nCols;
			rows=p[n]->nRows;
			anchorX=p[n]->anchorX;
			anchorY=p[n]->anchorY;
			values=p[n]->values;
			
			fwrite(&cols, sizeof(int), 1, fp);
			fwrite(&rows, sizeof(int), 1, fp);
			fwrite(&anchorX, sizeof(int), 1, fp);
			fwrite(&anchorY, sizeof(int), 1, fp);
			fwrite(values, sizeof(float), cols*rows, fp);
		}
	}
	
	fclose(fp);

	return 0;
}


int YARPHMax::loadPatches(const char* name)
{
	FILE  *fp = fopen(name, "rb");
	
	if (!fp) {
		fprintf(stderr, "Error - cannot open file for reading\n");
		return -1;
	}

	fread(&_numPatch, sizeof(int), 1, fp);
	fread(&_numFil, sizeof(int), 1, fp);
	printf("Loading %d random patches\n", _numPatch);

	for (int i=0; i<_numPatch; i++) {
		IplConvKernelFP** tmp=new IplConvKernelFP*[_numFil];
		
		for (int n=0; n<_numFil; n++) {
			int cols, rows, anchorX, anchorY;
			float *values;
			
			fread(&cols, sizeof(int), 1, fp);
			fread(&rows, sizeof(int), 1, fp);
			fread(&anchorX, sizeof(int), 1, fp);
			fread(&anchorY, sizeof(int), 1, fp);
			values = new float [cols*rows];
			fread(values, sizeof(float), cols*rows, fp);
			
			tmp[n]=_createConvKernelFP(cols, rows, anchorX, anchorY, values);

			delete [] values;
		}
		_patch[i]=tmp;
	}
	
	fclose(fp);

	return 0;
}


int YARPHMax::apply(IplImage* src, double* resp)
{
	int i;

	_firstStep(src);
	_maxResponse(resp);

	for (i=0; i<_numPatch; i++)
		resp[i]=exp(-(resp[i]*resp[i])/2);

	//for (i=0; i<_numPatch; i++)
	//	fprintf(stdout, "%1.2f ", resp[i]);
	//fprintf(stdout, "\r");
	//fprintf(stdout, "\n");
	
	return -1;
}


void YARPHMax::_freeFilters()
{
	if (_numFil>0) {
		for (int i=0; i<_numFil; i++)
			delete _filters[i];
		delete [] _filters;
	}
}


int YARPHMax::_readFiltersFromFile(const char *src)
{
	int cols, rows, anchorX, anchorY;
	float* values;

	FILE  *fp = fopen(src, "r");
	
	if (!fp) {
		fprintf(stderr, "Error - cannot open file for reading\n");
		return -1;
	}

	_freeFilters();

	fscanf(fp,"%d ",&_numFil);
	
	_filters = new IplConvKernelFP* [_numFil];

	for (int i=0; i<_numFil; i++) {
		fscanf(fp,"%d ",&rows);
		fscanf(fp,"%d ",&cols);
		fscanf(fp,"%d ",&anchorX);
		fscanf(fp,"%d ",&anchorY);
		values = new float [rows*cols];
		for (int n=0; n<rows*cols; n++)
			fscanf(fp,"%f ",values+n);
		
		_normFilter(values,rows*cols);

		_filters[i]=iplCreateConvKernelFP(cols, rows, anchorX, anchorY, values);
	}
	
	fclose(fp);

	return 0;
}


float YARPHMax::_sumSqFilter(float *values, int dim)
{
	float sum=0;
	for (int i=0; i<dim; i++)
		sum+=values[i]*values[i];

	return sum;
}


void YARPHMax::_normFilter(float *values, int dim)
{
	float sum=_sumSqFilter(values, dim);

	sum=(float)sqrt(sum);
	for (int i=0; i<dim; i++)
		values[i]=values[i]/sum;
}


void YARPHMax::_convertFP(IplImage* src, IplImage* dst)
{
	const int xmax=src->width;
	const int ymax=src->height;

	for (int x=0; x<xmax; x++)
		for (int y=0; y<ymax; y++)
			IPL_IMAGE_ELEM( dst, float, y, x )=IPL_IMAGE_ELEM( src, unsigned char, y, x );
}


void YARPHMax::_sqrtImage(IplImage* src, IplImage* dst)
{
	const int xmax=src->width;
	const int ymax=src->height;

	for (int x=0; x<xmax; x++)
		for (int y=0; y<ymax; y++) {
			float tmp=IPL_IMAGE_ELEM( src, float, y, x );
			IPL_IMAGE_ELEM( dst, float, y, x )=(float)sqrt(tmp);
		}
}


void YARPHMax::_absDiv(IplImage* src1, IplImage* src2, IplImage* dst)
{
	const int xmax=src1->width;
	const int ymax=src1->height;

	for (int x=0; x<xmax; x++)
		for (int y=0; y<ymax; y++) {
			float tmp1=IPL_IMAGE_ELEM( src1, float, y, x );
			float tmp2=IPL_IMAGE_ELEM( src2, float, y, x );
			//IPL_IMAGE_ELEM( dst, float, y, x )=(float)(fabs(tmp1)/sqrt(tmp2));
			IPL_IMAGE_ELEM( dst, float, y, x )=(float)(fabs(tmp1)/tmp2);
			//IPL_IMAGE_ELEM( dst, float, y, x )=(UABS(tmp1)/tmp2);
		}
}


void YARPHMax::_maxFilter(IplImage* src, IplImage* dst, const int dim)
{
	//Note: iplMaxFilter doesn't work on float images
	
	const int xmax=src->width;
	const int ymax=src->height;

	for (int y=0+dim; y<ymax-dim; y++) {
		for (int x=0+dim; x<xmax-dim; x++) {
			float mx=0;
			for (int yy=y-dim; yy<=y+dim; yy++) {
				for (int xx=x-dim; xx<=x+dim; xx++) {
					//mx=__max(IPL_IMAGE_ELEM( src, float, y+yy, x+xx ),mx);
					mx=__max(IPL_IMAGE_ELEM( src, float, yy, xx ),mx);
				}
			}
			IPL_IMAGE_ELEM( dst, float, y, x )=mx;
		}
	}
	
}


void YARPHMax::_maxFilterAndSubsample(IplImage* src, IplImage* dst, const int dim, const int fact)
{
	//Note: iplMaxFilter doesn't work on float images
	
	const int xmax=src->width;
	const int ymax=src->height;

	for (int y=0+dim; y<ymax-dim; y+=fact) {
		for (int x=0+dim; x<xmax-dim; x+=fact) {
			float mx=0;
			for (int yy=y-dim; yy<=y+dim; yy++) {
				for (int xx=x-dim; xx<=x+dim; xx++) {
					//mx=__max(IPL_IMAGE_ELEM( src, float, y+yy, x+xx ),mx);
					mx=__max(IPL_IMAGE_ELEM( src, float, yy, xx ),mx);
				}
			}
			IPL_IMAGE_ELEM( dst, float, y/fact, x/fact )=mx;
		}
	}
	
}


void YARPHMax::_subsample(IplImage* src, IplImage* dst, int fact)
{
	const int xmax=src->width;
	const int ymax=src->height;

	for (int x=0; x<xmax; x+=fact)
		for (int y=0; y<ymax; y+=fact)
			IPL_IMAGE_ELEM( dst, float, y/fact, x/fact )=IPL_IMAGE_ELEM( src, float, y, x );
}


IplConvKernelFP** YARPHMax::_randomPatch(IplImage** src, int dim, int idx)
{
	IplConvKernelFP** tmp=new IplConvKernelFP*[_numFil];

	float *patch=new float [(2*dim+1)*(2*dim+1)];
		
	const int xmax=src[0]->width;
	const int ymax=src[0]->height;

	const int x=_ran1N(dim,xmax-dim-1);
	const int y=_ran1N(dim,xmax-dim-1);

	_x[idx]=x;
	_y[idx]=y;

	for (int n=0; n<_numFil; n++) {
		int idx=0;
		for (int yy=-dim; yy<=dim; yy++)
			for (int xx=-dim; xx<=dim; xx++) {
				patch[idx]=IPL_IMAGE_ELEM( src[n], float, y+yy, x+xx );
				idx++;
			}
		//tmp[n]=iplCreateConvKernelFP(2*dim+1, 2*dim+1, dim, dim, patch);
		tmp[n]=_createConvKernelFP(2*dim+1, 2*dim+1, dim, dim, patch);
	}
	
	delete [] patch;

	return tmp;
}


float YARPHMax::_findMin(IplImage* src)
{
	const int xmax=src->width;
	const int ymax=src->height;

	float mn=FLT_MAX;
	
	for (int x=0; x<xmax; x++)
		for (int y=0; y<ymax; y++)
			mn=__min(IPL_IMAGE_ELEM( src, float, y, x ),mn);

	return mn;
}


float YARPHMax::_findMax(IplImage* src)
{
	const int xmax=src->width;
	const int ymax=src->height;

	float mx=-FLT_MAX;
	int xmx=0;
	int ymx=0;
	
	for (int x=0; x<xmax; x++)
		for (int y=0; y<ymax; y++) {
			//mx=__max(src(x,y),mn);
			float tmp=IPL_IMAGE_ELEM( src, float, y, x );
			if (mx<tmp) {
				mx=tmp;
				xmx=x;
				ymx=y;
			}
		}

	return mx;
}


IplConvKernelFP* YARPHMax::_createConvKernelFP(int rows, int cols, int anchorX, int anchorY, float *values)
{
	IplConvKernelFP* tmp = iplCreateConvKernelFP(rows, cols, anchorX, anchorY, values);
	for (int i=0; i<rows*cols; i++)
		tmp->values[i]=values[i];

	return tmp;
}

int YARPHMax::_ran1N(int min, int max) {
	return int ((double (rand()) / double (RAND_MAX+1)) * (max - min + 1) + min);
}

IplConvKernelFP* YARPHMax::_createGaborFilter(double rot, int size, double div)
{
	const double PI=3.14159265358979;
	
	const double lambda = size*2./div;
	const double sigma = lambda*0.8;
	const double sigmaq=sigma*sigma;
	const double G = 0.3;
	const int center=(size+1)/2;
	const int halfSize=(size-1)/2;
	const double theta=rot*PI/180.;

	float sum=0;
	float sumq=0;

	float *values=new float [size*size];
	
	for (int i=-halfSize; i<=halfSize; i++)
		for (int j=-halfSize; j<=halfSize; j++) {
			float E;
			double x,y;
			if ( sqrt(i*i+j*j)>size/2. )
				E = 0;
			else {
				x = i*cos(theta) - j*sin(theta);
				y = i*sin(theta) + j*cos(theta);
				E = (float)(exp(-(x*x+G*G*y*y)/(2.*sigmaq))*cos(2.*PI*x/lambda));
			}
			values[size*(j+halfSize)+i+halfSize]=E;
			sum+=E;
			sumq+=(E*E);
		}

	for (i=0; i<size*size; i++)
		values[i]=(values[i]-sum/(size*size))/(float)sqrt(sumq);

	IplConvKernelFP* tmp = _createConvKernelFP(size, size, center, center, values);

	delete [] values;

	return tmp;
}


void YARPHMax::_saveImgData(char* filename, IplImage* src)
{
	// Save the image like a matlab matrix, for easy loading&processing
	
	const int xmax=src->width;
	const int ymax=src->height;

	FILE *fp = fopen(filename, "wb");
	
	fprintf(fp, "a=[\n");
	
	for (int y=0; y<ymax; y++) {
		for (int x=0; x<xmax; x++) {
			fprintf(fp, "%f ", IPL_IMAGE_ELEM( src, float, y, x ));
		}
		fprintf(fp, "\n");
	}

	fprintf(fp, "];\n");

	fclose(fp);
}
