/*
 *  optical flow library. computes dense optical flow on a sequence of images
 *
 *  Copyright (C) 2007 Fabio Berton, LIRA-Lab
 *  RobotCub Consortium, European Commission FP6 Project IST-004370
 *  email:   fberton@dist.unige.it
 *  website: www.robotcub.org
 *
 *  Permission is granted to copy, distribute, and/or modify this program 
 *  under the terms of the GNU General Public License, version 2 or any later
 *  version published by the Free Software Foundation. A copy of the license can be 
 *  found at http://www.robotcub.org/icub/license/gpl.txt
 *
 *  This program is distributed in the hope that it will be useful, but WITHOUT ANY
 *  WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A 
 *  PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 */

#ifndef optFlow_h 
#define optFlow_h

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

#define GRAY 1
#define COLOR 3

#define CART 0
#define LP 1
#define HYB 2

#if !defined(__QNX__) && !defined(__LINUX__)
#include <windows.h>
#endif	/// __WIN32__
#include "RC_DIST_FB_logpolar_mapper.h"

class oFlow
{
	
public:
	oFlow();
	~oFlow();

	int procmode;
	int inputmode;
	float prsig,posig;
	float prsigT;
	bool XYTproc;

	int Wid,Hei;
	int nEcc,nAng;
	int sX,sY;

	int ImgSize;
	int hybISize;
	int fov;
	int cartfov;
	float lambda;
	float r0;
	float firstRing;
	float scaleFactor;

	int border;
	int lplimit;

	int XYkSz;
	int TkSz;
	double XYker[7];
	double Tker[6];
	int sizeGxy;
	int sizeGt;

//	unsigned char *imageSet;
//	float *imageSet;
	float **FimageSet;
	float **GimageSet;
	float * tempCart;
//	float **FovimageSet;

	cart2LpPixel * c2ltable;
	lp2CartPixel * l2ctable;


//Acquisition
	void getFileName (char * partFileName, char * fileName, int frameNumber);
	float ** convertFrame(int inputType,int outputType, int origColor,int currSlot);
	unsigned char *loadFrame(char *File_Name, int type);
//	float ** acquireImageSet(char *fileName,int fFrame);
//	float ** loadImageSet(int *XSize,int *YSize,char *File_Name, int fFrame);
	unsigned char  * openBitmap(int *XSize,int *YSize,int *planes,char *fileName);
	void saveFloat  (float *fImage,int XSize,int YSize,int planes,char *fileName,float mult,float bias);
	void saveBitmap  (unsigned char *image, int XSize,int YSize,int planes,char *fileName);
	void procC2LMap(int planes);
	void procL2CMap(int planes);
	void getLpImg(float *lpImg, unsigned char *cartImg,bool color);
	void getLpImg(float *lpImg, float *cartImg,bool color);
	void getCartImg(float *cartImg, unsigned char *lpImg,bool color);
	void getFovea(float *cartImg, unsigned char *lpImg,bool color);
	void getLpFovea(float *lpImg, float *cartImg);
	unsigned char * currentFrame;
	int planes;

//Smoothing
	void preSmooth(int currSlot);
	void postSmooth();
	void intPostSmooth();
	void setGaussian(float sigma, float *gaussian);
	void initSmooth(float **LUTV, float **LUTH, int **VPosLo, int **HPosLo,int **VPosHi, int **HPosHi, float sig);
	void getLPCoord(float *rho, float *theta, float x, float y);
	void getCCoord(float rho, float theta, float *x, float *y);
	void imsmoothLP(float * inImg, float * outImg,float **LUTV, float **LUTH, int *VPosLo, int *HPosLo,int *VPosHi, int *HPosHi);
	void intImsmoothLP(float * inImg, float * outImg,float **LUTV, float **LUTH, int *VPosLo, int *HPosLo,int *VPosHi, int *HPosHi);
	void imsmoothCart(float * inImg, float * outImg, float sig, float *gauss);
	void intImsmoothCart(float * inImg, float * outImg, float sig, float *gauss);
	float * preGaussian;
	float * preGaussianT;
	float *postGaussian;
	float *radius;
	float *recFieldRadSize;
	float *recFieldTanSize;

	int *rhoLoPr, *rhoHiPr, *thetaLoPr,*thetaHiPr;
	int *rhoLoPo, *rhoHiPo, *thetaLoPo,*thetaHiPo;

	float **preGLUTHor;
	float **preGLUTVert;
	float **postGLUTHor;
	float **postGLUTVert;
	float *xpdx;
	float *ypdy;
	bool *procMask;
	
	
//Derivatives
	void buildDerKernel();
	void getDx(float *inImg, float **outImg);
	void getDy(float *inImg, float **outImg);
	void getDt(float **inImg, float **outImg, int currSlot);
	float *fx,*fy,*ft;
	float *derKernel[3];

	float *fxx,*fxy,*fxt;
	float *fyy,*fyt,*ftt;
	float * bufImgL;
	float * bufImgC;

	float * bufRowL;
	float * bufRowC;
	float * bufColL;
	float * bufColC;
	float * colMultL;
	
	void crossProducts();

	void computeFlow();

	float *u,*v;
	float *uDen, *vDen;
	void getModule(float *uflow, float *vflow);
	float * module;
	void getOrientation(float *uflow, float *vflow);
	float * orientation;
	long getTime();
	float *X0eq,*Y0eq;
	float *deX,*deY;


	void HSV2RGB(unsigned char *outImg, float *Hue, float *Sat, float *Bri, float maxMod,unsigned char *outImgRem);
	void drawFlow(float * module, float * Cmodule, int frame, float *CdeX, float *CdeY);


	void warpFlow();

};





#endif

