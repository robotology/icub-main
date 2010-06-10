/*
*  logpolar mapper library. subsamples rectangular images into logpolar.
*
*  Copyright (C) 2005 
*  RobotCub Consortium, European Commission FP6 Project IST-004370
*  Author Fabio Berton.
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
*  $Id: optFlow.cpp,v 1.9 2008/07/24 16:02:07 nat Exp $
*/

#define _CRT_SECURE_NO_DEPRECATE

#include <iCub/vis/optFlow.h>
//#include "opticflow.h"
#include <stdio.h>
#include <math.h>
#include <memory.h>

#include <stdlib.h>
#include <string.h>
#include <time.h>
//#include "../../logPolar/include/iCub/RC_DIST_FB_logpolar_mapper.h"


oFlow::oFlow()
{
	
	bufColC=NULL;
	bufColL=NULL;
	bufImgC = NULL;
	bufImgL = NULL;
	bufRowC=NULL;
	bufRowL=NULL;
	colMultL=NULL;
	deX = NULL;
	deY = NULL;
	tempCart = NULL;
	xpdx = NULL;
	ypdy = NULL;
	procMask = NULL;
	X0eq = NULL;
	Y0eq = NULL;
	currentFrame = NULL;
	FimageSet = NULL;
	GimageSet = NULL;
	c2ltable = NULL;
	l2ctable = NULL;
	orientation = NULL;
	module = NULL;
	preGaussian = NULL;
	preGaussianT = NULL;
	postGaussian = NULL;
	radius = NULL;
	recFieldRadSize = NULL;
	recFieldTanSize = NULL;
	rhoLoPr = NULL;
	rhoHiPr = NULL;
	thetaLoPr = NULL;
	thetaHiPr = NULL;
	rhoLoPo = NULL;
	rhoHiPo = NULL;
	thetaLoPo = NULL;
	thetaHiPo = NULL;
	preGLUTVert = NULL;
	preGLUTHor = NULL;
	postGLUTVert = NULL;
	postGLUTHor = NULL;

	fx = NULL;
	fy = NULL;
	ft = NULL;
	fxx = NULL;
	fxy = NULL;
	fxt = NULL;
	fyy = NULL;
	fyt = NULL;
	ftt = NULL;

	u=NULL;
	v=NULL;

	uDen = NULL;
	vDen = NULL;

	double tmpXYker [] = {-1.0f/60.0f,3.0f/20.0f,-3.0f/4.0f,0.0f,3.0f/4.0f,-3.0f/20.0f,1.0f/60.0f};
	double tmpTker  [] = {-9.0f/1920.0f,125.0f/1920.0f,-2250.0f/1920.0f,2250.0f/1920.0f,-125.0f/1920.0f,9.0f/1920.0f};

	memcpy(XYker,tmpXYker,sizeof(tmpXYker));
	memcpy(Tker,tmpTker,sizeof(tmpTker));

	XYkSz = sizeof(XYker)/sizeof(double);
	TkSz  = sizeof(Tker)/sizeof(double);
	
//	if (XYTproc)
//		if (1)
//	{
		sizeGxy = 2*TkSz;
		sizeGt  = TkSz;
//	}
//	else
//	{
//		sizeGxy = TkSz;
//		sizeGt  = 0;
//	}


}

oFlow::~oFlow()
{
	int i;
	
	if (FimageSet!=NULL)
	{
		for (i=0; i<sizeGxy; i++)
			if (FimageSet[i]!=NULL)
				delete [] FimageSet[i];
			delete [] FimageSet;
	}
	
	
	if (currentFrame!=NULL)
		delete [] currentFrame;
}
/*
float ** oFlow::acquireImageSet(char *fileName,int fFrame)
{
	int *HSize, *VSize;

	if (inputmode==CART)
	{
		HSize = &sX;
		VSize = &sY;
	}
	else
	{
		HSize = &nAng;
		VSize = &nEcc;
	}

	if (strncmp(fileName, "CAM", 3)==0)
	{
		return NULL;
	}
	else 
		FimageSet = loadImageSet(HSize,VSize,fileName,fFrame);

	return FimageSet;
}
*/
unsigned char *oFlow::loadFrame(char *File_Name, int type)
{
	if (currentFrame != NULL)
		delete [] currentFrame;

	if (type == LP)
		currentFrame = openBitmap(&nAng,&nEcc,&planes,File_Name);
	else
		currentFrame = openBitmap(&sX,&sY,&planes,File_Name);

	return currentFrame;

}

float ** oFlow::convertFrame(int inputType,int outputType, int inputColor, int currSlot)
{
	if (inputType == HYB)
		inputType = LP;

	inputmode = inputType;
	procmode = outputType;

	bool firstpass = false;

	int i,j,k;
	int ISize;
	int noF;
	
	if (XYTproc)
	{
		sizeGxy = 2*TkSz;
		sizeGt  = TkSz;
		noF = 2*((int)(3*prsigT+1))+sizeGt;
	}
	else
	{
		sizeGxy = TkSz;
		sizeGt  = 0;
		noF = TkSz;
	}


	if (FimageSet==NULL)
	{
		FimageSet = new float *[noF];
		for (i=0; i<noF; i++)
			FimageSet[i]=NULL;
		firstpass = true;
	}

	if (currentFrame==NULL)
		return NULL;
	
	if (inputType == LP)
		ISize = nEcc*nAng;
	else
		ISize = sX*sY;

	if (outputType == CART)
	{
		Wid = sX;
		Hei = sY;
		ImgSize = Wid*Hei;
		hybISize = ImgSize;
	}
	else
	{
		Wid = nAng;
		Hei = nEcc;
		lambda = (float)((1.0+sin(PI/nAng))/(1.0-sin(PI/nAng)));
		fov = (int)(lambda/(lambda-1));
		firstRing = 0.5f * ((lambda/(lambda-1)) - fov);
		r0 = 1.0f/((float)pow(lambda,fov)*(lambda-1));
		scaleFactor = (float)RCcomputeScaleFactor (nEcc, nAng, sX,sY, 1.0);
		ImgSize = Wid*Hei;

		border = (int)(3*prsig+1)+(int)(3*posig+1)+XYkSz;

		if(firstpass==true)
		{
			if(procmode==HYB)
			{
				int lps;
				int cts;
				float xxx,yyy;
				for (i=(fov+nEcc)/2; i<nEcc; i++)
				{
					lps = i*nAng;
					
					getCCoord((float)i,0.0,&xxx,&yyy);
					
					cts = 2*border+2*((int)(xxx+0.5));
					cts = cts*cts;
					
					if(cts>lps)
					{
						lplimit = i-1;
						break;
					}
				}
				
				getCCoord((float)(i-0.5),0.0,&xxx,&yyy);
				
				cartfov = (int)(border+xxx+0.5);
				
				//			cartfov = (int)((fov+80)*scaleFactor+0.5);
				//			cartfov += (int)(3*prsig+1)+(int)(3*posig+1)+XYkSz;
				//			cartfov = 92;
				hybISize = ImgSize+cartfov*cartfov*4;
			}
			else
			{
				hybISize = ImgSize;
			}
		}

	}


	if ((procmode == inputmode) || ((inputmode==LP)&&(procmode==HYB)))
	{
		if (FimageSet[currSlot]==NULL)
			FimageSet[currSlot] = new float[hybISize];

		if (inputColor==3) 
			for (i=0; i<ISize; i++)
				FimageSet[currSlot][i] = 0.5f+(currentFrame[3*i]+currentFrame[3*i+1]+currentFrame[3*i+2])/3.0f;
		else
			for (i=0; i<ISize; i++)
				FimageSet[currSlot][i] = (float)(currentFrame[i]);

		if (procmode==HYB)
		{
			if (firstpass)
			{
				if(l2ctable==NULL)
					l2ctable = new lp2CartPixel[sX*sY];
				
				if(c2ltable==NULL)
					c2ltable = new cart2LpPixel[nEcc*nAng];

				RCbuildL2CMap(nEcc,nAng,sX,sY,1.0,scaleFactor,0,0,ELLIPTICAL,"D:\\Test Images\\tables\\");
				RCallocateL2CTable(l2ctable,sX,sY,"D:\\Test Images\\tables\\");
				procL2CMap(inputColor);

				RCbuildC2LMap(nEcc,nAng,sX,sY,1.0,scaleFactor,ELLIPTICAL,"D:\\Test Images\\tables\\");
				RCallocateC2LTable(c2ltable,nEcc,nAng,false,"D:\\Test Images\\tables\\");
				procC2LMap(inputColor);
			}
		
			getFovea(FimageSet[currSlot]+ImgSize,currentFrame,false);
		}
	}
	else //remap  
	{
		if (procmode!=CART) // (Cart to LP or Cart to Hyb)
		{
			if (firstpass)
			{
				if(c2ltable==NULL)
					c2ltable = new cart2LpPixel[ImgSize];

				RCbuildC2LMap(nEcc,nAng,sX,sY,1.0,scaleFactor,ELLIPTICAL,"D:\\Test Images\\tables\\");
				RCallocateC2LTable(c2ltable,nEcc,nAng,false,"D:\\Test Images\\tables\\");
				procC2LMap(inputColor);

				if(procmode==HYB)
				{
					if(l2ctable==NULL)
						l2ctable = new lp2CartPixel[sX*sY];

					RCbuildL2CMap(nEcc,nAng,sX,sY,1.0,scaleFactor,0,0,ELLIPTICAL,"D:\\Test Images\\tables\\");
					RCallocateL2CTable(l2ctable,sX,sY,"D:\\Test Images\\tables\\");
					procL2CMap(inputColor);
				}
			}
			
			if (FimageSet[currSlot]==NULL)
				FimageSet[currSlot] = new float[hybISize];

			getLpImg(FimageSet[currSlot],currentFrame,false);
//			getLpImg(FimageSet[currSlot],tmpfimage,false);

			if (procmode == HYB)
			{
				if (inputColor == 1)
				{
					for (j=-cartfov; j<cartfov; j++)
						for (i=-cartfov; i<cartfov; i++)
							FimageSet[currSlot][(j+cartfov)*2*cartfov+i+cartfov+ImgSize] = (float)(currentFrame[(j+sY/2)*sX+(i+sX/2)]);
//							FimageSet[currSlot][(j+cartfov)*2*cartfov+i+cartfov+ImgSize] = (float)(tmpfimage[(j+sY/2)*sX+(i+sX/2)]);
				}
				else
				{
					float a;
					for (j=-cartfov; j<cartfov; j++)
						for (i=-cartfov; i<cartfov; i++)
						{
							a=0;
							for (k=0; k<3; k++)
								a+=(currentFrame[3*((j+sY/2)*sX+(i+sX/2))+k]);
							a /=3.0f;
							FimageSet[currSlot][(j+cartfov)*2*cartfov+i+cartfov+ImgSize] = a;
						}
				}
			}
		}
		else //LP to Cart 
		{

			if (firstpass)
			{
//				lambda = (float)((1.0+sin(PI/nAng))/(1.0-sin(PI/nAng)));
//				fov = (int)(lambda/(lambda-1));
//				firstRing = 0.5f * ((lambda/(lambda-1)) - fov);
//				r0 = 1.0f/((float)pow(lambda,fov)*(lambda-1));
				
				if(l2ctable==NULL)
					l2ctable = new lp2CartPixel[ImgSize];
				
//				scaleFactor = (float)RCcomputeScaleFactor (nEcc,nAng,sX,sY,1.0);
				RCbuildL2CMap(nEcc,nAng,sX,sY,1.0,scaleFactor,0,0,ELLIPTICAL,"D:\\Test Images\\tables\\");
				RCallocateL2CTable(l2ctable,sX,sY,"D:\\Test Images\\tables\\");
				procL2CMap(inputColor);
			}
			
			if (FimageSet[currSlot]==NULL)
				FimageSet[currSlot] = new float[hybISize];

			if(procmode==HYB)
			{
				if (tempCart==NULL)
					tempCart = new float[cartfov*cartfov*4];
			}

			
			if (procmode == HYB)
			{
				getCartImg(tempCart,currentFrame,false);
				
				for (j=-cartfov; j<cartfov; j++)
					for (i=-cartfov; i<cartfov; i++)
						FimageSet[currSlot][(j+cartfov)*2*cartfov+i+cartfov+ImgSize] = tempCart[(j+sY/2)*sX+(i+sX/2)];
			}			
		}
	}

//	if (((procmode==LP)||(procmode==HYB))&&(firstpass))
//	{
//		lambda = (float)((1.0+sin(PI/nAng))/(1.0-sin(PI/nAng)));
//		fov = (int)(lambda/(lambda-1));
//		firstRing = 0.5f * ((lambda/(lambda-1)) - fov);
//		r0 = 1.0f/((float)pow(lambda,fov)*(lambda-1));
//		
//		scaleFactor = (float)RCcomputeScaleFactor (nEcc,nAng, sX,sY, 1.0);
//	}
//	delete [] tmpfimage;
	return FimageSet;
}
/*
float ** oFlow::loadImageSet(int *XSize,int *YSize,char *File_Name, int fFrame)
{
	if (inputmode == HYB)
		inputmode = LP;
	
	int noF = 2*((int)(3*prsig+1))+sizeGt;
	if (!XYTproc)
		noF = TkSz;
	
	char fullFN [256];
	int frame;
	int planes;
	int i,j,k;
	
	if (FimageSet==NULL)
	{
		FimageSet = new float *[noF];

		for (frame=0; frame<noF; frame++)
			FimageSet[frame]=NULL;
	}

//	if (procmode==HYB)
//	{
//		if(FovimageSet==NULL) 
//		{
//			FovimageSet = new float * [noF];
//			
//			for (frame=0; frame<noF; frame++)
//				FovimageSet[frame]=NULL;
//		}
//	}

	for (frame=0; frame<noF; frame++)
	{
		sprintf(fullFN,"%s%03d%s",File_Name,fFrame+frame,".bmp");
		currentFrame = openBitmap(XSize,YSize,&planes,fullFN);


//		*XSize = 640;
//		*YSize = 480;
//		planes = 3;
//		
//		imageSet = new float[640*480*3];

//		for (j=0; j<*YSize; j++)
//			for (i=0; i<*XSize; i++)
//				for (k=0; k<3; k++)
//						imageSet[3*(j**XSize+i)+k] = ((float)(i)/4.0f)+frame;

		if (currentFrame==NULL)
			return NULL;
		
		int ISize = *XSize**YSize;

		
		if ((procmode == inputmode) || ((inputmode==LP)&&(procmode==HYB)))
		{
			Wid = *XSize;
			Hei = *YSize;
			FimageSet[frame] = new float[ISize];
			if (planes==3) 
			{
				for (i=0; i<ISize; i++)
					FimageSet[frame][i] = (float)(0.5f+(currentFrame[3*i]+currentFrame[3*i+1]+currentFrame[3*i+2])/3.0f);
			}
			else
			{
				for (i=0; i<ISize; i++)
					FimageSet[frame][i] = (float)(currentFrame[i]);
			}
#ifdef HYBPR
			if (procmode==HYB)
			{
				if (frame==0)
				{
					ImgSize = Wid*Hei;
					
					lambda = (1.0+sin(PI/nAng))/(1.0-sin(PI/nAng));
					fov = (int)(lambda/(lambda-1));
					firstRing = 0.5 * ((lambda/(lambda-1)) - fov);
					r0 = 1.0/(pow(lambda,fov)*(lambda-1));
					
					scaleFactor = RCcomputeScaleFactor (*YSize, *XSize, sX,sY, 1.0);
					if(l2ctable==NULL)
						l2ctable = new lp2CartPixel[sX*sY];
					
					RCbuildL2CMap(*YSize,*XSize,sX,sY,1.0,scaleFactor,0,0,ELLIPTICAL,"D:\\Test Images\\tables\\");
					RCallocateL2CTable(l2ctable,sX,sY,"D:\\Test Images\\tables\\");
					procL2CMap(planes);
				}
				cartfov = (int)(fov*scaleFactor+0.5);
				FovimageSet[frame] = new float[cartfov*cartfov*4];

				getFovea(FovimageSet[frame],imageSet,false);
				
			}
#endif
		}
		else //remap
		{
			if (procmode!=CART) //Cart to LP 
			{
				
				if (frame==0)
				{
					Wid = nAng;
					Hei = nEcc;
										
					lambda = (float)((1.0+sin(PI/nAng))/(1.0-sin(PI/nAng)));
					fov = (int)(lambda/(lambda-1));
					firstRing = 0.5f * ((lambda/(lambda-1)) - fov);
					r0 = 1.0f/((float)pow(lambda,fov)*(lambda-1));
					
					scaleFactor = (float)RCcomputeScaleFactor (nEcc, nAng, *XSize,*YSize, 1.0);
					ImgSize = nEcc*nAng;
					
					if(c2ltable==NULL)
						c2ltable = new cart2LpPixel[ImgSize];
					RCbuildC2LMap(nEcc,nAng,*XSize,*YSize,1.0,scaleFactor,ELLIPTICAL,"D:\\Test Images\\tables\\");
					RCallocateC2LTable(c2ltable,nEcc,nAng,false,"D:\\Test Images\\tables\\");
					procC2LMap(planes);
				}

				FimageSet[frame] = new float[ImgSize];
				getLpImg(FimageSet[frame],currentFrame,false);
				if (procmode == HYB)
				{
					cartfov = (int)(fov*scaleFactor+0.5);
					FovimageSet[frame] = new float[cartfov*cartfov*4];
					
					if (planes == 1)
					{
						for (j=-cartfov; j<cartfov; j++)
							for (i=-cartfov; i<cartfov; i++)
								FovimageSet[frame][(j+cartfov)*2*cartfov+i+cartfov] = (float)(currentFrame[(j+*YSize/2)**XSize+(i+*XSize/2)]);
					}
					else
					{
						float a;
						for (j=-cartfov; j<cartfov; j++)
							for (i=-cartfov; i<cartfov; i++)
							{
								a=0;
								for (k=0; k<3; k++)
									a+=(float)(currentFrame[3*((j+*YSize/2)**XSize+(i+*XSize/2))+k]);
								a /=3.0f;
								FovimageSet[frame][(j+cartfov)*2*cartfov+i+cartfov] = a;
							}
					}
				}
			}
			else //LP to Cart
			{
				scaleFactor = (float)RCcomputeScaleFactor (*YSize,*XSize,sX,sY,1.0);
				if (frame==0)
				{
					Wid = sX;
					Hei = sY;

					ImgSize = sX*sY;
					
					lambda = (float)((1.0+sin(PI/nAng))/(1.0-sin(PI/nAng)));
					fov = (int)(lambda/(lambda-1));
					firstRing = 0.5f * ((lambda/(lambda-1)) - fov);
					r0 = 1.0f/((float)pow(lambda,fov)*(lambda-1));

					if(l2ctable==NULL)
						l2ctable = new lp2CartPixel[ImgSize];

					RCbuildL2CMap(*YSize,*XSize,sX,sY,1.0,scaleFactor,0,0,ELLIPTICAL,"D:\\Test Images\\tables\\");
					RCallocateL2CTable(l2ctable,sX,sY,"D:\\Test Images\\tables\\");
					procL2CMap(planes);
				}

				FimageSet[frame] = new float[ImgSize];
//				getCartImg(FimageSet[frame],imageSet,false);

				if (procmode == HYB)
				{
					cartfov = (int)(fov*scaleFactor+0.5);
					FovimageSet[frame] = new float[cartfov*cartfov*4];
					
					for (j=-cartfov; j<cartfov; j++)
						for (i=-cartfov; i<cartfov; i++)
							FovimageSet[frame][(j+cartfov)*2*cartfov+i+cartfov] = FimageSet[frame][(j+*YSize/2)**XSize+(i+*XSize/2)];
				}			
			}
		}
		
	}

	if ((procmode==LP)||(procmode==HYB))
	{
		lambda = (float)((1.0+sin(PI/nAng))/(1.0-sin(PI/nAng)));
		fov = (int)(lambda/(lambda-1));
		firstRing = 0.5f * ((lambda/(lambda-1)) - fov);
		r0 = 1.0f/((float)pow(lambda,fov)*(lambda-1));
		
		scaleFactor = (float)RCcomputeScaleFactor (nEcc,nAng, sX,sY, 1.0);
	}
	ImgSize = Wid*Hei;
	
	return FimageSet;
}

*/
void oFlow::procC2LMap(int planes)
{
	int i,j;
	
	if (planes == 1)
	{
		for (j=0; j<ImgSize; j++)
		{
			for(i=0; i<c2ltable[j].divisor; i++)
				c2ltable[j].position[i] = c2ltable[j].position[i]/3;
		}
	}
}

void oFlow::procL2CMap(int planes)
{
	int i,j;
	
	if (planes == 1)
	{
		for (j=0; j<sX*sY; j++)
		{
			for(i=0; i<l2ctable[j].iweight; i++)
				l2ctable[j].position[i] = l2ctable[j].position[i]/3;
		}
	}
}

void oFlow::getLpImg(float *lpImg, unsigned char *cartImg, bool color)
//void oFlow::getLpImg(float *lpImg, float *cartImg, bool color)
{
    int i, j, k, z;
//    int r[3];
    float r[3];
    int pos;
    int t = 0;
	
    z = 1;

	memset(lpImg,0,ImgSize*sizeof(float));
	
    if (color)
        z = 3;
	
    for (j = 0; j < ImgSize; j++)
    {
		memset(r,0,z*sizeof(float));
		
        t = 0;
		
        for (i = 0; i < c2ltable[j].divisor; i++)
        {
            pos = c2ltable[j].position[i];

            for (k = 0; k < z; k++)
                r[k] += cartImg[pos + k] * c2ltable[j].iweight[i];
            
			t += c2ltable[j].iweight[i];
        }

        for (k = 0; k < z; k++)
        {
            if (t == 0)
                printf ("t is zero!, position %d\n",j);
            lpImg[z * j + k] = (float) (r[k] / (float)(z*t));

        }
    }
}


void oFlow::getLpImg(float *lpImg, float *cartImg, bool color)
//void oFlow::getLpImg(float *lpImg, float *cartImg, bool color)
{
    int i, j, k, z;
	//    int r[3];
    float r[3];
    int pos;
    int t = 0;
	
    z = 1;
	
	memset(lpImg,0,ImgSize*sizeof(float));
	
    if (color)
        z = 3;
	
    for (j = 0; j < ImgSize; j++)
    {
		memset(r,0,z*sizeof(float));
		
        t = 0;
		
        for (i = 0; i < c2ltable[j].divisor; i++)
        {
            pos = c2ltable[j].position[i];
			
            for (k = 0; k < z; k++)
                r[k] += cartImg[pos + k] * c2ltable[j].iweight[i];
            
			t += c2ltable[j].iweight[i];
        }
		
        for (k = 0; k < z; k++)
        {
            if (t == 0)
                printf ("t is zero!, position %d\n",j);
            lpImg[z * j + k] = (float) (r[k] / (float)(z*t));
			
        }
    }
}

void oFlow::getLpFovea(float *lpImg, float *cartImg)
{
	int i, j;
    float r;
    int pos,posX,posY;
    int t = 0;

//	int min,max;
//
//	min = sX;
//	max = 0;
//
//	for (j = 0; j < fov*nAng; j++)
//        for (i = 0; i < c2ltable[j].divisor; i++)
//		{
//			pos = c2ltable[j].position[i]/planes;
//			posX = (pos%sX)-((sX-2*cartfov)/2);
//			
//			if (posX<min)
//				min = posX;
//			if (posX>max)
//				max=posX;
//		}
//
//	min = sY;
//	max = 0;
//	for (j = 0; j < fov*nAng; j++)
//        for (i = 0; i < c2ltable[j].divisor; i++)
//		{
//			pos = c2ltable[j].position[i]/planes;
//			posY = (pos/sX)-((sY-2*cartfov)/2);
//			
//			if (posY<min)
//				min = posY;
//			if (posY>max)
//				max=posY;
//		}
		
    for (j = 0; j < lplimit*nAng; j++)
    {
		r = 0;
        t = 0;
		
		for (i = 0; i < c2ltable[j].divisor; i++)
        {
			pos = (c2ltable[j].position[i])/planes;
			posX = (pos%sX)-((sX-2*cartfov)/2);
			posY = (pos/sX)-((sY-2*cartfov)/2);
			pos = posY*cartfov*2+posX;

			r += cartImg[pos] * c2ltable[j].iweight[i];
			t += c2ltable[j].iweight[i];
        }

        if (t == 0)
            printf ("t is zero!, position %d\n",j);
        lpImg[j] = (float) (r / (float)(t));
   }
}

void oFlow::getCartImg(float *cartImg, unsigned char *lpImg,bool color)
{
    int i, j, k;
	
    float tempPixel[3];
	
	int z=1;
	
	if(color)
		z=3;
	
    for (j = 0; j < ImgSize; j++)
    {
        for (k = 0; k < z; k++)
            tempPixel[k] = 0;
		
        if (l2ctable[j].iweight != 0)
        {
            for (i = 0; i < l2ctable[j].iweight; i++)
                for (k = 0; k < 3; k++)
                {
                    tempPixel[k] += (float)lpImg[l2ctable[j].position[i] + k];
                }
				for (k = 0; k < 3; k++)
					cartImg[z * j + k] = tempPixel[k] / l2ctable[j].iweight;
        }
    }
}


void oFlow::getFovea(float *cartImg, unsigned char *lpImg,bool color)
{
    int i, j, k;
	
    float tempPixel[3];
	
	int z=1;
	
	if(color)
		z=3;

	int l,m;
	
    for (l = -cartfov; l < cartfov; l++)
		for (m = -cartfov; m < cartfov; m++)
		{
			j=(l+sY/2)*sX+m+sX/2;
			for (k = 0; k < z; k++)
				tempPixel[k] = 0;
			
			if (l2ctable[j].iweight != 0)
			{
				for (i = 0; i < l2ctable[j].iweight; i++)
					for (k = 0; k < z; k++)
					{
						tempPixel[k] += (float)lpImg[l2ctable[j].position[i] + k];
					}
					for (k = 0; k < z; k++)
						cartImg[z * ((l+cartfov)*2*cartfov+m+cartfov) + k] = tempPixel[k] / l2ctable[j].iweight;
			}
		}
}


unsigned char  * oFlow::openBitmap(int *XSize,int *YSize,int *planes,char *fileName)
{
	
#if defined(__QNX__) || defined(__LINUX__)
	printf("Format not supported under Linux\n");
	return NULL;
#endif
	
	unsigned char *image;	
	unsigned char c=0;
	int x,y,z;
	int Offset;
	FILE* fin;
	BITMAPFILEHEADER bmpfh;
	BITMAPINFOHEADER bmpih;
	RGBQUAD palette[256];
	
	if ((fin = fopen(fileName,"rb")) != NULL)
	{
		fread (&bmpfh,sizeof(BITMAPFILEHEADER),1,fin);
		fread (&bmpih,sizeof(BITMAPINFOHEADER),1,fin);
		*XSize = bmpih.biWidth;
		*YSize = bmpih.biHeight;
		*planes = bmpih.biBitCount/8;
		image = (unsigned char *)malloc(*XSize * *YSize * *planes *sizeof(unsigned char));
		Offset = (4 - ((*XSize * *planes) %4))%4;
		
		if (*planes == 1)
			fread (&palette,sizeof(RGBQUAD),256,fin);
		
		for (y=*YSize-1; y>=0; y--)
		{
			for (x=0; x<*XSize; x++)
				for (z=*planes-1; z>=0; z--)
				{
					fread (&c,1,1,fin);
					image[y**planes* *XSize + *planes *x+z] = (unsigned char)(c);
				}
				for (x=0;x<Offset;x++)
					fread(&c,1,1,fin);
		}
		fclose(fin);
	}
	else
		image = NULL;
	
	return image;
}

void oFlow::saveFloat  (float *fImage,int XSize,int YSize,int planes,char *fileName,float mult,float bias)
{
	int i;
	int ISize = XSize*YSize*planes;
	
	unsigned char * outimg = new unsigned char [ISize];
	
	for (i=0; i<ISize; i++)
		outimg[i] = (unsigned char)(mult*(fImage[i]-bias));
	
	saveBitmap(outimg,XSize,YSize,planes,fileName);
	
	delete [] outimg;
}

void oFlow::saveBitmap  (unsigned char *image, int XSize,int YSize,int planes,char *fileName)
{
	FILE* fout;
	int size = XSize * YSize * planes;
	int x,y,z;
	int Offset = (4 - ((XSize * planes) %4))%4;
	BITMAPFILEHEADER bmpfh;
	BITMAPINFOHEADER bmpih;
	
	fout = fopen(fileName,"wb");
	
	bmpfh.bfType = 'MB';
	bmpfh.bfOffBits = 54;
	if (planes==1)
		bmpfh.bfOffBits = 54+1024;
	
	bmpfh.bfSize = size + bmpfh.bfOffBits;
	bmpfh.bfReserved1 = 0;
	bmpfh.bfReserved2 = 0;
	
	bmpih.biSize = 40;
	bmpih.biWidth = XSize;
	bmpih.biHeight = YSize;
	bmpih.biPlanes = 1;
	bmpih.biBitCount = planes *8;
	bmpih.biCompression = 0;
	bmpih.biSizeImage = YSize * (XSize * planes + Offset);
	bmpih.biXPelsPerMeter = 2835;
	bmpih.biYPelsPerMeter = bmpih.biXPelsPerMeter;
	bmpih.biClrUsed = 0;
	bmpih.biClrImportant = 0;
	if (planes==1)
	{
		bmpih.biClrUsed = 256;
		bmpih.biClrImportant = 256;
	}
	
	fwrite(&bmpfh,sizeof(BITMAPFILEHEADER),1,fout);
	fwrite(&bmpih,sizeof(BITMAPINFOHEADER),1,fout);
	
	if (planes ==1)
		for (x=0; x<256;x++)
		{
			y=0;
			fwrite(&x,sizeof(unsigned char),1,fout);
			fwrite(&x,sizeof(unsigned char),1,fout);
			fwrite(&x,sizeof(unsigned char),1,fout);
			fwrite(&y,sizeof(unsigned char),1,fout);
		}
		
		for (y=YSize-1; y>=0; y--)
		{	
			for (x=0; x<XSize; x++)
				for (z=planes-1;z>=0; z--)
					fwrite(image+(planes*(y*XSize+x)+z),sizeof(unsigned char),1,fout);
				for (x=0;x<Offset;x++)
					fwrite(image,sizeof(unsigned char),1,fout);
		}
		
		fclose(fout);
}

void oFlow::preSmooth(int currStep)
{
	int i;
	int klim;

	bool setGXY = false;
	bool setGT  = false;
	klim = 2*((int)(3*prsigT+1))+1;
	int noF = klim+sizeGt-1;
	
	if ((procmode==CART)||(procmode==HYB))
		if (preGaussian==NULL)
		{
			preGaussian = new float [2*((int)(3*prsig+1))+1];
			setGXY = true;
		}

	if (XYTproc)
		if (preGaussianT==NULL)
		{
			preGaussianT = new float [klim];
			setGT = true;
		}
	
	if (setGXY)
		setGaussian(prsig,preGaussian);
	
	if (setGT)
		setGaussian(prsigT,preGaussianT);
		
	if ((procmode == LP)||(procmode==HYB))
	{
		setGXY = false;
		
		if (preGLUTVert==NULL)
		{
			preGLUTVert = new float * [Hei];
			setGXY = true;
		}
		if (preGLUTHor==NULL)
		{
			preGLUTHor = new float * [Hei];
			setGXY = true;
		}
		
		if (setGXY)
			initSmooth(preGLUTVert,preGLUTHor,&rhoLoPr,&thetaLoPr,&rhoHiPr,&thetaHiPr,prsig);
	}
	
	if (GimageSet ==NULL)
	{
		GimageSet = new float* [sizeGxy];
		for (i=0; i<sizeGxy; i++)
			GimageSet[i] =NULL;
	}

	if((XYTproc)&&(currStep>=klim-1))
	{
		int llim;
		int slot;
		float acc;
		int j,k,l;

		llim = (currStep-klim+1)%noF;
		slot = ((currStep-klim+1)%sizeGt)+sizeGt;
		
		if (GimageSet[slot]==NULL)
			GimageSet[slot] = new float [hybISize];
		
		if(prsigT > 0.01) 
		{
			for (i=0; i<hybISize; i++)
				{
					acc = 0.0f;
					for (k=0,l=llim; k<klim; k++,l++)
						acc+=preGaussianT[k]*FimageSet[l%noF][i];
					
					GimageSet[slot][i] = acc; 
				}
		}
		else
			for (j=0; j<hybISize; j++)
				GimageSet[slot][j]=(float)(FimageSet[(currStep-klim/2)%noF][j]);
	}
	
	int currSlot;

	if((XYTproc)&&(currStep>=klim-1))
	{
		currSlot = ((currStep-klim+1)%sizeGt);
		
		if (GimageSet[currSlot]==NULL)
			GimageSet[currSlot] = new float [hybISize];
		
		if ((procmode == LP)||(procmode==HYB))
			imsmoothLP(GimageSet[currSlot+TkSz],GimageSet[currSlot],preGLUTVert,preGLUTHor,rhoLoPr,thetaLoPr,rhoHiPr,thetaHiPr);
		else
			imsmoothCart(GimageSet[currSlot+TkSz],GimageSet[currSlot],prsig,preGaussian);

		if(procmode==HYB)
			imsmoothCart(GimageSet[currSlot+TkSz]+ImgSize,GimageSet[currSlot]+ImgSize,prsig,preGaussian);
	}
	else if (!XYTproc)
	{
		currSlot = (currStep%sizeGxy);

		if (GimageSet[currSlot]==NULL)
			GimageSet[currSlot] = new float [hybISize];

		if(prsig>0.1)
		{
			if ((procmode == LP)||(procmode==HYB))
				imsmoothLP(FimageSet[currSlot],GimageSet[currSlot],preGLUTVert,preGLUTHor,rhoLoPr,thetaLoPr,rhoHiPr,thetaHiPr);
			else
				imsmoothCart(FimageSet[currSlot],GimageSet[currSlot],prsig,preGaussian);
			if(procmode==HYB)
				imsmoothCart(FimageSet[currSlot]+ImgSize,GimageSet[currSlot]+ImgSize,prsig,preGaussian);
		}
		else
			memcpy(GimageSet[currSlot],FimageSet[currSlot],hybISize*sizeof(float));
	}
		
}

void oFlow::imsmoothLP(float * inImg, float * outImg,float **LUTV, float **LUTH, int *VPosLo, int *HPosLo,int *VPosHi, int *HPosHi)
{
	int rho,theta,j,pos;
	int startRho = 0;

	if (bufImgL==NULL)
	{
		if (procmode==LP)
			bufImgL = new float[ImgSize];
		else
			bufImgL = new float[ImgSize];
	}

	memset(bufImgL,0,ImgSize*sizeof(float));

	if(procmode==LP)
		startRho = 0;
	else
		if(postGLUTVert==LUTV)
			startRho = lplimit;

	for(rho=startRho; rho<Hei; rho++)
		for (theta = 0; theta<Wid; theta++)
		{
			for (j=VPosLo[rho]; j<=VPosHi[rho]; j++)
				bufImgL[rho*Wid+theta] += inImg[j*Wid+theta]*LUTV[rho][j-VPosLo[rho]];
		}
		
	memset(outImg,0,ImgSize*sizeof(float));

	for(rho=startRho; rho<Hei; rho++)
		for (theta = 0; theta<Wid; theta++)
		{
			for (j=HPosLo[rho]; j<=HPosHi[rho]; j++)
			{
				pos = ((j+theta+Wid)%Wid);
				outImg[rho*Wid+theta] += bufImgL[rho*Wid+pos]*LUTH[rho][j-HPosLo[rho]];
			}
		}
			
}


void oFlow::intImsmoothLP(float * inImg, float * outImg,float **LUTV, float **LUTH, int *VPosLo, int *HPosLo,int *VPosHi, int *HPosHi)
{
	int rho,theta;
	int startRho = 0;
	int lineSize;
	int up,down;
	float multiplier;
	
	if (bufImgL==NULL)
		bufImgL = new float[ImgSize];

	
//	memset(bufImgL,0,ImgSize*sizeof(float));

	if(procmode==LP)
		startRho = 0;
	else
		if(postGLUTVert==LUTV)
			startRho = lplimit;

	if (bufRowL==NULL)
		bufRowL = new float[nAng+HPosHi[lplimit]-HPosLo[lplimit]+1];

	if (bufColL==NULL)
		bufColL = new float[nEcc];
	
//	memset(bufRowL,0,nAng*sizeof(float));
	if (colMultL==NULL)
	{
		colMultL = new float[nEcc];
		for (rho=0; rho<nEcc; rho++)
			if(VPosLo[rho]==0)
				VPosLo[rho] = 1;

		for (rho=0; rho<nEcc; rho++)
			colMultL[rho] = 1.0f/(VPosHi[rho]-VPosLo[rho]+1);
	}

	//Vert Smooth
	for (theta=0; theta<Wid; theta++)
	{
		//Pass1
		bufColL[0] = inImg[theta];
		for (rho = 1; rho<Hei; rho++)
			bufColL[rho] = inImg[rho*Wid+theta]+bufColL[rho-1];
		
		for (rho = startRho; rho<Hei; rho++)
			bufImgL[rho*Wid+theta] = colMultL[rho]*(bufColL[VPosHi[rho]]-bufColL[VPosLo[rho]-1]);
		
		//Pass2
		bufColL[0] = bufImgL[theta];
		for (rho = 1; rho<Hei; rho++)
			bufColL[rho] = bufImgL[rho*Wid+theta]+bufColL[rho-1];
		
		for (rho = startRho; rho<Hei; rho++)
			bufImgL[rho*Wid+theta] = colMultL[rho]*(bufColL[VPosHi[rho]]-bufColL[VPosLo[rho]-1]);
		
		//Pass3
		bufColL[0] = bufImgL[theta];
		for (rho = 1; rho<Hei; rho++)
			bufColL[rho] = bufImgL[rho*Wid+theta]+bufColL[rho-1];
		
		for (rho = startRho; rho<Hei; rho++)
			bufImgL[rho*Wid+theta] = colMultL[rho]*(bufColL[VPosHi[rho]]-bufColL[VPosLo[rho]-1]);
		
	}

	//Hor Smooth


	for(rho=startRho; rho<Hei; rho++)
	{
		lineSize = HPosHi[rho]-HPosLo[rho]+1;
		multiplier = 1.0f/lineSize;
		multiplier *= (multiplier*multiplier);
		up = lineSize/2;
		down = -up;
		up++;
		
		//Pass 1

		bufRowL[0] = bufImgL[rho*Wid];
		for (theta = 1; theta<Wid; theta++)
			bufRowL[theta] = bufImgL[rho*Wid+theta]+bufRowL[theta-1];
		
		for (theta = Wid; theta<Wid+lineSize+1; theta++)
			bufRowL[theta] = bufImgL[rho*Wid+theta-Wid]+bufRowL[theta-1];
		
		for (theta = -down+1; theta<Wid; theta++)
			bufImgL[rho*Wid+theta] = (bufRowL[theta+up-1]-bufRowL[theta+down-1]);
		
		for (theta = Wid; theta<Wid-down+1; theta++)
			bufImgL[rho*Wid+theta-Wid] = (bufRowL[theta+up-1]-bufRowL[theta+down-1]);
		
		//Pass 2
		
		bufRowL[0] = bufImgL[rho*Wid];
		for (theta = 1; theta<Wid; theta++)
			bufRowL[theta] = bufImgL[rho*Wid+theta]+bufRowL[theta-1];
		
		for (theta = Wid; theta<Wid+lineSize+1; theta++)
			bufRowL[theta] = bufImgL[rho*Wid+theta-Wid]+bufRowL[theta-1];
		
		for (theta = -down+1; theta<Wid; theta++)
			bufImgL[rho*Wid+theta] = (bufRowL[theta+up-1]-bufRowL[theta+down-1]);
		
		for (theta = Wid; theta<Wid-down+1; theta++)
			bufImgL[rho*Wid+theta-Wid] = (bufRowL[theta+up-1]-bufRowL[theta+down-1]);

		//Pass 3
		
		bufRowL[0] = bufImgL[rho*Wid];
		for (theta = 1; theta<Wid; theta++)
			bufRowL[theta] = bufImgL[rho*Wid+theta]+bufRowL[theta-1];
		
		for (theta = Wid; theta<Wid+lineSize+1; theta++)
			bufRowL[theta] = bufImgL[rho*Wid+theta-Wid]+bufRowL[theta-1];
		
		for (theta = -down+1; theta<Wid; theta++)
			bufImgL[rho*Wid+theta] = multiplier*(bufRowL[theta+up-1]-bufRowL[theta+down-1]);
		
		for (theta = Wid; theta<Wid-down+1; theta++)
			bufImgL[rho*Wid+theta-Wid] = multiplier*(bufRowL[theta+up-1]-bufRowL[theta+down-1]);
	}


}

void oFlow::intImsmoothCart(float * inImg, float * outImg, float sig, float *gauss)
{
	int j,i;
	int startRho = 0;
	float multiplier;
	int cartWid, cartHei;
	int trunc = (int)(3.0*sig+1);
	
	if (bufImgC == NULL)
	{
		if (procmode==CART)
		{
			bufImgC = new float [ImgSize];
			cartWid = Wid;
			cartHei = Hei;
		}
		else
		{
			bufImgC = new float [hybISize-ImgSize];
			cartWid = 2*cartfov;
			cartHei = 2*cartfov;
		}
	}
	
	if (procmode==CART)
	{
		cartWid = Wid;
		cartHei = Hei;
	}
	else
	{
		cartWid = 2*cartfov;
		cartHei = 2*cartfov;
	}

	if (bufRowC==NULL)
		bufRowC = new float[cartWid];

	if (bufColC==NULL)
		bufColC = new float[cartHei];

	
	multiplier = 1.0f/(2*trunc+1);

	multiplier *= multiplier*multiplier;
	multiplier *= multiplier;

	//Vert Smooth
	for (i=0; i<cartWid; i++)
	{
		//Pass1
		bufColC[0] = inImg[i];
		for (j = 1; j<cartHei; j++)
		{
			bufColC[j] = inImg[j*cartWid+i]+bufColC[j-1];
		}
		
		for (j = trunc+1; j<cartHei-trunc; j++)
			bufImgC[j*cartWid+i] = (bufColC[j+trunc]-bufColC[j-trunc-1]);

		for (j = 0; j<trunc+1; j++)
			bufImgC[j*cartWid+i] = (bufColC[j+trunc]-bufColC[0]);
		
		for (j = cartHei-trunc; j<cartHei; j++)
			bufImgC[j*cartWid+i] = (bufColC[cartHei-1]-bufColC[j-trunc-1]);
		
		//Pass2
		bufColC[0] = bufImgC[i];
		for (j = 1; j<cartHei; j++)
			bufColC[j] = bufImgC[j*cartWid+i]+bufColC[j-1];
		
		for (j = trunc+1; j<cartHei-trunc; j++)
			bufImgC[j*cartWid+i] = (bufColC[j+trunc]-bufColC[j-trunc-1]);
		
		for (j = 0; j<trunc+1; j++)
			bufImgC[j*cartWid+i] = (bufColC[j+trunc]-bufColC[0]);
		
		for (j = cartHei-trunc; j<cartHei; j++)
			bufImgC[j*cartWid+i] = (bufColC[cartHei-1]-bufColC[j-trunc-1]);
		
		//Pass3
		bufColC[0] = bufImgC[i];
		for (j = 1; j<cartHei; j++)
			bufColC[j] = bufImgC[j*cartWid+i]+bufColC[j-1];
		
		for (j = trunc+1; j<cartHei-trunc; j++)
			bufImgC[j*cartWid+i] = (bufColC[j+trunc]-bufColC[j-trunc-1]);
		
		for (j = 0; j<trunc+1; j++)
			bufImgC[j*cartWid+i] = (bufColC[j+trunc]-bufColC[0]);
		
		for (j = cartHei-trunc; j<cartHei; j++)
			bufImgC[j*cartWid+i] = (bufColC[cartHei-1]-bufColC[j-trunc-1]);
		
	}

	//Hor Smooth
	for (j=0; j<cartHei; j++)
	{
		//Pass1
		bufRowC[0] = bufImgC[j];
		for (i = 1; i<cartWid; i++)
			bufRowC[i] = bufImgC[j*cartWid+i]+bufRowC[i-1];
		
		for (i = trunc+1; i<cartWid-trunc; i++)
			bufImgC[j*cartWid+i] = (bufRowC[i+trunc]-bufRowC[i-trunc-1]);
		
		for (i = 0; i<trunc+1; i++)
			bufImgC[j*cartWid+i] = (bufRowC[i+trunc]-bufRowC[0]);
		
		for (i= cartHei-trunc; i<cartHei; i++)
			bufImgC[j*cartWid+i] = (bufRowC[cartWid-1]-bufRowC[i-trunc-1]);
		
		//Pass2
		bufRowC[0] = bufImgC[j];
		for (i = 1; i<cartWid; i++)
			bufRowC[i] = bufImgC[j*cartWid+i]+bufRowC[i-1];
		
		for (i = trunc+1; i<cartWid-trunc; i++)
			bufImgC[j*cartWid+i] = (bufRowC[i+trunc]-bufRowC[i-trunc-1]);
		
		for (i = 0; i<trunc+1; i++)
			bufImgC[j*cartWid+i] = (bufRowC[i+trunc]-bufRowC[0]);
		
		for (i= cartHei-trunc; i<cartHei; i++)
			bufImgC[j*cartWid+i] = (bufRowC[cartWid-1]-bufRowC[i-trunc-1]);
		
		//Pass3
		bufRowC[0] = bufImgC[j];
		for (i = 1; i<cartWid; i++)
			bufRowC[i] = bufImgC[j*cartWid+i]+bufRowC[i-1];
		
		for (i = trunc+1; i<cartWid-trunc; i++)
			bufImgC[j*cartWid+i] = (bufRowC[i+trunc]-bufRowC[i-trunc-1]);
		
		for (i = 0; i<trunc+1; i++)
			bufImgC[j*cartWid+i] = (bufRowC[i+trunc]-bufRowC[0]);
		
		for (i= cartHei-trunc; i<cartHei; i++)
			bufImgC[j*cartWid+i] = multiplier*(bufRowC[cartWid-1]-bufRowC[i-trunc-1]);
	}
}


void oFlow::imsmoothCart(float * inImg, float * outImg, float sig, float *gauss)
{
	int i,j,k;
	int cartWid, cartHei;
	int offSet;
	if(sig > 0.01) 
	{
		if (bufImgC == NULL)
		{
			if (procmode==CART)
			{
				bufImgC = new float [ImgSize];
//				memset(bufImg,0,ImgSize*sizeof(float));
				cartWid = Wid;
				cartHei = Hei;
			}
			else
			{
				bufImgC = new float [hybISize-ImgSize];
//				memset(bufImg,0,(hybISize-ImgSize)*sizeof(float));
				cartWid = 2*cartfov;
				cartHei = 2*cartfov;
			}
		}
		
		if (procmode==CART)
		{
			cartWid = Wid;
			cartHei = Hei;
			offSet = 0;
		}
		else
		{
			cartWid = 2*cartfov;
			cartHei = 2*cartfov;
			offSet = 0;
		}

		int trunc = (int)(3.0f*sig+1);
		float acc;
		float acc2;
		
		//Horz Smoothing
		for (j=0; j<cartHei; j++)
			for (i=0; i<cartWid-(2*trunc); i++)
			{
				acc = 0.0f;
				for (k=0; k<2*trunc+1; k++)
				{
					acc+=gauss[k]*inImg[j*cartWid+i+k];
				}
				bufImgC[j*cartWid+i+trunc+offSet] = acc; 
			}
			
		for (j=0; j<cartHei; j++)
			for (i=0; i<trunc; i++)
			{
				acc = 0.0f;
				acc2 = 0.0f;
				for (k=trunc-i; k<2*trunc+1; k++)
				{
					acc+=gauss[k]*inImg[j*cartWid+i+k-trunc];
					acc2+=gauss[k];
				}
				bufImgC[j*cartWid+i+offSet] = acc/acc2; 
			}
				
		for (j=0; j<cartHei; j++)
			for (i=cartWid-trunc; i<cartWid; i++)
			{
				acc = 0.0f;
				acc2 = 0.0f;
				for (k=0; k<trunc+cartWid-i; k++)
				{
					acc+=gauss[k]*inImg[j*cartWid+i+k-trunc];
					acc2+=gauss[k];
				}
				bufImgC[j*cartWid+i+offSet] = acc/acc2; 
			}
					
		//Vert Smoothing
		for (i=0; i<cartWid; i++)
			for (j=0; j<cartHei-(2*trunc); j++)
			{
				acc = 0.0f;
				for (k=0; k<2*trunc+1; k++)
				{
					acc+=gauss[k]*bufImgC[(j+k)*cartWid+i+offSet];
				}
				outImg[(j+trunc)*cartWid+i] = acc; 
			}
			
		for (i=0; i<cartWid; i++)
			for (j=0; j<trunc; j++)
			{
				acc = 0.0f;
				acc2 = 0.0f;
				for (k=trunc-j; k<2*trunc+1; k++)
				{
					acc+=gauss[k]*bufImgC[(j+k-trunc)*cartWid+i+offSet];
					acc2+=gauss[k];
				}
				outImg[j*cartWid+i] = acc/acc2; 
			}
			
		for (i=0; i<cartWid; i++)
			for (j=cartHei-trunc; j<cartHei; j++)
			{
				acc = 0.0f;
				acc2 = 0.0f;
				for (k=0; k<trunc+cartHei-j; k++)
				{
					acc+=gauss[k]*bufImgC[(j+k-trunc)*cartWid+i+offSet];
					acc2+=gauss[k];
				}
				outImg[j*cartWid+i] = acc/acc2; 
			}
	}
	else
	{
		for (i=0; i<hybISize; i++) 
			outImg[i] = (float)inImg[i];
	}
}


void oFlow::setGaussian(float sigma, float *gaussian)
{
	int trunc;
	int i;
	float scale;
	float acc;
	
	trunc = (int)(3*sigma+1);
	
	if (sigma>0.01)
	{
		scale = (float)(1.0/(sigma*sqrt(2.0*PI)));
		acc = 0.0f;
		
		for (i=-trunc; i<=trunc; i++)
		{
			gaussian[i+trunc] = (float)(scale*exp (-(i*i)/(2.0*sigma*sigma)));
			acc += gaussian[i+trunc];
		}
		for (i=-trunc; i<=trunc; i++)
			gaussian[i+trunc] /= acc;
	}
	else
		memset(gaussian,0,(2*trunc+1)*sizeof(float));
	
}



void oFlow::initSmooth(float **LUTV, float **LUTH, int **VPosLo, int **HPosLo,int **VPosHi, int **HPosHi, float sig)
{

	if (sig > 0.0)
	{
		int rho;
		int j;

		float x,y,x0,y0;
		float gLim;

		float acc;
		float varx,vary;

		if (radius==NULL)
			radius = new float[nEcc];
		
		if (recFieldRadSize==NULL)
			recFieldRadSize = new float[nEcc];

		if (recFieldTanSize==NULL)
			recFieldTanSize = new float[nEcc];

		if(*VPosLo == NULL)
			*VPosLo = new int[nEcc];
		if(*VPosHi == NULL)
			*VPosHi = new int[nEcc];
		if(*HPosLo == NULL)
			*HPosLo = new int[nEcc];
		if(*HPosHi == NULL)
			*HPosHi = new int[nEcc];
		
		float frho, ftheta;

		for (rho=0; rho<nEcc; rho++)	//computes the length of the radius of the center of the rho-th ring
		{
			if (rho<fov)
			{
				if (rho==0)
				{
					radius[rho] = (float)(firstRing* scaleFactor);	
					recFieldRadSize[rho] = 2*radius[rho];
				}
				else
				{
					radius[rho] = (float)((rho+2*firstRing-0.5) * scaleFactor);
					recFieldRadSize[rho]= (float)(1.0f*scaleFactor);
				}
			}
			else
			{
				radius[rho] = (float)(pow(lambda,rho) * (r0 * 0.5*(lambda+1)) * scaleFactor);//was /scalefactor
				recFieldRadSize[rho] = (float)((pow(lambda,(float)(rho+0.5))-pow(lambda,(float)(rho-0.5))) * (r0 * 0.5*(lambda+1)) * scaleFactor);//was /scalefactor
			}
			recFieldTanSize[rho] = (float)(2.0f*PI*radius[rho]/nAng);

			gLim =  3*sig+1;
			if(sig==posig)
				gLim = 3*sig;

			if (gLim<radius[rho])
			{
				x=(float)(2*radius[rho]*radius[rho]-gLim*gLim)/(2*radius[rho]);
				y=(float)sqrt(radius[rho]*radius[rho]-x*x);

				(*HPosHi)[rho] = (int)(ceil(nAng*atan2(y,x)/PI));
				(*HPosLo)[rho] = -(*HPosHi)[rho];

				getLPCoord(&frho,&ftheta,(float)(radius[rho]-gLim),0);
				(*VPosLo)[rho] = (int)floor(frho);

			}
			else
			{
				(*HPosHi)[rho] = nAng/2;
				(*HPosLo)[rho] = -(*HPosHi)[rho];
				(*VPosLo)[rho] = 0;
			}
			getLPCoord(&frho,&ftheta,(float)(radius[rho]+gLim),0);
			(*VPosHi)[rho] = (int)ceil(frho);
			if((*VPosHi)[rho]>=nEcc)
				(*VPosHi)[rho] = nEcc-1;

			LUTV[rho] = new float[(*VPosHi)[rho]-(*VPosLo)[rho]+1];
			LUTH[rho] = new float[(*HPosHi)[rho]-(*HPosLo)[rho]+1];

			acc=0.0f;
			
			getCCoord((float)rho,0.0f,&x0,&y0);
			for(j=(*VPosLo)[rho]; j<=(*VPosHi)[rho]; j++)
			{
				getCCoord((float)j,0.0f,&x,&y);
				
				varx = (x-x0)/sig;
				vary = (y-y0)/sig;
				varx *= varx;
				vary *= vary;
				LUTV[rho][j-(*VPosLo)[rho]]= (float)(exp(-varx)*exp(-vary)*recFieldRadSize[rho]);
				acc += LUTV[rho][j-(*VPosLo)[rho]];
			}
			for(j=(*VPosLo)[rho]; j<=(*VPosHi)[rho]; j++)
				LUTV[rho][j-(*VPosLo)[rho]]/=acc;

			acc=0.0f;
			
			for(j=(*HPosLo)[rho]; j<=(*HPosHi)[rho]; j++)
			{
				getCCoord((float)rho,(float)j,&x,&y);
				
				varx = (x-x0)/sig;
				vary = (y-y0)/sig;
				varx *= varx;
				vary *= vary;
				LUTH[rho][j-(*HPosLo)[rho]]= (float)(exp(-varx)*exp(-vary)*recFieldTanSize[rho]);
				acc += LUTH[rho][j-(*HPosLo)[rho]];
			}
			for(j=(*HPosLo)[rho]; j<=(*HPosHi)[rho]; j++)
				LUTH[rho][j-(*HPosLo)[rho]]/=acc;
		}
	}
}

void oFlow::getCCoord(float rho, float theta, float *x, float *y)
{
	//the cartesian origin is the center of the image
	float r;
	if (rho<fov)
	{
		if (rho<=0.5)
			r = rho*(4*firstRing);
		else
			r=rho + 2*firstRing - 0.5f;
	}
	else
		r = (float)pow (lambda, rho) * (r0 + 0.5f / (float)pow (lambda, fov));
	
	r *= scaleFactor;
	
	float ang;
	ang = (float)(2*PI*theta/nAng);
	
	*x = (float)(r*cos(ang));
	*y = (float)(r*sin(ang));
}

void oFlow::getLPCoord(float *rho, float *theta, float x, float y)
{
	float r;
	
	r  = (float)sqrt(x*x+y*y);
	r /= scaleFactor;

	*rho = (float)(log(r/(r0 + 0.5 / pow (lambda, fov)))/log(lambda));
	
	if (r<fov)
	{
		if (r<=2*firstRing)
			*rho = r/(4*firstRing);
		else
			*rho= (float)(r - 2*firstRing + 0.5);
	}
	else
		*rho = (float)(log(r/(r0 + 0.5 / pow (lambda, fov)))/log(lambda));

	float ang;

	ang = (float)(PI+atan2(-y,-x));
	
	*theta = (float)(nAng*ang/(2*PI));
}


void oFlow::getDx(float *inImg, float **outImg)
{
	int i,j,k;
	double acc;
	int cartWid, cartHei;
	int startRho = 0;

	if (*outImg==NULL)
	{
		*outImg = new float[hybISize];
		memset(*outImg,0,hybISize*sizeof(float));
	}

	float *xDer = *outImg;

	if (procmode == HYB)
	{
		cartWid = 2*cartfov;
		cartHei = 2*cartfov;
//		startRho = fov-6;
	}
	else
	{
		cartWid = Wid;
		cartHei = Hei;
//		startRho = 0;
	}
	
	startRho = 0;

	int krnCent = XYkSz/2; 

	if ((procmode==LP)||(procmode==HYB))	//LP
	{
		for (j=startRho; j<nEcc; j++)
		{
			for (i=0; i<nAng; i++)
			{
				acc = 0.0f;
				for (k=0; k<XYkSz; k++)
					acc+=XYker[k]*inImg[j*nAng+((i+k)%nAng)];
				
				xDer[j*nAng+((i+krnCent)%nAng)]=(float)acc;
			}
		}
	}
	else
	{
		for (j=0; j<cartHei; j++)
		{
			for (i=XYkSz/2; i<cartWid-XYkSz/2; i++)
			{
				acc = 0.0f;
				for (k=0; k<XYkSz; k++)
				{
//					acc+=derKernel[cartHei/2][k]*inImg[j*cartWid+(i+k-XYkSz/2)];
					acc+=derKernel[0][k]*inImg[j*cartWid+(i+k-XYkSz/2)];
				}
				xDer[j*cartWid+i] = (float)acc;
			}
		}

		for (j=0; j<cartHei; j++)
		{
			i=2;
			acc = 0.0f;
			for (k=1; k<XYkSz-1; k++)
			{
//				acc+=derKernel[cartHei-3][k]*inImg[j*cartWid+(i+k-XYkSz/2)];
				acc+=derKernel[1][k]*inImg[j*cartWid+(i+k-XYkSz/2)];
			}
			xDer[j*cartWid+i] = (float)acc;
			
			i=1;
			acc = 0.0f;
			for (k=2; k<XYkSz-2; k++)
			{
//				acc+=derKernel[cartHei-2][k]*inImg[j*cartWid+(i+k-XYkSz/2)];
				acc+=derKernel[2][k]*inImg[j*cartWid+(i+k-XYkSz/2)];
			}
			xDer[j*cartWid+i] = (float)acc;
			
			i=cartWid-3;
			acc = 0.0f;
			for (k=1; k<XYkSz-1; k++)
			{
//				acc+=derKernel[cartHei-3][k]*inImg[j*cartWid+(i+k-XYkSz/2)];
				acc+=derKernel[1][k]*inImg[j*cartWid+(i+k-XYkSz/2)];
			}
			xDer[j*cartWid+i] = (float)acc;
			
			i=cartWid-2;
			acc = 0.0f;
			for (k=2; k<XYkSz-2; k++)
			{
//				acc+=derKernel[cartHei-2][k]*inImg[j*cartWid+(i+k-XYkSz/2)];
				acc+=derKernel[2][k]*inImg[j*cartWid+(i+k-XYkSz/2)];
			}
			xDer[j*cartWid+i] = (float)acc;
		}
	}

	if (procmode==HYB)
	{
		for (j=0; j<cartHei; j++)
		{
			for (i=XYkSz/2; i<cartWid-XYkSz/2; i++)
			{
				acc = 0.0f;
				for (k=0; k<XYkSz; k++)
				{
					acc+=derKernel[0][k]*inImg[j*cartWid+(i+k-XYkSz/2)+ImgSize];
				}
				xDer[j*cartWid+i+ImgSize] = (float)acc;
			}
		}
		
		for (j=0; j<cartHei; j++)
		{
			i=2;
			acc = 0.0f;
			for (k=1; k<XYkSz-1; k++)
			{
//				acc+=derKernel[cartHei-3][k]*inImg[j*cartWid+(i+k-XYkSz/2)+ImgSize];
				acc+=derKernel[1][k]*inImg[j*cartWid+(i+k-XYkSz/2)+ImgSize];
			}
			xDer[j*cartWid+i+ImgSize] = (float)acc;
			
			i=1;
			acc = 0.0f;
			for (k=2; k<XYkSz-2; k++)
			{
//				acc+=derKernel[cartHei-2][k]*inImg[j*cartWid+(i+k-XYkSz/2)+ImgSize];
				acc+=derKernel[2][k]*inImg[j*cartWid+(i+k-XYkSz/2)+ImgSize];
			}
			xDer[j*cartWid+i+ImgSize] = (float)acc;
			
			i=cartWid-3;
			acc = 0.0f;
			for (k=1; k<XYkSz-1; k++)
			{
//				acc+=derKernel[cartHei-3][k]*inImg[j*cartWid+(i+k-XYkSz/2)+ImgSize];
				acc+=derKernel[1][k]*inImg[j*cartWid+(i+k-XYkSz/2)+ImgSize];
			}
			xDer[j*cartWid+i+ImgSize] = (float)acc;
			
			i=cartWid-2;
			acc = 0.0f;
			for (k=2; k<XYkSz-2; k++)
			{
//				acc+=derKernel[cartHei-2][k]*inImg[j*cartWid+(i+k-XYkSz/2)+ImgSize];
				acc+=derKernel[2][k]*inImg[j*cartWid+(i+k-XYkSz/2)+ImgSize];
			}
			xDer[j*cartWid+i+ImgSize] = (float)acc;
		}
	}

//	return fx;
}



void oFlow::getDy(float *inImg, float **outImg)
{
	int i,j,k;
	double acc;

	int startRho=0;


	if (*outImg==NULL)
	{
		*outImg = new float[hybISize];
		memset(*outImg,0,hybISize*sizeof(float));
	}
	
	float *yDer = *outImg;

	int cartWid, cartHei;

	if (procmode == HYB)
	{
		cartWid = 2*cartfov;
		cartHei = 2*cartfov;
//		startRho = fov-6;
	}

	
	
	int krnCent = XYkSz/2; 

	for (i=0; i<Wid; i++)
	{
		for (j=startRho+XYkSz/2; j<Hei-XYkSz/2; j++)
		{
			acc = 0.0f;
			for (k=0; k<XYkSz; k++)
			{
//				acc+=derKernel[j][k]*inImg[(j+k-XYkSz/2)*Wid+i];
				acc+=derKernel[0][k]*inImg[(j+k-XYkSz/2)*Wid+i];
			}
			yDer[j*Wid+i] = (float)acc;
		}
	}
			
	if ((procmode == LP)||(procmode==HYB))
//	if ((procmode==HYB))
	{
		for (i=0; i<Wid; i++)
		{
			j=2;
			acc = 0.0f;
			for (k=1; k<XYkSz; k++)
			{
//				acc+=derKernel[j][k]*inImg[(j+k-XYkSz/2)*nAng+i];
				acc+=derKernel[0][k]*inImg[(j+k-XYkSz/2)*nAng+i];
			}
			
			acc+=derKernel[0][0]*inImg[(i+nAng/2)%nAng];
			
			yDer[2*nAng+i] = (float)acc;
			
			j=1;
			acc = 0.0f;
			for (k=2; k<XYkSz; k++)
			{
//				acc+=derKernel[j][k]*inImg[(j+k-XYkSz/2)*nAng+i];
				acc+=derKernel[0][k]*inImg[(j+k-XYkSz/2)*nAng+i];
			}
			acc+=derKernel[0][0]*inImg[nAng+((i+nAng/2)%nAng)];
			acc+=derKernel[0][1]*inImg[(i+nAng/2)%nAng];
			yDer[nAng+i] = (float)acc;
			
			j=0;
			acc = 0.0f;
			for (k=3; k<XYkSz; k++)
			{
//				acc+=derKernel[j][k]*inImg[(j+k-XYkSz/2)*nAng+i];
				acc+=derKernel[0][k]*inImg[(j+k-XYkSz/2)*nAng+i];
			}
			acc+=derKernel[0][0]*inImg[2*nAng+((i+nAng/2)%nAng)];
			acc+=derKernel[0][1]*inImg[  nAng+((i+nAng/2)%nAng)];
			acc+=derKernel[0][2]*inImg[(i+nAng/2)%nAng];
			yDer[i] = (float)acc;
		}
	}
	else if (procmode==CART)
	{
		for (i=0; i<Wid; i++)
		{
			j=2;
			acc = 0.0f;
			for (k=1; k<XYkSz-1; k++)
			{
//				acc+=derKernel[Hei-3][k]*inImg[(j+k-XYkSz/2)*Wid+i];
				acc+=derKernel[1][k]*inImg[(j+k-XYkSz/2)*Wid+i];
			}
			yDer[j*Wid+i] = (float)acc;
			
			j=1;
			acc = 0.0f;
			for (k=2; k<XYkSz-2; k++)
			{
//				acc+=derKernel[Hei-2][k]*inImg[(j+k-XYkSz/2)*Wid+i];
				acc+=derKernel[2][k]*inImg[(j+k-XYkSz/2)*Wid+i];
			}
			yDer[j*Wid+i] = (float)acc;
		}
	}
		
	for (i=0; i<Wid; i++)
	{
		j=Hei-3;
		acc = 0.0f;
		for (k=1; k<XYkSz-1; k++)
		{
//			acc+=derKernel[j][k]*inImg[(j+k-XYkSz/2)*Wid+i];
			acc+=derKernel[1][k]*inImg[(j+k-XYkSz/2)*Wid+i];
		}
		yDer[j*Wid+i] = (float)acc;
		
		j=Hei-2;
		acc = 0.0f;
		for (k=2; k<XYkSz-2; k++)
		{
//			acc+=derKernel[j][k]*inImg[(j+k-XYkSz/2)*Wid+i];
			acc+=derKernel[2][k]*inImg[(j+k-XYkSz/2)*Wid+i];
		}
		yDer[j*Wid+i] = (float)acc;
	}

	if (procmode == HYB)
	{
		for (i=0; i<cartWid; i++)
		{
			for (j=XYkSz/2; j<cartWid-XYkSz/2; j++)
			{
				acc = 0.0f;
				for (k=0; k<XYkSz; k++)
				{
//					acc+=derKernel[j][k]*inImg[(j+k-XYkSz/2)*cartWid+i+ImgSize];
					acc+=derKernel[0][k]*inImg[(j+k-XYkSz/2)*cartWid+i+ImgSize];
				}
				yDer[j*cartWid+i+ImgSize] = (float)acc;
			}
		}
		
		for (i=0; i<cartWid; i++)
		{
			j=2;
			acc = 0.0f;
			for (k=1; k<XYkSz-1; k++)
			{
//				acc+=derKernel[Hei-3][k]*inImg[(j+k-XYkSz/2)*cartWid+i+ImgSize];
				acc+=derKernel[1][k]*inImg[(j+k-XYkSz/2)*cartWid+i+ImgSize];
			}
			yDer[j*cartWid+i+ImgSize] = (float)acc;
			
			j=1;
			acc = 0.0f;
			for (k=2; k<XYkSz-2; k++)
			{
//				acc+=derKernel[Hei-2][k]*inImg[(j+k-XYkSz/2)*cartWid+i+ImgSize];
				acc+=derKernel[2][k]*inImg[(j+k-XYkSz/2)*cartWid+i+ImgSize];
			}
			yDer[j*cartWid+i+ImgSize] = (float)acc;
			
			j=cartHei-3;
			acc = 0.0f;
			for (k=1; k<XYkSz-1; k++)
			{
//				acc+=derKernel[j][k]*inImg[(j+k-XYkSz/2)*cartWid+i+ImgSize];
				acc+=derKernel[1][k]*inImg[(j+k-XYkSz/2)*cartWid+i+ImgSize];
			}
			yDer[j*cartWid+i+ImgSize] = (float)acc;
			
			j=cartHei-2;
			acc = 0.0f;
			for (k=2; k<XYkSz-2; k++)
			{
//				acc+=derKernel[j][k]*inImg[(j+k-XYkSz/2)*cartWid+i+ImgSize];
				acc+=derKernel[2][k]*inImg[(j+k-XYkSz/2)*cartWid+i+ImgSize];
			}
			yDer[j*cartWid+i+ImgSize] = (float)acc;
		}
	}

//	return fy;
}

void oFlow::getDt(float **inImg, float **outImg, int currSlot)
{
	int i,k;
	double acc;

	if (*outImg==NULL)
		*outImg = new float[hybISize];
	
	float *tDer = *outImg;
	
//		for (j=0; j<Hei; j++)
//	for (i=0; i<Wid; i++)
	for (i=0; i<hybISize; i++)
		{
			acc = 0.0f;
			for (k=0; k<TkSz; k++)
			{
				acc+=Tker[k]*inImg[(k+currSlot)%TkSz][i];
			}
			tDer[i] = (float)(acc); 
		}
}


void oFlow::buildDerKernel()
{
	int i;

//	if (derKernel==NULL)
//		derKernel=new float *[3];
		
	for (i=0; i<3; i++)
		derKernel[i]=new float [XYkSz];
	
	derKernel[0][0] =  -1.0f/60.0f;
	derKernel[0][1] =   9.0f/60.0f;
	derKernel[0][2] = -45.0f/60.0f;
	derKernel[0][3] =   0.0f;
	derKernel[0][4] = -derKernel[0][2];
	derKernel[0][5] = -derKernel[0][1];
	derKernel[0][6] = -derKernel[0][0];
	
	//Border
	/*i = Hei-3;*/
	i=1;
	derKernel[i][0] =  -0.0f/12.0f;
	derKernel[i][1] =   1.0f/12.0f;
	derKernel[i][2] =  -8.0f/12.0f;
	derKernel[i][3] =   0.0f;
	derKernel[i][4] = -derKernel[i][2];
	derKernel[i][5] = -derKernel[i][1];
	derKernel[i][6] = -derKernel[i][0];
	
	i = 2;
	derKernel[i][0] =   0.0f/2.0f;
	derKernel[i][1] =   0.0f/2.0f;
	derKernel[i][2] =  -1.0f/2.0f;
	derKernel[i][3] =   0.0f;
	derKernel[i][4] = -derKernel[i][2];
	derKernel[i][5] = -derKernel[i][1];
	derKernel[i][6] = -derKernel[i][0];	
	
}


void oFlow::crossProducts()
{
	int i;
	
	if (fxx==NULL)
		fxx = new float [hybISize];
	if (fxy==NULL)
		fxy = new float [hybISize];
	if (fxt==NULL)
		fxt = new float [hybISize];
	if (fyy==NULL)
		fyy = new float [hybISize];
	if (fyt==NULL)
		fyt = new float [hybISize];
	if (ftt==NULL)
		ftt = new float [hybISize];
	
	for (i=0; i<hybISize; i++)
	{
		fxx[i] = fx[i]*fx[i];
		fxy[i] = fx[i]*fy[i];
		fxt[i] = fx[i]*ft[i];
		fyy[i] = fy[i]*fy[i];
		fyt[i] = fy[i]*ft[i];
		ftt[i] = ft[i]*ft[i];
	}
}


void oFlow::postSmooth()
{
	bool setG = false;
	
	if ((XYTproc)||(procmode==CART)||procmode==HYB)
		if (postGaussian==NULL)
		{
			postGaussian = new float [2*((int)(3*posig+1))+1];
			setG = true;
		}
	
	if (setG)
		setGaussian(posig,postGaussian);
	
	if ((procmode == LP)||(procmode==HYB))
	{
		setG = false;
		
		if (postGLUTVert==NULL)
		{
			postGLUTVert = new float * [Hei];
			setG = true;
		}
		if (postGLUTHor==NULL)
		{
			postGLUTHor = new float * [Hei];
			setG = true;
		}
		
		if ((setG))
			initSmooth(postGLUTVert,postGLUTHor,&rhoLoPo,&thetaLoPo,&rhoHiPo,&thetaHiPo,posig);
	}
	

	if ((procmode == LP)||(procmode==HYB))
	{
		imsmoothLP(fxx,fxx,postGLUTVert,postGLUTHor,rhoLoPo,thetaLoPo,rhoHiPo,thetaHiPo);
		imsmoothLP(fxy,fxy,postGLUTVert,postGLUTHor,rhoLoPo,thetaLoPo,rhoHiPo,thetaHiPo);
		imsmoothLP(fxt,fxt,postGLUTVert,postGLUTHor,rhoLoPo,thetaLoPo,rhoHiPo,thetaHiPo);
		imsmoothLP(fyy,fyy,postGLUTVert,postGLUTHor,rhoLoPo,thetaLoPo,rhoHiPo,thetaHiPo);
		imsmoothLP(fyt,fyt,postGLUTVert,postGLUTHor,rhoLoPo,thetaLoPo,rhoHiPo,thetaHiPo);
		imsmoothLP(ftt,ftt,postGLUTVert,postGLUTHor,rhoLoPo,thetaLoPo,rhoHiPo,thetaHiPo);
	}
	else
	{
		imsmoothCart(fxx,fxx,posig,postGaussian);
		imsmoothCart(fxy,fxy,posig,postGaussian);
		imsmoothCart(fxt,fxt,posig,postGaussian);
		imsmoothCart(fyy,fyy,posig,postGaussian);
		imsmoothCart(fyt,fyt,posig,postGaussian);
		imsmoothCart(ftt,ftt,posig,postGaussian);
	}

	if(procmode==HYB)
	{
		imsmoothCart(fxx+ImgSize,fxx+ImgSize,posig,postGaussian);
		imsmoothCart(fxy+ImgSize,fxy+ImgSize,posig,postGaussian);
		imsmoothCart(fxt+ImgSize,fxt+ImgSize,posig,postGaussian);
		imsmoothCart(fyy+ImgSize,fyy+ImgSize,posig,postGaussian);
		imsmoothCart(fyt+ImgSize,fyt+ImgSize,posig,postGaussian);
		imsmoothCart(ftt+ImgSize,ftt+ImgSize,posig,postGaussian);
	}
		
}


void oFlow::intPostSmooth()
{
	bool setG = false;
	
	if ((XYTproc)||(procmode==CART)||procmode==HYB)
		if ((procmode == LP)||(procmode==HYB))
		{
			setG = false;
			
			if (postGLUTVert==NULL)
			{
				postGLUTVert = new float * [Hei];
				setG = true;
			}
			if (postGLUTHor==NULL)
			{
				postGLUTHor = new float * [Hei];
				setG = true;
			}
			
			if ((setG))
				initSmooth(postGLUTVert,postGLUTHor,&rhoLoPo,&thetaLoPo,&rhoHiPo,&thetaHiPo,posig);
		}
		
		if (posig>0.1)
		{
			if ((procmode == LP)||(procmode==HYB))
			{
				intImsmoothLP(fxx,fxx,postGLUTVert,postGLUTHor,rhoLoPo,thetaLoPo,rhoHiPo,thetaHiPo);
				intImsmoothLP(fxy,fxy,postGLUTVert,postGLUTHor,rhoLoPo,thetaLoPo,rhoHiPo,thetaHiPo);
				intImsmoothLP(fxt,fxt,postGLUTVert,postGLUTHor,rhoLoPo,thetaLoPo,rhoHiPo,thetaHiPo);
				intImsmoothLP(fyy,fyy,postGLUTVert,postGLUTHor,rhoLoPo,thetaLoPo,rhoHiPo,thetaHiPo);
				intImsmoothLP(fyt,fyt,postGLUTVert,postGLUTHor,rhoLoPo,thetaLoPo,rhoHiPo,thetaHiPo);
				intImsmoothLP(ftt,ftt,postGLUTVert,postGLUTHor,rhoLoPo,thetaLoPo,rhoHiPo,thetaHiPo);
			}
			else
			{
				intImsmoothCart(fxx,fxx,posig,postGaussian);
				intImsmoothCart(fxy,fxy,posig,postGaussian);
				intImsmoothCart(fxt,fxt,posig,postGaussian);
				intImsmoothCart(fyy,fyy,posig,postGaussian);
				intImsmoothCart(fyt,fyt,posig,postGaussian);
				intImsmoothCart(ftt,ftt,posig,postGaussian);
			}
			
			if(procmode==HYB)
			{
				intImsmoothCart(fxx+ImgSize,fxx+ImgSize,posig,postGaussian);
				intImsmoothCart(fxy+ImgSize,fxy+ImgSize,posig,postGaussian);
				intImsmoothCart(fxt+ImgSize,fxt+ImgSize,posig,postGaussian);
				intImsmoothCart(fyy+ImgSize,fyy+ImgSize,posig,postGaussian);
				intImsmoothCart(fyt+ImgSize,fyt+ImgSize,posig,postGaussian);
				intImsmoothCart(ftt+ImgSize,ftt+ImgSize,posig,postGaussian);
			}
		}
}


void oFlow::computeFlow()
{
//	long TimeI, TimeF;
//	TimeI = getTime();

	float Msensitivity = 0.001f;
	float Asensitivity = 0.0005f;
	Msensitivity = 0.001f;//was 0.01
	Asensitivity = 0.005f;//was 0.05
	float omega = 1.98f;
	float alpha = 1000.0f;
	bool firstpass = false;

	int i,j;

	int itRho, itThe;
	int itX,itY;


	if (procmode!=CART)
	{
		if ((X0eq==NULL)||(Y0eq==NULL))
		{
			if (X0eq==NULL)
				X0eq = new float [ImgSize];
			if (Y0eq==NULL)
				Y0eq = new float [ImgSize];
			
			for (i=0; i<ImgSize; i++)
				getCCoord((float)(i/nAng),(float)(i%nAng),&(X0eq[i]),&(Y0eq[i]));
		}
		
		if ((deX==NULL)||(deY==NULL))
		{
			if (deX==NULL)
				deX = new float [ImgSize];
			if (deY==NULL)
				deY = new float [ImgSize];
			
			memset(deX,0,ImgSize*sizeof(float));
			memset(deY,0,ImgSize*sizeof(float));
		}
		
		if ((xpdx==NULL)||(ypdy==NULL))
		{
			if(xpdx==NULL)
				xpdx = new float [ImgSize];
			
			if(ypdy==NULL)
				ypdy = new float [ImgSize];

			memcpy(xpdx,X0eq,ImgSize*sizeof(float));
			memcpy(ypdy,Y0eq,ImgSize*sizeof(float));
		}
	}

	if (u==NULL)
	{
		u = new float [hybISize];
		firstpass = true;
		memset(u,0,hybISize*sizeof(float));
	}

	if (v==NULL)
	{
		v = new float [hybISize];
		memset(v,0,hybISize*sizeof(float));
	}

	if (uDen==NULL)
		uDen = new float [hybISize];
	
	if (vDen==NULL)
		vDen = new float [hybISize];

	float avgdX,avgdY;
	float temprho, temptheta;

	if(procMask==NULL)
		procMask = new bool[hybISize];


	int pixCount=0;

	for (i=0; i<hybISize; i++)
	{
		if(ftt[i]>=1.0f)
		{
			procMask[i] = true;
			pixCount++;
		}
		else
		{
//			u[i] = 0.0f;
//			v[i] = 0.0f;
//			if (i<ImgSize)
//			{
//				deX[i] = 0.0f;
//				deY[i] = 0.0f;
//				xpdx[i] = X0eq[i];
//				ypdy[i] = Y0eq[i];
//			}
			procMask[i] = false;
		}
		fxy[i] = fxy[i]/alpha;
		fxt[i] = fxt[i]/alpha;
		fyt[i] = fyt[i]/alpha;
		uDen[i] = omega/(4.0f+fxx[i]/alpha);
		vDen[i] = omega/(4.0f+fyy[i]/alpha);
	}

#ifdef NONLINEAR

	float *dxu,*dxv,*dyu,*dyv;
	float *w2;

	float beta1, beta2;
	
	dxu = new float [hybISize];
	dxv = new float [hybISize];
	dyu = new float [hybISize];
	dyv = new float [hybISize];

	w2  = new float [hybISize];

	getDx(deX,dxu);
	getDx(deY,dxv);
	getDy(deX,dyu);
	getDy(deY,dyv);

	for (i=0; i<ImgSize; i++)
		w2[i] = (dxu[i]*dxu[i]+dxv[i]*dxv[i]+dyu[i]*dyu[i]+dyv[i]*dyv[i])/beta2;

	delete [] dxu;
	delete [] dxv;
	delete [] dyu;
	delete [] dyv;
	delete [] w2;




#endif


	float sumMu, sumPu;
	float sumMv, sumPv;
	int startPix, endPix;

	float err,tempUFlow,tempVFlow;

	float ratioU,ratioV;
	
	int cartWid = 2*cartfov;
	int cartHei = 2*cartfov;

	int Xlim, Ylim;
//	Xlim = cartfov-(int)((fov+80)*scaleFactor+0.5);
	Xlim = border;
	Ylim = Xlim;

//	TimeF = getTime();
//	printf("Initialization = %f msec \n", ((TimeF-TimeI)/1.0f));
//
//	return;
	
	if (procmode == HYB)
	{
		startPix = (1*cartWid)+1+ImgSize;
		endPix = (cartHei-1-0)*cartWid-1-0+ImgSize;

		for (j=0; j<10000; j++)
		{
			err = 0.0f;
			pixCount = 0;


			for (i=startPix; i<endPix; i++)
			{
				if (procMask[i]==false)
					continue;

				itX = (i-ImgSize)%cartWid;
				itY = (i-ImgSize)/cartWid;

				if ((itX<Xlim)||(itX>cartWid-Xlim-1))
					continue;
				if ((itY<Ylim)||(itY>cartWid-Ylim-1))
					continue;

				pixCount++;
				tempUFlow= u[i]; 
				tempVFlow= v[i];

				sumMu = u[i-1]+u[i-cartWid];
				sumPu = u[i+1]+u[i+cartWid];
				sumMv = v[i-1]+v[i-cartWid];
				sumPv = v[i+1]+v[i+cartWid];
				
				u[i] = (float)((1.0f-omega)*u[i] + uDen[i]*(sumMu+sumPu-(fxy[i]*v[i]+fxt[i])));
				v[i] = (float)((1.0f-omega)*v[i] + vDen[i]*(sumMv+sumPv-(fxy[i]*u[i]+fyt[i])));

				err += ((u[i]-tempUFlow)*(u[i]-tempUFlow));
				err += ((v[i]-tempVFlow)*(v[i]-tempVFlow));

				if (/*(tempUFlow*tempVFlow!=0.0)&&*/(v[i]*u[i]!=0.0))
				{
					if ((j>0)||(firstpass==false))
					{
						ratioU = (float)fabs(tempUFlow/u[i]);
						if ((ratioU <1.0f)&&(ratioU>0.0f))
							ratioU = 1.0f/ratioU;
						else if (ratioU==0.0f)
							ratioU = 100000.0f;

						ratioV = (float)fabs(tempVFlow/v[i]);
						if ((ratioV <1.0f)&&(ratioV>0.0f))
							ratioV = 1.0f/ratioV;
						else if (ratioV==0.0f)
							ratioV = 100000.0f;

						ratioU = ratioU-1.0f;
						ratioV = ratioV-1.0f;
						
						if(((ratioU+ratioV)<Msensitivity/1.0f)||(fabs(tempUFlow-u[i])<Asensitivity/1.0f)||(fabs(tempVFlow-v[i])<Asensitivity/1.0f))
							procMask[i] = false;
						else
						{
							procMask[i+1]=true;
							procMask[i+cartWid]=true;
							procMask[i-1]=true;
							procMask[i-cartWid]=true;
							procMask[i]=true;
						}
					}
				}
			}

			if (j==0)
				printf("CartMask:%d",pixCount);

			int xx,yy;
//			for (xx=0; xx<cartWid; xx++)
			for (xx=Xlim; xx<cartWid-Xlim; xx++)
			{
				u[(Ylim-1)*cartWid+xx+ImgSize] = u[(Ylim)*cartWid+xx+ImgSize];
				v[(Ylim-1)*cartWid+xx+ImgSize] = v[(Ylim)*cartWid+xx+ImgSize];
				u[(cartHei-Ylim)*cartWid+xx+ImgSize] = u[(cartHei-Ylim-1)*cartWid+xx+ImgSize];
				v[(cartHei-Ylim)*cartWid+xx+ImgSize] = v[(cartHei-Ylim-1)*cartWid+xx+ImgSize];
//				u[xx+ImgSize+0*cartWid] = u[cartWid+xx+ImgSize+0*cartWid];
//				v[xx+ImgSize+0*cartWid] = v[cartWid+xx+ImgSize+0*cartWid];
//				u[(cartHei-1-0)*cartWid+xx+ImgSize] = u[(cartHei-2-0)*cartWid+xx+ImgSize];
//				v[(cartHei-1-0)*cartWid+xx+ImgSize] = v[(cartHei-2-0)*cartWid+xx+ImgSize];
			}
			for (yy=Ylim; yy<cartHei-Ylim; yy++)
			{
				u[yy*cartWid+(Xlim-1)+ImgSize]=u[yy*cartWid+(Xlim)+ImgSize];
				v[yy*cartWid+(Xlim-1)+ImgSize]=v[yy*cartWid+(Xlim)+ImgSize];
				u[yy*cartWid+(cartWid-Xlim)+ImgSize]=u[yy*cartWid+(cartWid-Xlim-1)+ImgSize];
				v[yy*cartWid+(cartWid-Xlim)+ImgSize]=v[yy*cartWid+(cartWid-Xlim-1)+ImgSize];
//				u[yy*cartWid+ImgSize+0]=u[yy*cartWid+1+0+ImgSize];
//				v[yy*cartWid+ImgSize+0]=v[yy*cartWid+1+0+ImgSize];
//				u[(yy+1)*cartWid+ImgSize-1]=u[(yy+1)*cartWid-1-1+ImgSize];
//				v[(yy+1)*cartWid+ImgSize-1]=v[(yy+1)*cartWid-1-1+ImgSize];
			}
		

			if ((pixCount==0)&&(j>0))
				break;
			if ((err<0.0000001f))
				break;
		}


		printf("Fov:%d,%d px. Per:",j, pixCount);

		getLpFovea(u,u+ImgSize);
		getLpFovea(v,v+ImgSize);
		
		for(i=0; i<nAng; i++)
		{
			float tempX = (float)((firstRing/2.0f)*cos(2*i*PI/nAng)*scaleFactor);
			float tempY = (float)((firstRing/2.0f)*sin(2*i*PI/nAng)*scaleFactor);
			
			float xpdx, ypdy;
			
			xpdx = tempX+u[i];
			ypdy = tempY+v[i];
			
			getLPCoord(&temprho,&temptheta,xpdx,ypdy);

			deX[i] = u[i];
			deY[i] = v[i];
			u[i] = temptheta-i;
			if (u[i]>=nAng/2)
				u[i]-=nAng;
			if (u[i]<-nAng/2)
				u[i]+=nAng;

			v[i] = temprho;
		}
		for(i=nAng; i<fov*nAng; i++)
		{
			float tempX = (float)(((i/nAng) + 2*firstRing - 0.5f)*cos(2*(i%nAng)*PI/nAng)*scaleFactor);
			float tempY = (float)(((i/nAng) + 2*firstRing - 0.5f)*sin(2*(i%nAng)*PI/nAng)*scaleFactor);
			
			float xpdx, ypdy;
			
			xpdx = tempX+u[i];
			ypdy = tempY+v[i];
			
			getLPCoord(&temprho,&temptheta,xpdx,ypdy);
			
			deX[i] = u[i];
			deY[i] = v[i];
			u[i] = temptheta-(i%nAng);
			if (u[i]>=nAng/2)
				u[i]-=nAng;
			if (u[i]<-nAng/2)
				u[i]+=nAng;
			v[i] = temprho-(i/nAng);
		}		
		
		for(i=fov*nAng; i<lplimit*nAng; i++)
		{
			float tempX = (float)((pow (lambda, i/nAng) * (r0 + 0.5f / (float)pow (lambda, fov)))*cos(2*(i%nAng)*PI/nAng)*scaleFactor);
			float tempY = (float)((pow (lambda, i/nAng) * (r0 + 0.5f / (float)pow (lambda, fov)))*sin(2*(i%nAng)*PI/nAng)*scaleFactor);
//			float tempY = (float)(((i/nAng) + 2*firstRing - 0.5f)*sin(2*(i%nAng)*PI/nAng)*scaleFactor);
			
			float xpdx, ypdy;
			
			xpdx = tempX+u[i];
			ypdy = tempY+v[i];
			
			getLPCoord(&temprho,&temptheta,xpdx,ypdy);
			
			deX[i] = u[i];
			deY[i] = v[i];
			u[i] = temptheta-(i%nAng);
			if (u[i]>=nAng/2)
				u[i]-=nAng;
			if (u[i]<-nAng/2)
				u[i]+=nAng;
			v[i] = temprho-(i/nAng);
		}
	}
	
	if ((procmode == LP)||(procmode == HYB))
	{
		startPix = (lplimit)*nAng;
		endPix = ImgSize-Wid; 
//		endPix = (100+fov)*nAng; 
	}
	else
	{
		startPix = Wid+1;
		endPix = ImgSize-Wid-1; 
	}


	for (j=0; j<1000; j++)
	{
		err = 0.0f;
	
		pixCount = 0;

		for (i=startPix; i<endPix; i++)
		{
			if (procMask[i]==false)
				continue;

//			if(i==11988)
//				i=i;

			if (procmode==CART)
			{
				itX = i%Wid;
				itY = i/Wid;
				if ((itX==0)||(itX==Wid-1))
					continue;
			}
			else
			{
				itRho = i/Wid;
				itThe = i%Wid;
			}

			pixCount++;

			if (procmode==CART)
			{
				sumMu = u[i-1]+u[i-Wid];
				sumPu = u[i+1]+u[i+Wid];
				sumMv = v[i-1]+v[i-Wid];
				sumPv = v[i+1]+v[i+Wid];
			}
			else
			{
				if(itThe==0)
				{
					avgdX = deX[i-Wid]+deX[(i-1)+Wid];
					avgdY = deY[i-Wid]+deY[(i-1)+Wid];
				}
				else
				{
					avgdX = deX[i-Wid]+deX[i-1];
					avgdY = deY[i-Wid]+deY[i-1];
				}
				
				if(itThe==Wid-1)
				{
					avgdX += deX[i+Wid]+deX[(i+1)-Wid];
					avgdY += deY[i+Wid]+deY[(i+1)-Wid];

					avgdX /=4.0;
					avgdY /=4.0;
				}
				else
				{
					avgdX += deX[i+Wid]+deX[i+1];
					avgdY += deY[i+Wid]+deY[i+1];

					avgdX /=4.0;
					avgdY /=4.0;
				}

				float tempxpdx = X0eq[i]+avgdX;
				float tempypdy = Y0eq[i]+avgdY;
				
				getLPCoord(&temprho,&temptheta,tempxpdx,tempypdy);

				temptheta = 4*(temptheta-itThe);
				temprho   = 4*(temprho-itRho);
				
				if (temptheta>=nAng*4)
					temptheta-=nAng*4;
				if (temptheta<0.0)
					temptheta+=nAng*4;

				if (temptheta>=nAng*4)
					temptheta-=nAng*4;
				if (temptheta<0.0)
					temptheta+=nAng*4;

				if (temptheta>=nAng*2)
					temptheta-=nAng*4;
		
			}

			tempUFlow= u[i]; 
			tempVFlow= v[i]; 

			if (procmode!=CART)
			{
				u[i] = (float)((1.0f-omega)*u[i] + uDen[i]*(temptheta-(fxy[i]*v[i]+fxt[i])));
				
				if ((u[i]>=nAng/2))
					u[i]-=nAng;
				
				if ((u[i]<-nAng/2))
					u[i]+=nAng;

				v[i] = (float)((1.0f-omega)*v[i] + vDen[i]*(temprho-(fxy[i]*u[i]+fyt[i])));
			}
			else
			{
				u[i] = (float)((1.0f-omega)*u[i] + uDen[i]*(sumMu+sumPu-(fxy[i]*v[i]+fxt[i])));
				v[i] = (float)((1.0f-omega)*v[i] + vDen[i]*(sumMv+sumPv-(fxy[i]*u[i]+fyt[i])));
			}

			if (/*(tempUFlow*tempVFlow!=0.0)&&*/(v[i]*u[i]!=0.0))
			{
				if ((j>0)||(firstpass==false))
				{
					ratioU = (float)fabs(tempUFlow/u[i]);
					if ((ratioU <1.0f)&&(ratioU>0.0f))
						ratioU = 1.0f/ratioU;
					else if (ratioU==0.0f)
						ratioU = 100000.0f;
					
					ratioV = (float)fabs(tempVFlow/v[i]);
					if ((ratioV <1.0f)&&(ratioV>0.0f))
						ratioV = 1.0f/ratioV;
					else if (ratioV==0.0f)
						ratioV = 100000.0f;
					
					ratioU = ratioU-1.0f;
					ratioV = ratioV-1.0f;
					
					if(((ratioU+ratioV)<Msensitivity/1.0f)||(fabs(tempUFlow-u[i])<Asensitivity*1.0f)||(fabs(tempVFlow-v[i])<Asensitivity*1.0f))
						procMask[i] = false;
					else
					{
						if (procmode!=CART)
						{
							if(itThe==0)
							{
								procMask[i+1]=true;
								procMask[i+Wid]=true;
								procMask[i-1+Wid]=true;
								procMask[i-Wid]=true;
								procMask[i]=true;
							}
							else if(itThe==Wid-1)
							{
								procMask[i+1-Wid]=true;
								procMask[i+Wid]=true;
								procMask[i-1]=true;
								procMask[i-Wid]=true;
								procMask[i]=true;
							}
							else
							{
								procMask[i+1]=true;
								procMask[i+Wid]=true;
								procMask[i-1]=true;
								procMask[i-Wid]=true;
								procMask[i]=true;
							}
						}
						else
						{
							procMask[i+1]=true;
							procMask[i+Wid]=true;
							procMask[i-1]=true;
							procMask[i-Wid]=true;
							procMask[i]=true;
						}
					}
				}
			}

			if ((tempUFlow==0.0)&&(u[i]==0.0))
				procMask[i] = false;
			if ((tempVFlow==0.0)&&(v[i]==0.0))
				procMask[i] = false;

			err += ((u[i]-tempUFlow)*(u[i]-tempUFlow));
			err += ((v[i]-tempVFlow)*(v[i]-tempVFlow));
			
			if(procmode!=CART)
			{
				getCCoord(itRho+v[i],itThe+u[i],&(xpdx[i]),&(ypdy[i]));
				deX[i]=xpdx[i]-X0eq[i];
				deY[i]=ypdy[i]-Y0eq[i];
			}
//			if ((err<0.00001f))
//				break;
		}

//		if (j==0)
//			printf("LPMask:%d - ",pixCount);
		
		if (procmode == CART)
		{
			int xx,yy;
			for (xx=0; xx<Wid; xx++)
			{
				u[xx] = u[Wid+xx];
				u[(Hei-1)*Wid+xx] = u[(Hei-2)*Wid+xx];
				v[xx] = v[Wid+xx];
				v[(Hei-1)*Wid+xx] = v[(Hei-2)*Wid+xx];
			}
			for (yy=0; yy<Hei; yy++)
			{
				u[yy*Wid]=u[yy*Wid+1];
				u[(yy+1)*Wid]=u[(yy+1)*Wid-1];
				v[yy*Wid]=v[yy*Wid+1];
				v[(yy+1)*Wid]=v[(yy+1)*Wid-1];
			}
		}
		else
		{
			int xx;
			for (xx=endPix; xx<endPix+Wid; xx++)
			{
				deX[xx]=deX[xx-Wid];
				deY[xx]=deY[xx-Wid];

				xpdx[xx] = xpdx[xx-Wid]-X0eq[xx-Wid]+X0eq[xx];
				ypdy[xx] = ypdy[xx-Wid]-Y0eq[xx-Wid]+Y0eq[xx];
				
				getLPCoord(&temprho,&temptheta,xpdx[xx],ypdy[xx]);
				
				v[xx] = temprho-((endPix)/nAng);
				u[xx] = temptheta-(xx%nAng);

				if(u[xx]>=nAng)
					u[xx]-=nAng;
				
				if(u[xx]<0.0)
					u[xx]+=nAng;

				if(u[xx]>=nAng/2)
					u[xx] -=nAng;
			}
			
		if (procmode==LP)
				for (xx=startPix-Wid; xx<startPix; xx++)
				{
					deX[xx]=deX[xx+Wid];
					deY[xx]=deY[xx+Wid];
					
					xpdx[xx] = xpdx[xx+Wid]-X0eq[xx+Wid]+X0eq[xx];
					ypdy[xx] = ypdy[xx+Wid]-Y0eq[xx+Wid]+Y0eq[xx];
					
					getLPCoord(&temprho,&temptheta,xpdx[xx],ypdy[xx]);
					
					v[xx] = temprho-((startPix-Wid)/nAng);
					u[xx] = temptheta-(xx%nAng);
					
					if(u[xx]>=nAng)
						u[xx]-=nAng;
					
					if(u[xx]<0.0)
						u[xx]+=nAng;
					
					if(u[xx]>=nAng/2)
						u[xx] -=nAng;
				}
		}

		if ((pixCount==0)&&(j>0))
			break;
		if ((err<0.00001f))
			break;
	}
	printf("%d,%d px",j, pixCount);
}

/*
void oFlow::getModule(float *uflow, float *vflow, float **module)
{
	int i,j;
	
	if (*module==NULL)
		*module = new float[ImgSize];
	
	float *a = *module;
	
	if (procmode==CART)
	{
		for (j=0; j<ImgSize; j++)
			a[j] = (float)sqrt(uflow[j]*uflow[j]+vflow[j]*vflow[j]);
	}
	else
	{
		for (j=0; j<Hei; j++)
			for (i=0; i<Wid; i++)
				a[j*Wid+i] = (float)(pow(lambda,2*j)*(1.0+pow(lambda,2*vflow[j*Wid+i])-2*pow(lambda,vflow[j*Wid+i])*cos(2*PI*uflow[j*Wid+i]/Wid)));
	}
}
*/
long oFlow::getTime()
{
#if !defined(__QNX__) && !defined(__LINUX__)

	LARGE_INTEGER perfTime;
	LARGE_INTEGER perfFreq;
	long l_tempo;
	float d_tempo;
	time_t offset_ora;

        QueryPerformanceFrequency(&perfFreq);
		time(&offset_ora);//secs from Jan 1st, 1970
		QueryPerformanceCounter(&perfTime);
	    d_tempo = (float)perfTime.QuadPart/(float)perfFreq.QuadPart;//Elapsed Time (secs from boot)
		l_tempo = (long)(d_tempo * 1000.0);//(msecs from boot)

		return l_tempo;
#else
		return -1;
#endif    
}

void oFlow::getFileName (char * partFileName, char * fileName, int frameNumber)
{
	sprintf(fileName,"%s%03d.bmp",partFileName,frameNumber);	
}

void oFlow::HSV2RGB(unsigned char *outImg, float *Hue, float *Sat, float *Bri, float maxMod, unsigned char *outImgRem)
{
	
	int tempPixelR;
	int tempPixelG;
	int tempPixelB;
	
	int H;
	double Hu;
	double F,P,Q,T,V,S;
	int i,j;
		
	for (i=0; i<ImgSize; i++)
	{
		Hu = 360.0*(Hue[i]+PI)/(2*PI);
		H = ((int)(Hu/60.0))%6;
		F = (Hu/60.0)-H;
		V = (127.5f+(Bri[i]-127.5f)/4.0f)/255.0f;
		S = Sat[i]/maxMod;
		P = V*(1-S);
		Q = V*(1-F*S);
		T = V*(1-(1-F)*S);
			
		switch(H) {
		case 0:
			outImg[3*i+0] = (unsigned char)(V*255.0+0.5);
			outImg[3*i+1] = (unsigned char)(T*255.0+0.5);
			outImg[3*i+2] = (unsigned char)(P*255.0+0.5);
			break;
		case 1:
			outImg[3*i+0] = (unsigned char)(Q*255.0+0.5);
			outImg[3*i+1] = (unsigned char)(V*255.0+0.5);
			outImg[3*i+2] = (unsigned char)(P*255.0+0.5);
			break;
		case 2:
			outImg[3*i+0] = (unsigned char)(P*255.0+0.5);
			outImg[3*i+1] = (unsigned char)(V*255.0+0.5);
			outImg[3*i+2] = (unsigned char)(T*255.0+0.5);
			break;
		case 3:
			outImg[3*i+0] = (unsigned char)(P*255.0+0.5);
			outImg[3*i+1] = (unsigned char)(Q*255.0+0.5);
			outImg[3*i+2] = (unsigned char)(V*255.0+0.5);
			break;
		case 4:
			outImg[3*i+0] = (unsigned char)(T*255.0+0.5);
			outImg[3*i+1] = (unsigned char)(P*255.0+0.5);
			outImg[3*i+2] = (unsigned char)(V*255.0+0.5);
			break;
		case 5:
			outImg[3*i+0] = (unsigned char)(V*255.0+0.5);
			outImg[3*i+1] = (unsigned char)(P*255.0+0.5);
			outImg[3*i+2] = (unsigned char)(Q*255.0+0.5);
			break;
		}
	}

	//Remap
	if (procmode!=CART)
	{
		for (j=0; j<sX*sY; j++)
		{
			tempPixelR = 0;
			tempPixelG = 0;
			tempPixelB = 0;
			
			if (l2ctable[j].iweight != 0)
			{
				for (i = 0; i < l2ctable[j].iweight; i++)
				{
					tempPixelR += outImg[3*l2ctable[j].position[i]+0];
					tempPixelG += outImg[3*l2ctable[j].position[i]+1];
					tempPixelB += outImg[3*l2ctable[j].position[i]+2];
				}
				outImgRem[3*j+0] = tempPixelR / l2ctable[j].iweight;
				outImgRem[3*j+1] = tempPixelG / l2ctable[j].iweight;
				outImgRem[3*j+2] = tempPixelB / l2ctable[j].iweight;
			}
		}
	}
}


void oFlow::drawFlow(float * module, float * Cmodule, int frame, float *CdeX, float *CdeY)
{
	float tempPixel;
	float tempPixelX;
	float tempPixelY;

	int i,j;

	memset(Cmodule,0,sX*sY*sizeof(float));
	memset(CdeX,0,sX*sY*sizeof(float));
	memset(CdeY,0,sX*sY*sizeof(float));
		
	for (j = 0; j < sX*sY; j++)
	{
		tempPixel = 0;
		tempPixelX = 0;
		tempPixelY = 0;
		
		if (l2ctable[j].iweight != 0)
		{
			for (i = 0; i < l2ctable[j].iweight; i++)
			{
				tempPixel  += module[l2ctable[j].position[i]];
				tempPixelX += deX[l2ctable[j].position[i]];
				tempPixelY += deY[l2ctable[j].position[i]];
			}
			Cmodule[j] = tempPixel / l2ctable[j].iweight;
			CdeX[j] = tempPixelX / l2ctable[j].iweight;
			CdeY[j] = tempPixelY / l2ctable[j].iweight;
		}
	}

	float centerX = 0;
	float centerY = 0;

		
	float modsum = 0;
	for (j = 0; j < sY; j++)
		for (i = 0; i < sX; i++)
		{
			if(Cmodule[j*sX+i]>2.0)
			{
				centerX+=(Cmodule[j*sX+i]*(i-(sX/2)));
				centerY+=(Cmodule[j*sX+i]*(j-(sY/2)));
				modsum += Cmodule[j*sX+i];
			}
		}

	if (modsum>0.0)
	{
		centerX = centerX/(modsum);
		centerY = centerY/(modsum);
	}

	float avgdeX = 0;
	float dexsum = 0;
	float avgdeY = 0;
	float deysum = 0;
		
	for (j = 0; j < sY; j++)
		for (i = 0; i < sX; i++)
		{
			if(Cmodule[j*sX+i]>2.0)
			{
				avgdeX+=(float)((CdeX[j*sX+i])*fabs(1.0/(i-sX/2-centerX)));
				dexsum += (float)fabs(1.0/(i-sX/2-centerX));
				avgdeY+=(float)((CdeY[j*sX+i])*fabs(1.0/(i-sY/2-centerY)));
				deysum += (float)fabs(1.0/(i-sY/2-centerY));
			}
		}
			
	if (dexsum>0.0)
		avgdeX = avgdeX/(dexsum);
	
	if (deysum>0.0)
		avgdeY = avgdeY/(deysum);

	double avgmod = sqrt(avgdeX*avgdeX+avgdeY*avgdeY);
	double avgori = atan2(avgdeY,avgdeX);
		
//	printf("Avg Module : %f, Avg Orient: %f\n",avgmod,180*avgori/PI);

	memset(Cmodule,0,sX*sY*sizeof(float));
	
	for (j = 0; j < sX*sY; j++)
	{
		tempPixel = 0;
		
		if (l2ctable[j].iweight != 0)
		{
			for (i = 0; i < l2ctable[j].iweight; i++)
				tempPixel += GimageSet[(4+frame-2*((int)(3*prsigT+1)))%sizeGt][l2ctable[j].position[i]];
			Cmodule[j] = tempPixel / l2ctable[j].iweight;
		}
	}

		for (j = 0; j < sY; j++)
		{
			Cmodule[j*sX+(int)(centerX+0.5+sX/2)-1]=255.0f;
			Cmodule[j*sX+(int)(centerX+0.5+sX/2)]=0.0f;
			Cmodule[j*sX+(int)(centerX+0.5+sX/2)+1]=255.0f;
		}
		
		for (i = 0; i < sX; i++)
		{
			Cmodule[((int)(1+centerY+0.5+sY/2))*sX+i]=255.0f;
			Cmodule[((int)(centerY+0.5+sY/2))*sX+i]=0.0f;
			Cmodule[((int)(-1+centerY+0.5+sY/2))*sX+i]=255.0f;
		}

		int xcoord,ycoord;
		double ncx = centerX+cos(avgori+PI/2);
		double ncy = centerY+sin(avgori+PI/2);
		for (j = 0; j <60; j++)
		{
			xcoord = (int)(0.5+ncx+j*0.2*avgmod*cos(avgori)+sX/2);
			ycoord = (int)(0.5+ncy+j*0.2*avgmod*sin(avgori)+sY/2);
			if ((xcoord>=0)&&(ycoord>=0))
				if ((xcoord<sX)&&(ycoord<sY))
					Cmodule[ycoord*sX+xcoord] = 255.0f;
		}
		
		ncx = centerX-cos(avgori+PI/2);
		ncy = centerY-sin(avgori+PI/2);
		for (j = 0; j <60; j++)
		{
			xcoord = (int)(0.5+ncx+j*0.2*avgmod*cos(avgori)+sX/2);
			ycoord = (int)(0.5+ncy+j*0.2*avgmod*sin(avgori)+sY/2);
			if ((xcoord>=0)&&(ycoord>=0))
				if ((xcoord<sX)&&(ycoord<sY))
					Cmodule[sX+xcoord] = 255.0f;
		}
		
		for (j = 0; j <60; j++)
		{
			xcoord = (int)(0.5+centerX+j*0.2*avgmod*cos(avgori)+sX/2);
			ycoord = (int)(0.5+centerY+j*0.2*avgmod*sin(avgori)+sY/2);
			if ((xcoord>=0)&&(ycoord>=0))
				if ((xcoord<sX)&&(ycoord<sY))
					Cmodule[ycoord*sX+xcoord] = 0.0f;
		}
}

void oFlow::warpFlow()
{
	int i,j;
	float posX, posY;
	int posXp, posYp;
	int posXm, posYm;

	float w0x,w1x,w0y,w1y;

	float * warpedU = new float[hybISize];
	float * warpedV = new float[hybISize];

	memset(warpedU,0,hybISize*sizeof(float));
	memset(warpedV,0,hybISize*sizeof(float));

	float * counter = new float [hybISize];
	memset(counter,0,hybISize*sizeof(float));

	for (j=0; j<nEcc; j++)
	{
		for (i=0; i<nAng; i++)
		{
			posX = i+u[j*nAng+i];
			posY = j+v[j*nAng+i];
			w1x = posX-(int)(posX);
			w0x = 1.0f-w1x;
			w1y = posY-(int)(posY);
			w0y = 1.0f-w1y;

			posXm = (int)(posX);
			posXp = (int)(posX+1);
			posYm = (int)(posY);
			posYp = (int)(posY+1);

			if(posXm<0)
			{
				posXm = posXm+nAng;

				if(posXm<nAng-1)
					posXp = posXm+1;
				else
					posXp = 0;
			}

			if(posXp>nAng-1)
			{
				posXp = posXp-nAng;

				if(posXp>0)
					posXm = posXp-1;
				else
					posXm = nAng-1;
			}
			
			if(posY<0)
			{
				posXm = (posXm+nAng/2)%nAng;
				posXp = (posXp+nAng/2)%nAng;
			}

			warpedU[posYm*nAng+posXm]+=w0x*w0y*u[j*nAng+i];
			warpedU[posYm*nAng+posXp]+=w1x*w0y*u[j*nAng+i];
			warpedU[posYp*nAng+posXm]+=w0x*w1y*u[j*nAng+i];
			warpedU[posYp*nAng+posXp]+=w1x*w1y*u[j*nAng+i];
			
			warpedV[posYm*nAng+posXm]+=w0x*w0y*v[j*nAng+i];
			warpedV[posYm*nAng+posXp]+=w1x*w0y*v[j*nAng+i];
			warpedV[posYp*nAng+posXm]+=w0x*w1y*v[j*nAng+i];
			warpedV[posYp*nAng+posXp]+=w1x*w1y*v[j*nAng+i];

			counter[posYm*nAng+posXm]+=w0x*w0y;
			counter[posYm*nAng+posXp]+=w1x*w0y;
			counter[posYp*nAng+posXm]+=w0x*w1y;
			counter[posYp*nAng+posXp]+=w1x*w1y;
		}
	}

	for(i=0; i<ImgSize; i++)
	{
		if(counter[i]>0)
		{
			warpedV[i]/=counter[i];
			warpedU[i]/=counter[i];
		}
	}

//	memcpy(u,warpedU,ImgSize*sizeof(float));
//	memcpy(v,warpedV,ImgSize*sizeof(float));

	float *cartWU = warpedU+ImgSize;
	float *cartWV = warpedV+ImgSize;
	float *cartU  = u+ImgSize;
	float *cartV  = v+ImgSize;
	float *cartC  = counter+ImgSize;

	for (j=0; j<2*cartfov; j++)
		for (i=0; i<2*cartfov; i++)
		{
			posX = i+cartU[j*2*cartfov+i];
			posY = j+cartV[j*2*cartfov+i];
			w1x = posX-(int)(posX);
			w0x = 1.0f-w1x;
			w1y = posY-(int)(posY);
			w0y = 1.0f-w1y;
			
			posXm = (int)(posX);
			posXp = (int)(posX+1);
			posYm = (int)(posY);
			posYp = (int)(posY+1);
			
			if((posXm>=0)&&(posXp<2*cartfov)&&(posYm>=0)&&(posYp<2*cartfov))
			{
				cartWU[posYm*2*cartfov+posXm]+=w0x*w0y*cartU[j*2*cartfov+i];
				cartWU[posYm*2*cartfov+posXp]+=w1x*w0y*cartU[j*2*cartfov+i];
				cartWU[posYp*2*cartfov+posXm]+=w0x*w1y*cartU[j*2*cartfov+i];
				cartWU[posYp*2*cartfov+posXp]+=w1x*w1y*cartU[j*2*cartfov+i];
				
				cartWV[posYm*2*cartfov+posXm]+=w0x*w0y*cartV[j*2*cartfov+i];
				cartWV[posYm*2*cartfov+posXp]+=w1x*w0y*cartV[j*2*cartfov+i];
				cartWV[posYp*2*cartfov+posXm]+=w0x*w1y*cartV[j*2*cartfov+i];
				cartWV[posYp*2*cartfov+posXp]+=w1x*w1y*cartV[j*2*cartfov+i];
				
				cartC[posYm*2*cartfov+posXm]+=w0x*w0y;
				cartC[posYm*2*cartfov+posXp]+=w1x*w0y;
				cartC[posYp*2*cartfov+posXm]+=w0x*w1y;
				cartC[posYp*2*cartfov+posXp]+=w1x*w1y;
			}
		}

	for(i=0; i<hybISize-ImgSize; i++)
	{
		if(cartC[i]>0)
		{
			cartWU[i]/=cartC[i];
			cartWV[i]/=cartC[i];
		}
	}

	memcpy(u,warpedU,hybISize*sizeof(float));
	memcpy(v,warpedV,hybISize*sizeof(float));

	for (j=0; j<nEcc; j++)
	{
		for (i=0; i<nAng; i++)
		{
			if(procmode!=CART)
			{
				getCCoord(j+v[j*nAng+i],i+u[j*nAng+i],&(xpdx[j*nAng+i]),&(ypdy[j*nAng+i]));
				deX[j*nAng+i]=xpdx[j*nAng+i]-X0eq[j*nAng+i];
				deY[j*nAng+i]=ypdy[j*nAng+i]-Y0eq[j*nAng+i];
			}
		}
	}

	delete [] warpedU;
	delete [] warpedV;
	delete [] counter;

}

void oFlow::getModule(float *uflow, float *vflow)
{
	int i;

	if (module==NULL)
		module = new float[ImgSize];

	for (i=0; i<ImgSize;i++)	
		module[i] = (float)sqrt(uflow[i]*uflow[i]+vflow[i]*vflow[i]);
}

void oFlow::getOrientation(float *uflow, float *vflow)
{
	int i;
	
	if (orientation==NULL)
		orientation = new float[ImgSize];

	for (i=0; i<ImgSize;i++)	
		orientation[i] = (float)atan2(vflow[i],uflow[i]);
}	
