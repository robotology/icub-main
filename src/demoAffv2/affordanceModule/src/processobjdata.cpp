#include <stdio.h>
#include <math.h>

#include <iostream>
#include <algorithm>
#include <string.h>

using namespace std;


#include "iCub/processobjdata.h"

processobjdata::processobjdata(void)
{
	maxnsamples = 10;
	index = 0;
	npoints = 0;

	objposx = (double*)malloc(maxnsamples * sizeof(double));	
	objposy = (double*)malloc(maxnsamples * sizeof(double));
	objw = (double*)malloc(maxnsamples * sizeof(double));	
	objh = (double*)malloc(maxnsamples * sizeof(double));	
	objangle = (double*)malloc(maxnsamples * sizeof(double));	

	lastshape = -1;
	lastcolor = -1;
}

processobjdata::~processobjdata(void)
{
	delete objposx;
	delete objposy;
	delete objw;
	delete objh;
	delete objangle;
}

int processobjdata::classifycolor(double* colorhist)
{
double *maxptr;

	maxptr = max_element(colorhist,colorhist+16);

	return (maxptr-colorhist);
}

int processobjdata::classifyshape(BlobInfo &obj)
{
  if( (obj.eccentricity>0.7) && (obj.circleness>0.7))
  //if ( (obj.circleness>0.70))
		return BALL;
	else
		return BOX;
}

int processobjdata::classifysize(BlobInfo &obj)
{
  //	if( (obj.eccentricity>0.75) && (obj.circleness>0.65))
  if ( (obj.area>0.60))
    return 0;
  else if ( (obj.area>1.0))
    return 1;
  else
    return 2;
}


int processobjdata::printhist(double* hist )
{
	for(int cnt=0;cnt<16;cnt++)
		printf("%1.3g ",hist[cnt]);

	printf("\n");

	return 0;
}

int processobjdata::getlastshape()
{
  return lastshape;
}

int processobjdata::getlastcolor()
{
  return lastcolor;
}

int processobjdata::setlastshape(int shape)
{
  lastshape = shape;
  return 0;
}

int processobjdata::setlastcolor(int color)
{
  lastcolor = color;
  return 0;
}

//add data point for computing effects
int processobjdata::addDataPoint(double* dataPoint)
{

	objposx[index]=dataPoint[0];	
	objposy[index]=dataPoint[1];
	objw[index]=dataPoint[2];
	objh[index]=dataPoint[3];
	objangle[index]=dataPoint[4];

	index++;
	if(index==maxnsamples)
		index = 0;

	npoints++;
	if(npoints>maxnsamples)
		npoints = maxnsamples;

	return npoints;
}



int processobjdata::restartDataAcquisition(void)
{
	index = 0;
	npoints = 0;
	
	deleteBuffers();

	lastcolor = -1;
	lastshape = -1;

	return 0;
}

int processobjdata::detectActionInit(void)
{
double aveh=0;
double avew=0;
double avex=0;
double avey=0;

 printf("npoints %d, index %d\n",npoints, index);
	if(npoints<=1)
		return 0;

	for(int cnt=0;cnt<npoints;cnt++)
	{
		int circularindex = cnt + index - npoints;
		if(circularindex<0)
			circularindex = circularindex + maxnsamples;
		else if(circularindex>=maxnsamples)
			circularindex = circularindex - maxnsamples;

//		printf("circularindex %d\n",circularindex);

		avew = (cnt*avew + objw[circularindex])/(cnt+1);
		aveh = (cnt*aveh + objh[circularindex])/(cnt+1);
		avex = (cnt*avex + objposx[circularindex])/(cnt+1);
		avey = (cnt*avey + objposy[circularindex])/(cnt+1);

		//printf("%g %g %g %g\n",avew,objw[circularindex],aveh,objh[circularindex]);
		if( (fabs(avew-objw[circularindex]) > (0.1*objw[circularindex]) ) ||
		    (fabs(aveh-objh[circularindex]) > (0.1*objh[circularindex]) ))
		  printf("Shape changed\n");
		if(    (fabs(avex-objposx[circularindex]) > (0.1) ) ||
		    (fabs(avey-objposy[circularindex]) > (0.1) )  )
		  printf("Motion detected\n");
		
		if( (fabs(avew-objw[circularindex]) > (0.1*objw[circularindex]) ) ||
                    (fabs(aveh-objh[circularindex]) > (0.1*objh[circularindex]) ) ||
		    (fabs(avex-objposx[circularindex]) > (0.1) ) ||
                    (fabs(avey-objposy[circularindex]) > (0.1) )  )
		  return 1;
	}
	return 0;
}

int processobjdata::deleteBuffers(void)
{
	memset ( objposx, 0, maxnsamples*sizeof(double) );
	memset ( objposy, 0, maxnsamples*sizeof(double) );
	memset ( objw, 0, maxnsamples*sizeof(double) );
	memset ( objh, 0, maxnsamples*sizeof(double) );
	memset ( objangle, 0, maxnsamples*sizeof(double) );

	return 0;
}

int processobjdata::bufferfull(void)
{
	return (npoints==maxnsamples);
}

// having the data gathered, compute the effects
int processobjdata::computeEffects(double* effects)
{
double avex=0;
double avey=0;
double inclx = 0;
double incly = 0;

	if(npoints==0)
		return -1;

	for(int cnt=0;cnt<npoints;cnt++)
	{
		avex += objposx[cnt];
		avey += objposy[cnt];
	}

	avex = avex / npoints;
	avey = avey / npoints;
	
	for(int cnt=0;cnt<npoints;cnt++)
	{
		inclx += fabs(objposx[cnt]-avex);
		incly += fabs(objposy[cnt]-avey);
	}

	effects[0] = inclx / npoints;
	effects[1] = incly / npoints;

	return 0;
}

int processobjdata::classifyeffects(double *effects, unsigned char *effectclassification)
{
	effectclassification[0] = (effects[0]>0.1);
	effectclassification[1] = (effects[1]>0.1);

	return 0;
}
