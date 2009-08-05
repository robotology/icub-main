#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include <iCub/optFlow.h>

void main (int argc, char *argv[])
{
    fprintf(stderr, "Starting of\n");

	char fileName[256];
	int ff = 479;  //starting frame
	
	float preSSigmaT = 1.770f;	//temporal gaussian
	float preSSigma  = 2.770f;	//spatial gaussian (presmoothing)
	float     Sigma  = 1.250f;  //spatial gaussian (postsmoothing)

	oFlow logp;
	
	logp.prsig = preSSigma;
	logp.posig = Sigma;
	logp.prsigT = preSSigmaT;
	logp.inputmode  = CART;
	logp.procmode = HYB;
	logp.XYTproc = false;
	logp.nEcc = 152;
	logp.nAng = 252;
	logp.sX = 640;
	logp.sY = 480;

	int frame;

	logp.sizeGxy = logp.TkSz;
	logp.sizeGt  = 0;

	if (logp.XYTproc)
	{
		logp.sizeGxy += logp.TkSz;
		logp.sizeGt  += logp.TkSz;
	}

	int noF = 2*((int)(3*logp.prsigT+1))+logp.sizeGt;

	if (!logp.XYTproc)
		noF = logp.TkSz;

	int index;

	for(frame=0;;frame++)
	{

		logp.getFileName(argv[1],fileName,ff+frame);
        fprintf(stderr, "Filename: %s\n", fileName);
		logp.loadFrame(fileName,logp.inputmode);

		if (logp.currentFrame==NULL)
			break;

        fprintf(stderr, "Frame read\n");

		logp.convertFrame(logp.inputmode,logp.procmode,logp.planes,frame%noF);

        fprintf(stderr, "Converted frame\n");

		logp.preSmooth(frame);

		if (frame==0)
			logp.buildDerKernel();

		if (frame<noF-1)
			continue;

		if (logp.XYTproc)
		{
			index = (1+frame-2*((int)(3*logp.prsigT+1)));
			
			logp.getDx(logp.GimageSet[(index+3)%logp.sizeGt], &logp.fx);
			logp.getDy(logp.GimageSet[(index+3)%logp.sizeGt], &logp.fy);
			logp.getDt(logp.GimageSet, &logp.ft,index%logp.sizeGt);
		}
		else
		{
			index = (1+frame);

			logp.getDx(logp.GimageSet[(index+3)%logp.sizeGxy], &logp.fx);
			logp.getDy(logp.GimageSet[(index+3)%logp.sizeGxy], &logp.fy);
			logp.getDt(logp.GimageSet, &logp.ft,index%logp.sizeGxy);
		}

		logp.crossProducts();
		logp.intPostSmooth();
		
		logp.computeFlow();

		if(logp.procmode==CART)
		{
			logp.getModule(logp.u,logp.v);
			logp.getOrientation(logp.u,logp.v);
		}
		else
		{
			logp.getModule(logp.deX,logp.deY);
			logp.getOrientation(logp.deX,logp.deY);
		}

		logp.warpFlow();
	}
}
