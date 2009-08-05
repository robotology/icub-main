#include <stdio.h>
#include "public.h"
#include "basicOpenCV.h"
#include "colordetect.h"
#include "gmmTracker.h"

GmmTracker::GmmTracker(CvSize size, u32 kernelCnt)
{
	// GMM initialization
	u32 xSize = size.width;
	u32 ySize = size.height;
	kernels = kernelCnt;
	ws = new f64[1];
	ws[0] = 0.1;
	DB = new Database(xSize, ySize, 255);
	mix = new CurvedGaussMixture(xSize, ySize, DB, kernels);
	emLimit = 10;
	age = 0;
}

void GmmTracker::Apply(IplImage *image)
{
	DB->Process(image);
	//mix->randomKernel();
	age++;
	FOR(i,emLimit){
		mix->EM(ws);
	}
}

void GmmTracker::Randomize()
{
	mix->randomKernel();
	age = 0;
}

void GmmTracker::DrawGmm(IplImage *image)
{
	mix->paint(image);
}

f32 GmmTracker::GetLikelihood()
{
	return (f32)mix->likelihood();
}
