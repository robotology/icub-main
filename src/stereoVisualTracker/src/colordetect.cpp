#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>
#include "public.h"
#include "basicOpenCV.h"
#include "colordetect.h"

using namespace std;

#define TRACK_MIN_HEIGHT 5
#define TRACK_MIN_WIDTH 5
#define ADAPTIVE_TIMESPAN 100
#define ADAPTIVE_ALPHA 0.9
#define BINS 64

u32 bHalt = 0;
IplImage *GetHistBackProjResponse(IplImage *img1,IplImage *img2, CvHistogram **multiHist, float *weights, int nbHist);
IplImage *GetHistBackProjResponse(IplImage *img1,IplImage *img2, IplImage *img3, CvHistogram **multiHist, float *weights, int nbHist);
void ShowHistBackproj(IplImage *img1, IplImage *img2, CvHistogram **multiHist, u32 nbHist);
IplImage *GetHistBackproj(IplImage *img1, IplImage *img2, CvHistogram *hist);
IplImage *GetHistBackproj(IplImage *img1, IplImage *img2, IplImage *img3, CvHistogram *hist);

ColorDetect::ColorDetect()
{
	mode = 0;
	singleMode = 1;
	bActive = false;
	sigma = 1.f;
	mask = NULL;
	resp = NULL;
	ycbcr_values = cvScalarAll(0);
	hsv_values = cvScalarAll(0);
	bDrawAdd = 1;
	histCbcr = NULL;
	histHsv = NULL;
	addHistCbcr = NULL;
	addHistHsv = NULL;
	hist1 = NULL;
	hist2 = NULL;
	hist3 = NULL;
	multiHist = new CvHistogram*[1];
	multiHist[0] = NULL;
	singleResp = new IplImage*[1];
	singleResp[0] = NULL;
	FOR(i,9)weights[i]=1;
	addHistCnt = 0;
	currentHist = 0;
	nbHist = 1;
	prevNbHist = 0;
	editing = 0;
}

ColorDetect::~ColorDetect()
{
	IMKILL(mask);
}

void ColorDetect::SetColor(IplImage *image, CvRect selection, IplImage *msk)
{
	if(!image) return;
	if(histCbcr){
		cvReleaseHist(&histCbcr);histCbcr = NULL;
	}
	IplImage *ycrcb = cvCreateImage(cvGetSize(image), 8, 3);
	IplImage *cr = cvCreateImage(cvGetSize(image), 8, 1);
	IplImage *cb = cvCreateImage(cvGetSize(image), 8, 1);
	IplImage *tmp = cvCreateImage(cvGetSize(image), 8, 1);
	tmp->origin = cb->origin = cr->origin = ycrcb->origin = image->origin;

	cvCvtColor( image, ycrcb, CV_BGR2YCrCb );
	cvSplit( ycrcb, tmp, cr, cb, 0 );

	CvScalar crmean, cbmean;
	CvScalar crsdv, cbsdv;
	ROI(cr, selection);
	ROI(cb, selection);
	ROI(tmp, selection);
	if(msk)ROI(msk, selection);
	cvAvgSdv(cr, &crmean, &crsdv, msk);
	cvAvgSdv(cb, &cbmean, &cbsdv, msk);
	ycbcr_values = cvScalar(crmean.val[0],crsdv.val[0],cbmean.val[0],cbsdv.val[0]);

	histCbcr = getHist(tmp, cb, cr,selection,msk);
	//histCbcr = getHist(cb,cr,selection,msk);
	if(!addHistCnt){
		if(addHistCbcr){
			cvReleaseHist(&addHistCbcr);addHistCbcr=NULL;
		}
		addHistCbcr = getHist(tmp, cb, cr,selection,msk);
		//addHistCbcr = getHist(cb,cr,selection,msk);
	}
	else{
		SumHist(histCbcr, addHistCbcr);
	}
	addHistCnt++;
	if(msk)unROI(msk);
	unROI(tmp);
	unROI(cr);
	unROI(cb);

	//DrawHist(addHistCbcr);
	//IplImage* backProject = GetHistBackproj(cb,cr,histCbcr);

	if (nbHist == 1 && currentHist == 0)
	{
		printf("nbHist: %d\ncurrentHist: %d\n",nbHist,currentHist);
		multiHist[0] = addHistCbcr;
	}
	else
	{
		AddHist(addHistCbcr,currentHist);
	}
	//ShowHistBackproj(cb,cr,multiHist,nbHist, bVerbose);
	//fullResp = GetHistBackProjResponse(cb,cr,multiHist,weights,nbHist);
	//IMKILL(fullResp);

	IMKILL(ycrcb);
	IMKILL(cr);
	IMKILL(cb);
	IMKILL(tmp);
	IMKILL(resp);

}

void ColorDetect::Apply(IplImage *image)
{
	if(!image) return;
	if( !mask){
		mask = cvCreateImage(cvGetSize(image),image->depth, 1);
		mask->origin = image->origin;
		cvZero(mask);
	}

	//mode: 0=ycrcb, 2=hsv
	if (mode == 0){
		IplImage *ycrcb = cvCreateImage(cvGetSize(image), 8, 3);
		IplImage *cr = cvCreateImage(cvGetSize(image), 8, 1);
		IplImage *cb = cvCreateImage(cvGetSize(image), 8, 1);
		IplImage *tmp= cvCreateImage(cvGetSize(image), 8, 1);
		IplImage *crmsk = cvCreateImage(cvGetSize(image), 8, 1);
		IplImage *cbmsk = cvCreateImage(cvGetSize(image), 8, 1);
		ycrcb->origin = cr->origin = cb->origin = tmp->origin = crmsk->origin = cbmsk->origin = image->origin;
		cvCvtColor( image, ycrcb, CV_BGR2YCrCb );
		cvSplit( ycrcb, tmp, cr, cb, 0 );

		if (multiHist[0] != NULL)
		{
			if(!singleMode)
			{
				if(multiHist[0]->mat.dims == 2)
					fullResp = GetHistBackProjResponse(cb,cr,multiHist,weights,nbHist);
				else
					fullResp = GetHistBackProjResponse(tmp, cb, cr,multiHist,weights,nbHist);
				if(bVerbose){
					cvNamedWindow("APPLY totalresp");
					cvShowImage("APPLY totalresp",fullResp);
				}
				cvCopy(fullResp, mask);
			}
			else
			{
				if(nbHist != prevNbHist)
				{
					FOR(i, prevNbHist) IMKILL(singleResp[i]);
					KILL(singleResp);
					singleResp = new IplImage*[nbHist];
					FOR(i,nbHist) singleResp[i]=NULL;
					prevNbHist = nbHist;
				}
				for(int i=0; i<nbHist; i++)
				{
					//cvReleaseImage(&singleResp[i]);
					IMKILL(singleResp[i]);
					if(multiHist[i]->mat.dims == 2)
						singleResp[i] = GetHistBackproj(cb,cr,multiHist[i]);
					else
						singleResp[i] = GetHistBackproj(tmp, cb, cr,multiHist[i]);

					if(bVerbose){
						char wName[10] = "";
						sprintf(wName,"hist%dresp",i+1);
						cvNamedWindow(wName);
						cvShowImage(wName,singleResp[i]);

						//cvMorphologyEx(singleResp[i], singleResp[i], 0, 0, CV_MOP_CLOSE,2);
						//cvDilate(singleResp[i], singleResp[i],0,1);
						//cvErode(singleResp[i], singleResp[i],0,4);
						//cvNamedWindow("eroded");
						//cvShowImage("eroded",singleResp[i]);
					}
				}
				cvCopy(singleResp[min(currentHist,nbHist-1)], mask);
			}
			bActive = true;
		}

		/* old stuff to compute a gaussian around the selected colors
		f32 minCr = (f32)ycbcr_values.val[0] - sigma*(f32)ycbcr_values.val[1];
		f32 minCb = (f32)ycbcr_values.val[2] - sigma*(f32)ycbcr_values.val[3];
		f32 maxCr = (f32)ycbcr_values.val[0] + sigma*(f32)ycbcr_values.val[1];
		f32 maxCb = (f32)ycbcr_values.val[2] + sigma*(f32)ycbcr_values.val[3];
		cvInRangeS( cr, cvScalar(minCr,0,0,0),cvScalar(maxCr,0,0,0), crmsk );
		cvInRangeS( cb, cvScalar(minCb,0,0,0),cvScalar(maxCb,0,0,0), cbmsk );
		cvAnd(crmsk, cbmsk, crmsk);
		cvCopy(crmsk,mask);
		*/
		IMKILL(ycrcb);
		IMKILL(cr);
		IMKILL(cb);
		IMKILL(tmp);
		IMKILL(crmsk);
		IMKILL(cbmsk);
	}
	else if (mode == 2)
	{
		IplImage *hsv = cvCreateImage(cvGetSize(image), 8, 3);
		IplImage *s = cvCreateImage(cvGetSize(image), 8, 1);
		IplImage *v = cvCreateImage(cvGetSize(image), 8, 1);
		IplImage *tmp= cvCreateImage(cvGetSize(image), 8, 1);
		IplImage *smsk = cvCreateImage(cvGetSize(image), 8, 1);
		IplImage *vmsk = cvCreateImage(cvGetSize(image), 8, 1);
		hsv->origin = s->origin = v->origin = tmp->origin = smsk->origin = vmsk->origin = image->origin;
		cvCvtColor( image, hsv, CV_BGR2HSV );
		cvSplit( hsv, tmp, s, v, 0 );

		if (multiHist[0] != NULL)
		{
			fullResp = GetHistBackProjResponse(s,v,multiHist,weights,nbHist);
			bActive = true;
		}

		f32 minS = (f32)hsv_values.val[0] - sigma*(f32)hsv_values.val[1];
		f32 minV = (f32)hsv_values.val[2] - sigma*(f32)hsv_values.val[3];
		f32 maxS = (f32)hsv_values.val[0] + sigma*(f32)hsv_values.val[1];
		f32 maxV = (f32)hsv_values.val[2] + sigma*(f32)hsv_values.val[3];
		cvInRangeS( s, cvScalar(minS,0,0,0),cvScalar(maxS,0,0,0), smsk );
		cvInRangeS( v, cvScalar(minV,0,0,0),cvScalar(maxV,0,0,0), vmsk );
		cvAnd(smsk, vmsk, smsk);
		cvCopy(smsk,mask);

		IMKILL(hsv);
		IMKILL(s);
		IMKILL(v);
		IMKILL(tmp);
		IMKILL(smsk);
		IMKILL(vmsk);
	}
	cvMorphologyEx(mask, mask, 0, 0, CV_MOP_CLOSE,2);
	FOR(x, mask->width)
	{
		rgb(mask, x) = 0; // zeroes the edges
		rgb(mask, (mask->height-1)*mask->width + x) = 0;
	}
	FOR(y, mask->height)
	{
		rgb(mask, y*mask->width) = 0; // zeroes the edges
		rgb(mask, y*mask->width + mask->width-1) = 0;
	}

	//cvDilate(mask, mask,0,1);
	//cvErode(mask, mask,0,2);

	density = f32(cvSum(mask).val[0])/(mask->width*mask->height*(1<<mask->depth));
	mask->origin = image->origin;
}

IplImage *ColorDetect::Process(IplImage *image)
{
	Apply(image);
	return GetMask();
}

void ColorDetect::DrawMask(IplImage *image)
{
	if(bVerbose){
		cvNamedWindow("asd");
		cvShowImage("asd",mask); 
	}
	if(bDrawAdd){
		if(image->nChannels==3){
			FOR(i, image->width*image->height)
			{
				float mult = 1 + (float)(u8)rgb(mask,i) / 255.f;
				FOR(c,3){
					float pixel = float((u8)rgb(image,i*3+c))*(mult);
					rgb(image,i*3+c) = (u8)min(255,(pixel));
				}
			}
		}
		else
		{
			FOR(i, image->width*image->height)
			{
				float mult = 1.f + (float)(u8)rgb(mask,i) / 255.f;
				rgb(image,i) = (u8)((u8)rgb(image,i)*(mult));
			}
		}
		cvAdd(image, image, image, mask);
	}
	else{
		IplImage *img = cvCreateImage(cvGetSize(image),image->depth,image->nChannels);
		img->origin = image->origin;
		cvZero(img);
		cvAdd(img, image, img, mask);
		cvCopy(img, image);
		IMKILL(img);
	}
}

void ColorDetect::Save(const char *filename)
{
	std::ofstream file;
	file.open(filename);

	f64 *v = ycbcr_values.val;
	file << v[0] << " " << v[1] << " " << v[2] << " " << v[3]<< std::endl;
	file << sigma << " " << mode << std::endl;
	file.close();

	char *cfgName = new char[255];
	sprintf(cfgName, "%s.cfg", filename);
	std::ofstream cfgFile;
	cfgFile.open(cfgName);
	/*
	cfgFile << "# Histograms configuration file" << std::endl;
	cfgFile << "# Weight factor is saved with a default value that you can change manually if needed" << std::endl;
	cfgFile << "# Structure: <histCount>\n <weight> <histogram name>" << std::endl;
	*/
	cfgFile << nbHist <<std::endl;

	for(int i=0; i<nbHist; i++)
	{
		char *histName = new char[255];
		sprintf(histName, "%s%d.hist", filename, i);
		cvSave(histName,multiHist[i]);
		cfgFile << "1.0 " << histName << std::endl;
		delete [] histName;
	}
	cfgFile.close();
}

int ColorDetect::Load(const char *filename)
{
	std::ifstream file;
	file.open(filename);	
	if(!file.is_open()){
	  return 0;
	}
	f64 v[4];
	file >> v[0];
	file >> v[1];
	file >> v[2];
	file >> v[3];
	ycbcr_values = cvScalar(v[0],v[1],v[2],v[3]);
	file >> sigma;
	file >> mode;
	file.close();

	char *cfgName = new char[255];
	sprintf(cfgName, "%s.cfg", filename);
	std::ifstream cfgFile;
	cfgFile.open(cfgName);
	cfgFile >> nbHist;

	KILL(multiHist);
	multiHist = new CvHistogram*[nbHist+1];

	for(int i=0; i<nbHist; i++)
	{
		cfgFile >> weights[i];
		char *histName = new char[255];
		cfgFile >> histName;
		multiHist[i] = (CvHistogram *)cvLoad(histName);
		delete [] histName;
	}
	
	cfgFile.close();
	multiHist[nbHist] = NULL;
	weights[nbHist] = 1.0;
	currentHist = nbHist-1;
	return 1;
}

CvHistogram *ColorDetect::getHist(IplImage* cbImg, IplImage* crImg, CvRect rect, IplImage *msk)
{
	CvHistogram* hist = 0;
	int cteHistDim    = BINS;
	int dims[]        = {cteHistDim, cteHistDim};
	float cbRanges[]  = {0, 255};
	float crRanges[]  = {0, 255};
	float* ranges[]   = {cbRanges, crRanges};

	hist = cvCreateHist(2, dims, CV_HIST_ARRAY, ranges, CV_HIST_UNIFORM);

	IplImage* imgs[] = {cbImg, crImg};

	cvCalcHist((IplImage **)imgs, hist, 0, msk);

	// Normalize the histogram,
	// since the number of value depends on rect size.
	// Note that we choose a big factor to avoid having too small value
	cvNormalizeHist(hist, 10000);

	return(hist);
}

CvHistogram *ColorDetect::getHist(IplImage* img1, IplImage* img2, IplImage* img3, CvRect rect, IplImage *msk)
{
	CvHistogram* hist = 0;
	int cteHistDim    = BINS;
	int dims[]        = {cteHistDim, cteHistDim, cteHistDim};
	float ranges1[]  = {10, 250};
	float ranges2[]  = {0, 255};
	float ranges3[]  = {0, 255};
	float* ranges[]   = {ranges1, ranges2, ranges3};

	hist = cvCreateHist(3, dims, CV_HIST_ARRAY, ranges, CV_HIST_UNIFORM);

	IplImage* imgs[] = {img1, img2, img3};

	cvCalcHist((IplImage **)imgs, hist, 0, msk);

	// Normalize the histogram,
	// since the number of value depends on rect size.
	// Note that we choose a big factor to avoid having too small value
	cvNormalizeHist(hist, 10000);

	return(hist);
}

void ColorDetect::DrawHist(CvHistogram *hist)
{
	CvMat mat;
	cvGetMat(hist->bins, &mat, 0, 1);
	u32 size = BINS;
	IplImage *img = cvCreateImage(cvSize(size,size), 8, 1);
	cvZero(img);
	float* pHist = mat.data.fl;
	FOR(i, size*size){
		rgb(img,i) = cvRound(pHist[i]);
	}
	cvNamedWindow("histogram",0);
	cvResizeWindow("histogram", img->width, img->height); // hack to avoid linux installs of opencv crashing
	cvShowImage("histogram", img);
	cvReleaseImage(&img);img = NULL;
}

void ColorDetect::DrawHists()
{
	for (int i=0; i<nbHist; i++)
	{
		CvMat mat;
		cvGetMat(multiHist[i]->bins, &mat, 0, 1);
		u32 size = BINS;
		IplImage *histImg = cvCreateImage(cvSize(size,size), 8, 1);
		cvZero(histImg);
		float* pHist = mat.data.fl;
		FOR(j, size*size){
			rgb(histImg,j) = cvRound(pHist[j]);
		}

		char wName[6] = "";
		sprintf(wName,"hist%d",i+1);
		cvNamedWindow(wName,0);
		cvShowImage(wName,histImg);
		cvReleaseImage(&histImg);histImg = NULL;
	}
}

void ColorDetect::SumHist(CvHistogram *a, CvHistogram *b)
{
	// Delete value at (0,0) since it means
	// the camera doesn't found any color information.
	CvMat mat;
	cvGetMat(a->bins, &mat, 0, 1);
	mat.data.fl[0] = 0.0;

	// Normalize the histogram,
	// since the number of value depends on rect size.
	// Note that we choose a big factor to avoid having too small values
	cvNormalizeHist(a, 10000);

	CvMat accMat;
	cvGetMat(b->bins, &accMat, 0, 1);
	cvAdd(&mat, &accMat, &accMat);
}

void ColorDetect::SelectHist(int index)
{
	if (index >= 0 && index < nbHist)
	{
		currentHist = index;
		histCbcr = NULL;
		histHsv = NULL;
		addHistCbcr = NULL;
		addHistHsv = NULL;
		addHistCnt = 0;
		editing = 0;
	} else {
		currentHist = index;
		histCbcr = NULL;
		histHsv = NULL;
		addHistCbcr = NULL;
		addHistHsv = NULL;
		addHistCnt = 0;
		editing = 0;
	}
}

void ColorDetect::AddHist(CvHistogram *hist, int pos)
{
	if(nbHist == 1 && pos > 0) singleMode = 0;
	if (pos >= nbHist)
	{
		CvHistogram **tempHist = new CvHistogram *[nbHist+1];
		for (int i = 0; i < nbHist; i++) tempHist[i] = multiHist[i];
		delete[] multiHist;
		multiHist = tempHist;
		multiHist[nbHist] = hist;
		nbHist++;
		editing = 1;
		//printf("added hist at pos %d\n",nbHist-1);
	}
	else if(!editing && pos < nbHist)
	{
		if(multiHist){
			if(multiHist[pos]) cvReleaseHist(&multiHist[pos]);
			multiHist[pos] = NULL;
			multiHist[pos] = hist;
			editing = 1;
			//printf("reinit hist at pos %d\n",pos-1);
		}
	}
}

/*!
	GetHistBackproj: calculates the response of the histogram backprojection for a single histogram
	\param img1 first plane of the image (cr, s, rNcc, ...)
	\param img2 second plane of the image (cb, v, gNcc, ...)
	\param hist histogram to analyze
*/


IplImage *GetHistBackproj(IplImage *img1, IplImage *img2, CvHistogram *hist)
{
	IplImage* backProject = cvCreateImage(cvGetSize(img1), 8, 1);
	IplImage* imgs[] = {img1, img2};

	cvCalcBackProject(imgs, backProject, hist);
	return backProject;

	IplImage* response = cvCreateImage(cvGetSize(img1), 8, 1);
	cvZero(response);
	response->origin = backProject->origin = img1->origin;
	
	IplImage *integral = 0;
	CV::integralImage(backProject,&integral);

	u32 minSearchSize = 10;
	f32 wFactor = 1.5;
	f32 theMax = 0;

	for(u32 w = minSearchSize; w >= minSearchSize; w = (u32)(w/wFactor))
	//for(u32 w = min(img1->width,img1->height)-2; w > minSearchSize; w = (u32)(w/wFactor))
	{
		u32 bMaxed = 0;

		f32 wRath = 1.f/(w*w);
		for (u32 j = 0; j<img1->height-w-1; j += w/5)
		{
			for (u32 i = 0; i<img1->width-w-1; i += w/5)
			{
				CvRect r = cvRect(i,j,w,w);
				f32 sum = CV::GetSum(integral,r)*wRath;

				if(theMax < sum){
					theMax = sum;
					bMaxed = 1;
				}
				r = cvRect(i+w/2,j+w/2,w/5,w/5);
				ROI(response,r);
				cvZero(response);
				cvAddS(response, cvScalarAll(sum),response);
				unROI(response);
			}
		}
		//cvNamedWindow("response");
		//cvShowImage("response",response);
		//if(cvWaitKey(40) == 27)break;
		if(!bMaxed) break;
	}

	IMKILL(integral);
	IMKILL(backProject);

	return response;
}

IplImage *GetHistBackproj(IplImage *img1, IplImage *img2, IplImage *img3, CvHistogram *hist)
{
	IplImage* backProject = cvCreateImage(cvGetSize(img1), 8, 1);
	IplImage* imgs[] = {img1, img2, img3};

	cvCalcBackProject(imgs, backProject, hist);
	return backProject;

	IplImage* response = cvCreateImage(cvGetSize(img1), 8, 1);
	cvZero(response);
	response->origin = backProject->origin = img1->origin;
	
	IplImage *integral = 0;
	CV::integralImage(backProject,&integral);

	u32 minSearchSize = 10;
	f32 wFactor = 1.5;
	f32 theMax = 0;

	for(u32 w = minSearchSize; w >= minSearchSize; w = (u32)(w/wFactor))
	//for(u32 w = min(img1->width,img1->height)-2; w > minSearchSize; w = (u32)(w/wFactor))
	{
		u32 bMaxed = 0;

		f32 wRath = 1.f/(w*w);
		for (u32 j = 0; j<img1->height-w-1; j += w/5)
		{
			for (u32 i = 0; i<img1->width-w-1; i += w/5)
			{
				CvRect r = cvRect(i,j,w,w);
				f32 sum = CV::GetSum(integral,r)*wRath;

				if(theMax < sum){
					theMax = sum;
					bMaxed = 1;
				}
				r = cvRect(i+w/2,j+w/2,w/5,w/5);
				ROI(response,r);
				cvZero(response);
				cvAddS(response, cvScalarAll(sum),response);
				unROI(response);
			}
		}
		//cvNamedWindow("response");
		//cvShowImage("response",response);
		//if(cvWaitKey(40) == 27)break;
		if(!bMaxed) break;
	}

	IMKILL(integral);
	IMKILL(backProject);

	return response;
}

/*!
	ShowHistBackproj: shows the response of the histogram backprojection for a single histogram
	\param img1 first plane of the image (cr, s, rNcc, ...)
	\param img2 second plane of the image (cb, v, gNcc, ...)
	\param multiHist pointer structure containing the histograms to analyze
	\param nbHist the number of histograms to analyze
*/
void ShowHistBackproj(IplImage *img1, IplImage *img2, CvHistogram **multiHist, u32 nbHist)
{
	bool bShow = true;
	//init
	IplImage *imgs[] = {img1, img2};
	IplImage **backProjects = new IplImage*[nbHist];
	IplImage **responses = new IplImage*[nbHist];
	IplImage **integrals = new IplImage*[nbHist];
	f32 *sums = new f32[nbHist];
	f32 *maximums = new f32[nbHist];
	u32 *bMaxes = new u32[nbHist];
	u32 bMax = 1;
	FOR(i,nbHist)
	{
		backProjects[i] = cvCreateImage(cvGetSize(img1), 8, 1);
		cvCalcBackProject(imgs, backProjects[i], multiHist[i]);
		responses[i] = cvCreateImage(cvGetSize(img1), 8, 1);
		responses[i]->origin = backProjects[i]->origin = img1->origin;
		cvZero(responses[i]);
		maximums[i] = 0;
	}

	u32 minSearchSize = 20;
	f32 wFactor = 1.5;

	FOR(i, nbHist)
	{
		integrals[i] = 0;
		CV::integralImage(backProjects[i],&integrals[i]);
		bMaxes[i] = 0;
	}

	//search each square inside the image, bigger to smaller
	//for(u32 w = minSearchSize; w >= minSearchSize; w = (u32)(w/wFactor))
	for(u32 w = min(img1->width,img1->height)-2; w > minSearchSize; w = (u32)(w/wFactor))
	{
		u32 bMaxed = 0;
		FOR(i,nbHist) bMaxes[i] = 0;
		f32 wRath = 1.f/(w*w);
		
		//move the square on the image
		for (u32 j = 0; j<img1->height-w; j=j+(w/5))
		{
			if (j > img1->height-w) continue;
			for (u32 i = 0; i<img1->width-w; i=i+(w/5))
			{
				if (i > img1->width-w) continue;
				//for each histogram, calc the sum from the integral image
				FOR(k, nbHist)
				{
					CvRect r = cvRect(i,j,w,w);
					sums[k] = CV::GetSum(integrals[k],r)*wRath;

					if(maximums[k] < sums[k])
					{
						maximums[k] = sums[k];
						bMaxes[k] = 1;
					}
					r = cvRect(i+w/2,j+w/2,w/5,w/5);
					ROI(responses[k],r);
					cvZero(responses[k]);
					cvAddS(responses[k],cvScalarAll(sums[k]),responses[k]);
					unROI(responses[k]);					
				}
			}
		}

		FOR(i,nbHist)
		{
			if(bShow)
			{
				char wName[255] = "";
				sprintf(wName,"hist%dresp",i+1);
				cvNamedWindow(wName);
				cvShowImage(wName,responses[i]);
			}
			if(bMaxes[i]) bMaxed = 1;
		}
		if(!bMaxed) break;
	}
	delete[] sums;
	delete[] maximums;
	delete[] bMaxes;
	FOR(i, nbHist) IMKILL(integrals[i]);
	FOR(i, nbHist) IMKILL(backProjects[i]);
	FOR(i, nbHist) IMKILL(responses[i]);
	KILL(integrals);
	KILL(backProjects);
	KILL(responses);
}

/*!
	GetHistBackProjResponse: calculates the total response of the histograms backprojections
	\param img1 first plane of the image (cr, s, ...)
	\param img2 second plane of the image (cb, v, ...)
	\param multiHist pointer structure containing the histograms to analyze
	\param weights assigned to the different histograms
	\param nbHist the number of histograms to analyze
*/
IplImage *GetHistBackProjResponse(IplImage *img1,IplImage *img2, CvHistogram **multiHist, float *weights, int nbHist)
{
	//init
	IplImage *response = cvCreateImage(cvGetSize(img1), 8, 1);
	IplImage *imgs[] = {img1, img2};
	IplImage **backProjects = new IplImage*[nbHist];
	cvZero(response);
	for(int i=0; i<nbHist; i++)
	{
		backProjects[i] = cvCreateImage(cvGetSize(img1), 8, 1);
		cvCalcBackProject(imgs, backProjects[i], multiHist[i]);
		backProjects[i]->origin = response->origin = img1->origin;
	}

	u32 minSearchSize = 20;
	f32 wFactor = 1.5;

	IplImage **integrals = new IplImage*[nbHist];

	for(int i=0; i<nbHist; i++)
	{
		integrals[i] = 0;
		CV::integralImage(backProjects[i],&integrals[i]);
	}

	f32 theMax = 0;

	bool bGotInOnce = false;

	//search each square inside the image, bigger to smaller
	for(int w = min(img1->width,img1->height-2); (u32)w > minSearchSize; w = (u32)(w/wFactor))
	{
		u32 bMaxed = 0;
		f32 wRath = 1.f/(w*w);

		//move the square on the image
		for (int j = 0; j<img1->height-w; j=j+(w/5))
		{
			if (j > img1->height-w) continue;
			for (int i = 0; i<img1->width-w; i=i+(w/5))
			{
				if (i > img1->width-w) continue;
				bGotInOnce = true;
				f32 res = 1;
				CvRect r = cvRect(i,j,w,w);

				//for each histogram, calc the sum from the integral image
				for(int k=0; k<nbHist; k++)
				{
					res *= CV::GetSum(integrals[k],r) * weights[k] * wRath;
				}

				if(theMax < res){
					theMax = res;
					bMaxed = 1;
				}
				r = cvRect(i+w/2,j+w/2,w/5,w/5);
				ROI(response,r);
				cvZero(response);
				cvAddS(response, cvScalarAll(res),response);
				unROI(response);
			}
		}
		if(!bMaxed && bGotInOnce) break;
	}
	FOR(i, nbHist) IMKILL(integrals[i]);
	FOR(i, nbHist) IMKILL(backProjects[i]);
	return response;
}


IplImage *GetHistBackProjResponse(IplImage *img1,IplImage *img2, IplImage *img3, CvHistogram **multiHist, float *weights, int nbHist)
{
	//init
	IplImage *response = cvCreateImage(cvGetSize(img1), 8, 1);
	IplImage *imgs[] = {img1, img2, img3};
	IplImage **backProjects = new IplImage*[nbHist];
	cvZero(response);
	for(int i=0; i<nbHist; i++)
	{
		backProjects[i] = cvCreateImage(cvGetSize(img1), 8, 1);
		cvCalcBackProject(imgs, backProjects[i], multiHist[i]);
		backProjects[i]->origin = response->origin = img1->origin;
	}

	u32 minSearchSize = 20;
	f32 wFactor = 1.5;

	IplImage **integrals = new IplImage*[nbHist];

	for(int i=0; i<nbHist; i++)
	{
		integrals[i] = 0;
		CV::integralImage(backProjects[i],&integrals[i]);
	}

	f32 theMax = 0;

	bool bGotInOnce = false;

	//search each square inside the image, bigger to smaller
	for(int w = min(img1->width,img1->height-2); (u32)w > minSearchSize; w = (u32)(w/wFactor))
	{
		u32 bMaxed = 0;
		f32 wRath = 1.f/(w*w);

		//move the square on the image
		for (int j = 0; j<img1->height-w; j=j+(w/5))
		{
			if (j > img1->height-w) continue;
			for (int i = 0; i<img1->width-w; i=i+(w/5))
			{
				if (i > img1->width-w) continue;
				bGotInOnce = true;
				f32 res = 1;
				CvRect r = cvRect(i,j,w,w);

				//for each histogram, calc the sum from the integral image
				for(int k=0; k<nbHist; k++)
				{
					res *= CV::GetSum(integrals[k],r) * weights[k] * wRath;
				}

				if(theMax < res){
					theMax = res;
					bMaxed = 1;
				}
				r = cvRect(i+w/2,j+w/2,w/5,w/5);
				ROI(response,r);
				cvZero(response);
				cvAddS(response, cvScalarAll(res),response);
				unROI(response);
			}
		}
		if(!bMaxed && bGotInOnce) break;
	}
	FOR(i, nbHist) IMKILL(integrals[i]);
	FOR(i, nbHist) IMKILL(backProjects[i]);
	return response;
}
