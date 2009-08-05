// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Plinio Moreno
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */


#include <iCub/HistTracker.h>

//#include <cv.h>
HistTracker::HistTracker(){
	hist2D =0;
	y0Hist = 0;
	y1Hist = 0;
	backgroundHist2D = 0;
	weightedHist2D = 0;
	weightedHist2DY0=0;
	redNorm=0;
	greenNorm=0;
	redNormCopy=0;
	greenNormCopy=0;
	_oldInSize.width = -1;
    _oldInSize.height = -1;
	_needInit=true;
	deltaScaleRatio=0.1f;
	cycleTimer=YarpTimer();
	cycleTimer.start();
}

HistTracker::~HistTracker(){
close();
}

bool HistTracker::open(Searchable &config){
	if (config.check("help","if present, display usage message")) {
        printf("Call with --name /module_prefix --file configFile.ini\n");
        return false;
    }
	backgroundWeight = (config.check("backgroundWeight",
                                    Value(1),
									"Flag to weight model histogram bins using background histogram").asInt()!=0);
	kalmanFilter = (config.check("kalmanFilter",
                                    Value(1),
									"Flag to add Kalman filter to histogram tracker").asInt()!=0);
	if ( kalmanFilter ){
		sigmaXNoise = (float)config.check("sigmaXNoise", Value(0.0), "x coordinate position noise").asDouble();
		sigmaYNoise = (float)config.check("sigmaYNoise", Value(0.0), "y coordinate position noise").asDouble();
		sigmaXVelNoise = (float)config.check("sigmaXVelNoise",	Value(10.0), "x coordinate velocity noise").asDouble();
		sigmaYVelNoise = (float)config.check("sigmaYVelNoise",	Value(10.0), "y coordinate velocity noise").asDouble();
		sigmaX = (float)config.check("sigmaX",	Value(2.0),	"x coordinate position covariance").asDouble();
		sigmaY = (float)config.check("sigmaY",	Value(2.0),	"y coordinate position covariance").asDouble();
		sigmaXVel = (float)config.check("sigmaXVel", Value(10.0), "x coordinate velocity covariance").asDouble();
		sigmaYVel = (float)config.check("sigmaYVel", Value(10.0), "y coordinate velocity covariance").asDouble();
	}
	adaptiveScale = (config.check("adaptiveScale",
                                    Value(1),
									"Flag to add scale adaptation to histogram tracker").asInt()!=0);
	if ( adaptiveScale ){
		deltaScaleRatio = (float)config.check("deltaScaleRatio", Value(0.1), "scale variation").asDouble();
		gamma = (float)config.check("gamma", Value(0.1), "scale filter value").asDouble();
	}
	bhattacharyyaThreshActive = (config.check("bhattacharyyaThreshActive",
                                    Value(1),
									"Flag to add bhattacharyya threshold stopping criterion").asInt()!=0);
	if ( bhattacharyyaThreshActive ){
		bhattacharyyaThresh = (float)config.check("bhattacharyyaThresh", Value(0.3), "bhattacharyya threshold value").asDouble();
	}
	nIterMax = config.check("nIterMax", Value(10),
									"Max number of mean shift iterations").asInt();
	binSize = config.check("histogramBinSize", Value(128), "Histogram bin size per dimension").asInt();
	epsilon = (float)config.check("epsilon", Value(1),
									"allowed convergence error in pixel location").asDouble();
	_needInit = true;
	totalTrackTime = 0;
	return true;
}

bool HistTracker::close(){

    cvReleaseImage(&imgGray);
	cvReleaseImage(&currentColorImage); 
	cvReleaseImage(&currentGrayFloatImage);
	if ( kalmanFilter ){
		//printf("Releasing kalman filter variables\n");
		cvReleaseMat(&P);
		cvReleaseMat(&Q);
		cvReleaseMat(&H);
		cvReleaseMat(&currentObservation);
		cvReleaseMat(&currentXHat);
		cvReleaseMat(&precedentXHat);
		cvReleaseMat(&SMatrix);
		cvReleaseMat(&KMatrix);
		cvReleaseMat(&identityMat); 
		cvReleaseMat(&RMatrix);
		//float myFVector []={1,0,timeDelta,0,0,1,0,timeDelta,0,0,1,0,0,0,0,1};
		//float myFVector []={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
		//F=cvMat(4,4,CV_32FC1, myFVector);
		cvReleaseMat(&F);
		cvReleaseMat(&Ft);
		cvReleaseMat(&innovation);
		cvReleaseMat(&Hx);
		cvReleaseMat(&Ht);
		cvReleaseMat(&KH);
		cvReleaseMat(&PH);
		cvReleaseMat(&tempRes);
	}
	if ( backgroundWeight ){
		free(myHist);
		free(myHistY0);
		//cvReleaseHist(&weightedHist2D);
		//cvReleaseHist(&weightedHist2DY0);
		cvReleaseHist(&backgroundHist2D);
		cvReleaseImage(&redNormCopy);
		cvReleaseImage(&greenNormCopy);
			for (int myCont=0;myCont<hist_size[0];myCont++){
			free(backgroundWeights[myCont]);
			free(weightedHistogramPointer[myCont]);
		}
		free(backgroundWeights);
		free(weightedHistogramPointer);
	}
	
	cvReleaseHist(&hist2D);
	cvReleaseHist(&y0Hist);
	cvReleaseHist(&y1Hist);

	cvReleaseImage(&redOriginal);
	cvReleaseImage(&greenOriginal);
	cvReleaseImage(&blueOriginal);
	cvReleaseImage(&redNorm);
	cvReleaseImage(&greenNorm);
	cvReleaseImage(&blueNorm);
	cvReleaseImage(&epanechnikovK);
	cvReleaseImage(&binLookUpTableRed);
	cvReleaseImage(&binLookUpTableGreen);
	cvReleaseImage(&binLookUpTableY0Red);
	cvReleaseImage(&binLookUpTableY0Green);

	return true;
}

void HistTracker::backgroundWeightedHistogramComputation ( IplImage** imageCopy, CvRect track_window, CvRect background_window,
											 CvHistogram *backgroundHistogram, CvHistogram *weightedHistogram,
											 CvHistogram *modelHistogram, int *hist_size, float **ranges,
											 float *histogramPointer,
											 float **backgroundWeights,
											 float **weightedHistogramPointer)
{
	//float **backgroundWeights;
	//float **weightedHistogramPointer;
	//float *tempPointer;
	/*backgroundWeights=(float **)malloc(sizeof(float)*hist_size[0]);
	weightedHistogramPointer=(float **)malloc(sizeof(float)*hist_size[0]);
	for (int myCont=0;myCont<hist_size[0];myCont++){
		backgroundWeights[myCont]=(float *)malloc(sizeof(float)*hist_size[1]);
		weightedHistogramPointer[myCont]=(float *)malloc(sizeof(float)*hist_size[1]);
	}*/
	cvSetImageROI( imageCopy[0], track_window );
	cvSetImageROI( imageCopy[1], track_window );
	cvSet( imageCopy[0], cvRealScalar(-1.0f), 0 );
	cvSet( imageCopy[1], cvRealScalar(-1.0f), 0 );
	cvResetImageROI(imageCopy[0]);
	cvResetImageROI(imageCopy[1]);
	cvSetImageROI( imageCopy[0], background_window );
	cvSetImageROI( imageCopy[1], background_window );
	cvClearHist( backgroundHistogram );
	cvCalcHist( imageCopy , backgroundHistogram, 0 );
	float minVal=1;
	int h,s;
	for( h = 0; h < hist_size[0]; h++ )
	{
		for( s = 0; s < hist_size[1]; s++ )
		{
			float bin_val = cvQueryHistValue_2D( backgroundHistogram, h, s );
			float bin_val1 = cvQueryHistValue_2D( modelHistogram, h, s );
			weightedHistogramPointer[h][s]=bin_val1;
			backgroundWeights[h][s] = bin_val;
			if (bin_val!=0 && bin_val<minVal)
				minVal=bin_val;
		}
	}
	//tempPointer=histogramPointer;
	float sum=0;
	for( h = 0; h < hist_size[0]; h++ )
	{
		for( s = 0; s < hist_size[1]; s++ )
		{
			if (backgroundWeights[h][s]!=0)
				backgroundWeights[h][s] = minVal / backgroundWeights[h][s];
			else
				backgroundWeights[h][s]=1;
			weightedHistogramPointer[h][s] *= backgroundWeights[h][s];
			//*histogramPointer++=weightedHistogramPointer[h][s];
			histogramPointer[h*hist_size[0]+s]=weightedHistogramPointer[h][s];
			sum+=weightedHistogramPointer[h][s];
		}
	}
	//histogramPointer=tempPointer;
	//weightedHistogram = cvMakeHistHeaderForArray( 2, hist_size, weightedHistogram, histogramPointer, ranges);
	//cvNormalizeHist( weightedHistogram, 1 );
	for (int myCont=0;myCont<hist_size[0]*hist_size[1];myCont++)
		histogramPointer[myCont]/=sum;
	/*for (int myCont=0;myCont<hist_size[0];myCont++){
		free(backgroundWeights[myCont]);
		free(weightedHistogramPointer[myCont]);
	}
	free(backgroundWeights);
	free(weightedHistogramPointer);*/
}

void HistTracker::imageBoundingBoxComputation ( CvRect actualWindow, CvRect *displayWindow, int imageWidth, int imageHeight )
{
	if (actualWindow.x < 0){
		//if ( actualWindow.x+actualWindow.width>=imageWidth )
		//	displayWindow->width
		displayWindow->width=actualWindow.width+actualWindow.x;
		displayWindow->x=0;
	}
	else{
		displayWindow->x=actualWindow.x;
		displayWindow->width=actualWindow.width;
	}
	if (actualWindow.y<0){
		displayWindow->height=actualWindow.height+actualWindow.y;
		displayWindow->y=0;
	}
	else{
		displayWindow->y=actualWindow.y;
		displayWindow->height=actualWindow.height;
	}
	if (actualWindow.x+actualWindow.width>=imageWidth && actualWindow.x>=0){
		displayWindow->width=imageWidth-actualWindow.x-1;
	}
	else if (actualWindow.x+actualWindow.width>=imageWidth && actualWindow.x<0){
		displayWindow->width=imageWidth-1;
	}
	//else
	//	displayWindow->width=actualWindow.width;
	if (actualWindow.y+actualWindow.height>=imageHeight && actualWindow.y>=0){
		displayWindow->height=imageHeight-actualWindow.y-1;
	}
	else if (actualWindow.y+actualWindow.height>=imageHeight && actualWindow.y<0){
		displayWindow->height=imageHeight-1;
	}
	//else
	//	displayWindow->height=actualWindow.height;

}

bool HistTracker::learnModel( const Image &in, const CvRect &track_window_initial ){
	if ( in.width()  != _oldInSize.width || 
         in.height() != _oldInSize.height || 
        _needInit)
        init(cvSize(in.width(), in.height()));
	track_window.x = track_window_actual.x=track_window_initial.x;
	track_window.y = track_window_actual.y=track_window_initial.y;
	track_window.width=track_window_actual.width=track_window_initial.width;
	track_window.height=track_window_actual.height=track_window_initial.height;
	cvCvtColor((IplImage*)in.getIplImage(),imgGray, CV_RGB2GRAY);//imgGray now contains gray scale image
	cvConvert( imgGray, currentGrayFloatImage );// Conversion from uchar to float
	
	cvCvtPixToPlane( (IplImage*)in.getIplImage(), redOriginal, greenOriginal, blueOriginal, 0 );
	cvConvert(redOriginal,redNorm);
	cvConvert(greenOriginal,greenNorm);
	cvConvert(blueOriginal,blueNorm);
	cvDiv( redNorm, currentGrayFloatImage, redNorm );
	cvConvertScale( redNorm, redNorm, 0.3333 );
	cvDiv( greenNorm, currentGrayFloatImage, greenNorm );
	cvConvertScale( greenNorm, greenNorm, 0.3333 );
	cvDiv( blueNorm, currentGrayFloatImage, blueNorm );
	cvConvertScale( blueNorm, blueNorm, 0.3333 );
	
	if ( backgroundWeight ){
		cvCopy(redNorm,redNormCopy);
		cvCopy(greenNorm,greenNormCopy);
	}
	/*printf("Track window: xPos: %d, yPos: %d, width: %d, height: %d\n",track_window_actual.x,track_window_actual.y
		,track_window_actual.width,track_window_actual.height);*/
	cvSetImageROI( redNorm, track_window_actual );
	cvSetImageROI( greenNorm, track_window_actual );
	cvSetImageROI( epanechnikovK, track_window_actual );
	cvSetImageROI( binLookUpTableRed, track_window_actual );
	cvSetImageROI( binLookUpTableGreen, track_window_actual );
	if ( kalmanFilter ){
	cvmSet(precedentXHat,0,0,double(track_window_actual.x+track_window_actual.width/2));
	cvmSet(precedentXHat,1,0,double(track_window_actual.y+track_window_actual.height/2));
	cvmSet(precedentXHat,2,0,0.0);
	cvmSet(precedentXHat,3,0,0.0);
	}
	int contX,contY;
	float x,y;
	float myWidth=(float)track_window_actual.width;
	float myHeight=(float)track_window_actual.height;
	myWidth-=1.0f;
	myHeight-=1.0f;
	for (x=-myWidth/2,contX=0;x<=myWidth/2;x++,contX++){
		for (y=-myHeight/2,contY=0;y<=myHeight/2;y++,contY++){
			float new_value=pow(x/(myWidth/2),2)+pow(y/(myHeight/2),2) ;
			if (new_value<=1)
				new_value=(2/(float)PI)*(1-new_value);
			else
				new_value=0;
			cvSet2D( epanechnikovK, contX, contY, cvRealScalar(new_value) );
		}
	}
	//printf("Before histogram model computation\n");
	IplImage* planes[] = {redNorm,greenNorm};
	IplImage* binLookUpTable[] = {binLookUpTableRed,binLookUpTableGreen};
	cvCalcHistKernelWeighted( planes, hist2D, epanechnikovK, binLookUpTable, 0 ,0);
	cvNormalizeHist( hist2D, 1 );
	//printf("After histogram model computation\n");
	if ( backgroundWeight ){
		//cvCopyHist( hist2D, &weightedHist2D );
		background_window_actual.x=round(track_window_actual.x-(track_window_actual.width/2)*sqrt(3.0f));
		background_window_actual.y=round(track_window_actual.y-(track_window_actual.height/2)*sqrt(3.0f));
		background_window_actual.width=round(track_window_actual.width*sqrt(3.0f));
		background_window_actual.height=round(track_window_actual.height*sqrt(3.0f));
		IplImage *imageCopy[]={redNormCopy,greenNormCopy};
		backgroundWeightedHistogramComputation ( imageCopy, track_window_actual, background_window_actual,
			backgroundHist2D, weightedHist2D,
			hist2D, hist_size, ranges,
			myHist,
			backgroundWeights,
			weightedHistogramPointer);
	}
	cycleTimer.stop();
	cycleTimer.endlap();
	cvResetImageROI(redNorm);
	cvResetImageROI(greenNorm);
	cvResetImageROI(epanechnikovK);
	cvResetImageROI(binLookUpTableRed);
	cvResetImageROI(binLookUpTableGreen);
	//printf("After model computation\n");
	totalTrackTime = 0;
	return true;

}
bool HistTracker::track ( const Image &in ){
	if ( in.width()  != _oldInSize.width || 
         in.height() != _oldInSize.height || 
        _needInit)
        init(cvSize(in.width(), in.height()));
	//printf("Before color conversion\n");
	cvCvtColor((IplImage*)in.getIplImage(),imgGray, CV_RGB2GRAY);//imgGray now contains gray scale image
	//printf("After color conversion\n");
	cvConvert( imgGray, currentGrayFloatImage );// Conversion from uchar to float
	
	cvCvtPixToPlane( (IplImage*)in.getIplImage(), redOriginal, greenOriginal, blueOriginal, 0 );
	cvConvert(redOriginal,redNorm);
	cvConvert(greenOriginal,greenNorm);
	cvConvert(blueOriginal,blueNorm);
	cvDiv( redNorm, currentGrayFloatImage, redNorm );
	cvConvertScale( redNorm, redNorm, 0.3333 );
	cvDiv( greenNorm, currentGrayFloatImage, greenNorm );
	cvConvertScale( greenNorm, greenNorm, 0.3333 );
	cvDiv( blueNorm, currentGrayFloatImage, blueNorm );
	cvConvertScale( blueNorm, blueNorm, 0.3333 );

	timeDelta=(float)cycleTimer.lastlap();//1.0/25.0;
	totalTrackTime += (double)timeDelta;
	//printf("Last lap of time: %f\n",timeDelta*1000);
	//nIter=1;

	//float deltaScaleRatio=0.1;
	
	
	currentXPos=(float)track_window_actual.x;
	currentYPos=(float)track_window_actual.y;
	currentWidth=(float)track_window_actual.width;
	currentHeight=(float)track_window_actual.height;
	/*printf("Track window 1: xPos: %d, yPos: %d, width: %d, height: %d\n",track_window_actual.x,track_window_actual.y
		,track_window_actual.width,track_window_actual.height);*/
	if ( currentXPos+currentWidth <= 0 || currentYPos+currentHeight <= 0
		|| currentXPos >= width || currentYPos >= height ){
			//trackingActive=0;
			//faceDetectedGlobal=0;
			//break;
			turnedOn = false;
			return true;
	}
	
	if (kalmanFilter){
		//xHatPrecedent=currentXPos+currentWidth/2;//+currentXVel*timeDelta;
		//yHatPrecedent=currentYPos+currentHeight/2;//+currentYVel*timeDelta;
		//float myFVector []={1,0,timeDelta,0,0,1,0,timeDelta,0,0,1,0,0,0,0,1};
		//F=cvMat(4,4,CV_32FC1, myFVector);
		cvmSet(F,0,2,timeDelta);
		cvmSet(F,1,3,timeDelta);
		//Ft=cvCloneMat(F);
		cvTranspose(F, Ft);
		cvMatMul(P, Ft, Ft);
		cvMatMulAdd(F, Ft, Q, P);
		cvMatMul(F, precedentXHat, precedentXHat);
		
	}
	//scaleAdaptation.start();
	for ( scaleCounter=initScale;scaleCounter<finalScale;scaleCounter++){
		track_window_actual.x=round(currentXPos-(currentWidth/2)*scaleRatio[scaleCounter]);
		track_window_actual.y=round(currentYPos-(currentHeight/2)*scaleRatio[scaleCounter]);
		track_window_actual.width=round(currentWidth+scaleRatio[scaleCounter]*currentWidth);
		track_window_actual.height=round(currentHeight+scaleRatio[scaleCounter]*currentHeight);
		//CvPoint pt1, pt2;
		////cvResetImageROI( redNorm);
		////cvResetImageROI( greenNorm);

		imageBoundingBoxComputation ( track_window_actual, &track_window, width, height );
		/*if ( track_window.x+track_window.width < 0 || track_window.y+track_window.height < 0
		|| track_window.x >= width || track_window.y >= height ){
		trackingActive=0;
		faceDetectedGlobal=0;
		break;
		}*/
		if ( track_window_actual.x+track_window_actual.width <= 0 || track_window_actual.y+track_window_actual.height <= 0
			|| track_window_actual.x >= width || track_window_actual.y >= height ){
				/*trackingActive=0;
				faceDetectedGlobal=0;
				break;*/
				turnedOn = false;
				return true;
		}
		/*if (drawingAll)
			drawBoundingBox(buffer, in_copy, track_window, 1, CV_RGB(255,0,255));*/
		/*printf("Track window 2: xPos: %d, yPos: %d, width: %d, height: %d\n",track_window_actual.x,track_window_actual.y
		,track_window_actual.width,track_window_actual.height);*/
		converge=0;
		nIter=1;
		deltaX[scaleCounter]=0.0;
		deltaY[scaleCounter]=0.0;
		//meanShift.start();
		while (nIter<nIterMax && converge==0)
		{
			//cycleTimer.start();
			imageBoundingBoxComputation ( track_window_actual, &track_window, width, height );
			/*if ( track_window.x+track_window.width < 0 || track_window.y+track_window.height < 0
			|| track_window.x >= width || track_window.y >= height ){
			trackingActive=0;
			faceDetectedGlobal=0;
			break;
			}*/
			if ( track_window_actual.x+track_window_actual.width <= 0 || track_window_actual.y+track_window_actual.height <= 0
				|| track_window_actual.x >= width || track_window_actual.y >= height ){
					/*trackingActive=0;
					faceDetectedGlobal=0;
					break;*/
					turnedOn = false;
					return true;
			}
			cvResetImageROI(redNorm);
			cvResetImageROI(greenNorm);
			if ( backgroundWeight ){
				cvResetImageROI(redNormCopy);
				cvResetImageROI(greenNormCopy);
				//cvReleaseImage(&redNormCopy);
				//cvReleaseImage(&greenNormCopy);
				cvCopy(redNorm,redNormCopy);
				cvCopy(greenNorm,greenNormCopy);
			}
			cvSetImageROI( redNorm, track_window );
			cvSetImageROI( greenNorm, track_window );
			cvResetImageROI(binLookUpTableY0Red);
			cvResetImageROI(binLookUpTableY0Green);
			cvResetImageROI(epanechnikovK);
			cvSet( epanechnikovK, cvRealScalar(0.0f), 0 );
			cvSet( binLookUpTableY0Red, cvScalar(0), 0 );
			cvSet( binLookUpTableY0Green, cvScalar(0), 0 );
			cvSetImageROI( epanechnikovK, track_window );
			cvSetImageROI( binLookUpTableY0Red, track_window );
			cvSetImageROI( binLookUpTableY0Green, track_window );
			cvClearHist( y0Hist );
			int contX,contY;
			float x,y;
			int posX,posY;
			float currentX,currentY;
			float myWidth=(float)track_window.width;
			float myHeight=(float)track_window.height;
			for (x=0,contX=0,posX=track_window.x;x<myWidth;x++,contX++,posX++){
				for (y=0,contY=0,posY=track_window.y;y<myHeight;y++,contY++,posY++){
					currentX=(float)(track_window_actual.x+track_window_actual.width/2-posX);
					currentY=(float)(track_window_actual.y+track_window_actual.height/2-posY);
					float new_value=pow(currentX/(track_window_actual.width/2),2)+pow(currentY/(track_window_actual.height/2),2) ;
					if (new_value<=1)
						new_value=(2/(float)PI)*(1-new_value);
					else
						new_value=0;
					cvSet2D( epanechnikovK, contY,contX,cvRealScalar(new_value) );
				}
			}
			//histogramComputation.start();
			IplImage* planes[] = {redNorm,greenNorm};
			IplImage* binLookUpTableY0[] = {binLookUpTableY0Red,binLookUpTableY0Green};
			cvCalcHistKernelWeighted( planes, y0Hist, epanechnikovK, binLookUpTableY0, 0 ,0);
			cvNormalizeHist( y0Hist, 1 );
			if ( backgroundWeight ){
				//cvCopyHist( y0Hist, &weightedHist2DY0 );
				//backgroundWeightComputation.start();
				background_window_actual.x=round(track_window_actual.x-(track_window_actual.width/2)*sqrt(3.0f));
				background_window_actual.y=round(track_window_actual.y-(track_window_actual.height/2)*sqrt(3.0f));
				background_window_actual.width=round(track_window_actual.width*sqrt(3.0f));
				background_window_actual.height=round(track_window_actual.height*sqrt(3.0f));

				imageBoundingBoxComputation ( background_window_actual, &background_window, width, height );
				/*if ( background_window.x+background_window.width < 0 || background_window.y+background_window.height < 0
				|| background_window.x >= width || background_window.y >= height ){
				trackingActive=0;
				faceDetectedGlobal=0;
				break;
				}*/
				if ( background_window_actual.x+background_window_actual.width <= 0 || background_window_actual.y+background_window_actual.height <= 0
					|| background_window_actual.x >= width || background_window_actual.y >= height ){
						/*trackingActive=0;
						faceDetectedGlobal=0;
						break;*/
						turnedOn = false;
						return true;
				}
				IplImage *imageCopy[]={redNormCopy,greenNormCopy};
				backgroundWeightedHistogramComputation ( imageCopy, track_window, background_window,
					backgroundHist2D, weightedHist2DY0,
					y0Hist, hist_size, ranges,
					myHistY0,
					backgroundWeights,
					weightedHistogramPointer);
			}
			//y0X=0.0;
			//y0Y=0.0;
			int h,s;
			/*histogramComputation.stop();
			backgroundWeightComputation.stop();
			backgroundWeightComputation.endlap();
			histogramComputation.endlap();*/
			bhattacharyyaCoef[scaleCounter]=0;
			float bin_val;
			float bin_val1;
			for( h = 0; h < binSize; h++ )
			{
				for( s = 0; s < binSize; s++ )
				{
					if ( backgroundWeight ){
						//bin_val = cvQueryHistValue_2D( weightedHist2D, h, s );
						bin_val = myHist[h*hist_size[0]+s];
						//bin_val1 = cvQueryHistValue_2D( weightedHist2DY0, h, s );
						bin_val1 = myHistY0[h*hist_size[0]+s];
					}
					else {
						bin_val = cvQueryHistValue_2D( hist2D, h, s );
						bin_val1 = cvQueryHistValue_2D( y0Hist, h, s );
					}
					bhattacharyyaCoef[scaleCounter]+=sqrt(bin_val*bin_val1);
				}
			}

			float histDist=0;
			float maxValue=0, sumHist=0;					
			float q;
			float p;
			CvScalar a,b;
			float y1X,y1Y;
			//float currentX,currentY;
			//int posX,posY;
			float weightSum=0,posXWeightSum=0,posYWeightSum=0,weight;
			for (x=0,contX=0,posX=track_window.x;x<myWidth;x++,contX++,posX++){
				for (y=0,contY=0,posY=track_window.y;y<myHeight;y++,contY++,posY++){
					currentX=track_window_actual.x+(float)track_window_actual.width/2-posX;
					currentY=track_window_actual.y+(float)track_window_actual.height/2-posY;
					a=cvGet2D( binLookUpTableY0Red, (int)y, (int)x );
					b=cvGet2D( binLookUpTableY0Green, (int)y, (int)x );
					if ( backgroundWeight ){
						//p=cvQueryHistValue_2D( weightedHist2DY0, (int)a.val[0], (int)b.val[0] );
						p = myHistY0[(int)a.val[0]*hist_size[0]+(int)b.val[0]];
						//q=cvQueryHistValue_2D( weightedHist2D, (int)a.val[0], (int)b.val[0] );
						q = myHist[(int)a.val[0]*hist_size[0]+(int)b.val[0]];
					}
					else{
						p=cvQueryHistValue_2D( y0Hist, (int)a.val[0], (int)b.val[0] );
						q=cvQueryHistValue_2D( hist2D, (int)a.val[0], (int)b.val[0] );
					}
					if (p!=0){
						weight=sqrt(q/p);
						weightSum+=weight;
						posXWeightSum+=(currentX/((float)track_window_actual.width/2))*weight;
						posYWeightSum+=(currentY/((float)track_window_actual.height/2))*weight;
					}
				}
			}						
			y1X=(posXWeightSum/weightSum)*(myWidth/2);
			y1Y=(posYWeightSum/weightSum)*(myHeight/2);
			if (((y1X)*(y1X)+(y1Y)*(y1Y))>=epsilon*epsilon){
				track_window_actual.x-=round(y1X);
				track_window_actual.y-=round(y1Y);
				deltaX[scaleCounter]+=round(y1X);
				deltaY[scaleCounter]+=round(y1Y);
				nIter++;
			}
			else
				converge=1;
		}
		cvResetImageROI( redNorm);
		cvResetImageROI( greenNorm);
		imageBoundingBoxComputation ( track_window_actual, &track_window, width, height );
		/*if ( track_window.x+track_window.width < 0 || track_window.y+track_window.height < 0
		|| track_window.x >= width || track_window.y >= height ){
		trackingActive=0;
		faceDetectedGlobal=0;
		break;
		}*/
		if ( track_window_actual.x+track_window_actual.width <= 0 || track_window_actual.y+track_window_actual.height <= 0
			|| track_window_actual.x >= width || track_window_actual.y >= height ){
				/*trackingActive=0;
				faceDetectedGlobal=0;
				break;*/
				turnedOn = false;
				return true;
		}
		/*if (drawingAll){
			drawBoundingBox(buffer, in_copy, track_window, 1, CV_RGB(0,0,255));
			cvShowImage( "Face tracker", buffer );
		}*/
	}
	//printf("After mean shift\n");
	if (adaptiveScale){
		if (bhattacharyyaCoef[0]>bhattacharyyaCoef[1] && bhattacharyyaCoef[0]>bhattacharyyaCoef[2]){
			optimalScaleRatio=scaleRatio[0];
			optimalDeltaX=deltaX[0];
			optimalDeltaY=deltaY[0];
			currentBhattacharyyaCoef=bhattacharyyaCoef[0];
		}
		else if (bhattacharyyaCoef[1]>bhattacharyyaCoef[0] && bhattacharyyaCoef[1]>bhattacharyyaCoef[2]){
			optimalScaleRatio=scaleRatio[1];
			optimalDeltaX=deltaX[1];
			optimalDeltaY=deltaY[1];
			currentBhattacharyyaCoef=bhattacharyyaCoef[1];
		}
		else if (bhattacharyyaCoef[2]>bhattacharyyaCoef[0] && bhattacharyyaCoef[2]>bhattacharyyaCoef[1]){
			optimalScaleRatio=scaleRatio[2];
			optimalDeltaX=deltaX[2];
			optimalDeltaY=deltaY[2];
			currentBhattacharyyaCoef=bhattacharyyaCoef[2];
		}
	}
	else{
		optimalScaleRatio=scaleRatio[1];
		optimalDeltaX=deltaX[1];
		optimalDeltaY=deltaY[1];
		currentBhattacharyyaCoef=bhattacharyyaCoef[1];
	}
	if ( bhattacharyyaThreshActive ){
		if ( currentBhattacharyyaCoef < bhattacharyyaThresh ){
			/*trackingActive=0;
			faceDetectedGlobal=0;
			break;*/
			turnedOn = false;
			return true;
		}
	}
	optimalWidth=currentWidth+optimalScaleRatio*currentWidth;;
	optimalHeight=currentHeight+optimalScaleRatio*currentHeight;
	newWidth=gamma*optimalWidth+(1-gamma)*currentWidth;
	newHeight=gamma*optimalHeight+(1-gamma)*currentHeight;
	//printf("newWidth: %.1f, newHeight: %.1f\n",newWidth,newHeight);
	track_window_actual.x=round(currentXPos-(currentWidth/2)*optimalScaleRatio-optimalDeltaX);
	track_window_actual.y=round(currentYPos-(currentHeight/2)*optimalScaleRatio-optimalDeltaY);
	track_window_actual.width=round(newWidth);
	track_window_actual.height=round(newHeight);
	/*printf("Track window 3: xPos: %d, yPos: %d, width: %d, height: %d\n",track_window_actual.x,track_window_actual.y
		,track_window_actual.width,track_window_actual.height);*/
	//scaleAdaptation.stop();
	//scaleAdaptation.endlap();
	//printf("Before Kalman filtering step\n");
	if (kalmanFilter){
		cvmSet(currentObservation,0,0,double(track_window_actual.x+newWidth/2));
		cvmSet(currentObservation,1,0,double(track_window_actual.y+newHeight/2));
		//printf("After cvmSet\n");
		/*CvMat *innovation = cvCreateMat(2,1,CV_32FC1);
		CvMat *Hx = cvCreateMat(2,1,CV_32FC1);
		CvMat *Ht = cvCreateMat(4,2,CV_32FC1);*/
		cvTranspose(H, Ht);
		//printf("after transpose\n");
		//CvMat *KH = cvCreateMat(4,4,CV_32FC1);
		cvMatMul(H, precedentXHat,Hx);
		//printf("after Hx mult\n");
		/*CvMat *PH = cvCreateMat(4,2,CV_32FC1);
		CvMat *tempRes = cvCreateMat(4,4,CV_32FC1);*/
		cvMatMul(P,Ht,PH);
		//printf("after PhP\n");
		cvMatMulAdd(H, PH, RMatrix, SMatrix);
		cvInvert( SMatrix, SMatrix, CV_LU );
		cvMatMul(PH, SMatrix, KMatrix);
		cvScaleAdd( Hx, cvScalar(-1), currentObservation, innovation );
		//printf("Before currentX\n");
		cvMatMulAdd( KMatrix, innovation, precedentXHat, currentXHat );
		//printf("After currentX\n");
		cvReleaseMat(&precedentXHat);
		precedentXHat = cvCloneMat( currentXHat );
		//printf("after cloning\n");
		cvMatMul(KMatrix, H, KH);
		//printf("After KH mult\n");
		cvScaleAdd( KH, cvScalar(-1), identityMat, tempRes );
		//printf("after tempRes comp\n");
		cvMatMul(tempRes, P, P);
		//printf("after P update\n");
	}
	cycleTimer.stop();
	imageBoundingBoxComputation ( track_window_actual, &track_window, width, height );
	/*if ( track_window.x+track_window.width < 0 || track_window.y+track_window.height < 0
	|| track_window.x >= width || track_window.y >= height ){
	trackingActive=0;
	faceDetectedGlobal=0;
	break;
	}*/
	if ( track_window_actual.x+track_window_actual.width <= 0 || track_window_actual.y+track_window_actual.height <= 0
		|| track_window_actual.x >= width || track_window_actual.y >= height ){
			/*trackingActive=0;
			faceDetectedGlobal=0;
			break;*/
			turnedOn = false;
			return true;
	}
	/*if (drawingAll || drawingResult)
		drawBoundingBox(buffer, in_copy, track_window, 1, CV_RGB(0,255,0));*/
	/*printf("Track window 4: xPos: %d, yPos: %d, width: %d, height: %d\n",track_window_actual.x,track_window_actual.y
		,track_window_actual.width,track_window_actual.height);*/
	if (kalmanFilter){
		track_window_actual.x=round((float)cvmGet( currentXHat, 0, 0 )-(newWidth/2));
		track_window_actual.y=round((float)cvmGet( currentXHat, 1, 0 )-(newHeight/2));
	}
	/*printf("Track window 5: xPos: %d, yPos: %d, width: %d, height: %d\n",track_window_actual.x,track_window_actual.y
		,track_window_actual.width,track_window_actual.height);*/
	//printf("After Kalman filtering step\n");
	imageBoundingBoxComputation ( track_window_actual, &track_window, width, height );
	/*if ( track_window.x+track_window.width < 0 || track_window.y+track_window.height < 0
	|| track_window.x >= width || track_window.y >= height ){
	trackingActive=0;
	faceDetectedGlobal=0;
	break;
	}*/
	if ( track_window_actual.x+track_window_actual.width <= 0 || track_window_actual.y+track_window_actual.height <= 0
		|| track_window_actual.x >= width || track_window_actual.y >= height ){
			/*trackingActive=0;
			faceDetectedGlobal=0;
			break;*/
			turnedOn = false;
			return true;
	}
	/*if ((drawingAll || drawingResult) && kalmanFilter)
		drawBoundingBox(buffer, in_copy, track_window, 1, CV_RGB(128,128,128));*/
	cycleTimer.endlap();
	//printf("Tracking ended\n");
	return true;
}

bool HistTracker::init(CvSize currImgSize){
	imgGray=cvCreateImage(currImgSize,  IPL_DEPTH_8U, 1);
	currentColorImage = cvCreateImage( currImgSize, 8, 3 );
	currentGrayFloatImage=cvCreateImage(currImgSize,  IPL_DEPTH_32F, 1);
	width = currImgSize.width;
	height = currImgSize.height;
	if ( kalmanFilter ){
		P=cvCreateMat(4,4,CV_32FC1);
		cvSetIdentity( P, cvRealScalar(1.0) );
		cvmSet(P,0,0,sigmaX);
		cvmSet(P,1,1,sigmaY);
		cvmSet(P,2,2,sigmaXVel);
		cvmSet(P,3,3,sigmaYVel);
		Q=cvCreateMat(4,4,CV_32FC1);;
		cvSetIdentity( Q, cvRealScalar(1.0) );
		cvmSet(Q,0,0,sigmaXNoise);
		cvmSet(Q,1,1,sigmaYNoise);
		cvmSet(Q,2,2,sigmaXVelNoise);
		cvmSet(Q,3,3,sigmaYVelNoise);
		//float myHVector []={1,0,0,0,0,1,0,0};
		//H=cvMat(2,4,CV_32FC1, myHVector);
		H=cvCreateMat(2,4,CV_32FC1);
		cvmSet(H,0,0,1);
		cvmSet(H,0,1,0);
		cvmSet(H,0,2,0);
		cvmSet(H,0,3,0);
		cvmSet(H,1,0,0);
		cvmSet(H,1,1,1);
		cvmSet(H,1,2,0);
		cvmSet(H,1,3,0);
		//float currentObserv[]={0,0,0,0};
		//currentObservation = cvMat(2,1,CV_32FC1,currentObserv);
		currentObservation = cvCreateMat(2,1,CV_32FC1);
		cvmSet(currentObservation,0,0,0);
		cvmSet(currentObservation,1,0,0);
		currentXHat = cvCreateMat(4,1,CV_32FC1);
		precedentXHat = cvCreateMat(4,1,CV_32FC1);
		//CvMat *predictedXHat = cvCreateMat(4,1,CV_32FC1);
		SMatrix = cvCreateMat(2,2,CV_32FC1);
		KMatrix = cvCreateMat(4,2,CV_32FC1);
		identityMat = cvCreateMat(4,4,CV_32FC1);
		cvSetIdentity( identityMat, cvScalar(1) );
		RMatrix=cvCreateMat(2,2,CV_32FC1);
		cvSetIdentity( RMatrix, cvRealScalar(1.0) );
		//float myFVector []={1,0,timeDelta,0,0,1,0,timeDelta,0,0,1,0,0,0,0,1};
		//float myFVector []={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
		//F=cvMat(4,4,CV_32FC1, myFVector);
		F=cvCreateMat(4,4,CV_32FC1);
		Ft=cvCreateMat(4,4,CV_32FC1);
		cvSetIdentity( F, cvRealScalar(1.0) );
		innovation = cvCreateMat(2,1,CV_32FC1);
		Hx = cvCreateMat(2,1,CV_32FC1);
		Ht = cvCreateMat(4,2,CV_32FC1);
		KH = cvCreateMat(4,4,CV_32FC1);
		PH = cvCreateMat(4,2,CV_32FC1);
		tempRes = cvCreateMat(4,4,CV_32FC1);
	}
	float r_ranges[] = { 0, 1 }; 
	float g_ranges[] = { 0, 1 };
	ranges[0] = r_ranges;
	ranges[1] = g_ranges;
	//float* ranges[] = { r_ranges, g_ranges };
	hist_size[0] = binSize;
	hist_size[1] = binSize;
	//int hist_size[] = {r_bins, g_bins};
	if ( backgroundWeight ){
		myHist=(float *)malloc(sizeof(float)*binSize*binSize);
		myHistY0=(float *)malloc(sizeof(float)*binSize*binSize);
		weightedHist2D = NULL;//cvCreateHist( 2, hist_size, CV_HIST_ARRAY, ranges, 1 );
		weightedHist2DY0= NULL;//cvCreateHist( 2, hist_size, CV_HIST_ARRAY, ranges, 1 );
		backgroundHist2D = cvCreateHist( 2, hist_size, CV_HIST_ARRAY, ranges, 1 );
		redNormCopy=cvCreateImage(cvSize(width,height),  IPL_DEPTH_32F, 1);
		greenNormCopy=cvCreateImage(cvSize(width,height),  IPL_DEPTH_32F, 1);
		backgroundWeights=(float **)malloc(sizeof(float *)*hist_size[0]);
		weightedHistogramPointer=(float **)malloc(sizeof(float *)*hist_size[0]);
		for (int myCont=0;myCont<hist_size[0];myCont++){
			backgroundWeights[myCont]=(float *)malloc(sizeof(float)*hist_size[1]);
			weightedHistogramPointer[myCont]=(float *)malloc(sizeof(float)*hist_size[1]);
		}
	}
	
	hist2D = cvCreateHist( 2, hist_size, CV_HIST_ARRAY, ranges, 1 );
	y0Hist= cvCreateHist( 2, hist_size, CV_HIST_ARRAY, ranges, 1 );
	y1Hist= cvCreateHist( 2, hist_size, CV_HIST_ARRAY, ranges, 1 );

	redOriginal=cvCreateImage(cvSize(width,height),  IPL_DEPTH_8U, 1);
	greenOriginal=cvCreateImage(cvSize(width,height),  IPL_DEPTH_8U, 1);
	blueOriginal=cvCreateImage(cvSize(width,height),  IPL_DEPTH_8U, 1);
	redNorm=cvCreateImage(cvSize(width,height),  IPL_DEPTH_32F, 1);
	greenNorm=cvCreateImage(cvSize(width,height),  IPL_DEPTH_32F, 1);
	blueNorm=cvCreateImage(cvSize(width,height),  IPL_DEPTH_32F, 1);
	epanechnikovK=cvCreateImage(cvSize(width,height),  IPL_DEPTH_32F, 1);
	cvSet2D(epanechnikovK,0,0,cvRealScalar(0.0f));
	binLookUpTableRed=cvCreateImage(cvSize(width,height),  IPL_DEPTH_8U, 1);
	binLookUpTableGreen=cvCreateImage(cvSize(width,height),  IPL_DEPTH_8U, 1);
	binLookUpTableY0Red=cvCreateImage(cvSize(width,height),  IPL_DEPTH_8U, 1);
	binLookUpTableY0Green=cvCreateImage(cvSize(width,height),  IPL_DEPTH_8U, 1);

	//imageCopy[] = {redNormCopy, greenNormCopy};
	//*imageCopy[1] = greenNormCopy;
	//planes[0][0] = redNorm;
	//planes[0][1] = greenNorm;
	/*binLookUpTable[0][0] = binLookUpTableRed;
	binLookUpTable[0][1] = binLookUpTableGreen;
	binLookUpTableY0[0][0] = binLookUpTableY0Red;
	binLookUpTableY0[0][1] = binLookUpTableY0Green;*/
	if (adaptiveScale){
		initScale=0;
		finalScale=3;
	}
	else
	{
		initScale=1;
		finalScale=2;
	}
	scaleRatio[0]=-deltaScaleRatio;
	scaleRatio[1]=0.0;
	scaleRatio[2]=deltaScaleRatio;
	_needInit = false;
	turnedOn = true;
	_oldInSize.width  = width;
    _oldInSize.height = height;
	//printf("After init\n");
	return true;
}
