// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <iCub/BlobFunctions.h>


void BlobFunctions::filterBiggest(CBlobResult *blobs){
	
	int i, numBlobs, indexBiggest = 0;
	numBlobs = blobs->GetNumBlobs();
	
	if (numBlobs > 0){
		
		double_stl_vector evalBlobs;
		double_stl_vector::iterator itEvalBlobs;
		double_stl_vector::iterator itBiggestBlob;

		CBlob *finalBlob;
		// this returns blobs wrapped in CBlobGetArea operadorBlobs!
		evalBlobs = blobs->GetSTLResult(CBlobGetArea());
		itEvalBlobs = evalBlobs.begin();
		itBiggestBlob = evalBlobs.begin();
			
		for(i=0; i < numBlobs; i++, itEvalBlobs++){
			if( *itEvalBlobs > *itBiggestBlob){
				indexBiggest = i;
				itBiggestBlob = itEvalBlobs;
			}
		}
		//cout << "index Biggest: " << indexBiggest << endl;
		finalBlob = new CBlob(blobs->GetBlob(indexBiggest));
		blobs->ClearBlobs();
		blobs->AddBlob(finalBlob);
        delete finalBlob;
	}
		
}

void BlobFunctions::filterClosest(CBlobResult *blobs, CvPoint target){
	
	int i, numBlobs, indexClosest = 0;
	numBlobs = blobs->GetNumBlobs();
	float minDistance = FLT_MAX;
	float tmpDistance;
	
	if (numBlobs > 0){
		
		double_stl_vector evalBlobsX, evalBlobsY;
		double_stl_vector::iterator itEvalBlobsX, itEvalBlobsY;
		double_stl_vector::iterator itClosestBlob;
		
		CBlob *finalBlob;
		// this returns blobs wrapped in CBlobGetArea operadorBlobs!
		evalBlobsX = blobs->GetSTLResult(CBlobGetXCenter());
		evalBlobsY = blobs->GetSTLResult(CBlobGetYCenter());
		itEvalBlobsX = evalBlobsX.begin();
		itEvalBlobsY = evalBlobsY.begin();
			
		for(i=0; i < numBlobs; i++, itEvalBlobsX++, itEvalBlobsY++){
			tmpDistance = pow((float)((*itEvalBlobsX)-target.x),2) + pow((float)((*itEvalBlobsY)-target.y),2);
			if( tmpDistance < minDistance){
				indexClosest = i;
				minDistance = tmpDistance;
			}
		}
		finalBlob = new CBlob(blobs->GetBlob(indexClosest));
		blobs->ClearBlobs();
		blobs->AddBlob(finalBlob);
        delete finalBlob;
	}
	
}

void BlobFunctions::filterRandom(CBlobResult *blobs){
	
	
	int numBlobs = blobs->GetNumBlobs();
	
	if (numBlobs > 0){
		CBlob *finalBlob;
		int rndBlobIndex;
		rndBlobIndex = (int)(numBlobs * ( (double)rand() / ((double)(RAND_MAX)+(double)(1.0)) ));
		//cout << "rnd: " << rndBlobIndex << endl;
		finalBlob = new CBlob(blobs->GetBlob(rndBlobIndex));
		blobs->ClearBlobs();
		blobs->AddBlob(finalBlob);
        delete finalBlob;
	}
	
	
}

void BlobFunctions::makeBlackBorder(IplImage* image, int borderSize){

	int arraypos = 0;
	int nChannels = image->nChannels;
    int y;  // to make visual studio 6 happy :-(

	// different channels are handled
	if( image->depth == IPL_DEPTH_8U){

		// left side
		for (y = 0; y < image->height; y++){
			arraypos = image->widthStep*y;
			for (int x = 0; x < borderSize; x++){
				for (int n = 0; n < nChannels; n++){
					((uchar*)(image->imageData + arraypos))[x*nChannels+n] = (uchar)0;
				}
			}
		}

		// right side
		for (y = 0; y < image->height; y++){
			arraypos = image->widthStep*y;
			for (int x = 0; x < borderSize; x++){
				for (int n = -1; n > -(nChannels+1); n--){
				((uchar*)(image->imageData + arraypos))[image->widthStep - x*nChannels +n] = (uchar)0;
				}
			}
		}

		// top 
		for (y = 0; y < borderSize; y++){
			arraypos = image->widthStep*y;
			for (int x = 0; x < image->width; x++){
				for (int n = 0; n < nChannels; n++){
					((uchar*)(image->imageData + arraypos))[x*nChannels+n] = (uchar)0;
				}
			}
		}

		// bottom
		for (y = image->height - borderSize; y < image->height; y++){
			arraypos = image->widthStep*y;
			for (int x = 0; x < image->width; x++){
				for (int n = 0; n < nChannels; n++){
					((uchar*)(image->imageData + arraypos))[x*nChannels+n] = (uchar)0;
				}
			}
		}
	}
	else if( image->depth == IPL_DEPTH_32F){

		// left side
		for (y = 0; y < image->height; y++){
			arraypos = image->widthStep*y;
			for (int x = 0; x < borderSize; x++){
				for (int n = 0; n < nChannels; n++){
					((float*)(image->imageData + arraypos))[x*nChannels+n] = 0.0f;
				}
			}
		}

		// right side
		for (y = 0; y < image->height; y++){
			arraypos = image->widthStep*y;
			for (int x = 0; x < borderSize; x++){
				for (int n = -1; n > -(nChannels+1); n--){
				((float*)(image->imageData + arraypos))[image->widthStep - x*nChannels -n] = 0.0f;
				}
			}
		}

		// top 
		for (y = 0; y < borderSize; y++){
			arraypos = image->widthStep*y;
			for (int x = 0; x < image->width; x++){
				for (int n = 0; n < nChannels; n++){
					((float*)(image->imageData + arraypos))[x*nChannels+n] = 0.0f;
				}
			}
		}

		// bottom
		for (y = image->height - borderSize; y < image->height; y++){
			arraypos = image->widthStep*y;
			for (int x = 0; x < image->width; x++){
				for (int n = 0; n < nChannels; n++){
					((float*)(image->imageData + arraypos))[x*nChannels+n] = 0.0f;
				}
			}
		}
	}
	else {
		printf("AilImageProcessing::makeBlackBorder() Image has to be of type IPL_DEPTH_8U or IPL_DEPTH_32F.");
	}
}

void BlobFunctions::grayFromFloat(IplImage* imgFloat, IplImage* imgGray, float scale){

    float flValue = 0.0f;
    int   arraypos = 0;
 
    for (int y = 0; y < imgFloat->height; y++){
		arraypos = imgGray->widthStep*y;
        for (int x = 0; x < imgFloat->width; x++){
            flValue = scale * ((float*)(imgFloat->imageData + imgFloat->widthStep*y))[x];
            if (flValue < 0.0f){
                flValue = -flValue;
                if (flValue > 255.0f)
                    flValue = 255.0f;
                //cout << "<0.0: " << flValue << endl;
                ((uchar*)(imgGray->imageData + arraypos))[x] = (uchar)flValue;
            }
            else if (flValue > 0.0f){
                if (flValue > 255.0f)
                    flValue = 255.0f;
                //cout << ">0.0: " << flValue << endl;
                ((uchar*)(imgGray->imageData + arraypos))[x] = (uchar)flValue;
            }
            else{
                ((uchar*)(imgGray->imageData + arraypos))[x] = (uchar)0;
            }
        }
    }   
}
