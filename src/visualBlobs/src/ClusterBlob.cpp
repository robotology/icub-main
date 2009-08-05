/*
 * Copyright (C) 2007 Jonas Ruesch
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */


#include <iCub/ClusterBlob.h>


ClusterBlob::ClusterBlob(){

    srand(static_cast<unsigned>(time(NULL)));
    _numBlobs = 0;
}

ClusterBlob::~ClusterBlob(){

}

void ClusterBlob::draw_clusters(IplImage* image, CBlobResult* blobs){
	
	// bbox values
	int iMaxx, iMinx, iMaxy, iMiny, iMeanx, iMeany;
	int i;
	
	//cout << "draw bbox: " << _blnDrawBBox << " area: " << _blnDrawArea << endl;
	
	// draw bounding box
	if (_blnDrawBBox){
		//cout << "inside" << endl;
		for (i = 0; i < blobs->GetNumBlobs(); i++){
			// get max, and min co-ordinates
			iMaxx=(int)blobs->GetBlob(i)->MaxX();
			iMinx=(int)blobs->GetBlob(i)->MinX();
			iMaxy=(int)blobs->GetBlob(i)->MaxY();
			iMiny=(int)blobs->GetBlob(i)->MinY();
			// find the average of the blob (i.e. estimate its centre)
			iMeanx=(iMinx+iMaxx)/2;
			iMeany=(iMiny+iMaxy)/2;
			// mark centre
			cvLine( image, cvPoint(iMeanx, iMeany), cvPoint(iMeanx, iMeany), CV_RGB(255, 0 , 0), 4, 8, 0 );
			// mark box around blob
			cvRectangle( image, cvPoint(iMinx , iMiny ), cvPoint ( iMaxx, iMaxy ), CV_RGB(255, 0, 0), 1, 8, 0);
		}
	}
	
	// draw area
	if (_blnDrawArea == 1)
		for (i = 0; i < blobs->GetNumBlobs(); i++)
			blobs->GetBlob(i)->FillBlob(image, CV_RGB(0, 255, 255));
}

void ClusterBlob::calculate_clusters(IplImage* inputImage, CBlobResult* blobs){
	
    int imageSize = inputImage->width * inputImage->height;
    
    // calculate blobs and add them to *blobs
    *blobs = CBlobResult(inputImage, NULL, _intThreshold, false) + *blobs;

	// sort out big blobs if required  
	blobs->Filter( *blobs, B_INCLUDE, CBlobGetArea(), B_LESS, _dblFilterLess * imageSize );

	// sort out small blobs if required
    blobs->Filter( *blobs, B_INCLUDE, CBlobGetArea(), B_GREATER, _dblFilterGreater * imageSize );
		
	switch (_intFilterOne){
		case FILTER_ONE_BIGGEST:
			BlobFunctions::filterBiggest(blobs);
			break;
			
		case FILTER_ONE_MIDDLE:
			BlobFunctions::filterClosest(blobs, cvPoint((int)(inputImage->width/2), (int)(inputImage->height/2)));
			//cout << "middle: " << inputImage->width/2 << " " << inputImage->height/2 << endl;
			break;
		case FILTER_ONE_RANDOM:
			BlobFunctions::filterRandom(blobs);
			//cout << "random" << endl;
			break;
        case FILTER_ONE_NO:
            break;
        default:
            break;
	}
	
	// remove blobs inside other blobs
	// TODO
	
	// merge blobs into one big pseudo-blob if requested
	// TODO
	/*
	 * create a new empty blob
	 * for each blob in blobs add it to blob (invent a +-function)
	 * clear all blobs in blobs
	 * add the new big blob to blobs
	 */
	
	_numBlobs = blobs->GetNumBlobs();
	//cout << "Number of detected blobs: " << blobs->GetNumBlobs() << endl;
      
}

bool ClusterBlob::open(Searchable &config){

    // blob config
    _dblFilterGreater = config.check("filterGreater",
                           Value(0.001),
                           "Filter blobs with area > value*imageSize.").asDouble();
    _dblFilterLess =    config.check("filterLess",
                           Value(0.8),
                           "Filter blobs with area < value*imageSize.").asDouble();
    _intThreshold =     config.check("threshold",
                           Value(0),
                           "Threshold (int [0,255]).").asInt();
    ConstString strOne= config.check("filterOne",
                           Value("no"),
                           "Filter one blob [no|biggest|middle|random].").asString();
    if      (strOne == "no")
        _intFilterOne = FILTER_ONE_NO;
	else if (strOne == "biggest")
		_intFilterOne = FILTER_ONE_BIGGEST;
	else if (strOne == "middle")
		_intFilterOne = FILTER_ONE_MIDDLE;
	else if (strOne == "random")
		_intFilterOne = FILTER_ONE_RANDOM;
    else
        _intFilterOne = FILTER_ONE_NO;

    // drawing
    _blnDrawBBox = (bool)config.check("drawBBox",
                                Value(1),
                                "Draw bounding box rectangle (0/1).").asInt();
    _blnDrawArea = (bool)config.check("drawArea",
                                Value(1),
                                "Draw blob area (0/1).").asInt();
	 	
    // some checks
   	if (_dblFilterGreater > 1.0)
   		_dblFilterGreater = 1.0;
   	if (_dblFilterGreater < 0.0)
   		_dblFilterGreater = 0.0;
   	if (_dblFilterLess > 1.0)
   		_dblFilterLess = 1.0;
   	if (_dblFilterLess < 0.0)
   		_dblFilterLess = 0.0;
   	if (_intThreshold < 0)
   		_intThreshold = 0;
   		
	return true;
}

int ClusterBlob::getNumBlobs(){
	return _numBlobs;	
}

