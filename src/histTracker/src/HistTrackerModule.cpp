// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2007 Plinio Moreno
* CopyPolicy: Released under the terms of the GNU GPL v2.0.
*
*/

#include <iCub/HistTrackerModule.h>

HistTrackerModule::HistTrackerModule(){
	active = false;
	anyTrackerActive = false;
	//needInit = true;
}

HistTrackerModule::~HistTrackerModule(){
	// data is released trough close/interrupt methods
}

bool HistTrackerModule::open(Searchable& config){

	if (config.check("help","if present, display usage message")) {
		printf("Call with --name /module_prefix --file configFile.ini --group CONFIGURATION_GROUP\n");
		return false;
	}

	oneTargetShortTerm = (config.check("oneTargetShortTerm",
                                    Value(0),
									"Flag to active short term tracking").asInt()!=0);
	/*_numPatternImagesRequired = config.check("numPatternImagesRequired",
	Value(25),
	"Number of images of the pattern required to run calibration (int").asInt();*/




	//bool initResult;
	//_prtImg.open(getName("image"));
	//printf("Before port opening\n");
	//if ( needInit ){
	_prtImgRgb.open(getName("rgb"));
	//printf("resRgb: %d\n",(int)resRgb);
	_detectorInput.open(getName("detectIn"));
	_prtImgTrackedRegionsRgb.open(getName("out"));
	_trackersignalOutput_port.open(getName("normOut"));
	_emotionOutput.open(getName("emotionOut"));
	//}
	
	/*
	if (!active){
		DetectorOutput *detectorInput = _detectorInput.read(false);
		if ( detectorInput == NULL)
			return false;
		if ( detectorInput->eyeFlag ){
			//printf("Entered to print out object\n");
			if ( detectorInput->nFaces > 0 ){
				list<VisualObject *>::iterator face = detectorInput->facesEyes.begin();
				list<VisualObject *>::iterator last_face = detectorInput->facesEyes.end();
				for(int i = 0; face != last_face; ++face, ++i){
					// Current face plot
					FaceObject *f = static_cast<FaceObject*>(*face);
					HistTracker *currentTracker = new HistTracker();
					currentTracker->open(config);
					currentTracker->learnModel( *(detectorInput->detectImage), cvRect( f->x, f->y, f->xSize, f->ySize ) );
					trackers.push_front(currentTracker);
				}
			}
		}
		else{
			//printf("Faces only print out\n");
			int nBoxes = detectorInput->facesOnly.size();
			if(nBoxes != 0) {
				while(!detectorInput->facesOnly.empty( ))
				{
					Square face = detectorInput->facesOnly.front();  
					detectorInput->facesOnly.pop_front();
					HistTracker *currentTracker = new HistTracker();
					currentTracker->open(config);
					currentTracker->learnModel( *(detectorInput->detectImage), cvRect( face.x, face.y, face.size, face.size ) );
					trackers.push_front(currentTracker);
				}
			}
		}
		active = true;
		//needInit = false;
	}
	*/
	trackerSetup.fromString(config.toString().c_str());
	//printf("After port opening\n");
	return true;
}

bool HistTrackerModule::close(){

	//_prtImg.close();
	_detectorInput.close();
	_prtImgRgb.close();
	_prtImgTrackedRegionsRgb.close();
	_emotionOutput.close();
	_trackersignalOutput_port.close();
	/*if (_ocvImgIn != NULL)
	cvReleaseImage(&_ocvImgIn);
	_ocvImgIn = NULL;
	if (_ocvImgTmp1 != NULL)
	cvReleaseImage(&_ocvImgTmp1);
	_ocvImgTmp1 = NULL;
	if (_ocvImgTmp2 != NULL)
	cvReleaseImage(&_ocvImgTmp2);
	_ocvImgTmp2 = NULL;
	if (_ocvImgOut != NULL)
	cvReleaseImage(&_ocvImgOut);
	_ocvImgOut = NULL;*/


	return true;
}

bool HistTrackerModule::interruptModule(){
	_detectorInput.interrupt();
	_prtImgRgb.interrupt();
	_prtImgTrackedRegionsRgb.interrupt();
	_trackersignalOutput_port.interrupt();
	_emotionOutput.interrupt();
	//_prtImg.interrupt();
	return true;
}

bool HistTrackerModule::updateModule(){

    //synchronizes with the input image
	/*yarp::sig::ImageOf<PixelRgb> *yrpImgRgbIn = _prtImgRgb.read(true);
    if(yrpImgRgbIn == NULL) //request to exit
        return false;*/

	DetectorOutput *detectorInput = _detectorInput.read(true);
    if( detectorInput == NULL ) //asked to quit
        return false;
    yarp::sig::ImageOf<PixelRgb>* yrpImgRgbIn = detectorInput->detectImage; 
    if( detectorInput->nFaces > 0 ) //New detection
    {
        if( oneTargetShortTerm || (!active) ) //create whole new list of targets
        {
            list<HistTracker *>::iterator currentTracker = trackers.begin();
		    list<HistTracker *>::iterator lastTracker = trackers.end();
            for(; currentTracker != lastTracker; ++currentTracker){
			    //printf("i: %d\n",i);
			    HistTracker *hTracker = *currentTracker;
                delete hTracker;
            }
		    trackers.clear();

            if ( detectorInput->eyeFlag )
            {
			    //printf("Entered to read %d faces\n",detectorInput->nFaces);
			    list<FaceObject *>::iterator face = detectorInput->facesEyes.begin();
				list<FaceObject *>::iterator last_face = detectorInput->facesEyes.end();
				//for(int i = 0; face != last_face; ++face, ++i){
				for(int i = 0; i < detectorInput->nFaces; ++face, ++i)
                {
					//printf("i: %d\n",i);
					// Current face plot
					FaceObject *f = static_cast<FaceObject*>(*face);
					HistTracker *currentTracker = new HistTracker();
					currentTracker->open(trackerSetup);
					currentTracker->learnModel( *(detectorInput->detectImage), cvRect( (int)f->x, (int)f->y, (int)f->xSize, (int)f->ySize ) );
					//printf("Before adding face to list\n");
					trackers.push_front(currentTracker);
					//printf("After adding face to list\n");
				}
			}
		    else // !detectorInput->eyeFlag 
            {
			    //printf("Faces only print out\n");
			    int nBoxes = detectorInput->nFaces;//detectorInput->facesOnly.size();
				//printf("nBoxes: %d\n",nBoxes);
				int myCont = 0;
			    if(nBoxes != 0) 
                {
				    //while(!detectorInput->facesOnly.empty( ))
					while ( myCont < nBoxes )
				    {
					    Square face = detectorInput->facesOnly.front();  
					    detectorInput->facesOnly.pop_front();
					    HistTracker *currentTracker = new HistTracker();
					    currentTracker->open(trackerSetup);
					    currentTracker->learnModel( *(detectorInput->detectImage), cvRect( face.x, face.y, face.size, face.size ) );
					    trackers.push_front(currentTracker);
						myCont++;
						//printf("Read face\n");
				    }
			    }
		    }
            active = true;
            list<HistTracker *>::iterator firstTracker = trackers.begin();
            HistTracker *hTracker = *firstTracker;
            //write the current detected face in the port
            Vector &normTrackOutput = _trackersignalOutput_port.prepare();
            normTrackOutput.resize(5);
            normTrackOutput[0] = 2*(double)(hTracker->track_window_actual.x+hTracker->track_window_actual.width/2)/(double)yrpImgRgbIn->width()-1;
            normTrackOutput[1] = 2*(double)(hTracker->track_window_actual.y+hTracker->track_window_actual.height/2)/(double)yrpImgRgbIn->height()-1;
            normTrackOutput[2] = 'r';
            normTrackOutput[3] = 'p';
			normTrackOutput[4] = hTracker->totalTrackTime;
            _trackersignalOutput_port.write();
        }
        else if( !oneTargetShortTerm && active )// !oneTargetShortTerm && active
        {
            //do not care about new detections
        }
    }
    else //  detectorInput == NULL
    {
        if( active )  // just track
        {
            list<HistTracker *>::iterator currentTracker = trackers.begin();
		    list<HistTracker *>::iterator lastTracker = trackers.end();
		    int i;
		    FaceBoxList tempResult= FaceBoxList();
		    DetectorOutput& outputPrepare = _trackerOutput.prepare();
		    outputPrepare.eyeFlag = false;
		    //outputPrepare.facesEyes = FaceBoxList();
		    //outputPrepare.facesOnly.push_front   nFaces = fEDetector.numberOfFacesFound;
		    anyTrackerActive = false;
		    for(i=0; currentTracker != lastTracker; ++currentTracker,++i){
			    //printf("i: %d\n",i);
			    HistTracker *hTracker = *currentTracker;
			    if ( hTracker->turnedOn ){
				    anyTrackerActive = true;
				    hTracker->track( *yrpImgRgbIn );
				    tempResult.push_back(TSquare<double> (hTracker->track_window.width, hTracker->track_window.x, hTracker->track_window.y, 0));
				    Vector &normTrackOutput = _trackersignalOutput_port.prepare();
				    normTrackOutput.resize(5);
				    normTrackOutput[0] = 2*(double)(hTracker->track_window_actual.x+hTracker->track_window_actual.width/2)/(double)yrpImgRgbIn->width()-1;
				    normTrackOutput[1] = 2*(double)(hTracker->track_window_actual.y+hTracker->track_window_actual.height/2)/(double)yrpImgRgbIn->height()-1;
                    normTrackOutput[2] = 'r';
                    normTrackOutput[3] = 'p';
					normTrackOutput[4] = hTracker->totalTrackTime;

				    _trackersignalOutput_port.write();
					//normTrackOutput.~Vector();
			    }
			    else{
				    tempResult.push_back(TSquare<double> (-1, -1, -1, 0));
			    }
		    }
		    outputPrepare.facesOnly = FaceBoxList(tempResult);
		    _trackerOutput.write();
		    if ( !anyTrackerActive ){
			    active = false;
                list<HistTracker *>::iterator currentTracker = trackers.begin();
		        list<HistTracker *>::iterator lastTracker = trackers.end();
                for(; currentTracker != lastTracker; ++currentTracker){
			        //printf("i: %d\n",i);
			        HistTracker *hTracker = *currentTracker;
                    delete hTracker;
                }
			    trackers.clear();
                Vector &normTrackOutput = _trackersignalOutput_port.prepare();
		        normTrackOutput.resize(5);
		        normTrackOutput[0] = 0.0;//2*(double)(hTracker->track_window_actual.x+hTracker->track_window_actual.width/2)/(double)yrpImgRgbIn->width()-1;
		        normTrackOutput[1] = 0.0;//2*(double)(hTracker->track_window_actual.y+hTracker->track_window_actual.height/2)/(double)yrpImgRgbIn->height()-1;
                normTrackOutput[2] = 'r';
                normTrackOutput[3] = 'p';
				normTrackOutput[4] = 0.0;

		        _trackersignalOutput_port.write();
		    }
			tempResult.clear();
			//tempResult.~FaceBoxList();

        }
        else  //!active
        {
            Vector &normTrackOutput = _trackersignalOutput_port.prepare();
	        normTrackOutput.resize(5);
	        normTrackOutput[0] = 0.0;//2*(double)(hTracker->track_window_actual.x+hTracker->track_window_actual.width/2)/(double)yrpImgRgbIn->width()-1;
	        normTrackOutput[1] = 0.0;//2*(double)(hTracker->track_window_actual.y+hTracker->track_window_actual.height/2)/(double)yrpImgRgbIn->height()-1;
            normTrackOutput[2] = 'r';
            normTrackOutput[3] = 'p';
			normTrackOutput[4] = 0.0;//hTracker->totalTrackTime;

	        _trackersignalOutput_port.write();
        }
    }
    yarp::sig::ImageOf<PixelRgb> yrpImgOut(*yrpImgRgbIn);
	// prepare output images
	//printf("update 3\n");
	ImageOf<PixelRgb>& yrpImgRgbOut = _prtImgTrackedRegionsRgb.prepare();
	drawTrackedRegions((IplImage*)yrpImgOut.getIplImage(),1,CV_RGB(128,0,128));
    yrpImgRgbOut = yrpImgOut;
	_prtImgTrackedRegionsRgb.write();
	Bottle &btmp = _emotionOutput.prepare();
    btmp.clear();
	if (active){
		btmp.addString("set");
		btmp.addString("all");
		btmp.addString("hap");
	}
	else{
		btmp.addString("set");
		btmp.addString("all");
		btmp.addString("neu");
	}
    _emotionOutput.write(true);
    return true;   
}

void HistTrackerModule::drawTrackedRegions(IplImage *currentImage, int lineThickness, CvScalar color){
	/*void drawBoundingBox(IplImage *currentImage, IplImage *imageCopy, 
	CvRect boundingBox, int lineThickness, CvScalar color)
	{*/
	CvPoint pt1, pt2;
	list<HistTracker *>::iterator currentTracker = trackers.begin();
	list<HistTracker *>::iterator lastTracker = trackers.end();
	for(; currentTracker != lastTracker; ++currentTracker){
		HistTracker *hTracker = *currentTracker;
		if ( hTracker->turnedOn ){
			//hTracker->track( *yrpImgRgbIn );
			pt1.x = hTracker->track_window.x;
			pt2.x = hTracker->track_window.x + hTracker->track_window.width;
			pt1.y = hTracker->track_window.y;
			pt2.y = hTracker->track_window.y + hTracker->track_window.height;
			if (currentImage->nChannels==1)
				cvRectangle( currentImage, pt1, pt2, cvScalar(255), lineThickness, 8, 0 );
			else if (currentImage->nChannels==3)
				cvRectangle( currentImage, pt1, pt2, color, lineThickness, 8, 0 );
		}
	}
}
