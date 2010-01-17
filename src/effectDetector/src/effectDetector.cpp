//PROBLEMS:
//* some initialization values are hardcoded, instead of read from an initialization file.


// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Alex Bernardino (VisLab/ISR/IST)
 */

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>

//OpenCV
#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <math.h>

#include <iCub/effectDetector.h>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;






//global variables - required by callback functions
IplImage *imagetodisplay = 0;
int select_object = 0;  //0: Not selecting an object, 1: Selecting an object
int track_object = 0;   //0: Idle, -1: Object just selected, 1: Object with valid color model 
CvPoint origin;
CvRect selection;
int vmin, vmax, smin;

//globalTimer - measures duration of the application
YarpTimer globalTimer;
// cycleTimer - measures the duration of each image cycle
YarpTimer cycleTimer;
// copyTimer counts the time required for image copy 
// done when frame changes
YarpTimer copyTimer;
// colorTimer counts the time required for color conversion 
// done when frame or roi change
YarpTimer colorTimer;
// thresholdTimer counts the time required for thresholding operations 
// done when frame or roi or thresholds change
YarpTimer thresholdTimer;
// modelTimer counts the time required for computing the histogram 
// done when object selection changes
YarpTimer modelTimer;
// trackTimer counts the time required for computing the backproject and camshift tracking 
// done when tracking is active and frame or threshold or roi or object selection change 
YarpTimer trackTimer;
// readTimer counts the time required for polling and reading data from the ports 
YarpTimer readTimer;
// writeTimer counts the time required for writing data to the ports 
YarpTimer writeTimer;
// interfaceTimer counts the time required for user interface (roi selection)
YarpTimer interfaceTimer;
// drawTimer counts the time required for image overlay drawing
YarpTimer drawTimer;
// displayTimer counts the time required for image display
YarpTimer displayTimer;

void on_mouse( int event, int x, int y, int flags, void* param )
{
	interfaceTimer.start();
    if( !imagetodisplay )
        return;

    if( imagetodisplay->origin )
        y = imagetodisplay->height - y;

//     if( select_object )
//     {
//         selection.x = MIN(x,origin.x);
//         selection.y = MIN(y,origin.y);
//         selection.width = selection.x + CV_IABS(x - origin.x);
//         selection.height = selection.y + CV_IABS(y - origin.y);
//         
//         selection.x = MAX( selection.x, 0 );
//         selection.y = MAX( selection.y, 0 );
//         selection.width = MIN( selection.width, imagetodisplay->width );
//         selection.height = MIN( selection.height, imagetodisplay->height );
//         selection.width -= selection.x;
//         selection.height -= selection.y;
//     }

    switch( event )
    {
    case CV_EVENT_LBUTTONDOWN:
        origin = cvPoint(x,y);
        selection = cvRect(x,y,0,0);
        select_object = 1;
		track_object = 0;
        break;
    case CV_EVENT_LBUTTONUP:
        select_object = 0;
        if( selection.width > 0 && selection.height > 0 )
            track_object = -1;
        break;
    }
    
    cvSetTrackbarPos( "Vmin", "CamShift", vmin );
    cvSetTrackbarPos( "Vmax", "CamShift", vmax );
    cvSetTrackbarPos( "Smin", "CamShift", smin );

    interfaceTimer.stop();
    interfaceTimer.endlap();
}




EffectDetector::EffectDetector()
{
};
EffectDetector::~EffectDetector()
{
};

float EffectDetector::computeSimilarity(IplImage *rawSegmImg, IplImage *rawCurrImg, int u, int v, int width, int height)
{
  int a, b;
  int r1, r2, g1, g2, b1, b2;
  float distance=0;
  for(a=u;a<u+width;a++)
  {
    for(b=v;b<v+height;b++)
    {
      r1=(((uchar*)(rawSegmImg->imageData + rawSegmImg->widthStep*v))[u*3+0]);
      g1=(((uchar*)(rawSegmImg->imageData + rawSegmImg->widthStep*v))[u*3+1]);
      b1=(((uchar*)(rawSegmImg->imageData + rawSegmImg->widthStep*v))[u*3+2]);
      r2=(((uchar*)(rawCurrImg->imageData + rawCurrImg->widthStep*v))[u*3+0]);
      g2=(((uchar*)(rawCurrImg->imageData + rawCurrImg->widthStep*v))[u*3+1]);
      b2=(((uchar*)(rawCurrImg->imageData + rawCurrImg->widthStep*v))[u*3+2]);
      
      distance+=(r1-r2)*(r1-r2)+(g1-g2)*(g1-g2)+(b1-b2)*(b1-b2);

    }
  }
      
  distance=sqrt(distance/(width*height*3))/255;

  return 1.0 - distance;
}





bool EffectDetector::configure(ResourceFinder &config) // equivalent to Module::open()
{
    if (config.check("help","if present, display usage message")) {
	printf("Call with --name </portprefix> --from <configfile.ini>\n");
	return false;
    }

	_default_vmin = config.check("default_vmin", 
		Value(0), 
		"If 0 use value from port, otherwise use this value").asInt();

    //set state of the system
    state = NOTPROCESSING;
    firstImageEver = true; //this will be turnet to false when the OpenCV image buffers are created.

    
    initPort.open(        getName("/init")                   );
    rawCurrImgPort.open(  getName("/rawcurrimg")             );
    rawSegmImgPort.open(  getName("/rawsegmimg")             );
    effectPort.open(      getName("/effect")                 ); 
    errorPort.open(       getName("/error")                  );             
    controlGazePort.open( getName("/effect2ControlGaze")     );             // Output: (normalized u, normalized v,"p") 

    rawCurrImg=NULL; //Make sure this pointer is initialized
    rawSegmImg=NULL; //Make sure this pointer is initialized
    tempImg1=NULL;   //Make sure this pointer is initialized
    tempImg2=NULL;   //Make sure this pointer is initialized



    buffer = 0; 
    image = 0; 
    hsv = 0; 
    hue = 0;
    mask = 0;
    maskTEST = 0;
    backproject = 0;
    histimg = 0;
    hist = 0;
    backproject_mode = 0;
    show_hist = 1;
    track_window;
    track_box;
    track_comp;
    hdims = 16;
    hranges_arr[0] = 0;
    hranges_arr[1] = 180;
    hranges = hranges_arr;
    
    //
    in=0;
    inRoi = 0;
    

    //These are set in the respond() method now.
    //     vmin = 0;
    //     vmax = 256;
    //     smin = 0;
    //     _vmin = vmin;
    //     _vmax = vmax;
    //     _smin = smin;
    simThreshold=0.5;
    attach(initPort); //all that is needed in the respond() method should be set up before this call.
        
    
    
    cvNamedWindow( "CamShift", 1 );
    cvResizeWindow( "CamShift", 640, 480);
    cvMoveWindow( "CamShift", 1400, 5);
    cvNamedWindow( "Histogram", 1 );
    cvResizeWindow( "Histogram", 320, 240);
    cvMoveWindow( "Histogram", 1400, 850);
    cvNamedWindow( "vFilter", 1 );
    cvResizeWindow( "vFilter", 320, 240);
    cvMoveWindow( "vFilter", 1400, 500);
    
    return true;
};



bool EffectDetector::respond(const Bottle & command, Bottle & reply)
{
  //this method is called any time the port that is attached to it (/init) receives a message
  cout<<"RESPOND(): has received a message and is starting.\n";

  //the data in the bottle must be arranged as:
  //u - initial position of the ROI
  //v - initial position of the ROI
  //width of the ROI
  //height of the ROI
  //h0 - value for the histogram bin 0
  //...
  //h15 - value for the histogram bin 15
  //vmin
  //vmax
  //smin
  
  int u =      (command.get(0)).asDouble();
  int v =      (command.get(1)).asDouble();
  int width =  (command.get(2)).asDouble();
  int height = (command.get(3)).asDouble();
  float similarity;
  float max_val;


  //*******************************************************
  //1. read rawSegmImg, convert it to HSV and OpenCV format.
  //*******************************************************
  yarpImg=rawSegmImgPort.read(); //warning: blocking read.
  tempImg1 = (IplImage*)yarpImg->getIplImage(); //the memory is somehow shared by the OpenCV and the YARP image... I should not release this memory

  //FIX: the selected ROI can be outside of the image: trim it to the image limits.
  if(width>(tempImg1->width-1-u))
  {
    width=tempImg1->width-1-u;
    cout<<"new width="<<width<<endl;
  }
  if(height>(tempImg1->height-1-v))
  {
    height=tempImg1->height-1-v;
    cout<<"new height="<<height<<endl;
  }
  if((height<=0)||(width<=0))
  {
    cout<<"One of the sizes of the ROI is NULL on negative: I cannot initialize the tracker like this."<<endl;
    cout<<"height="<<height<<endl;
    cout<<"width="<<width<<endl;
    reply.addInt(0);
    state=NOTPROCESSING;

  }
  else
  {
    if((u<0)||(v<0)||(u>=tempImg1->width)||(v>=tempImg1->height))
    {
      cout<<"The top-left corner of the specified ROI is outside the image: I cannot initialize the tracker like this."<<endl;
      cout<<"left="<<u<<endl;
      cout<<"top="<<v<<endl;
      reply.addInt(0);
      state=NOTPROCESSING;
    }
    else
    {
      //check if the memory for the tempImg2 buffer has already been allocated, otherwise allocate it now.
      if(tempImg2==NULL)
      {
	tempImg2 = cvCreateImage(cvSize(tempImg1->width, tempImg1->height), tempImg1->depth, tempImg1->nChannels);
      }
      //check if the memory for rawSegmImg has already been allocated, otherwise allocate it now.
      if(rawSegmImg==NULL)
      {
	rawSegmImg = cvCreateImage(cvSize(tempImg1->width, tempImg1->height), tempImg1->depth, tempImg1->nChannels);
      }

      cvCvtColor( tempImg1, tempImg2, CV_RGB2BGR);  //RGB->BGR //this function does not allocate memory, right?
      cvCvtColor( tempImg2, rawSegmImg, CV_BGR2HSV);//BGR->HSV //this function does not allocate memory, right?



      //******************************************************************
      //2. if needed, read rawCurrImg, convert it to HSV and OpenCV format.
      //******************************************************************
      if(rawCurrImg==NULL)
      {
	yarpImg=rawCurrImgPort.read(); //warning: blocking read.
	tempImg1 = (IplImage*)yarpImg->getIplImage(); //the memory is somehow shared by the OpenCV and the YARP image... I shouldn't release this memory

	//check if the memory for the tempImg2 buffer has already been allocated, otherwise allocate it now.
	if(tempImg2==NULL)
	{
	  tempImg2 = cvCreateImage(cvSize(tempImg1->width, tempImg1->height), tempImg1->depth, tempImg1->nChannels);
	}
	//check if the memory for rawCurrImg has already been allocated, otherwise allocate it now.
	if(rawCurrImg==NULL)
	{
	  rawCurrImg = cvCreateImage(cvSize(tempImg1->width, tempImg1->height), tempImg1->depth, tempImg1->nChannels);
	}
	
	cvCvtColor( tempImg1, tempImg2, CV_RGB2BGR);  //RGB->BGR //this function does not allocate memory, right?
	cvCvtColor( tempImg2, rawCurrImg, CV_BGR2HSV);//BGR->HSV //this function does not allocate memory, right?
      }

      //cvSaveImage("currImg.jpeg", rawCurrImg); //save with false colors: OpenCV assumes the images are coded as BGR.
      //cvSaveImage("segmImg.jpeg", rawSegmImg); //save with false colors: OpenCV assumes the images are coded as BGR.


      //************************************************************
      //3. compute the similarity between the ROI on the two images.
      //************************************************************

      //similarity assumes the HSV pixels have minimum values of [0,0,0] and maximum values of [255,255,255]
      similarity=computeSimilarity(rawSegmImg, rawCurrImg, u, v, width, height);
      
      cout<<"RESPOND(): similarity= "<<similarity<<endl;



      //*****************************************************
      //4. set new state, reply to the init message. in case,
      //   create the new initial histogram and set limits   
      //*****************************************************
      if(similarity>simThreshold)
      {
    
	//??? !!! ??? not sure if this is right ??? !!! ???
	hranges_arr[0] = 0;
	hranges_arr[1] = 180;
	hranges = hranges_arr;
	//create the histogram
	hist = cvCreateHist( 1, &hdims, CV_HIST_ARRAY, &hranges, 1 );
	//fill the histogram with the values received from the port
	int count;
	for(count=0;count<hdims;count++)
	{
	  *(cvGetHistValue_1D(hist, count))=(command.get(count+4)).asDouble(); //start with the fift value in the bottle
	}

	//rescale the histogram so that it covers the range [0..255]
	//should I do it? ??? !!! ???
	//cvGetMinMaxHistValue( hist, 0, &max_val, 0, 0 );
	//cvConvertScale( hist->bins, hist->bins, max_val ? 255. / max_val : 0., 0 );

	//set data that's going to be used in the update() method.
	    if(_default_vmin)
		    _vmin = _default_vmin;
	    else
		    _vmin = (command.get(20)).asDouble();
	_vmax = (command.get(21)).asDouble();
	_smin = (command.get(22)).asDouble();
	vmin=_vmin; //global variables, needed by the callback functions
	vmax=_vmax; //global variables, needed by the callback functions
	smin=_smin; //global variables, needed by the callback functions
	_w = tempImg1->width;
	_h = tempImg1->height;

	//tracking window initialization: position of the upper left corner and size.
	track_window.x=u;
	track_window.y=v;
	track_window.width=width;
	track_window.height=height;
	
	//if it is the first time that we decide to start tracking, initialize the stuff OpenCV needs for meanShift
	if(firstImageEver)
	{
	  cvSetMouseCallback( "CamShift", on_mouse, 0 );
	  cvCreateTrackbar( "Vmin", "CamShift", &vmin, 256, 0 );
	  cvCreateTrackbar( "Vmax", "CamShift", &vmax, 256, 0 );
	  cvCreateTrackbar( "Smin", "CamShift", &smin, 256, 0 );
      
	  CvSize sz = cvSize(_w,_h);
	  buffer = cvCreateImage(sz, 8, 3 );
	  image = cvCreateImage( sz, 8, 3 );
	  imagetodisplay = cvCreateImage( sz, 8, 3 );
	  hsv = cvCreateImage( sz, 8, 3 );
	  hue = cvCreateImage( sz, 8, 1 );
	  mask = cvCreateImage( sz, 8, 1 );
	  maskTEST = cvCreateImage( sz, 8, 1 );
	  
	  //cout<<"RESPOND(): mask = "<<mask<<endl;
	  histimg = cvCreateImage( cvSize(320,200), 8, 3 );
      
	  out.resize(_w,_h);
	  backproject = (IplImage*)out.getIplImage(); //backproject (opencv image) is associated with out (yarp image)
	  //so that when backproject is computed, also out is ready to be sent out.
	  

	  refreshDelay = 10;
	  timeToRefresh = 10;
	  overlaydisplay = false;
	  firstImageEver = false;
	}

	//Prepares histogram image buffer
	cvZero( histimg );
	//prepare histogram image
	bin_w = histimg->width / hdims;
	for( i = 0; i < hdims; i++ )
	{
	  cout<<"RESPOND(): histogram "<<cvGetReal1D(hist->bins,i)<<"\n";
	  int val = cvRound( cvGetReal1D(hist->bins,i)*histimg->height/255 );
	  CvScalar color = hsv2rgb(i*180.f/hdims);
	  cvRectangle( histimg, cvPoint(i*bin_w,histimg->height),
	  cvPoint((i+1)*bin_w,histimg->height - val),
	  color, -1, 8, 0 );
	}
	reply.addInt(1);
	state=PROCESSING;
      }
      else
      {
	reply.addInt(0);
	state=NOTPROCESSING;
      }
    }
  }
  
  cout<<"RESPOND(): returning.\n";
  return true; //everything is fine
}






bool EffectDetector::close()
{
    initPort.close();
    rawCurrImgPort.close();
    rawSegmImgPort.close();
    effectPort.close();
    errorPort.close();             
    controlGazePort.close();
    
    //fclose(fp);

    return true;
};




bool EffectDetector::interruptModule()
{
    initPort.interrupt();
    rawCurrImgPort.interrupt();
    rawSegmImgPort.interrupt();
    effectPort.interrupt();
    errorPort.interrupt();             
    controlGazePort.interrupt();
    
    return true;
};




CvScalar EffectDetector::hsv2rgb( float hue )
{
    int rgb[3], p, sector;
    static const int sector_data[][3]=
	{{0,2,1}, {1,2,0}, {1,0,2}, {2,0,1}, {2,1,0}, {0,1,2}};
    hue *= 0.033333333333333333333333333333333f;
    sector = cvFloor(hue);
    p = cvRound(255*(hue - sector));
    p ^= sector & 1 ? 255 : 0;

    rgb[sector_data[sector][0]] = 255;
    rgb[sector_data[sector][1]] = 0;
    rgb[sector_data[sector][2]] = p;

    return cvScalar(rgb[2], rgb[1], rgb[0],0);
}





bool EffectDetector::updateModule()
{
  cvWaitKey(1);

  //read the new image, if there is one.
  yarpImg=rawCurrImgPort.read(false); //non-blocking read.
  if(yarpImg!=NULL)
  {
    tempImg1 = (IplImage*)yarpImg->getIplImage(); //the memory is somehow shared by the OpenCV and the YARP image... I shouldn't release this memory

    //check if the memory for the tempImg2 buffer has already been allocated, otherwise allocate it now.
    if(tempImg2==NULL)
    {
      tempImg2 = cvCreateImage(cvSize(tempImg1->width, tempImg1->height), tempImg1->depth, tempImg1->nChannels);
    }
    //check if the memory for rawCurrImg has already been allocated, otherwise allocate it now.
    if(rawCurrImg==NULL)
    {
      rawCurrImg = cvCreateImage(cvSize(tempImg1->width, tempImg1->height), tempImg1->depth, tempImg1->nChannels);
    }
    
    cvCvtColor( tempImg1, tempImg2, CV_RGB2BGR);  //RGB->BGR //this function does not allocate memory, right?
    cvCvtColor( tempImg2, rawCurrImg, CV_BGR2HSV);//BGR->HSV //this function does not allocate memory, right?



    if(state==PROCESSING)
    {
	_vmin=vmin;
	_vmax=vmax;
	_smin=smin;
	cout<<"vmin= "<<_vmin<<" vmax="<<_vmax<<" smin="<<_smin<<endl;
	// thresholds the saturation channel - creates a mask indicating pixels with "good" saturation. 
	cvInRangeS( rawCurrImg, cvScalar(0,_smin,MIN(_vmin,_vmax+1),0),
				cvScalar(180,256,MAX(_vmin,_vmax+1),0), mask );
	// splits the hue channel from the color image
	cvSplit( rawCurrImg, hue, 0, 0, 0 );

	//TEST: which pixels have V==255?
	cvInRangeS( rawCurrImg, cvScalar(0,0,255,0),
				cvScalar(181,256,256,0), maskTEST );
	cvShowImage( "vFilter", maskTEST );


	// segments image pixels with good match to the histogram
	cvCalcBackProject( &hue, backproject, hist );
	//cvSaveImage("backprojectec.jpeg",backproject);
	cvAnd( backproject, mask, backproject, 0 );

	// searches iterativelly for the closest center of mass to the current search window 
	// Returns:
	//    track_window - contains object bounding box
	//    track box - contains object size and orientation
	cvCamShift( backproject, track_window,
				cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ),
				&track_comp, &track_box );

	// set the search window for the next time
	track_window = track_comp.rect;

	//what are these? Jonas's coordinates?
	double xnorm, ynorm;
	xnorm = 2*track_box.center.x/_w - 1;
	ynorm = 2*track_box.center.y/_h - 1;
	cout<<"Update: X="<<track_box.center.x<<" Y="<<track_box.center.y<<"\n";

	//Write data on the output port
	Bottle& output=effectPort.prepare();
	output.clear();
	output.addDouble(track_box.center.x);
	output.addDouble(track_box.center.y);
	effectPort.write();

	//Write data to Control Gaze
	Bottle& outputCG=controlGazePort.prepare();
	outputCG.clear();
	outputCG.addDouble(xnorm);
	outputCG.addDouble(ynorm);
	outputCG.addDouble((double)(int)('p'));
	controlGazePort.write();

	cvSetTrackbarPos( "Vmin", "CamShift", vmin );
	cvSetTrackbarPos( "Vmax", "CamShift", vmax );
	cvSetTrackbarPos( "Smin", "CamShift", smin );

	//prepare image to be shown
	cvCvtColor( backproject, image, CV_GRAY2BGR );
	if( !image->origin )
	{
	track_box.angle = -track_box.angle;
	}
	cvEllipseBox( image, track_box, CV_RGB(255,0,0), 3, CV_AA, 0 );


	cvShowImage( "CamShift", image );
	cvShowImage( "Histogram", histimg );
	//c = cvWaitKey(1);
	c = cvWaitKey(1);
    

    }
  }
  

  return true;
}

double EffectDetector::getPeriod() {
  return 0.0;
}
