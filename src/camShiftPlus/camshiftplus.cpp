
/**
 * @ingroup icub_module
 *
 * \defgroup icub_camshiftplus camshiftplus
 *
 *
 *\section intro_sec Description 
 *
 * This module is a wrapper of the \ref camshift tracker in OpenCV. It also provides information about the tracked object.
 *
 *\section lib_sec Libraries
 *
 * OpenCV
 *
 *\section parameters_sec Parameters
 *
 *
 *\section portssa_sec Ports Accessed 
 *
 * Input ports\n
 * <ul>
 * <li> image from a camera (calibrated or not) 
 *</ul>
 *
*
*\section portsc_sec Ports Created
*
* Input ports\n
* /camshiftplus/img/i - Port to receive the image
* /camshiftplus/quit  - Quit port
* /camshiftplus/roi/i - Port to reset the tracke
*
* Output ports\n
* /camshiftplus/all/o     - all descriptors output
* /camshiftplus/color/o   - color descriptor 
* /camshiftplus/contour/o - contour descriptor
* /camshiftplus/img/o     - output image with overimposed information
* /camshiftplus/moments/roi/o - roi 
*
*\section conf_file_sec Configuration Files
*
* This module does not require configuration files
*
*\section tested_os_sec Tested OS
*
* This module has been tested on Linux and Windows. 
*
*\section example_sec Example Instantiation of the Module
*
* ./camshiftplus 
*
* This file can be edited at \in src/artoolkittracker/src
*
*\authors Manuel Lopes and Alexandre Bernardino
*
**/

#include <yarp/sig/Image.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/ImageFile.h>
#include <yarp/sig/ImageDraw.h>

#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Property.h>
#include <yarp/os/Terminator.h>

#include <iostream>
#include <string>
#include <math.h>
#include <cv.h>
#include <highgui.h>



//Port names

const char *DEFAULT_NAME = "/camshiftplus";



using namespace yarp;

using namespace yarp::os;

using namespace yarp::sig;



// helper function:

// finds a cosine of angle between vectors

// from pt0->pt1 and from pt0->pt2 

double angle( CvPoint* pt1, CvPoint* pt0, CvPoint* pt2 )

{

    double dx1 = pt1->x - pt0->x;

    double dy1 = pt1->y - pt0->y;

    double dx2 = pt2->x - pt0->x;

    double dy2 = pt2->y - pt0->y;

    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);

}



class YarpTimer 

{

private:

	double global_start, global_end, global_acc, lap_partial, lap_end, last_lap, lap_acc;

	int lap_counter;

public:

	YarpTimer()

	{

		reset();

	};



	~YarpTimer()

	{

	};



	

	void reset()

	{

		lap_counter = 1;

		last_lap = 0;

		global_acc = 0;

		lap_acc = 0;

		global_start = Time::now();

	};

		

	double now()

	{

		return Time::now() - global_start;

	};





	void endlap()

	{

		lap_counter++;

		global_acc += lap_acc;

		last_lap = lap_acc;

		lap_acc = 0;

		start();

	}



	void start()

	{

		lap_partial = Time::now();

	}

	void stop()

	{

		double delta = Time::now() - lap_partial;	

		lap_acc += delta;

	}

		

	// acessors

	int cyc()

	{

		return lap_counter;

	}



	double lap()

	{

		return lap_acc;

	};



	double lastlap()

	{

		return last_lap;

	};



	double tot()

	{

		return lap_acc;

	}



	double avg()

	{

		if(lap_counter)

			return (global_acc/lap_counter);

		else

			return 0;

	}

};



void namePorts(int argc, char *argv[], BufferedPort<ImageOf<PixelRgb> > & inPort, BufferedPort<ImageOf<PixelMono> > & outPort, BufferedPort<Bottle> & inRoiPort, BufferedPort<Vector> & outMomentsPort, BufferedPort<Vector> & outColorPort, BufferedPort<Vector> & outContourPort, BufferedPort<Vector> & outAllPort, Terminee **terminee)

{

    Property p;

    p.fromCommand(argc,argv);



    if (argc<=1) {

        printf("default port names are:\n");

        printf("    %s/img/i				(for incoming images)\n", DEFAULT_NAME);

        printf("    %s/img/o				(for outgoing (backprojected) images)\n", DEFAULT_NAME);

		printf("    %s/roi/i				(for input selection)\n", DEFAULT_NAME);

		printf("    %s/moments/o			(for object moments : x, y, width, height, angle)\n", DEFAULT_NAME);

		printf("    %s/color/o				(for object color histogram)\n", DEFAULT_NAME);

		printf("    %s/contour/o			(for object contour orientation histogram)\n", DEFAULT_NAME);

		printf("    %s/all/o			    (for all object descriptors)\n", DEFAULT_NAME);

        printf("can change \"%s\" prefix with --name option\n\n", DEFAULT_NAME);

    }

	ConstString name = p.check("name",Value(DEFAULT_NAME)).asString();

    std::string inImageName = name.c_str();
	inImageName += "/img/i";

    std::string outImageName = name.c_str();
	outImageName += "/img/o";

    std::string roiInName = name.c_str();
	roiInName += "/roi/i";

    std::string momentsOutName = name.c_str();
	momentsOutName += "/moments/roi/o";

	std::string colorOutName = name.c_str();
	colorOutName += "/color/o";

	std::string contourOutName = name.c_str();
	contourOutName += "/contour/o";

    std::string allOutName = name.c_str();

	allOutName += "/all/o";

	inPort.open(inImageName.c_str());
	outPort.open(outImageName.c_str());
	inRoiPort.open(roiInName.c_str());

	outMomentsPort.open(momentsOutName.c_str());
	outColorPort.open(colorOutName.c_str());
	outContourPort.open(contourOutName.c_str());
	outAllPort.open(allOutName.c_str());

	std::string term = name.c_str();

	term += "/quit";

    *terminee = new Terminee(term.c_str());
    if(*terminee == 0) 
		printf("Can't allocate terminator socket port\n");

    if (!((*terminee)->isOk())) 
       printf("Failed to create terminator socket port\n");
}

//global variables - required by callback functions

IplImage *imagetodisplay = 0;
int select_object = 0;  //0: Not selecting an object, 1: Selecting an object
int track_object = 0;   //0: Idle, -1: Object just selected, 1: Object with valid color model 
CvPoint origin;
CvRect selection;
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

    if( select_object )
    {
        selection.x = MIN(x,origin.x);
        selection.y = MIN(y,origin.y);
        selection.width = selection.x + CV_IABS(x - origin.x);
        selection.height = selection.y + CV_IABS(y - origin.y);
        
        selection.x = MAX( selection.x, 0 );
        selection.y = MAX( selection.y, 0 );
        selection.width = MIN( selection.width, imagetodisplay->width );
        selection.height = MIN( selection.height, imagetodisplay->height );
        selection.width -= selection.x;
        selection.height -= selection.y;
	
    }

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
	interfaceTimer.stop();
	interfaceTimer.endlap();
}


CvScalar hsv2rgb( float hue )
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

int main(int argc, char *argv[])
{
    Network::init();
	Time::turboBoost();

	IplImage *smoothed = 0, *image = 0, *hsv = 0, *hue = 0, *mask = 0, *objmask = 0,
 			 *backprojected, *segmented = 0, *edges=0, 
			 *camshifthistimg = 0, *colorhistimg = 0, *contourhistimg, 
			 *objbuffer = 0;

	
	CvHistogram *camshifthist = 0, *colorhist = 0, *contourhist = 0, *conditionalhist = 0;
	int backproject_mode = 0;
	int show_hist = 1;
	CvRect track_window;
	CvBox2D track_box;
	CvConnectedComp track_comp;
	int camshifthdims = 16;
	int colorhdims = 16;
	int contourhdims[] = {8,8};
	float camshifthranges_arr[] = {0,180};
	float colorhranges_arr[] = {0,180};
	float contourhranges_arr[] = {0,180};
	float* camshifthranges = camshifthranges_arr;
	float* colorhranges = colorhranges_arr;
	float* contourhranges[2];
	contourhranges[0] = contourhranges_arr;
	contourhranges[1] = contourhranges_arr;
	int vmin = 0, vmax = 256, smin = 0;
	int lo_diff = 10, up_diff = 10;
	int ffill_case = 1;
	int connectivity = 4;  //for flood fill with neighbor connectivity 4

	int conditionaldims = contourhdims[0]+contourhdims[1];
	float *conditionalhistdata = new float[conditionaldims];

	
	// storage for contours
	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq *contours = 0;
	CvSeq *convexhull = 0;

	ImageOf<PixelRgb> *in=0;
	ImageOf<PixelMono> out;
	Bottle *inRoi = 0;
    
	BufferedPort< ImageOf<PixelRgb> > inPort;
    BufferedPort< ImageOf<PixelMono> > outPort;
	BufferedPort<Bottle> inRoiPort;
	BufferedPort<Vector> outMomentsPort;
	BufferedPort<Vector> outColorPort;
	BufferedPort<Vector> outContourPort;
	BufferedPort<Vector> outAllPort;


	Terminee *terminee = 0;

	namePorts(argc, argv, inPort, outPort, inRoiPort, outMomentsPort, outColorPort, outContourPort, outAllPort, &terminee);

	while(in==0)
		in=inPort.read(true); //wait until connection is established

	//retrieve image dimensions
	const int width=in->width();
	const int height=in->height();

	//cvNamedWindow( "Camshift Histogram", 1 );
	//cvNamedWindow( "Color Histogram", 1 );
	//cvNamedWindow( "Contour Histogram", 1 );
    cvNamedWindow( "CamShiftPlus", 1 );
	//cvNamedWindow( "BackProjected", 1);
	//cvNamedWindow( "Segmented", 1);
	//cvNamedWindow( "Mask", 1);
	//cvNamedWindow( "Edges", 1);
    cvSetMouseCallback( "CamShiftPlus", on_mouse, 0 );
    cvCreateTrackbar( "Vmin", "CamShiftPlus", &vmin, 256, 0 );
    cvCreateTrackbar( "Vmax", "CamShiftPlus", &vmax, 256, 0 );
    cvCreateTrackbar( "Smin", "CamShiftPlus", &smin, 256, 0 );
	cvCreateTrackbar( "lo_diff", "Segmented", &lo_diff, 255, NULL );
    cvCreateTrackbar( "up_diff", "Segmented", &up_diff, 255, NULL );

	int _vmin = vmin, _vmax = vmax, _smin = smin, _lo_diff = lo_diff, _up_diff = up_diff;

	smoothed = cvCreateImage( cvSize(width,height), 8, 3 );
	image = cvCreateImage( cvSize(width,height), 8, 3 );
	imagetodisplay = cvCreateImage( cvSize(width,height), 8, 3 );
    hsv = cvCreateImage( cvSize(width,height), 8, 3 );
    hue = cvCreateImage( cvSize(width,height), 8, 1 );
	backprojected = cvCreateImage( cvSize(width,height), 8, 1 );
    mask = cvCreateImage( cvSize(width,height), 8, 1 );
	objmask = cvCreateImage( cvSize(width,height), 8, 1 );
	objbuffer = cvCreateImage( cvSize(width,height), 8, 1 );
	edges = cvCreateImage( cvSize(width+2,height+2), 8, 1 );
	camshifthist = cvCreateHist( 1, &camshifthdims, CV_HIST_ARRAY, &camshifthranges, 1 );
	camshifthistimg = cvCreateImage( cvSize(320,200), 8, 3 );
    colorhist = cvCreateHist( 1, &colorhdims, CV_HIST_ARRAY, &colorhranges, 1 );
    colorhistimg = cvCreateImage( cvSize(320,200), 8, 3 );
	//contourhist = cvCreateHist( 1, &contourhdims, CV_HIST_ARRAY, &contourhranges, 1 );
	contourhist = cvCreateHist(2, contourhdims, CV_HIST_ARRAY, contourhranges, 1 );
    contourhistimg = cvCreateImage( cvSize(320,200), 8, 3 );
	cvZero( colorhistimg );
    cvZero( contourhistimg );
	cvZero( camshifthistimg );

	out.resize(width,height);
	segmented = (IplImage*)out.getIplImage();
	
	int bin_w,i,c;

	
	double refreshDelay = 10;
	double timeToRefresh = refreshDelay;

	float max_val = 0.0f;
			

	bool newimage;
	bool newselection;
	bool onselection;
	bool ontracking;
	bool newtrackbar;		//trackbar changed position
	bool newtrackbar2;
	bool overlaydisplay = false; 

	double contour_area, contour_perimeter, convex_perimeter, major_axis, 
					minor_axis, rect_area, convexity, eccentricity, compactness, 
					circleness, squareness;


	//elapsedStart = Time::now();
	while(!terminee->mustQuit())
	{                  
		globalTimer.start();
		readTimer.start();

		in = inPort.read(false);						//read image
		if(in != 0)
			cycleTimer.start();
		
		inRoi = inRoiPort.read(false);					// read roi
		if((inRoi!=0)  && (inRoi->size() == 4))			// got a valid selection
		{
			track_object = -1;
			
		// getting position
		selection.x = (int)(inRoi->get(0).asDouble() * width/2.0 + width/2.0 - inRoi->get(2).asDouble()/2*width);
			selection.y = (int)(inRoi->get(1).asDouble() * height/2.0 + height/2.0 - inRoi->get(3).asDouble()/2*height);
			selection.width = (int)(inRoi->get(2).asDouble() * width);
			selection.height = (int)(inRoi->get(3).asDouble() * height);

			//check limits 
			if(selection.x < 0) selection.x = 0;
			if(selection.x >= width-1) selection.x = width-2;
			if(selection.y < 0) selection.y = 0;
			if(selection.y >= height-1) selection.y = height-2;
			if(selection.width <= 0) selection.width = 1;
			if(selection.height <= 0) selection.height = 1;
			if(selection.width+selection.x > width) selection.width = width - selection.x;
			if(selection.height+selection.y > height) selection.height = height - selection.y;

			//inicializacao do porto, temos que recalcular os thresholds
		}


		newimage = (in != 0);	
		onselection = ( select_object && selection.width > 0 && selection.height > 0 );
		newselection = (track_object == -1);
		ontracking = (track_object == 1);
		newtrackbar = ((_vmin != vmin) || (_vmax != vmax) || (_smin != smin ) );
		newtrackbar2 = ((_lo_diff != lo_diff ) || (_up_diff != up_diff ));


	

		if(newtrackbar2)
		{ 
			_lo_diff = lo_diff;
			_up_diff = up_diff;
		}

		readTimer.stop();

		if(newimage)  //get image into local memory - take the chance to convert to rgb
		{
    		readTimer.endlap();
			copyTimer.start();

			//Get input image into opencv format
			IplImage *iplin = (IplImage*)in->getIplImage();
			//OpenCV display routines use BGR format
			cvCvtColor( iplin, image, CV_RGB2BGR );  //buffer now contains the original rgb image
			cvSmooth(image, smoothed, CV_GAUSSIAN, 5); //
			copyTimer.stop();
			copyTimer.endlap();
		
			colorTimer.start();
			//Converts to HSV
			cvCvtColor( smoothed, hsv, CV_BGR2HSV );
			colorTimer.stop();
			colorTimer.endlap();


		}

			if( newselection)
			{
				cvSetImageROI( hsv, selection );
				double min_val, max_val;
				cvSetImageCOI( hsv, 2 );
				cvMinMaxLoc( hsv, &min_val, &max_val );
				smin = min_val;
				cvSetImageCOI( hsv, 3 );
				cvMinMaxLoc( hsv, &min_val, &max_val );
				vmin = min_val;
				vmax = max_val;
				cvResetImageROI( hsv );

				newtrackbar = 1;

                printf("updating thresholds\n");
				//cvSetTrackbarPos( "Vmin", "CamShiftPlus", vmin );
				//cvSetTrackbarPos( "Vmax", "CamShiftPlus", vmax );
				cvSetTrackbarPos( "Smin", "CamShiftPlus", smin );
			}

		if(newtrackbar)
		{ 
			_vmin = vmin;
			_vmax = vmax;
			_smin = smin;
		}


		if( newimage || newtrackbar )
		{

			thresholdTimer.start();
			// thresholds the saturation channel - creates a mask indicating pixels with "good" saturation. 
			cvInRangeS( hsv, cvScalar(0,_smin,MIN(_vmin,_vmax),0),
						cvScalar(180,256,MAX(_vmin,_vmax),0), mask );

			// splits the hue channel from the color image
			cvSplit( hsv, hue, 0, 0, 0 );

			thresholdTimer.stop();
			thresholdTimer.endlap();
		}
		

		// Object being selected ? Draw bounding box
		if( onselection )
		{
			drawTimer.start();
			//draw a translucid region in the selection
			cvCopy(smoothed, imagetodisplay);
			cvSetImageROI( imagetodisplay, selection );
			cvXorS( imagetodisplay, cvScalarAll(255), imagetodisplay, 0 );
			cvResetImageROI( imagetodisplay );
			overlaydisplay = true;
			drawTimer.stop();
			drawTimer.endlap();
		}


		if( newselection ) //object just selected - create a color model of the object
		{

			modelTimer.start();

			// computes the hue histogram of pixels with good saturation in the selected region 
			cvSetImageROI( hue, selection );
			cvSetImageROI( mask, selection );
			cvCalcHist( &hue, camshifthist, 0, mask );

			// Scales histogram such that maximum value is 255
			cvGetMinMaxHistValue( camshifthist, 0, &max_val, 0, 0 );
			cvConvertScale( camshifthist->bins, camshifthist->bins, max_val ? 255. / max_val : 0., 0 );

			// Prepares full image tracking
			cvResetImageROI( hue );
			cvResetImageROI( mask );
			track_window = selection;
			track_object = 1;
			
			//Prepares histogram display
			cvZero( camshifthistimg );
			bin_w = camshifthistimg->width / camshifthdims;
			for( i = 0; i < camshifthdims; i++ )
			{
				int val = cvRound( cvGetReal1D(camshifthist->bins,i)*camshifthistimg->height/255 );
				CvScalar color = hsv2rgb(i*180.f/camshifthdims);
				cvRectangle( camshifthistimg, cvPoint(i*bin_w,camshifthistimg->height),
							 cvPoint((i+1)*bin_w,camshifthistimg->height - val),
							 color, -1, 8, 0 );
			}

			ontracking = true; //to process the next if-then-else
			modelTimer.stop();
			modelTimer.endlap();
		}


		if( ontracking && ( newimage || newselection || newtrackbar || newtrackbar2) )  //object is selected - enter tracking mode
		{	
			trackTimer.start();

			// segments image pixels with good match to the histogram
			cvCalcBackProject( &hue, backprojected, camshifthist );
			cvAnd( backprojected, mask, backprojected, 0 );

			// searches iterativelly for the closest center of mass to the current search window 
			// Returns:
			//    track_window - contains object bounding box
			//    track box - contains object size and orientation
			cvCamShift( backprojected, track_window,
						cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ),
						&track_comp, &track_box );



			// Compute edges inside tracked region
			//void cvCanny( const CvArr* image, CvArr* edges, double threshold1,
            // double threshold2, int aperture_size=3 );
			
			// initialize next search window
			track_window = track_comp.rect;
    
			// Display backprojected image ?
			/*if( backproject_mode )
				cvCvtColor( backprojected, image, CV_GRAY2BGR );*/

			// upside down image ?
			if( !imagetodisplay->origin )
				track_box.angle = -track_box.angle;

			//printf("width: %f height: %f\n", track_box.size.width, track_box.size.height);
			//printf("x: %f y: %f\n", track_box.center.x, track_box.center.y);

			// testing for NAN's TO BE IMPROVED!!!!!!!!!!!!!!!!!!!!!!
			if( (track_box.size.width > 10000) || (track_box.size.width < 1) )
			  //if (_isnan(track_box.size.width))
			{
				printf("Numeric error\n");
			}
			else
			{
				// Draws an Ellipse over the image to display
				cvCopy(smoothed, imagetodisplay);
				cvEllipseBox( imagetodisplay, track_box, CV_RGB(255,0,0), 3, CV_AA, 0 );
				overlaydisplay = true;
			};
			trackTimer.stop();
			trackTimer.endlap();

			// Segment the blob using connected components
			int flags;
			CvConnectedComp comp;
			CvPoint seed;
			seed.x = track_box.center.x;
			seed.y = track_box.center.y;
			
			/*int mask_val = 255;
			flags = connectivity + (mask_val << 8) + CV_FLOODFILL_MASK_ONLY;
			cvSet(edges, cvScalar(0));
			cvFloodFill( image, seed, cvScalarAll(128), cvScalarAll(_lo_diff), cvScalarAll(_up_diff), &comp, flags, edges);*/
			
			flags = connectivity  + (ffill_case == 1 ? CV_FLOODFILL_FIXED_RANGE : 0);
			cvThreshold(backprojected, segmented, 128, 170, CV_THRESH_BINARY);
			cvFloodFill( segmented, seed, cvScalar(255), cvScalar(_lo_diff), cvScalar(_up_diff), &comp, flags, NULL);

			
			cvSetImageROI(segmented, comp.rect);
			cvSetImageROI(objbuffer, comp.rect);
			cvThreshold(segmented, objbuffer, 200, 255, CV_THRESH_BINARY );
			cvResetImageROI(segmented);
			//dilate image to fill in gaps
			cvDilate(objbuffer, objbuffer);
			//printf("\ndid cvDilate\n");
			
			//recompute color histogram
			cvSetImageROI( hue, comp.rect );
			cvCalcHist( &hue, colorhist, 0, objbuffer );

			// Scales histogram such that maximum value is 255
			cvGetMinMaxHistValue( colorhist, 0, &max_val, 0, 0 );
			cvConvertScale( colorhist->bins, colorhist->bins, max_val ? 255. / max_val : 0., 0 );

			cvResetImageROI( hue );
		
			//Prepares histogram display
			cvZero( colorhistimg );
			bin_w = colorhistimg->width / colorhdims;
			for( i = 0; i < colorhdims; i++ )
			{
				int val = cvRound( cvGetReal1D(colorhist->bins,i)*colorhistimg->height/255 );
				CvScalar color = hsv2rgb(i*180.f/colorhdims);
				cvRectangle( colorhistimg, cvPoint(i*bin_w,colorhistimg->height),
							 cvPoint((i+1)*bin_w,colorhistimg->height - val),
							 color, -1, 8, 0 );
			}
			cvNormalizeHist( colorhist, 1 );

			//compute moments of the segmented region
			
			//find edges
			cvFindContours(	
				objbuffer, 
				storage, 
				&contours, 
				sizeof(CvContour),
				CV_RETR_EXTERNAL,		//only what the outer contour 
				CV_CHAIN_APPROX_SIMPLE, 
				cvPoint(0,0)
			);
		
			
			CvPoint drawcircle;
			if(contours) 
			{

				// closest point to bottom left - for grasping
				CvPoint* pt = (CvPoint*)cvGetSeqElem( contours, 0 );
				double min_dist2 = (width - pt->x - comp.rect.x)*(width - pt->x - comp.rect.x)+(height - pt->x - comp.rect.y)*(height - pt->x - comp.rect.y);
				int index = 0;
				double dist2;
				for(i = 1; i < contours->total; i++)
				{
					pt = (CvPoint*)cvGetSeqElem( contours, i );
					dist2 = (width - pt->x - comp.rect.x)*(width - pt->x - comp.rect.x)+(height - pt->y - comp.rect.y)*(height - pt->y - comp.rect.y);
					if(dist2 < min_dist2){
						min_dist2 = dist2;
						index=i;
					}
				}
				pt = (CvPoint*)cvGetSeqElem( contours, index );

				//display pt coordinates
				
				drawcircle.x = pt->x + comp.rect.x;
				drawcircle.y = pt->y + comp.rect.y;
				cvCircle( imagetodisplay, drawcircle, 3, CV_RGB(0,0,255), -1 );

				contours = cvApproxPoly( 
					contours, 
					sizeof(CvContour), 
					storage, 
					CV_POLY_APPROX_DP, 
					cvContourPerimeter(contours)*0.02, 
					1);

				convexhull = cvConvexHull2( contours, storage, CV_CLOCKWISE, 1 );

				//mesurements of the contour
    			contour_area = fabs(cvContourArea( contours, CV_WHOLE_SEQ ));
				contour_perimeter = cvArcLength( contours, CV_WHOLE_SEQ, 1 );
				convex_perimeter = cvArcLength( convexhull, CV_WHOLE_SEQ, 1 );
				CvBox2D enclosing_rect = cvMinAreaRect2( convexhull, storage );
				major_axis = (enclosing_rect.size.width > enclosing_rect.size.height ? enclosing_rect.size.width : enclosing_rect.size.height);
				minor_axis = (enclosing_rect.size.width > enclosing_rect.size.height ? enclosing_rect.size.height : enclosing_rect.size.width);
				rect_area = major_axis*minor_axis;
				
				CvPoint2D32f center;
				float radius;
				cvMinEnclosingCircle( contours, &center, &radius );

				//shape descriptors
				if(contour_perimeter > 0)
					convexity = convex_perimeter/contour_perimeter;
				else
					convexity = 0;
		
				if(major_axis > 0)
				   eccentricity = minor_axis/major_axis;
				else
					eccentricity = 0;

			
				if(contour_perimeter > 0)
					compactness = contour_area/(contour_perimeter*contour_perimeter);

				if( radius > 0)
					circleness = contour_area/(3.1415*radius*radius);

				if(rect_area > 0)
					squareness = contour_area/rect_area;

				/*printf("\nArea: %f\n", contour_area);			// 1
				printf("Convexity: %f\n", convexity);			// 2
				printf("Eccentricity: %f\n", eccentricity);		// 3
				printf("Compactness: %f\n", compactness);		// 4
				printf("Circleness: %f\n", circleness);			// 5
				printf("Squareness: %f\n\n", squareness);		// 6
				*/
				//display contours in image

				cvDrawContours(
					imagetodisplay, 
					contours, 
					CV_RGB(0,255,0), //External color
					CV_RGB(0,0,255), //Hole color
					0,				 //Contour level - only the external contour is required	
					1, 
					CV_AA, 
					cvPoint(comp.rect.x, comp.rect.y)			// roi offset
				);
				//printf("\ndid cvDrawContours\n");
			}




			//compute contour histogram - pairwise geometrical histogram method 
			if( contours && contours->total >= 3)
			{
				cvCalcPGH( contours, contourhist );
				cvNormalizeHist( contourhist, 1 );
				// Scales histogram such that maximum value is 255
				cvGetMinMaxHistValue( contourhist, 0, &max_val, 0, 0 );
				//cvConvertScale( contourhist->bins, contourhist->bins, max_val ? 255. / max_val : 0., 0 );
				//Prepares histogram display
				cvZero( contourhistimg );
				bin_w = contourhistimg->width / (contourhdims[0]*contourhdims[1]);
				for( i = 0; i < contourhdims[0]*contourhdims[1]; i++ )
				{
					int val = cvRound( cvGetReal2D(contourhist->bins,i/contourhdims[1],i%contourhdims[1])*contourhistimg->height );
					//printf("val: %d");
					//CvScalar color = hsv2rgb(i*180.f/contourhdims[0]*contourhdims[1]);
					CvScalar color = cvScalarAll(255);
					cvRectangle( contourhistimg, cvPoint(i*bin_w,contourhistimg->height),
						cvPoint((i+1)*bin_w,contourhistimg->height - val),
						color, -1, 8, 0 );
				}
				//printf("\n");

								//writing data to ports
				writeTimer.start();

				Vector &colorData=outColorPort.prepare();
				colorData.size(colorhdims);
				for(i = 0; i < colorhdims; i++)
					colorData[i] = cvQueryHistValue_1D( colorhist, i );

				Vector &contourData=outContourPort.prepare();
				contourData.size(contourhdims[0]*contourhdims[1]);
				for(i = 0; i < contourhdims[0]*contourhdims[1]; i++)
					contourData[i] = cvQueryHistValue_2D( contourhist, i/contourhdims[1], i%contourhdims[1] );
				
				Vector &momentsData=outMomentsPort.prepare();
				momentsData.size(5);
				momentsData[0] = track_box.center.x;  
				momentsData[1] = track_box.center.y;

            	momentsData[2] = track_box.size.width;

				momentsData[3] = track_box.size.height;
				momentsData[4] = track_box.angle;

				Vector &allData=outAllPort.prepare();
				allData.size(7+colorhdims+contourhdims[0]*contourhdims[1]);
				allData[0] = (double)(track_box.center.x - (width/2.0))/((double)1.0 * width/2.0);  
				allData[1] = -(double)(track_box.center.y - (height/2.0))/((double)1.0 * height/2.0);  
				allData[2] = track_box.size.width / width;
				allData[3] = track_box.size.height / height;
				allData[4] = track_box.angle;
				allData[5] = drawcircle.x;
				allData[6] = drawcircle.y;
				for(i = 0; i < colorhdims; i++)
					allData[i+7] = cvQueryHistValue_1D( colorhist, i );
				/*for(i = 0; i < contourhdims[0]*contourhdims[1]; i++)
					allData[i+7+colorhdims] = cvQueryHistValue_2D( contourhist, i/contourhdims[1], i%contourhdims[1] );*/

				allData[7+colorhdims+0] = contour_area;
				allData[7+colorhdims+1] = convexity;
				allData[7+colorhdims+2] = eccentricity;
				allData[7+colorhdims+3] = compactness;
				allData[7+colorhdims+4] = circleness;
				allData[7+colorhdims+5] = squareness;
				for(i = 6; i < contourhdims[0]*contourhdims[1]; i++)
					allData[i+7+colorhdims] = 0;


				
				printf("\nArea: %f Convexity: %f Eccentricity: %f Compactness: %f Circleness: %f Squareness: %f\n",
					contour_area, convexity, eccentricity, compactness, circleness, squareness);
				
				printf("seems a %s\n", circleness>0.6?"circle":"square");

				/*for(i = 0; i < contourhdims[0]*contourhdims[1]; i++)
					printf("%f ",cvQueryHistValue_2D( contourhist, i/contourhdims[1], i%contourhdims[1] ));
				printf("\n");*/
				outMomentsPort.write();
			outColorPort.write();
			outContourPort.write();
			outAllPort.write();
			ImageOf<PixelMono> &outMono=outPort.prepare();
			outMono.resize(width, height);
			outMono = out;
			outPort.write();
			writeTimer.stop();
			writeTimer.endlap();
			//printf("\ndid Writes\n");
		}

			//transform features into probabilities 

			//TODO
		}
				
		displayTimer.start();
		//refresh image displays
		if(overlaydisplay)
			cvShowImage( "CamShiftPlus", imagetodisplay );
		else
			cvShowImage( "CamShiftPlus", smoothed );
		//cvShowImage( "BackProjected", backprojected );
		//cvShowImage( "Camshift Histogram", camshifthistimg );
		//cvShowImage( "Color Histogram", colorhistimg );
		//cvShowImage( "Contour Histogram", contourhistimg);
		//cvShowImage( "Segmented", segmented);
		//cvShowImage( "Mask", mask );
		//cvShowImage( "Edges", edges );

		c = cvWaitKey(1);
        if( c == 27 )
            break;
        switch( c )
        {
        case 'b':
            backproject_mode ^= 1;
            break;
        case 'c':
            track_object = 0;
			overlaydisplay = false;
			cvZero( camshifthistimg );
            cvZero( colorhistimg );
			cvZero( contourhistimg );
            break;
        case 'h':
            show_hist ^= 1;
            if( !show_hist )
                cvDestroyWindow( "Histogram" );
            else
                cvNamedWindow( "Histogram", 1 );
            break;
			case 's':
            printf("Simple floodfill mode is set\n");
            ffill_case = 1;
            break;

       case 'f':
            printf("Fixed Range floodfill mode is set\n");
            ffill_case = 1;
            break;
        case 'g':

           printf("Gradient (floating range) floodfill mode is set\n");
            ffill_case = 2;
            break;
        case '4':

           printf("4-connectivity mode is set\n");
            connectivity = 4;
            break;
        case '8':
            printf("8-connectivity mode is set\n");
            connectivity = 8;
            break;
        default:
            ;
        }

		displayTimer.stop();
		displayTimer.endlap();
		
		if( globalTimer.now() >= timeToRefresh)
		{
			timeToRefresh += refreshDelay;

			printf("Cycle %d; Time %03.2f Duration (ms) %03.2f Avg frame rate %03.2f\n", cycleTimer.cyc(), globalTimer.now(), cycleTimer.lastlap()*1000, cycleTimer.cyc()/globalTimer.now());
			printf("Copy   time (ms): %03.2f. (Average: %03.2f)\n", copyTimer.lastlap()*1000, copyTimer.avg()*1000);
			printf("Color  time (ms): %03.2f. (Average: %03.2f)\n", colorTimer.lastlap()*1000, colorTimer.avg()*1000);
			printf("Thresh time (ms): %03.2f. (Average: %03.2f)\n", thresholdTimer.lastlap()*1000, thresholdTimer.avg()*1000);
			printf("Model  time (ms): %03.2f. (Average: %03.2f)\n", modelTimer.lastlap()*1000, modelTimer.avg()*1000);
			printf("Track  time (ms): %03.2f. (Average: %03.2f)\n", trackTimer.lastlap()*1000, trackTimer.avg()*1000);
			printf("Read   time (ms): %03.2f. (Average: %03.2f)\n", readTimer.lastlap()*1000, readTimer.avg()*1000);
			printf("Write  time (ms): %03.2f. (Average: %03.2f)\n", writeTimer.lastlap()*1000, writeTimer.avg()*1000);
			printf("Interf.time (ms): %03.2f. (Average: %03.2f)\n", interfaceTimer.lastlap()*1000, interfaceTimer.avg()*1000);
			printf("Draw   time (ms): %03.2f. (Average: %03.2f)\n", drawTimer.lastlap()*1000, drawTimer.avg()*1000);
			printf("Disp.  time (ms): %03.2f. (Average: %03.2f)\n", displayTimer.lastlap()*1000, displayTimer.avg()*1000);
		}
		
		if(newimage) 
		{	
			globalTimer.stop();
			globalTimer.endlap();
			cycleTimer.stop();
			cycleTimer.endlap();

	}

		
	}
	delete terminee;
	//Network::fini();
	return 0;
}
