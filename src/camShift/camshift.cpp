// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Alex Bernardino (VisLab/ISR/IST)
 */

// yarp
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>

//OpenCV
#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <math.h>


using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;


// ISNAN, from inkscape isnan.h, public domain code
// deal with some messes in the possible names of isnan
#if defined(__isnan)
# define isNaN(_a) (__isnan(_a))
#elif defined(__APPLE__) && __GNUC__ == 3
# define isNaN(_a) (__isnan(_a))    /* MacOSX/Darwin definition < 10.4 */
#elif defined(WIN32) || defined(_isnan)
# define isNaN(_a) (_isnan(_a))     /* Win32 definition */
#elif defined(isnan) || defined(__FreeBSD__) || defined(__osf__)
# define isNaN(_a) (isnan(_a))      /* GNU definition */
#elif defined (SOLARIS_2_8) && __GNUC__ == 3 && __GNUC_MINOR__ == 2
# define isNaN(_a) (isnan(_a))      /* GNU definition */
#else
# define isNaN(_a) (std::isnan(_a))
#endif

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



class camshift : public Module
{
private:
    
    IplImage *buffer, *image, *hsv, *hue, *mask, *backproject, *histimg;
	CvHistogram *hist;
	int backproject_mode;
	int show_hist;
	CvRect track_window;
	CvBox2D track_box;
	CvConnectedComp track_comp;
	int hdims;
	float hranges_arr[2];
	float* hranges;
	int vmin, vmax, smin, _vmin, _vmax, _smin;
    int bin_w,i,c;
    double refreshDelay;
    double timeToRefresh;
    int _w; //image width
    int _h; //image height


    bool newimage;
    bool newselection;
    bool onselection;
    bool ontracking;
    bool newtrackbar;		//trackbar changed position
    bool overlaydisplay; 

    ImageOf<PixelRgb> *in;
	ImageOf<PixelMono> out;
	Bottle *inRoi;
    
	BufferedPort< ImageOf<PixelRgb> > inPort;
    BufferedPort< ImageOf<PixelMono> > outPort;
	BufferedPort<Bottle> inRoiPort;
	BufferedPort<Vector> outRoiPort;
	BufferedPort<Vector> outObjPort;
public:

    camshift(){};
    ~camshift(){};
    
    virtual bool open(Searchable& config)
    {
        if (config.check("help","if present, display usage message")) {
            printf("Call with --name </portprefix> --file <configfile.ini>\n");
            return false;
        }
        
        inPort.open(getName("img:i"));
        outPort.open(getName("img:o"));
        inRoiPort.open(getName("roi:i"));
        outRoiPort.open(getName("roi:o"));
        outObjPort.open(getName("obj:o"));

        //fp = fopen("histograms.txt","w");
	    buffer = 0; 
        image = 0; 
        hsv = 0; 
        hue = 0;
        mask = 0;
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
	    

	    in=0;
	    inRoi = 0;
        
	    vmin = 0;
        vmax = 256;
        smin = 0;
        _vmin = vmin;
        _vmax = vmax;
        _smin = smin;


	    cvNamedWindow( "Histogram", 1 );
        cvNamedWindow( "CamShift", 1 );
        cvSetMouseCallback( "CamShift", on_mouse, 0 );
        cvCreateTrackbar( "Vmin", "CamShift", &vmin, 256, 0 );
        cvCreateTrackbar( "Vmax", "CamShift", &vmax, 256, 0 );
        cvCreateTrackbar( "Smin", "CamShift", &smin, 256, 0 );

        // waits for one image to get the dimensions
        in = inPort.read(true);	
        _w = in->width();
        _h = in->height();

        CvSize sz = cvSize(_w,_h);
	    buffer = cvCreateImage(sz, 8, 3 );
	    image = cvCreateImage( sz, 8, 3 );
	    imagetodisplay = cvCreateImage( sz, 8, 3 );
        hsv = cvCreateImage( sz, 8, 3 );
        hue = cvCreateImage( sz, 8, 1 );
        mask = cvCreateImage( sz, 8, 1 );
        hist = cvCreateHist( 1, &hdims, CV_HIST_ARRAY, &hranges, 1 );
        histimg = cvCreateImage( cvSize(320,200), 8, 3 );
        cvZero( histimg );

	    out.resize(_w,_h);
	    backproject = (IplImage*)out.getIplImage();
    	
	    refreshDelay = 10;
        timeToRefresh = 10;
        overlaydisplay = false;

        return true;
    };

    virtual bool close()
    {
       inPort.close();
       outPort.close();
       inRoiPort.close();
       outRoiPort.close();
       outObjPort.close();
       
       //fclose(fp);

       return true;
    };
 
    virtual bool interruptModule()
    {
       inPort.interrupt();
       outPort.interrupt();
       inRoiPort.interrupt();
       outRoiPort.interrupt();
       outObjPort.interrupt();
       return true;
    };

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

    virtual bool updateModule()
    {
        /*yarp::sig::ImageOf<PixelRgb> *yrpImgIn;
        yrpImgIn = portImgIn.read();
        if (yrpImgIn == NULL)   
            return true;
        yarp::sig::ImageOf<PixelRgb> yrpImgOut; 
        yrpImgOut.resize(_output_cols, _output_lines);
        IplImage *inptr = (IplImage*)yrpImgIn->getIplImage();
	    IplImage *outptr = (IplImage*)yrpImgOut.getIplImage();
        //cvRemap(inptr,outptr,_mapx,_mapy,CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS,cvScalarAll(0));
        cvRemap(inptr,outptr,_mapx,_mapy,CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS,cvScalarAll(0));
        yarp::sig::ImageOf<PixelRgb>& yrpOut = portImgOut.prepare();
        yrpOut = yrpImgOut;
        portImgOut.write();*/

        globalTimer.start();
		readTimer.start();

		in = inPort.read(false);						//read image
		if(in != 0)
			cycleTimer.start();
		
		inRoi = inRoiPort.read(false);					// read roi
		if((inRoi!=0)  && (inRoi->size() == 4))			// got a valid selection
		{
			track_object = -1;
			
			selection.x = inRoi->get(0).asInt();
			selection.y = inRoi->get(1).asInt();
			selection.width = inRoi->get(2).asInt();
			selection.height = inRoi->get(3).asInt();

			//check limits 
			if(selection.x < 0) selection.x = 0;
			if(selection.x >= _w-1) selection.x = _w-2;
			if(selection.y < 0) selection.y = 0;
			if(selection.y >= _h-1) selection.y = _h-2;
			if(selection.width <= 0) selection.width = 1;
			if(selection.height <= 0) selection.height = 1;
			if(selection.width+selection.x > _w) selection.width = _w - selection.x;
			if(selection.height+selection.y > _h) selection.height = _h - selection.y;
		}


		newimage = (in != 0);	
		onselection = ( select_object && selection.width > 0 && selection.height > 0 );
		newselection = (track_object == -1);
		ontracking = (track_object == 1);
		newtrackbar = ((_vmin != vmin) || (_vmax != vmax) || (_smin != smin ));

		if(newtrackbar)
		{ 
			_vmin = vmin;
			_vmax = vmax;
			_smin = smin;
		}

		readTimer.stop();

		if(newimage)  //get image into local memory - take the chance to convert to rgb
		{
			readTimer.endlap();
			copyTimer.start();
			//Get input image into opencv format
			IplImage *iplin = (IplImage*)in->getIplImage();
			//OpenCV display routines use BGR format
			cvCvtColor( iplin, buffer, CV_RGB2BGR);  //buffer now contains the original rgb image
			copyTimer.stop();
			copyTimer.endlap();
		
			colorTimer.start();
			//Converts to HSV
			cvCvtColor( buffer, hsv, CV_BGR2HSV );
			colorTimer.stop();
			colorTimer.endlap();
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
			cvCopy(buffer, imagetodisplay);
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

			float max_val = 0.0f;
			
			// computes the hue histogram of pixels with good saturation in the selected region 
			cvSetImageROI( hue, selection );
			cvSetImageROI( mask, selection );
			cvCalcHist( &hue, hist, 0, mask );

			// write histogram to file - for debugging
			//for(int b = 0; b < hdims; b++)
			//	fprintf(fp, "%03.2f ", cvQueryHistValue_1D( hist, b ) );
			//fprintf(fp, "\n");


			// Scales histogram such that maximum value is 255
			cvGetMinMaxHistValue( hist, 0, &max_val, 0, 0 );
			cvConvertScale( hist->bins, hist->bins, max_val ? 255. / max_val : 0., 0 );

			// Prepares full image tracking
			cvResetImageROI( hue );
			cvResetImageROI( mask );
			track_window = selection;
			track_object = 1;
			
			//Prepares histogram display
			cvZero( histimg );
			bin_w = histimg->width / hdims;
			for( i = 0; i < hdims; i++ )
			{
				int val = cvRound( cvGetReal1D(hist->bins,i)*histimg->height/255 );
				CvScalar color = hsv2rgb(i*180.f/hdims);
				cvRectangle( histimg, cvPoint(i*bin_w,histimg->height),
							 cvPoint((i+1)*bin_w,histimg->height - val),
							 color, -1, 8, 0 );
			}
			ontracking = true; //to process the next if-then-else
			modelTimer.stop();
			modelTimer.endlap();
		}


		if( ontracking && ( newimage || newselection || newtrackbar ) )  //object is selected - enter tracking mode
		{	
			trackTimer.start();

			// segments image pixels with good match to the histogram
			cvCalcBackProject( &hue, backproject, hist );
			cvAnd( backproject, mask, backproject, 0 );

			// searches iterativelly for the closest center of mass to the current search window 
			// Returns:
			//    track_window - contains object bounding box
			//    track box - contains object size and orientation
			cvCamShift( backproject, track_window,
						cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ),
						&track_comp, &track_box );
			
			// initialize next search window
			track_window = track_comp.rect;
    
			// Display backprojected image ?
			if( backproject_mode )
				cvCvtColor( backproject, image, CV_GRAY2BGR );

			// upside down image ?
			if( !imagetodisplay->origin )
				track_box.angle = -track_box.angle;

			//printf("width: %f height: %f\n", track_box.size.width, track_box.size.height);
			//printf("x: %f y: %f\n", track_box.center.x, track_box.center.y);

			// testing for NAN's TO BE IMPROVED!!!!!!!!!!!!!!!!!!!!!!
			//if( (track_box.size.width > 10000) || (track_box.size.width < 1) )
			if (isNaN(track_box.size.width))
			{
				printf("Numeric error\n");
			}
			else
			{
				// Draws an Ellipse over the image to display
				cvCopy(buffer, imagetodisplay);
				printf("%f %f %f %f %f\n", track_box.center.x, 
					track_box.center.y, track_box.size.width, track_box.size.height, track_box.angle);
				cvEllipseBox( imagetodisplay, track_box, CV_RGB(255,0,0), 3, CV_AA, 0 );
				overlaydisplay = true;
			};
			trackTimer.stop();
			trackTimer.endlap();


			//writing data to ports
			writeTimer.start();

			Vector &roiData=outRoiPort.prepare();
			roiData.size(4);
			roiData[0] = track_comp.rect.x;
			roiData[1] = track_comp.rect.y;
			roiData[2] = track_comp.rect.width;
			roiData[3] = track_comp.rect.height;

			Vector &objData=outObjPort.prepare();
			objData.size(5);
            double xnorm, ynorm;
            // convert to normalized coordinates
            xnorm = 2*track_box.center.x/_w - 1;
            ynorm = 2*track_box.center.y/_h - 1;
            objData[0] = xnorm;
            objData[1] = ynorm;
			//objData[2] = track_box.size.width;
			//objData[3] = track_box.size.height;
			//objData[4] = track_box.angle;
			objData[2] = 'r';  //relative mode
			objData[3] = 'p';  //pursuit
			objData[4] = 0;    //not used

			outRoiPort.write();
			outObjPort.write();

			ImageOf<PixelMono> &outMono=outPort.prepare();
			outMono.resize(_w, _h);
			outMono = out;
			outPort.write();

			writeTimer.stop();
			writeTimer.endlap();
		}
				
		displayTimer.start();
		//refresh image displays
		if(overlaydisplay)
			cvShowImage( "CamShift", imagetodisplay );
		else
			cvShowImage( "CamShift", buffer );

		cvShowImage( "Histogram", histimg );
		c = cvWaitKey(1);
        
        switch( c )
        {
        case 'b':
            backproject_mode ^= 1;
            break;
        case 'c':
            track_object = 0;
			overlaydisplay = false;
            cvZero( histimg );
            break;
        case 'h':
            show_hist ^= 1;
            if( !show_hist )
                cvDestroyWindow( "Histogram" );
            else
                cvNamedWindow( "Histogram", 1 );
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
        return true;
    };
};

int main(int argc, char *argv[]) {
    Network yarp;
    yarp::os::Time::turboBoost();

    camshift module;
    module.setName("/camshift"); // set default name of module
    return module.runModule(argc,argv);
}
