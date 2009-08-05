// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Julio Gomes, Alex Bernardino (VisLab/ISR/IST)
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
#include <simio.h> // more portable
#include "FaceDetector.h"


using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;


static CvMemStorage* storage = 0;
static CvHaarClassifierCascade* cascade = 0;

void detect_and_draw( IplImage* image );

const char* cascade_name =
    "haarcascade_frontalface_alt.xml";
/*    "haarcascade_profileface.xml";*/


/*sending vectors of doubles containing:
	x - horizontal position (normalized image coordinates)
	y - vertical position (normalized image coordinates)
*/




//#undef main

class detect_face : public Module
{
private:
    int _input_lines;
    int _input_cols;
    
    double _fx;            
    double _fy;           
    double _cx;
    double _cy;

	ImageOf<PixelRgb> *in;
	ImageOf<PixelRgb> out;
	    
	BufferedPort< ImageOf<PixelRgb> > imagePort;
    BufferedPort<Vector> outPort;

    IplImage *frame, *frame_copy;
	FaceDetector* mysensor;

    

	public:

    detect_face(){};
    ~detect_face(){};
    
    virtual bool open(Searchable& config)
    {
        if (config.check("help","if present, display usage message")) {
            printf("Call with --name </portprefix> --file <configfile.ini>\n");
            return false;
        }

        // Defaults will correspond to a view field of 90 deg.
        _input_lines = config.check("h", 480, "Input image lines").asInt();
        _input_cols = config.check("w", 640, "Input image columns").asInt();
        _fx = config.check("fx", 320, "Focal distance (on horizontal pixel size units)").asDouble();
        _fy = config.check("fy", 240, "Focal distance (on vertical pixel size units)").asDouble();
        _cx = config.check("cx", 320, "Image center (on horizontal pixel size units)").asDouble();
        _cy = config.check("cy", 240, "Image center (on vertical pixel size units)").asDouble();
        

        imagePort.open(getName("image"));
        outPort.open(getName("pos"));
        
        in=0;
	    
	    out.resize(_input_cols,_input_lines);
		frame_copy = cvCreateImage( cvSize(_input_cols,_input_lines), IPL_DEPTH_8U, 3 );

		cascade = (CvHaarClassifierCascade*)cvLoad( "haarcascade_frontalface_alt.xml", 0, 0, 0 );
    
		if( !cascade )
		{
			fprintf( stderr, "ERROR: Could not load classifier cascade\n" );
			return false;
		}
		storage = cvCreateMemStorage(0);
		cvNamedWindow( "result", 1 );
		mysensor=new FaceDetector(_input_cols,_input_lines);
    
	    return true;
    };

    virtual bool close()
    {
       imagePort.close();
       outPort.close();
	   cvReleaseImage( &frame_copy );
       return true;
    };
 
    virtual bool interruptModule()
    {
       imagePort.interrupt();
       outPort.interrupt();
       return true;
    };

	virtual bool updateModule()
	{
		double x, y;
		in = imagePort.read(false);
		if(in != 0)
		{

			/*
			//Get input image into opencv format
			frame = (IplImage*)in->getIplImage();
			if( frame->origin == IPL_ORIGIN_TL )
                cvCopy( frame, frame_copy, 0 );
            else
                cvFlip( frame, frame_copy, 0 );
            
            detect_and_draw( frame_copy, &x, &y );
			*/

            mysensor->try_detect_features((IplImage*)in->getIplImage());
			x=mysensor->get_feature(0);
			y=mysensor->get_feature(1);
			printf("%lf, %lf \n", x, y);
			
		
			if (x!=0.0 && y!=0.0) 
			{	
				//send a saccade command with the orientation expressed in the camera frame 
				Vector &posdata=outPort.prepare();
				posdata.size(4);

                posdata[0] = (x - _cx)/_fx;  
			    posdata[1] = (y - _cy)/_fy;
				posdata[2] = 'r';
				posdata[3] = 's';
				outPort.write();
            }
			
			
			//cvShowImage( "left eye targets", iplin );
			//cvWaitKey(10);
		}
		return true;
	}

	void detect_and_draw( IplImage* img, double *x, double *y )
	{
		int scale = 1;
		IplImage* temp = cvCreateImage( cvSize(img->width/scale,img->height/scale), 8, 3 );
		CvPoint pt1, pt2;
		int i;

		//cvPyrDown( img, temp, CV_GAUSSIAN_5x5 );
		cvClearMemStorage( storage );

		if( cascade )
		{
			CvSeq* faces = cvHaarDetectObjects( img, cascade, storage,
												1.1, 2, CV_HAAR_DO_CANNY_PRUNING,
												cvSize(40, 40) );
			for( i = 0; i < (faces ? faces->total : 0); i++ )
			{
				CvRect* r = (CvRect*)cvGetSeqElem( faces, i );
				pt1.x = r->x*scale;
				pt2.x = (r->x+r->width)*scale;
				pt1.y = r->y*scale;
				pt2.y = (r->y+r->height)*scale;
				cvRectangle( img, pt1, pt2, CV_RGB(255,0,0), 3, 8, 0 );
			}
			if( i == 0) //no faces found
			{
				*x  = 0;
				*y  = 0;
			}
			else
			{
				*x = (pt1.x+pt2.x)/2.0;
				*y = (pt1.y+pt2.y)/2.0;
			}
		}
		else
		{
			*x = 0;
			*y = 0;
		}

		cvShowImage( "result", img );
		cvReleaseImage( &temp );
	}
};

int main(int argc, char *argv[]) {
    Network yarp;
    yarp::os::Time::turboBoost();

    detect_face module;
    module.setName("/detect_face"); // set default name of module
    return module.runModule(argc,argv);
}
