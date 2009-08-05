#include <ace/OS.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/all.h>
#include <yarp/os/Network.h>
#include <yarp/os/Module.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/all.h>
#include <yarp/String.h>

#include <ace/OS.h>
#include <ace/Log_Msg.h>
#include <ace/Sched_Params.h>
#include <ace/String_Base.h>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>

#include <stdio.h>
#include <cv.h>
#include <highgui.h>

// MATLAB Engine Library
#include "engine.h"


using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace yarp::dev;

using namespace std;

int LOOK_R = 1, LOOK_L = 2;

const int ARM_JOINTS=16;
const int NUM_CONTROLLED_JOINTS = 7;
const int NUM_FEATURES = 4;

void chageMLToCurDir(Engine *ep);

int dimTheta, dimY;
int mouseParam = 0;

int backproject_mode = 0;
int select_object = 0;
int track_object = 0;
int show_hist = 1;
CvPoint origin;
CvRect selection;
CvRect track_window;
CvBox2D track_box;
CvConnectedComp track_comp;
int hdims = 16;
float hranges_arr[] = {0,180};
float* hranges = hranges_arr;
int vmin = 10, vmax = 256, smin = 30;

IplImage *image = NULL;


int backproject_mode_left = 0;
int select_object_left = 0;
int track_object_left = 0;
int show_hist_left = 1;
CvPoint origin_left;
CvRect selection_left;
CvRect track_window_left;
CvBox2D track_box_left;
CvConnectedComp track_comp_left;
int hdims_left = 16;
float hranges_arr_left[] = {0,180};
float* hranges_left = hranges_arr_left;
int vmin_left = 10;
int vmax_left = 256;
int smin_left = 30;

IplImage *image_left = NULL;

/*Function to capture the right mouse movement*/
void on_mouse( int event, int x, int y, int flags, void* param )
{
	if( !image )
		return;	

		if( image->origin )
			y = image->height - y;

		if( select_object )
		{
			selection.x = MIN(x,origin.x);
			selection.y = MIN(y,origin.y);
			selection.width = selection.x + CV_IABS(x - origin.x);
			selection.height = selection.y + CV_IABS(y - origin.y);

			selection.x = MAX( selection.x, 0 );
			selection.y = MAX( selection.y, 0 );
			selection.width = MIN( selection.width, image->width );
			selection.height = MIN( selection.height, image->height );
			selection.width -= selection.x;
			selection.height -= selection.y;
		}

	

	switch( event )
	{
	case CV_EVENT_LBUTTONDOWN:
		origin = cvPoint(x,y);
		selection = cvRect(x,y,0,0);
		select_object = 1;
		break;
	case CV_EVENT_LBUTTONUP:
		select_object = 0;
		if( selection.width > 0 && selection.height > 0 )
			track_object = -1;
		break;
	}
}


/*Function to capture the left mouse movement*/
void on_mouse_left( int event, int x, int y, int flags, void* param )
{
	if( !image_left )
		return;

	
	
		if( image_left->origin )
			y = image_left->height - y;

		if( select_object_left )
		{
			selection_left.x = MIN(x,origin_left.x);
			selection_left.y = MIN(y,origin_left.y);
			selection_left.width = selection_left.x + CV_IABS(x - origin_left.x);
			selection_left.height = selection_left.y + CV_IABS(y - origin_left.y);

			selection_left.x = MAX( selection_left.x, 0 );
			selection_left.y = MAX( selection_left.y, 0 );
			selection_left.width = MIN( selection_left.width, image_left->width );
			selection_left.height = MIN( selection_left.height, image_left->height );
			selection_left.width -= selection_left.x;
			selection_left.height -= selection_left.y;
		}

	

	switch( event )
	{
	case CV_EVENT_RBUTTONDOWN:
		origin_left = cvPoint(x,y);
		selection_left = cvRect(x,y,0,0);
		select_object_left = 1;
		break;
	case CV_EVENT_RBUTTONUP:
		select_object_left = 0;
		if( selection_left.width > 0 && selection_left.height > 0 )
			track_object_left = -1;
		break;
	}
}

/*change of color space from HSV to RGB*/
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





class rxPort : public BufferedPort<Bottle>
{
public:
	rxPort(unsigned int _dim) : dim(_dim) { curr.resize(dim); curr=0.0; }
	Vector &get_curr()                    { return curr;                }

private:
	unsigned int dim;
	Vector curr;

	virtual void onRead(Bottle &b)
	{
		for (unsigned int i=0; i<dim; i++)
			curr[i]=b.get(i).asDouble();
	}
};



class GatewayThread : public RateThread
{
private:

	Port                  portReadFeatures;
	unsigned int          dim;

	int    period;
	string visibility;
	string portName;

	Engine  *ep;

	mxArray *varoutTheta;
	mxArray *varoutY;
	mxArray *varoutYD;
	mxArray *varoutQ;
	mxArray *varoutSimStep;

	int w, h;//image width and height


	unsigned int N,idx;

	double t0;
	double go_old;

	PolyDriver *_dd;
	IEncoders *_iencs;
	IPositionControl *_ipos;
	IVelocityControl *_ivel;
	double *encoders;

	//image processing 
	// Make a port for reading and writing images
	BufferedPort<ImageOf<PixelRgb> >   port;
	BufferedPort<ImageOf<PixelRgb> >   portInputLeft;
	BufferedPort<ImageOf<PixelRgb> >   portInputRight;
	BufferedPort<ImageOf<PixelRgb> >   portOutputLeft;
	BufferedPort<ImageOf<PixelRgb> >   portOutputRight;
	Port cmdPort;
	Port outputFeatures;
	int ct;
	IplImage                            *cvImageCam;
	ImageOf<PixelRgb>                   *imgRight;
	ImageOf<PixelRgb>                   *imgLeft;

	Bottle                              bot;
	Bottle                              b;

	IplImage *hsv;
	IplImage *hue;
	IplImage *mask;
	IplImage *backproject;
	IplImage *histimg;

	CvHistogram *hist;


	IplImage *hsv_left;
	IplImage *hue_left;
	IplImage *mask_left;
	IplImage *backproject_left;
	IplImage *histimg_left;

	CvHistogram *hist_left;


public:
	GatewayThread(int _period, string &_portName, string &_visibility) : RateThread(_period),
		period(_period), portName(_portName), visibility(_visibility) 
	{
		dim   =7;
		N     =500;
		idx   =1;
		go_old=0.0;
		encoders=new double [ARM_JOINTS];

	    hsv = NULL;
	    hue = NULL;
	    mask = NULL;
	    backproject = NULL;
	    histimg = NULL;

	    hist = NULL;

		hsv_left = NULL;
	    hue_left = NULL;
	    mask_left = NULL;
	    backproject_left = NULL;
	    histimg_left = NULL;

	    hist_left = NULL;	

		imgLeft = NULL;
		imgRight = NULL;

	}

	virtual bool threadInit()
	{
		dimY = NUM_FEATURES;
		dimTheta = NUM_CONTROLLED_JOINTS;	

		Property armOptions;
		armOptions.put("robot", "icubSim");
		armOptions.put("part", "arm");
		armOptions.put("device", "remote_controlboard");
		armOptions.put("local", "/icubSim/left_arm");   //local port names
		armOptions.put("remote", "/icubSim/left_arm");         //where we connect to

		//armOptions.put("robot", "icub");
		//armOptions.put("part", "arm");
		//armOptions.put("device", "remote_controlboard");
		//armOptions.put("local", "/icub/left_arm");   //local port names
		//armOptions.put("remote", "/icub/left_arm");         //where we connect to


		// create arm device
		_dd = new PolyDriver(armOptions);

		bool ok;
		ok = _dd->view(_iencs);
		ok = _dd->view(_ipos);
		ok = _dd->view(_ivel);

		if (!(ep=engOpen(NULL)))
		{
			cerr << "Opening MATLAB engine failed!" << endl;
			return false;
		}

		// show matlab console
		chageMLToCurDir(ep);
		engSetVisible(ep,true);

		engEvalString(ep,"q = zeros(7,1);");
		engEvalString(ep,"q(1,1) = -22;");
		engEvalString(ep,"q(2,1)= 22;");
		engEvalString(ep,"theta = zeros(7,1);");
		engEvalString(ep,"simstep = 0;");
		engEvalString(ep,"simstep0 = 0;");
		engEvalString(ep,"vel = zeros(7,1);");
		engEvalString(ep,"colorThresholds = [56 66 140 170];");
		engEvalString(ep,"y = zeros(4,1);");
		engEvalString(ep,"yd = y;");
		engEvalString(ep,"global y; global yd;");
		engEvalString(ep,"global theta;");
		engEvalString(ep,"global simstep;");
		engEvalString(ep,"global simstep0;");
		engEvalString(ep,"global vel;");
		engEvalString(ep,"global colorThresholds;");
		engEvalString(ep,"global q;");


		if (!(varoutQ=mxCreateDoubleMatrix(dimTheta,1,mxREAL)))
			cerr << "Unable to create mxMatrix" << endl;

		if (!(varoutTheta=mxCreateDoubleMatrix(dimTheta,1,mxREAL)))
			cerr << "Unable to create mxMatrix" << endl;

		if (!(varoutY=mxCreateDoubleMatrix(dimY,1,mxREAL)))
			cerr << "Unable to create mxMatrix" << endl;

		if (!(varoutYD=mxCreateDoubleMatrix(dimY,1,mxREAL)))
			cerr << "Unable to create mxMatrix" << endl;

		if (!(varoutSimStep=mxCreateDoubleMatrix(1,1,mxREAL) ))
			cerr << "Unable to create mxMatrix" << endl;


		cout << "Starting main thread..." << endl;

		ct = 0;

		//port.open("/test/cam/left");
		//Network::connect("/icubSim/cam/left","/test/cam/left");*/

		portInputLeft.open("/test/cam/left");
//		Network::connect("/icubSim/cam/left","/test/cam/left");
		Network::connect("/icubSim/cam/left","/test/cam/left");
//		Network::connect("/icub/cam/left","/test/cam/left");
		//Network::connect("/grabber","/test/cam/left");
		portInputRight.open("/test/cam/right");
		Network::connect("/icubSim/cam/right","/test/cam/right");
		//Network::connect("/icub/cam/right","/test/cam/right");

	
		//wecapture a first image to set the height and width of the image
		imgLeft = portInputLeft.read();

		w = imgLeft->width();//defining the image width

		h = imgLeft->height();//defining the image height

		//I capture an image to initialize just once the images variables
		cvImageCam = cvCreateImage(cvSize(w,h), IPL_DEPTH_8U, 3 );

		cvCvtColor((IplImage*)imgLeft->getIplImage(), cvImageCam, CV_RGB2BGR);

		image = cvCreateImage( cvGetSize(cvImageCam), 8, 3 );
		image->origin = cvImageCam->origin;
		hsv = cvCreateImage( cvGetSize(cvImageCam), 8, 3 );
		hue = cvCreateImage( cvGetSize(cvImageCam), 8, 1 );
		mask = cvCreateImage( cvGetSize(cvImageCam), 8, 1 );
		backproject = cvCreateImage( cvGetSize(cvImageCam), 8, 1 );
		hist = cvCreateHist( 1, &hdims, CV_HIST_ARRAY, &hranges, 1 );
		histimg = cvCreateImage( cvSize(320,200), 8, 3 );
		cvZero( histimg );

					//	// allocate all the buffers 
		image_left = cvCreateImage( cvGetSize(cvImageCam), 8, 3 );
		image_left->origin = cvImageCam->origin;
		hsv_left = cvCreateImage( cvGetSize(cvImageCam), 8, 3 );
		hue_left = cvCreateImage( cvGetSize(cvImageCam), 8, 1 );
		mask_left = cvCreateImage( cvGetSize(cvImageCam), 8, 1 );
		backproject_left = cvCreateImage( cvGetSize(cvImageCam), 8, 1 );
		hist_left = cvCreateHist( 1, &hdims_left, CV_HIST_ARRAY, &hranges_left, 1 );
		histimg_left = cvCreateImage( cvSize(320,200), 8, 3 );
		cvZero( histimg_left );


		return true;
	}

	virtual void afterStart(bool s)
	{
		t0=Time::now();

		if (s)
			cout << "Thread started successfully" << endl;
		else
			cout << "Thread did not start" << endl;
	}

	virtual void run()
	{

		imgRight = portInputRight.read();

		imgLeft  = portInputLeft.read();

		if ((imgRight != NULL)&&(imgLeft != NULL))

		{

			sendImageLeft(LOOK_L);

			sendImage(LOOK_R);
		
		}

		static double simStep = 0;

		static double data_c[NUM_CONTROLLED_JOINTS];
		//// read the joint positions
		_iencs->getEncoders(encoders);
		for (unsigned int i=0; i<dimTheta; i++)
			data_c[i]=encoders[i];
		//
		memcpy((char*)mxGetPr(varoutTheta),(char*)data_c, dimTheta*sizeof(double));
		if (engPutVariable(ep,"theta",varoutTheta))
			cerr << "Unable to update MATLAB workspace" << endl;



		// read matlab workspace
		mxArray *vel = engGetVariable(ep,"vel");
		double jointvels[16] = {0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0,  0.0};
		
		if (vel)
		{
			mwSize n = mxGetNumberOfElements(vel);
			double *_vel=mxGetPr(vel);
			for (int i=0; i < NUM_CONTROLLED_JOINTS; i++)
				jointvels[i] = _vel[i];

			// send velocities to joints
			_ivel->velocityMove(jointvels);

			mxDestroyArray(vel);
		}



		mxArray *pos = engGetVariable(ep,"q");
		double jointpos[16] =  {-22.0, 22.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0,  0.0};
		if (pos)
		{
			mwSize nq = mxGetNumberOfElements(pos);
			double *_pos=mxGetPr(pos);
			for (int i=0; i < NUM_CONTROLLED_JOINTS; i++)
				jointpos[i] = _pos[i];

			printf("%f %f %f %f %f %f %f\n", _pos[0], _pos[1], _pos[2], _pos[3],_pos[4], _pos[5], _pos[6]);
			//	// send positions to the joints
			_ipos->positionMove(jointpos);

			mxDestroyArray(pos);
		}

		double data_Y[NUM_FEATURES];

		for (int i =0; i < NUM_FEATURES; i++){
			data_Y[i]=b.get(i).asDouble();
		}

		b.clear();



		memcpy((char*)mxGetPr(varoutY),(char*)data_Y, dimY*sizeof(double));
		if (engPutVariable(ep,"y",varoutY))
			cerr << "Unable to update MATLAB workspace" << endl;	


		static double data_s[1];
		data_s[0] = simStep;
		memcpy((char*)mxGetPr(varoutSimStep),(char*)(data_s), 1*sizeof(double));
		if(engPutVariable(ep, "simstep", varoutSimStep))
			cerr<< "Unable to update MATLAB workspace" << endl;

		simStep++;
		printf("simStep=%f\n",simStep);





		double t=Time::now()-t0;

	}



	bool sendImage(int cam){

			//img = portInputRight.read(); 

			// wait for a key
			cvWaitKey(10);
			cvNamedWindow( "HistogramRight", 1 );
			cvNamedWindow( "CamShiftDemoRight", 1 );
			cvSetMouseCallback( "CamShiftDemoRight", on_mouse);
			cvCreateTrackbar( "Vmin", "CamShiftDemoRight", &vmin, 256, 0 );
			cvCreateTrackbar( "Vmax", "CamShiftDemoRight", &vmax, 256, 0 );
			cvCreateTrackbar( "Smin", "CamShiftDemoRight", &smin, 256, 0 );

			// image grabbed?
			if (imgRight == NULL) 
			{
				printf("no image found\n");
				return false;
			}

		int i, bin_w, c;
		
		cvCvtColor((IplImage*)imgRight->getIplImage(), cvImageCam, CV_RGB2BGR);

			cvCopy( cvImageCam, image, 0 );

			cvCvtColor( image, hsv, CV_BGR2HSV );

			if( track_object )
			{
				int _vmin = vmin, _vmax = vmax;

				cvInRangeS( hsv, cvScalar(0,smin,MIN(_vmin,_vmax),0),
					cvScalar(180,256,MAX(_vmin,_vmax),0), mask );
				cvSplit( hsv, hue, 0, 0, 0 );

				if( track_object < 0 )
				{
					float max_val = 0.f;
					cvSetImageROI( hue, selection );
					cvSetImageROI( mask, selection );
					cvCalcHist( &hue, hist, 0, mask );
					cvGetMinMaxHistValue( hist, 0, &max_val, 0, 0 );
					cvConvertScale( hist->bins, hist->bins, max_val ? 255. / max_val : 0., 0 );
					cvResetImageROI( hue );
					cvResetImageROI( mask );
					track_window = selection;
					track_object = 1;

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
				}

				cvCalcBackProject( &hue, backproject, hist );
				cvAnd( backproject, mask, backproject, 0 );
				cvCamShift( backproject, track_window,
					cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ),
					&track_comp, &track_box );
				track_window = track_comp.rect;

				if( backproject_mode )
					cvCvtColor( backproject, image, CV_GRAY2BGR );
				if( image->origin )
					track_box.angle = -track_box.angle;
				cvEllipseBox( image, track_box, CV_RGB(0,255,0), 3, CV_AA, 0 );
				//printf("rightangle=%f rightcenter=%f",track_box.angle,track_box.center.x);
			}

			if( select_object && selection.width > 0 && selection.height > 0 )
			{
				cvSetImageROI( image, selection );
				cvXorS( image, cvScalarAll(255), image, 0 );
				cvResetImageROI( image );
			}


			cvShowImage( "CamShiftDemoRight", image );
			cvShowImage( "HistogramRight", histimg );               			

		
		
		// Put blob values in the matlab workspace

		double posx = 0, posy = 0;
		double area = 0;
		double width = 0, height = 0;
		// for each blob

		if(track_box.center.x!=0)
		{
		    bot.addDouble(track_box.center.x);
		}else{
			bot.addDouble(0.0);
		}


		if(track_box.center.y!=0)
		{
		    bot.addDouble(track_box.center.y);
		}else{
			bot.addDouble(0.0);
		}

	

		
			
		
		printf("x=%s\n",bot.toString().c_str());





		if(cam == LOOK_R){
				double t=Time::now();

				for(int i = 0; i < NUM_FEATURES; i++)
				{
				 b.addDouble(bot.get(i).asDouble());
				}
				printf("x=%s\n",b.toString().c_str());

				bot.addDouble(t);

				outputFeatures.write(bot);
				bot.clear();
		}




		// output the image

		

		return true;	
	}



	bool sendImageLeft(int cam){




		

			//img = portInputLeft.read(); 

			// wait for a key
			cvWaitKey(10);
			cvNamedWindow( "HistogramLeft", 1 );
			cvNamedWindow( "CamShiftDemoLeft", 1 );
			cvSetMouseCallback( "CamShiftDemoLeft", on_mouse_left);
			cvCreateTrackbar( "Vmin", "CamShiftDemoLeft", &vmin_left, 256, 0 );
			cvCreateTrackbar( "Vmax", "CamShiftDemoLeft", &vmax_left, 256, 0 );
			cvCreateTrackbar( "Smin", "CamShiftDemoLeft", &smin_left, 256, 0 );

			// image grabbed?
			if (imgLeft == NULL) 
			{

				printf("no image found\n");
				return false;
			}

		

		

		int i, bin_w, c;


		
		cvCvtColor((IplImage*)imgLeft->getIplImage(), cvImageCam, CV_RGB2BGR);

		cvCopy( cvImageCam, image_left, 0 );

		cvCvtColor( image_left, hsv_left, CV_BGR2HSV );

			if( track_object_left )
			{
				int _vmin_left = vmin_left;
				int _vmax_left = vmax_left;

				cvInRangeS( hsv_left, cvScalar(0,smin_left,MIN(_vmin_left,_vmax_left),0),
					cvScalar(180,256,MAX(_vmin_left,_vmax_left),0), mask_left );
				cvSplit( hsv_left, hue_left, 0, 0, 0 );

				if( track_object_left < 0 )
				{
					float max_val_left = 0.f;
					cvSetImageROI( hue_left, selection_left );
					cvSetImageROI( mask_left, selection_left );
					cvCalcHist( &hue_left, hist_left, 0, mask_left );
					cvGetMinMaxHistValue( hist_left, 0, &max_val_left, 0, 0 );
					cvConvertScale( hist_left->bins, hist_left->bins, max_val_left ? 255. / max_val_left : 0., 0 );
					cvResetImageROI( hue_left );
					cvResetImageROI( mask_left );
					track_window_left = selection_left;
					track_object_left = 1;

					cvZero( histimg_left );
					bin_w = histimg_left->width / hdims_left;
					for( i = 0; i < hdims_left; i++ )
					{
						int val_left = cvRound( cvGetReal1D(hist_left->bins,i)*histimg_left->height/255 );
						CvScalar color_left = hsv2rgb(i*180.f/hdims_left);
						cvRectangle( histimg_left, cvPoint(i*bin_w,histimg_left->height),
							cvPoint((i+1)*bin_w,histimg_left->height - val_left),
							color_left, -1, 8, 0 );
					}
				}

				cvCalcBackProject( &hue_left, backproject_left, hist_left );
				cvAnd( backproject_left, mask_left, backproject_left, 0 );
				cvCamShift( backproject_left, track_window_left,
					cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ),
					&track_comp_left, &track_box_left );
				track_window_left = track_comp_left.rect;

				if( backproject_mode_left )
					cvCvtColor( backproject_left, image_left, CV_GRAY2BGR );
				if( image_left->origin )
					track_box_left.angle = -track_box_left.angle;
				cvEllipseBox( image_left, track_box_left, CV_RGB(0,255,0), 3, CV_AA, 0 );
				//printf("rightangle=%f rightcenter=%f",track_box.angle,track_box.center.x);
			}

			if( select_object_left && selection_left.width > 0 && selection_left.height > 0 )
			{
				cvSetImageROI( image_left, selection_left );
				cvXorS( image_left, cvScalarAll(255), image_left, 0 );
				cvResetImageROI( image_left );
			}


			cvShowImage( "CamShiftDemoLeft", image_left );
			cvShowImage( "HistogramLeft", histimg_left );               			

		//
		//// Put blob values in the matlab workspace

		double posx = 0, posy = 0;
		double area = 0;
		double width = 0, height = 0;
		// for each blob

		if(track_box_left.center.x!=0)
		{
		    bot.addDouble(track_box_left.center.x);
		}else{
			bot.addDouble(0.0);
		}


		if(track_box_left.center.y!=0)
		{
		    bot.addDouble(track_box_left.center.y);
		}else{
			bot.addDouble(0.0);
		}

	

		//printf("x=%s\n",bot.toString().c_str());


		return true;	
	}



	virtual void threadRelease()
	{

		portReadFeatures.close();

		delete [] encoders;

        cvReleaseImage(&cvImageCam);

		mxDestroyArray(varoutTheta);
		mxDestroyArray(varoutY);
		mxDestroyArray(varoutYD);
		mxDestroyArray(varoutQ);
		mxDestroyArray(varoutSimStep);

		engClose(ep);
	}
};



class GatewayModule: public Module
{
private:
	Network               &yarp;
	BufferedPort<Bottle>  rpcPort;
	GatewayThread         *thread;
	string                portName;





public:
	GatewayModule(Network &_yarp) : yarp(_yarp) {


	}

	virtual bool open(Searchable &s)
	{
		Property options(s.toString());
		string visibility;

		if (options.check("help"))
		{
			cout << "Options:" << endl << endl;
			cout << "\t--name        name:   port name (default /mlViewer)"                      << endl;
			cout << "\t--visibility  switch: set MATLAB session visibility on/off (default off)" << endl;

			return false;
		}

		if (options.check("name"))
			portName=options.find("name").asString();
		else
			portName="/mlViewer";


		string rpcPortName=portName+"/rpc";
		rpcPort.open(rpcPortName.c_str());
		attach(rpcPort,true);



		thread=new GatewayThread(20,portName,visibility);
		thread->start();

		return true;
	}

	virtual bool close()
	{
		thread->stop();
		rpcPort.close();


		delete thread;        

		return true;
	}

	virtual double getPeriod()    { return 1.0;  }
	virtual bool   updateModule() { return true; }
};



void chageMLToCurDir(Engine *ep)
{
	ACE_OS::system("cd > curDir.txt");

	char curDir[255];
	ifstream fin("curDir.txt");
	fin.getline(&curDir[0],255,'\n');
	fin.close();

	string cmd("cd '");
	cmd+=curDir;
	cmd+="/..'";

	engEvalString(ep,cmd.c_str());
	ACE_OS::system("rm curDir.txt");
}



int main(int argc, char *argv[])
{
	Network yarp;

	if (!yarp.checkNetwork())
		return false;

	GatewayModule mod(yarp);

	return mod.runModule(argc,argv);
}
