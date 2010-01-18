/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author Fernando Gamarra SSSA
 * email:   <firstname.secondname>@robotcub.org
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/
/** 
 * @ingroup icub_module
 *
 * \defgroup servoIcubSSSA servoIcubSSSA
 *
 * This module develops a reaching controller based on visual servoing.
 * the image based visual servocontroller uses an image Jacobian that
 * is initialized with the forward model created in the babbling phase. 
 *
 * \section lib_sec Libraries
 * - YARP libraries.
 * - OpenCV libraries.
 *
 * \section portsa_sec Ports Accessed
 * - /icub/head/rpc:i 
 *   is connected temporally just to put move the head in a position to see the hand
 * /icub/left_arm
 *   is connected to control the arm and babble the arm
 *   in the robot workspace
 * /icub/cam/left
 *   connected to the left camera
 * /icub/cam/right
 *   connected to the right camera
 * Command-line Parameters
 * 
 * The following key-value pairs can be specified as command-line parameters by prefixing -- to the key 
 * (e.g. --from file.ini. The value part can be changed to suit your needs; the default values are shown below. 
 *
 * - <key> <value>           
 *   <description>
 *
 * - ...
 *
 *
 * Configuration File Parameters
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 *   
 * - <key> <value>           
 *   <description> 
 *
 * - ... 
 *
 * 
 * \section portsa_sec Ports Accessed
 * 
 * - <portName>              
 *   <description>
 * 
 * - ...
 *                      
 * \section portsc_sec Ports Created
 *
 *  Input ports
 *
 *  - <portName>
 *    <description>
 *
 *  - ...
 *
 * Output ports
 *
 *  - <portName>
 *    <description>
 * 
 *  - ...
 *
 * Port types 
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 * - <portType>   <portName>        
 *
 *
 * \section in_files_sec Input Data Files
 *
 *  - <fileName>
 *    <description>
 *
 * \section out_data_sec Output Data Files
 *   -  jointAngles.mat 
 *      the joint angles of the robotic manipulator
 *   -  endEffectorImgPts.mat
 *      the end-effector position in the image space
  * \section conf_file_sec Configuration Files
 *
 *  - <fileName>
 *    <description>
 * 
 * \section tested_os_sec Tested OS
 *
 * Linux and Windows
 *  the servoIcubSSSA pops 4 windows, the first pair is for     
 *  defining the target and the second pair choses the 
 *  end effector.
 *the algorithm is based on the paper published by Piepmeier in:
 *J.Armstrong Piepmeier, G.V. McMurray, and H.Lipkin. A dynamic 
 *Jacobian estimation method for uncalibrated visual servoing", In
 *In Proceedings of the IEEE Internationalconference
 *onAdvanced Mechatronics(ASME), 1999
 * \author Fernando Gamarra
 * 
 * Copyright (C) 2009 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * This file can be edited at src/<moduleName>/src/<moduleName.h>.
 * 
 **/

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

#include <gsl/gsl_linalg.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>



#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>

#include <stdio.h>
#include <cv.h>
#include <highgui.h>

#include <cstdlib> 



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

int backproject_mode_target = 0;
int select_object_target = 0;
int track_object_target = 0;
int show_hist_target = 1;
CvPoint origin_target;
CvRect selection_target;
CvRect track_window_target;
CvBox2D track_box_target;
CvConnectedComp track_comp_target;
int hdims_target = 16;
float hranges_arr_target[] = {0,180};
float* hranges_target = hranges_arr_target;
int vmin_target = 10;
int vmax_target = 256;
int smin_target = 30;

IplImage *image_target = NULL;


int backproject_mode_target_left = 0;
int select_object_target_left = 0;
int track_object_target_left = 0;
int show_hist_target_left = 1;
CvPoint origin_target_left;
CvRect selection_target_left;
CvRect track_window_target_left;
CvBox2D track_box_target_left;
CvConnectedComp track_comp_target_left;
int hdims_target_left = 16;
float hranges_arr_target_left[] = {0,180};
float* hranges_target_left = hranges_arr_target_left;
int vmin_target_left = 10;
int vmax_target_left = 256;
int smin_target_left = 30;

IplImage *image_target_left = NULL;

bool initServo = false;
//Global Variables defined for the visual servoing

double meanValuefk;

double lambda;

FILE *vPk = fopen("vPk.dat", "w");

FILE *vJ = fopen("J.dat", "w");

FILE *vq = fopen("q.dat", "w");

FILE *vError = fopen("error.dat", "w");


//desired features
double udL, vdL, udR, vdR;		

//joint vector position

double qv[] = {0.0,
		       0.0,
			   0.0,
			   0.0,
			   0.0,
			   0.0};

gsl_matrix_view q = gsl_matrix_view_array(qv,6,1);
	
//initial features
double uL, vL, uR, vR;

//initial Jacobian

		/*double Jm[] ={1.00, 0.00, 0.00, 0.00, 0.00, 0.00,
	           0.00, 1.00, 0.00, 0.00, 0.00, 0.00,
			   0.00, 0.00, 1.00, 0.00, 0.00, 0.00,
	           0.00, 0.00, 0.00, 1.00, 0.00, 0.00};
*/
double Jm[] ={rand(), rand(), rand(), rand(), rand(), rand(),
	           rand(), rand(), rand(), rand(), rand(), rand(),
			   rand(), rand(), rand(), rand(), rand(), rand(),
	           rand(), rand(), rand(), rand(), rand(), rand()};

		gsl_matrix_view J = gsl_matrix_view_array(Jm,4,6);
    
      //Jacobian pseudo inverse
		double pinvJm[] ={0.00, 0.00, 0.00, 0.00,
	           0.00, 0.00, 0.00, 0.00, 
			   0.00, 0.00, 0.00, 0.00,
			   0.00, 0.00, 0.00, 0.00,
			   0.00, 0.00, 0.00, 0.00,
	           0.00, 0.00, 0.00, 0.00};

		gsl_matrix_view pinvJ = gsl_matrix_view_array(pinvJm,6,4);
    
		//covariance matrix
		double Pkm[] ={1000.00, 0.00, 0.00, 0.00, 0.00, 0.00,
	           0.00, 1000.00, 0.00, 0.00, 0.00, 0.00,
			   0.00, 0.00, 1000.00, 0.00, 0.00, 0.00,
	           0.00, 0.00, 0.00, 1000.00, 0.00, 0.00,
		       0.00, 0.00, 0.00, 0.00,  1000.00, 0.00,
			   0.00, 0.00, 0.00, 0.00, 0.00, 1000.00		
		};

		gsl_matrix_view Pk = gsl_matrix_view_array(Pkm,6,6);
    
		
		//target features
		double ydm[] = {udL,
		                vdL,
						udR,
						vdR};

		gsl_matrix_view yd = gsl_matrix_view_array(ydm,4,1);
	

		//end effector features
		double ym[] = {uL,
		               vL,
					   uR,
					   vR};

		gsl_matrix_view y = gsl_matrix_view_array(ym,4,1);
	
        //variation of the features
		double fm[] = {uL,
		               vL,
					   uR,
					   vR};

		gsl_matrix_view f = gsl_matrix_view_array(fm,4,1);
	

		double fkm[] = {uL,
		               vL,
					   uR,
					   vR};

		gsl_matrix_view fk = gsl_matrix_view_array(fkm,4,1);
	
		//variation of the joint positions

		double qPerturb = 0.01;

		double qkv[] = {0.0,
		                0.0,
						0.0,
						0.0,
						0.0,
					    0.0};

		gsl_matrix_view qk = gsl_matrix_view_array(qkv,6,1);
	

		double deltaqm[] = {0.00,
		                   0.00,
						   0.00,
						   0.00,
						   0.00,
					       0.00};

		gsl_matrix_view deltaq = gsl_matrix_view_array(deltaqm,6,1);
	
		//variation of the features

		double deltafm[] = {0.00,
		               0.00,
					   0.00,
					   0.00};

		gsl_matrix_view deltaf = gsl_matrix_view_array(deltafm,4,1);


		//variation of the joint positions

		double hthetam[] = {0.00,
		               0.00,
					   0.00,
					   0.00,
					   0.00,
					   0.00};

		gsl_matrix_view htheta = gsl_matrix_view_array(hthetam,6,1);


		
	//defining temporal variables for J
		double temp1m[] = {0.00,
		               0.00,
					   0.00,
					   0.00};

		gsl_matrix_view temp1 = gsl_matrix_view_array(temp1m,4,1);

//		PrintMatrix(temp1);

		double temp2m[] = {0.00,
		               0.00,
					   0.00,
					   0.00};

		gsl_matrix_view temp2 = gsl_matrix_view_array(temp2m,4,1);


		double hthetaTm[] = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00};

		gsl_matrix_view hthetaT = gsl_matrix_view_array(hthetaTm,1,6);


		double temp3m[] ={0.00, 0.00, 0.00, 0.00, 0.00, 0.00};

		gsl_matrix_view temp3 = gsl_matrix_view_array(temp3m,1,6);
    


		double temp4m[] ={0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
	           0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
			   0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
	           0.00, 0.00, 0.00, 0.00, 0.00, 0.00};

		gsl_matrix_view temp4 = gsl_matrix_view_array(temp4m,4,6);
    

		double temp5m[] ={0.00,
			              0.00,
						  0.00,
						  0.00,
						  0.00,
						  0.00};

		gsl_matrix_view temp5 = gsl_matrix_view_array(temp5m,6,1);
    

		double temp6m[] ={0.00};

		gsl_matrix_view temp6 = gsl_matrix_view_array(temp6m,1,1);
    

		double temp7 = 0.00;

		//defining temporal variables for P in the numerator

		double temp1PkNumm[] = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00};

		gsl_matrix_view temp1PkNum = gsl_matrix_view_array(temp1PkNumm,1,6);


        double temp2PkNumm[] ={0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
	           0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
			   0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
	           0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
		       0.00, 0.00, 0.00, 0.00,  0.00, 0.00,
			   0.00, 0.00, 0.00, 0.00, 0.00, 0.00		
		};

		gsl_matrix_view temp2PkNum = gsl_matrix_view_array(temp2PkNumm,6,6);
    

		double temp3PkNumm[] ={0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
	           0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
			   0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
	           0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
		       0.00, 0.00, 0.00, 0.00,  0.00, 0.00,
			   0.00, 0.00, 0.00, 0.00, 0.00, 0.00		
		};

		gsl_matrix_view temp3PkNum = gsl_matrix_view_array(temp3PkNumm,6,6);
    

		//defining temporal variables for P in the denominator

		double temp1PkDenm[] = {0.00,
			                    0.00,
								0.00, 
								0.00,
								0.00, 
								0.00};

		gsl_matrix_view temp1PkDen = gsl_matrix_view_array(temp1PkDenm,6,1);


		double temp2PkDenm[] = {0.00};

		gsl_matrix_view temp2PkDen = gsl_matrix_view_array(temp2PkDenm,1,1);


		double temp3PkDen;

		double tempInvlambda;

		ifstream infileuL, infilevL, infileuR, infilevR;

		int nIterationsVS = 0;

		bool initialPos = false;

		bool initFeatures = false;



void PrintMatrix(gsl_matrix_view M){

	int col = M.matrix.size1;
	int row = M.matrix.size2;
	double temp;

	for(int i=0; i<col ;i++) 
	{
		printf("\n ");
		for(int j=0; j<row ;j++){
		    temp =  gsl_matrix_get( &M.matrix, i, j );
	 	    printf(" %g " , temp ) ;
		}
		 
	}
	printf("\n"); printf("\n ");

}

/**on_mouse 
*Function to capture the right mouse movement
*/
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



/**on_mouse_left
*Function to capture the left mouse movement
*/
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

/**on_mouse_target
*Function to capture the right mouse target movement
*/
void on_mouse_target( int event, int x, int y, int flags, void* param )
{
	if( !image_target )
		return;

		if( image_target->origin )
			y = image_target->height - y;

		if( select_object_target )
		{
			selection_target.x = MIN(x,origin_target.x);
			selection_target.y = MIN(y,origin_target.y);
			selection_target.width = selection_target.x + CV_IABS(x - origin_target.x);
			selection_target.height = selection_target.y + CV_IABS(y - origin_target.y);

			selection_target.x = MAX( selection_target.x, 0 );
			selection_target.y = MAX( selection_target.y, 0 );
			selection_target.width = MIN( selection_target.width, image_target->width );
			selection_target.height = MIN( selection_target.height, image_target->height );
			selection_target.width -= selection_target.x;
			selection_target.height -= selection_target.y;
		}

	

	switch( event )
	{
	case CV_EVENT_LBUTTONDOWN:
		origin_target = cvPoint(x,y);
		selection_target = cvRect(x,y,0,0);
		select_object_target = 1;
		break;
	case CV_EVENT_LBUTTONUP:
		select_object_target = 0;
		if( selection_target.width > 0 && selection_target.height > 0 )
			track_object_target = -1;
		break;
	}
}

/**on_mouse_target_left
*Function to capture the right mouse target movement
*/
void on_mouse_target_left( int event, int x, int y, int flags, void* param )
{
	if( !image_target_left )
		return;

		if( image_target_left->origin )
			y = image_target_left->height - y;

		if( select_object_target_left )
		{
			selection_target_left.x = MIN(x,origin_target_left.x);
			selection_target_left.y = MIN(y,origin_target_left.y);
			selection_target_left.width = selection_target_left.x + CV_IABS(x - origin_target_left.x);
			selection_target_left.height = selection_target_left.y + CV_IABS(y - origin_target_left.y);

			selection_target_left.x = MAX( selection_target_left.x, 0 );
			selection_target_left.y = MAX( selection_target_left.y, 0 );
			selection_target_left.width = MIN( selection_target_left.width, image_target_left->width );
			selection_target_left.height = MIN( selection_target_left.height, image_target_left->height );
			selection_target_left.width -= selection_target_left.x;
			selection_target_left.height -= selection_target_left.y;
		}

	

	switch( event )
	{
	case CV_EVENT_RBUTTONDOWN:
		origin_target_left = cvPoint(x,y);
		selection_target_left = cvRect(x,y,0,0);
		select_object_target_left = 1;
		break;
	case CV_EVENT_RBUTTONUP:
		select_object_target_left = 0;
		if( selection_target_left.width > 0 && selection_target_left.height > 0 )
			track_object_target_left = -1;
		break;
	}
}


/**hsv2rgb
*change of color space from HSV to RGB
*/
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


/**GatewayThread
*main class where we can find all the functions that will
*deploy when the module is launched
*/

class GatewayThread : public RateThread
{
private:

	Port                  portReadFeatures;
	unsigned int          dim;

	int    period;
	string visibility;
	string portName;
	
	int w, h;//image width and height

	unsigned int N,idx;

	double t0;
	double go_old;

	PolyDriver *_dd;
	IEncoders *_iencs;
	IPositionControl *_ipos;
	IVelocityControl *_ivel;
	double *encoders;
	dev::IEncoders *iHeadEncs;

	//image processing 
	// Make a port for reading and writing images
	BufferedPort<ImageOf<PixelRgb> >   port;
	BufferedPort<ImageOf<PixelRgb> >   portInputLeft;
	BufferedPort<ImageOf<PixelRgb> >   portInputRight;
	BufferedPort<ImageOf<PixelRgb> >   portOutputLeft;
	BufferedPort<ImageOf<PixelRgb> >   portOutputRight;
	Port cmdPort;
	Port outputFeatures;	
	Port outputFeaturesTarget;
	int ct;
	IplImage                            *cvImageCam;
	//ImageOf<PixelRgb>                   *img;
	ImageOf<PixelRgb>                   *imgRight;
	ImageOf<PixelRgb>                   *imgLeft;


	Bottle                              bot;
	Bottle                              botYd;
	Bottle                              b;
	Bottle                              bYd;

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

	IplImage *hsv_target;
	IplImage *hue_target;
	IplImage *mask_target;
	IplImage *backproject_target;
	IplImage *histimg_target;

	CvHistogram *hist_target;

	IplImage *hsv_target_left;
	IplImage *hue_target_left;
	IplImage *mask_target_left;
	IplImage *backproject_target_left;
	IplImage *histimg_target_left;

	CvHistogram *hist_target_left;


public:
	GatewayThread(int _period, string &_portName, string &_visibility) : RateThread(_period),
		period(_period), portName(_portName), visibility(_visibility) 
	{
		dim   =7;
		N     =500;
		idx   =1;
		go_old=0.0;
		encoders = new double [ARM_JOINTS];
		


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

		hsv_target = NULL;
	    hue_target = NULL;
	    mask_target = NULL;
	    backproject_target = NULL;
	    histimg_target = NULL;

	    hist_target = NULL;

		hsv_target_left = NULL;
	    hue_target_left = NULL;
	    mask_target_left = NULL;
	    backproject_target_left = NULL;
	    histimg_target_left = NULL;

	    hist_target_left = NULL;

		imgRight = NULL;
	    imgLeft  = NULL;


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

		Property headOptions;
     //   headOptions.put("robot", "icubSim");
     //   headOptions.put("part", "head");
     //   headOptions.put("device", "remote_controlboard");
	    //headOptions.put("subdevice", "jameshead");
     //   headOptions.put("local", "/icubSim/head");   //local port names
     //   headOptions.put("remote", "/icubSim/head");         //where we connect to

		//headOptions.put("robot", "icub");
  //      headOptions.put("part", "head");
  //      headOptions.put("device", "remote_controlboard");
	 //   headOptions.put("subdevice", "jameshead");
  //      headOptions.put("local", "/icub/head");   //local port names
  //      headOptions.put("remote", "/icub/head");         //where we connect to

		// create arm device
		_dd = new PolyDriver(armOptions);
		// create head device
		//dev::PolyDriver head;

		//head.open(headOptions);
		bool ok;
		ok = _dd->view(_iencs);
		ok = _dd->view(_ipos);
		ok = _dd->view(_ivel);

		/*iHeadEncs=0;
		bool ret;

		ret=ret&&head.view(iHeadEncs);
	    if (!ret)
		    fprintf(stderr, "Problem acquiring head encoder interface, thread will not start\n");
		if (!ret)
		    return false;*/


		BufferedPort<Bottle> outNeck;

        outNeck.open("/nat/out/neck");
	    Network::connect("/nat/out/neck","/icubSim/head/rpc:i");
		//Network::connect("/nat/out/neck","/icub/head/rpc:i");
        Bottle& outbotNeck1 = outNeck.prepare(); //get the object
	    outbotNeck1.clear();
	    outbotNeck1.addString("set"); //put "set" command in the bottle
	    outbotNeck1.addString("pos"); //put "pos" command in the bottle
	    outbotNeck1.addInt(0);//control joint 0
	    outbotNeck1.addInt(-45);// move to -45
	    printf("writing bottle Neck 1 (%s)\n", outbotNeck1.toString().c_str());
	    outNeck.write();            //now send it on its way
		Time::delay(1);

		outNeck.close();
	    Network::disconnect("/icubSim/head/rpc:i", "/nat/out/neck");
		//Network::disconnect("/icub/head/rpc:i", "/nat/out/neck");
	
		cout << "Starting main thread..." << endl;

		ct = 0;

		//port.open("/test/cam/left");
		//Network::connect("/icubSim/cam/left","/test/cam/left");*/

		portInputLeft.open("/test/cam/left");
//		Network::connect("/icubSim/cam/left","/test/cam/left");
		Network::connect("/icubSim/cam/left","/test/cam/left");
    	//Network::connect("/icub/cam/left","/test/cam/left");
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

			//	// allocate all the buffers for the target
		image_target = cvCreateImage( cvGetSize(cvImageCam), 8, 3 );
		image_target->origin = cvImageCam->origin;
		hsv_target = cvCreateImage( cvGetSize(cvImageCam), 8, 3 );
		hue_target = cvCreateImage( cvGetSize(cvImageCam), 8, 1 );
		mask_target = cvCreateImage( cvGetSize(cvImageCam), 8, 1 );
		backproject_target = cvCreateImage( cvGetSize(cvImageCam), 8, 1 );
		hist_target = cvCreateHist( 1, &hdims_target, CV_HIST_ARRAY, &hranges_target, 1 );
		histimg_target = cvCreateImage( cvSize(320,200), 8, 3 );
		cvZero( histimg_target );


		//	// allocate all the buffers for the left target
		image_target_left = cvCreateImage( cvGetSize(cvImageCam), 8, 3 );
		image_target_left->origin = cvImageCam->origin;
		hsv_target_left = cvCreateImage( cvGetSize(cvImageCam), 8, 3 );
		hue_target_left = cvCreateImage( cvGetSize(cvImageCam), 8, 1 );
		mask_target_left = cvCreateImage( cvGetSize(cvImageCam), 8, 1 );
		backproject_target_left = cvCreateImage( cvGetSize(cvImageCam), 8, 1 );
		hist_target_left = cvCreateHist( 1, &hdims_target_left, CV_HIST_ARRAY, &hranges_target_left, 1 );
		histimg_target_left = cvCreateImage( cvSize(320,200), 8, 3 );
		cvZero( histimg_target_left );

		infileuL.open("uLVec.dat");

		infilevL.open("vLVec.dat");

		infileuR.open("uRVec.dat");

		infilevR.open("vRVec.dat");

		double jointpos[16] =  {-22.0, 22.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0,  0.0};

		// send positions to the joints
		_ipos->positionMove(jointpos);

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

			sendImage(LOOK_R);

			sendImageLeft(LOOK_L);			

			//if (initServo == false)
			if (initFeatures == false)
			{

				sendImageTarget(LOOK_R);

				sendImageTargetLeft(LOOK_L);

			}
		
		}

		double data_Y[NUM_FEATURES];
		double data_YD[NUM_FEATURES];

		for (int i =0; i < NUM_FEATURES; i++){
			data_Y[i]  = b.get(i).asDouble();
			data_YD[i] = bYd.get(i).asDouble();
		}

		b.clear();

		bYd.clear();

		static double simStep = 0;

		static double data_c[NUM_CONTROLLED_JOINTS];
		//// read the joint positions
		_iencs->getEncoders(encoders);
		for (unsigned int i=0; i<dimTheta; i++)
			data_c[i]=encoders[i];

		
		if(initServo == false){

			if((data_YD[0] != 0)&& (data_YD[1] != 0)&&(data_YD[2] != 0)&&(data_YD[3] != 0))
			{
				if(initFeatures == false)
						{

								udL = data_YD[0];

								vdL = data_YD[1];

								udR = data_YD[2];

								vdR = data_YD[3];

								initFeatures = true;


				        }

			}
					if((data_Y[0] != 0)&& (data_Y[1] != 0)&&(data_Y[2] != 0)&&(data_Y[3] != 0))
					{
									//if (initialPos == false)	
									//{
						                uL = data_Y[0];

				                        vL = data_Y[1];

				                        uR = data_Y[2];

				                        vR = data_Y[3];
									//}

						
										//if (initialPos == false)
										//    {
										//		  double jointpos[16] =  {-22.0, 22.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0,  0.0};
										//		  // send positions to the joints
										//		  _ipos->positionMove(jointpos);
										//		  //Time::delay(3);
										//		  initialPos = true;


										//	  } else{

												   gsl_matrix_set(&q.matrix, 0, 0, data_c[0]);
												   gsl_matrix_set(&q.matrix, 1, 0, data_c[1]);
												   gsl_matrix_set(&q.matrix, 2, 0, data_c[2]);
												   gsl_matrix_set(&q.matrix, 3, 0, data_c[3]);
												   gsl_matrix_set(&q.matrix, 4, 0, data_c[4]);
												   gsl_matrix_set(&q.matrix, 5, 0, data_c[5]);

                                                   //Time::delay(5);
			    								   initServo = initServoing();


											 // }
			                            

					}		
			
			//}
				
		} 


        if ((nIterationsVS < 800)&&(initServo == true))
		{
				
			  

           if(nIterationsVS != 0)
		   {
			  uL = data_Y[0];

			  vL = data_Y[1];

			  uR = data_Y[2];

			  vR = data_Y[3];

		      updatingServoingVariablesP();
           }

		   if(initServo == true)
		   {
  
		      visualServoingP();

			  nIterationsVS++;

			  printf("iteration=%d\n",nIterationsVS);

			  printf("Error=%f\n",norm_matrix(&f.matrix));

			  fprintf(vError, "%f \n", norm_matrix(&f.matrix));


		   }

		}

		

		if (nIterationsVS == 800)
		{
		
	       fclose(vq);	
		   fclose(vError);
		}


		//
//		memcpy((char*)mxGetPr(varoutTheta),(char*)data_c, dimTheta*sizeof(double));
//		if (engPutVariable(ep,"theta",varoutTheta))
//			cerr << "Unable to update MATLAB workspace" << endl;



		// read matlab workspace
//		mxArray *vel = engGetVariable(ep,"vel");
//		double jointvels[16] = {0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0,  0.0};
		
		//if (vel)
		//{
		//	mwSize n = mxGetNumberOfElements(vel);
		//	double *_vel=mxGetPr(vel);
		//	for (int i=0; i < NUM_CONTROLLED_JOINTS; i++)
		//		jointvels[i] = _vel[i];

		//	// send velocities to joints
		//	_ivel->velocityMove(jointvels);

		//	mxDestroyArray(vel);
		//}



		//mxArray *pos = engGetVariable(ep,"q");
		double jointpos[16] =  {-22.0, 22.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0,  0.0, 0.0, 0.0, 0.0, 0.0,  0.0};
		
		if ((initServo == true))
		   {
			for (int i=0; i < NUM_CONTROLLED_JOINTS; i++)
				jointpos[i] = gsl_matrix_get(&q.matrix, 0, i);

			// send positions to the joints
			_ipos->positionMove(jointpos);
			//Time::delay(1);
			//Time::delay(2);
			Time::delay(0.25);

			printf("%f %f %f %f %f %f %f\n", jointpos[0], jointpos[1], jointpos[2], jointpos[3],jointpos[4], jointpos[5], jointpos[6]);
		   }
	

	


		//memcpy((char*)mxGetPr(varoutY),(char*)data_Y, dimY*sizeof(double));
		//if (engPutVariable(ep,"y",varoutY))
		//	cerr << "Unable to update MATLAB workspace" << endl;	

		//memcpy((char*)mxGetPr(varoutYD),(char*)data_YD, dimY*sizeof(double));
		//if (engPutVariable(ep,"yd",varoutYD))
		//	cerr << "Unable to update MATLAB workspace" << endl;	



		static double data_s[1];
		data_s[0] = simStep;
		//memcpy((char*)mxGetPr(varoutSimStep),(char*)(data_s), 1*sizeof(double));
		//if(engPutVariable(ep, "simstep", varoutSimStep))
		//	cerr<< "Unable to update MATLAB workspace" << endl;

		simStep++;
		printf("simStep=%f\n",simStep);

		double t=Time::now()-t0;

	}



	bool initServoing(void){

		//defining yd

		gsl_matrix_set(&yd.matrix, 0, 0, udL);

		gsl_matrix_set(&yd.matrix, 1, 0, vdL);

		gsl_matrix_set(&yd.matrix, 2, 0, udR);

		gsl_matrix_set(&yd.matrix, 3, 0, vdR);

		//defining y

		gsl_matrix_set(&y.matrix, 0, 0, uL);

		gsl_matrix_set(&y.matrix, 1, 0, vL);

		gsl_matrix_set(&y.matrix, 2, 0, uR);

		gsl_matrix_set(&y.matrix, 3, 0, vR);

       //defining f

		gsl_matrix_set(&f.matrix, 0, 0, uL);

		gsl_matrix_set(&f.matrix, 1, 0, vL);

		gsl_matrix_set(&f.matrix, 2, 0, uR);

		gsl_matrix_set(&f.matrix, 3, 0, vR);

        //defining fk
        gsl_matrix_set(&fk.matrix, 0, 0, uL);

		gsl_matrix_set(&fk.matrix, 1, 0, vL);

		gsl_matrix_set(&fk.matrix, 2, 0, uR);

		gsl_matrix_set(&fk.matrix, 3, 0, vR);

		 gsl_matrix_sub(&f.matrix,&yd.matrix);

		 PrintMatrix(f);

		 gsl_matrix_memcpy(&htheta.matrix,&q.matrix);

		 PrintMatrix(htheta);

		 gsl_matrix_memcpy(&qk.matrix,&q.matrix);

		 
		 gsl_matrix_add_constant(&qk.matrix, qPerturb);

		 PrintMatrix(qk);





		 gsl_matrix_sub(&fk.matrix,&yd.matrix);

		 PrintMatrix(fk);

		 
		 gsl_matrix_memcpy(&deltaf.matrix,&f.matrix);

		 PrintMatrix(deltaf);


		 
		 gsl_matrix_sub(&deltaf.matrix,&fk.matrix);

		 PrintMatrix(deltaf);


		 gsl_matrix_sub(&htheta.matrix,&qk.matrix);	

		 PrintMatrix(htheta);

	
	     return true;
	}

	void updatingServoingVariables(){
			  //updating feature points of the endeffector

		   
		   gsl_matrix_set(&f.matrix, 0, 0, uL);

		   gsl_matrix_set(&f.matrix, 1, 0, vL);

		   gsl_matrix_set(&f.matrix, 2, 0, uR);

		   gsl_matrix_set(&f.matrix, 3, 0, vR);

		   PrintMatrix(f);

		   //calculating a new diference between the image features f = y - yd
           PrintMatrix(yd); 

		   gsl_matrix_sub(&f.matrix,&yd.matrix);
	
		   PrintMatrix(f);

		   //calculating a new diference between the image feature variations deltaf = f - fk

		   gsl_matrix_memcpy(&deltaf.matrix,&f.matrix);

    	   PrintMatrix(deltaf);

		   gsl_matrix_sub(&deltaf.matrix,&fk.matrix);

	       PrintMatrix(deltaf);

		   //calculating a new diference between the joint angles htheta = q - qk

		   	gsl_matrix_memcpy(&htheta.matrix,&q.matrix);

    	    PrintMatrix(htheta);

		    gsl_matrix_sub(&htheta.matrix,&qk.matrix);

	        PrintMatrix(htheta);

			//update previous value of fk

			gsl_matrix_memcpy(&fk.matrix,&f.matrix);

			//update previous value of qk

			gsl_matrix_memcpy(&qk.matrix,&q.matrix);

			//writing values of Pk, J and q

			//fprintf(vPk, "%f\n", Make);

			//fprintf(vJ, "%f\n", Make);

			fprintf(vq, "%f %f %f %f %f %f\n", gsl_matrix_get(&q.matrix, 0, 0), gsl_matrix_get(&q.matrix, 1, 0), gsl_matrix_get(&q.matrix, 2, 0), gsl_matrix_get(&q.matrix, 3, 0), gsl_matrix_get(&q.matrix, 4, 0), gsl_matrix_get(&q.matrix, 5, 0));

	
	
	
	}



	void updatingServoingVariablesP(){
			  //updating feature points of the endeffector

		   
		   gsl_matrix_set(&f.matrix, 0, 0, uL);

		   gsl_matrix_set(&f.matrix, 1, 0, vL);

		   gsl_matrix_set(&f.matrix, 2, 0, uR);

		   gsl_matrix_set(&f.matrix, 3, 0, vR);

		   //PrintMatrix(f);

		   //calculating a new diference between the image features f = y - yd
           PrintMatrix(yd); 

		   gsl_matrix_sub(&f.matrix,&yd.matrix);
	
		   //PrintMatrix(f);

		   //calculating a new diference between the image feature variations deltaf = f - fk

		   gsl_matrix_memcpy(&deltaf.matrix,&f.matrix);

    	   //PrintMatrix(deltaf);

		   gsl_matrix_sub(&deltaf.matrix,&fk.matrix);

	       //PrintMatrix(deltaf);

		   //calculating a new diference between the joint angles htheta = q - qk

		   	gsl_matrix_memcpy(&htheta.matrix,&q.matrix);

    	    //PrintMatrix(htheta);

		    gsl_matrix_sub(&htheta.matrix,&qk.matrix);

	        //PrintMatrix(htheta);

			//update previous value of fk

			gsl_matrix_memcpy(&fk.matrix,&f.matrix);

			//update previous value of qk

			gsl_matrix_memcpy(&qk.matrix,&q.matrix);

			//writing values of Pk, J and q

			//fprintf(vPk, "%f\n", Make);

			//fprintf(vJ, "%f\n", Make);

			fprintf(vq, "%f %f %f %f %f %f\n", gsl_matrix_get(&q.matrix, 0, 0), gsl_matrix_get(&q.matrix, 1, 0), gsl_matrix_get(&q.matrix, 2, 0), gsl_matrix_get(&q.matrix, 3, 0), gsl_matrix_get(&q.matrix, 4, 0), gsl_matrix_get(&q.matrix, 5, 0));

	
	
	
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


	bool sendImageTarget(int cam){


      //img = portInputRight.read();

	  // wait for a key
	  cvWaitKey(10);
	  cvNamedWindow( "HistogramTarget", 1 );
	  cvNamedWindow( "CamShiftDemoTarget", 1 );
	  cvSetMouseCallback( "CamShiftDemoTarget", on_mouse_target);
	  cvCreateTrackbar( "Vmin", "CamShiftDemoTarget", &vmin_target, 256, 0 );
	  cvCreateTrackbar( "Vmax", "CamShiftDemoTarget", &vmax_target, 256, 0 );
	  cvCreateTrackbar( "Smin", "CamShiftDemoTarget", &smin_target, 256, 0 );


	  // image grabbed?
			if (imgRight == NULL) 
			{

				printf("no image found\n");
				return false;
			}

		

		

		int i, bin_w, c;


		
		cvCvtColor((IplImage*)imgRight->getIplImage(), cvImageCam, CV_RGB2BGR);

		cvCopy( cvImageCam, image_target, 0 );

		cvCvtColor( image_target, hsv_target, CV_BGR2HSV );

		if( track_object_target )
		{



     			int _vmin_target = vmin_target;
				int _vmax_target = vmax_target;

				cvInRangeS( hsv_target, cvScalar(0,smin_target,MIN(_vmin_target,_vmax_target),0),
					cvScalar(180,256,MAX(_vmin_target,_vmax_target),0), mask_target );
				cvSplit( hsv_target, hue_target, 0, 0, 0 );

				if( track_object_target < 0 )
				{
					float max_val_target = 0.f;
					cvSetImageROI( hue_target, selection_target );
					cvSetImageROI( mask_target, selection_target );
					cvCalcHist( &hue_target, hist_target, 0, mask_target );
					cvGetMinMaxHistValue( hist_target, 0, &max_val_target, 0, 0 );
					cvConvertScale( hist_target->bins, hist_target->bins, max_val_target ? 255. / max_val_target : 0., 0 );
					cvResetImageROI( hue_target );
					cvResetImageROI( mask_target );
					track_window_target = selection_target;
					track_object_target = 1;

					cvZero( histimg_target );
					bin_w = histimg_target->width / hdims_target;
					for( i = 0; i < hdims_target; i++ )
					{
						int val_target = cvRound( cvGetReal1D(hist_target->bins,i)*histimg_target->height/255 );
						CvScalar color_target = hsv2rgb(i*180.f/hdims_target);
						cvRectangle( histimg_target, cvPoint(i*bin_w,histimg_target->height),
							cvPoint((i+1)*bin_w,histimg_target->height - val_target),
							color_target, -1, 8, 0 );
					}


				}

				cvCalcBackProject( &hue_target, backproject_target, hist_target );
				cvAnd( backproject_target, mask_target, backproject_target, 0 );
				cvCamShift( backproject_target, track_window_target,
					cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ),
					&track_comp_target, &track_box_target );
				track_window_target = track_comp_target.rect;

				if( backproject_mode_target )
					cvCvtColor( backproject_target, image_target, CV_GRAY2BGR );
				if( image_target->origin )
					track_box_target.angle = -track_box_target.angle;
				cvEllipseBox( image_target, track_box_target, CV_RGB(0,255,0), 3, CV_AA, 0 );
				//printf("rightangle=%f rightcenter=%f",track_box.angle,track_box.center.x);




		}

		if( select_object_target && selection_target.width > 0 && selection_target.height > 0 )
			{
				cvSetImageROI( image_target, selection_target );
				cvXorS( image_target, cvScalarAll(255), image_target, 0 );
				cvResetImageROI( image_target );
			}


		cvShowImage( "CamShiftDemoTarget", image_target );
		cvShowImage( "HistogramTarget", histimg_target );    


		// Put blob values in the matlab workspace for the target

		double posx = 0, posy = 0;
		double area = 0;
		double width = 0, height = 0;
		// for each blob

		/*if(track_box_target.center.x!=0)
		{
		    botYd.addDouble(track_box_target.center.x);
		}else{
			botYd.addDouble(0.0);
		}


		if(track_box_target.center.y!=0)
		{
		    botYd.addDouble(track_box_target.center.y);
		}else{
			botYd.addDouble(0.0);
		}*/

		if(track_box.angle!=0)
		{
		    bot.addDouble(track_box.center.x);
		}else{
			bot.addDouble(0.0);
		}


		if(track_box.angle!=0)
		{
		    bot.addDouble(track_box.center.y);
		}else{
			bot.addDouble(0.0);
		}


		if(cam == LOOK_R){
				double t=Time::now();

				for(int i = 0; i < NUM_FEATURES; i++)
				{
				 bYd.addDouble(botYd.get(i).asDouble());
				}
				printf("xd=%s\n",bYd.toString().c_str());

				botYd.addDouble(t);

				outputFeaturesTarget.write(botYd);
				botYd.clear();
		}

	  return true;



	}

	bool sendImageTargetLeft(int cam){


      //img = portInputLeft.read();

	  // wait for a key
	  cvWaitKey(10);
	  cvNamedWindow( "HistogramTargetLeft", 1 );
	  cvNamedWindow( "CamShiftDemoTargetLeft", 1 );
	  cvSetMouseCallback( "CamShiftDemoTargetLeft", on_mouse_target_left);
	  cvCreateTrackbar( "Vmin", "CamShiftDemoTargetLeft", &vmin_target_left, 256, 0 );
	  cvCreateTrackbar( "Vmax", "CamShiftDemoTargetLeft", &vmax_target_left, 256, 0 );
	  cvCreateTrackbar( "Smin", "CamShiftDemoTargetLeft", &smin_target_left, 256, 0 );


	  // image grabbed?
			if (imgLeft == NULL) 
			{

				printf("no image found\n");
				return false;
			}

		int i, bin_w, c;


		cvCvtColor((IplImage*)imgLeft->getIplImage(), cvImageCam, CV_RGB2BGR);

		cvCopy( cvImageCam, image_target_left, 0 );

		cvCvtColor( image_target_left, hsv_target_left, CV_BGR2HSV );

		if( track_object_target_left )
		{



     			int _vmin_target_left = vmin_target_left;
				int _vmax_target_left = vmax_target_left;

				cvInRangeS( hsv_target_left, cvScalar(0,smin_target_left,MIN(_vmin_target_left,_vmax_target_left),0),
					cvScalar(180,256,MAX(_vmin_target_left,_vmax_target_left),0), mask_target_left );
				cvSplit( hsv_target_left, hue_target_left, 0, 0, 0 );

				
				if( track_object_target_left < 0 )
				{
					float max_val_target_left = 0.f;
					cvSetImageROI( hue_target_left, selection_target_left );
					cvSetImageROI( mask_target_left, selection_target_left );
					cvCalcHist( &hue_target_left, hist_target_left, 0, mask_target_left );
					cvGetMinMaxHistValue( hist_target_left, 0, &max_val_target_left, 0, 0 );
					cvConvertScale( hist_target_left->bins, hist_target_left->bins, max_val_target_left ? 255. / max_val_target_left : 0., 0 );
					cvResetImageROI( hue_target_left );
					cvResetImageROI( mask_target_left );
					track_window_target_left = selection_target_left;
					track_object_target_left = 1;

					cvZero( histimg_target_left );
					bin_w = histimg_target_left->width / hdims_target_left;
					for( i = 0; i < hdims_target_left; i++ )
					{
						int val_target_left = cvRound( cvGetReal1D(hist_target_left->bins,i)*histimg_target_left->height/255 );
						CvScalar color_target_left = hsv2rgb(i*180.f/hdims_target_left);
						cvRectangle( histimg_target_left, cvPoint(i*bin_w,histimg_target_left->height),
							cvPoint((i+1)*bin_w,histimg_target_left->height - val_target_left),
							color_target_left, -1, 8, 0 );
					}


				}

				cvCalcBackProject( &hue_target_left, backproject_target_left, hist_target_left );
				cvAnd( backproject_target_left, mask_target_left, backproject_target_left, 0 );
				cvCamShift( backproject_target_left, track_window_target_left,
					cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ),
					&track_comp_target_left, &track_box_target_left );
				track_window_target_left = track_comp_target_left.rect;

				if( backproject_mode_target_left )
					cvCvtColor( backproject_target_left, image_target_left, CV_GRAY2BGR );
				if( image_target_left->origin )
					track_box_target_left.angle = -track_box_target_left.angle;
				cvEllipseBox( image_target_left, track_box_target_left, CV_RGB(0,255,0), 3, CV_AA, 0 );
				//printf("rightangle=%f rightcenter=%f",track_box.angle,track_box.center.x);



		}

		if( select_object_target_left && selection_target_left.width > 0 && selection_target_left.height > 0 )
			{
				cvSetImageROI( image_target_left, selection_target_left );
				cvXorS( image_target_left, cvScalarAll(255), image_target_left, 0 );
				cvResetImageROI( image_target_left );
			}


		cvShowImage( "CamShiftDemoTargetLeft", image_target_left );
		cvShowImage( "HistogramTargetLeft", histimg_target_left );    


		// Put blob values in the matlab workspace for the target

		double posx = 0, posy = 0;
		double area = 0;
		double width = 0, height = 0;
		// for each blob

		/*if(track_box_target_left.center.x!=0)
		{
		    botYd.addDouble(track_box_target_left.center.x);
		}else{
			botYd.addDouble(0.0);
		}


		if(track_box_target_left.center.y!=0)
		{
		    botYd.addDouble(track_box_target_left.center.y);
		}else{
			botYd.addDouble(0.0);
		}*/

		if(track_box.angle!=0)
		{
		    bot.addDouble(track_box.center.x);
		}else{
			bot.addDouble(0.0);
		}


		if(track_box.angle!=0)
		{
		    bot.addDouble(track_box.center.y);
		}else{
			bot.addDouble(0.0);
		}



			return true;
	}

	int visualServoing(void){

		    PrintMatrix(yd);


			meanValuefk = mean_matrix(&fk.matrix);

            meanValuefk = abs(meanValuefk);

			if (meanValuefk > 5.00)    lambda = 0.4;
            else lambda = 1.0;    

			/* Compute temp1Pk = htheta'*Pk*/

			gsl_matrix_transpose_memcpy(&hthetaT.matrix,&htheta.matrix);

		     
			PrintMatrix(hthetaT);

			PrintMatrix(Pk);

			gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
                               1.0, &hthetaT.matrix, &Pk.matrix,
                                0.0, &temp1PkNum.matrix);

		   PrintMatrix(temp1PkNum);

		   	/* Compute temp2PkNum = htheta*htheta'*Pk*/

		   	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
                               1.0, &htheta.matrix, &temp1PkNum.matrix,
                                0.0, &temp2PkNum.matrix);

		   PrintMatrix(temp2PkNum);

		   /* Compute temp3PkNum = Pk*htheta*htheta'*Pk*/
		    gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
                               1.0, &Pk.matrix, &temp2PkNum.matrix,
                                0.0, &temp3PkNum.matrix);

		   PrintMatrix(temp3PkNum);

           

		    /* Compute temp1PkDen = Pk*htheta*/
		    gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
                               1.0, &Pk.matrix, &htheta.matrix,
                                0.0, &temp1PkDen.matrix);

		   PrintMatrix(temp1PkDen);

		   /*compute temp2PkDen = htheta'*Pk*htheta*/
		   gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
                               1.0, &hthetaT.matrix, &temp1PkDen.matrix,
                                0.0, &temp2PkDen.matrix);

		   PrintMatrix(temp2PkDen);

		   /*temp3PkDen = lambda+htheta'*Pk*htheta*/

		   temp3PkDen = lambda + gsl_matrix_get(&temp2PkDen.matrix, 0, 0);

		   /*temp3PkDen = 1/(lambda+htheta'*Pk*htheta)*/

           temp3PkDen = 1/temp3PkDen; 

		   /*calculating (Pk*htheta*htheta'*Pk)/(lambda + htheta'*Pk*htheta) => temp3PkNum/temp3PkDen*/

		   gsl_matrix_scale(&temp3PkNum.matrix,temp3PkDen);

		   PrintMatrix(temp3PkNum);

		   /*calculating Pk - (Pk*htheta*htheta'*Pk)/(lambda + htheta'*Pk*htheta) => Pk = Pk - temp3PkNum*/

		   gsl_matrix_sub(&Pk.matrix,&temp3PkNum.matrix);
	
		   PrintMatrix(Pk);

		   /*calculating Pk = (1/lambda)*(Pk - (Pk*htheta*htheta'*Pk)/(lambda + htheta'*Pk*htheta))*/

		   tempInvlambda = 1/lambda;

		   gsl_matrix_scale(&Pk.matrix,tempInvlambda);

		   PrintMatrix(Pk);


			/* Compute temp1 = J*htheta */

		   PrintMatrix(J);

           gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
                               1.0, &J.matrix, &htheta.matrix,
                                0.0, &temp1.matrix);

		   PrintMatrix(temp1);

		   /*temp2 = deltaf - J*htheta(temp1)*/

		   gsl_matrix_memcpy(&temp2.matrix,&deltaf.matrix);

    	   PrintMatrix(temp2);

		   gsl_matrix_sub(&temp2.matrix,&temp1.matrix);
	
		   PrintMatrix(temp2);

		   /*temp3 = htheta'*Pk*/

		   gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
                               1.0, &hthetaT.matrix, &Pk.matrix,
                                0.0, &temp3.matrix);

		   PrintMatrix(temp3);

	   
		   /*calculating (deltaf - J*htheta)*htheta'*Pk => temp4 = temp2*temp3*/

		   gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
                               1.0, &temp2.matrix, &temp3.matrix,
                                0.0, &temp4.matrix);

		   PrintMatrix(temp4);

		   /*calculating Pk*htheta => temp5 = Pk*htheta*/

		   gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
                               1.0, &Pk.matrix, &htheta.matrix,
                                0.0, &temp5.matrix);

		   PrintMatrix(temp5);

		   /*calculating htheta'*Pk*htheta => temp6 = htheta'*temp5*/

		   gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
                               1.0, &hthetaT.matrix, &temp5.matrix,
                                0.0, &temp6.matrix);

		   PrintMatrix(temp6);

		   /*calculating lambda+htheta'*Pk*htheta => temp7 = lambda + temp6*/

           temp7 = lambda + gsl_matrix_get(&temp6.matrix, 0, 0);

		   /*calculating 1/(lambda+htheta'*Pk*htheta) => temp7 = 1/(lambda+htheta'*Pk*htheta)*/

		   temp7 = 1.00/temp7;

		   /*calculating ((deltaf - J*htheta)*htheta'*Pk)/(lambda+htheta'*Pk*htheta) => temp4 = temp4*inv(temp7)*/
      
		   gsl_matrix_scale(&temp4.matrix,temp7);

		   PrintMatrix(temp4);

		   /* calculating J = J + ((deltaf - J*htheta)*htheta'*Pk)/(lambda+htheta'*Pk*htheta)  => J = J + temp4*/

		   PrintMatrix(J);

		   gsl_matrix_add(&J.matrix, &temp4.matrix);

		   PrintMatrix(J);

		  // matrix_pseudo_inverse(gsl_matrix *dest, const gsl_matrix *src);
		   matrix_pseudo_inverse(&pinvJ.matrix,&J.matrix);

		   PrintMatrix(pinvJ);

		   // deltaq = pinv(J)*f

		   gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
                               1.0, &pinvJ.matrix, &f.matrix,
                                0.0, &deltaq.matrix);

		   PrintMatrix(deltaq);	

		   //Clamping the deltaq values

		   matrix_clamp(&deltaq.matrix, &deltaq.matrix);

		   PrintMatrix(deltaq);

		  //calculating q =  q - deltaq;

		   gsl_matrix_sub(&q.matrix,&deltaq.matrix);
	
		   PrintMatrix(q);

		
//		}



		


	
	     return 0;
	}



		int visualServoingP(void){

		    PrintMatrix(yd);

			PrintMatrix(y);
            
			double normValuefk;

			//meanValuefk = mean_matrix(&fk.matrix);

            //meanValuefk = abs(meanValuefk);

			normValuefk = norm_matrix(&f.matrix);

			//printf("meanValuefk = %f\n",meanValuefk);
			printf("meanValuefk = %f\n",normValuefk);

			/*if (meanValuefk > 5.00)    lambda = 0.3;
            else lambda = 0.98;    */
			if (normValuefk > 15.00)    lambda = 0.3;
            else lambda = 0.98;    

			if (normValuefk > 15){

			/* Compute temp1Pk = htheta'*Pk*/

			gsl_matrix_transpose_memcpy(&hthetaT.matrix,&htheta.matrix);

		     
			//PrintMatrix(hthetaT);

			//PrintMatrix(Pk);

			gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
                               1.0, &hthetaT.matrix, &Pk.matrix,
                                0.0, &temp1PkNum.matrix);

		   //PrintMatrix(temp1PkNum);

		   	/* Compute temp2PkNum = htheta*htheta'*Pk*/

		   	gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
                               1.0, &htheta.matrix, &temp1PkNum.matrix,
                                0.0, &temp2PkNum.matrix);

		   //PrintMatrix(temp2PkNum);

		   /* Compute temp3PkNum = Pk*htheta*htheta'*Pk*/
		    gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
                               1.0, &Pk.matrix, &temp2PkNum.matrix,
                                0.0, &temp3PkNum.matrix);

		   //PrintMatrix(temp3PkNum);

           

		    /* Compute temp1PkDen = Pk*htheta*/
		    gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
                               1.0, &Pk.matrix, &htheta.matrix,
                                0.0, &temp1PkDen.matrix);

		   //PrintMatrix(temp1PkDen);

		   /*compute temp2PkDen = htheta'*Pk*htheta*/
		   gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
                               1.0, &hthetaT.matrix, &temp1PkDen.matrix,
                                0.0, &temp2PkDen.matrix);

		   //PrintMatrix(temp2PkDen);

		   /*temp3PkDen = lambda+htheta'*Pk*htheta*/

		   temp3PkDen = lambda + gsl_matrix_get(&temp2PkDen.matrix, 0, 0);

		   /*temp3PkDen = 1/(lambda+htheta'*Pk*htheta)*/

           temp3PkDen = 1/temp3PkDen; 

		   /*calculating (Pk*htheta*htheta'*Pk)/(lambda + htheta'*Pk*htheta) => temp3PkNum/temp3PkDen*/

		   gsl_matrix_scale(&temp3PkNum.matrix,temp3PkDen);

		   //PrintMatrix(temp3PkNum);

		   /*calculating Pk - (Pk*htheta*htheta'*Pk)/(lambda + htheta'*Pk*htheta) => Pk = Pk - temp3PkNum*/

		   gsl_matrix_sub(&Pk.matrix,&temp3PkNum.matrix);
	
		   //PrintMatrix(Pk);

		   /*calculating Pk = (1/lambda)*(Pk - (Pk*htheta*htheta'*Pk)/(lambda + htheta'*Pk*htheta))*/

		   tempInvlambda = 1/lambda;

		   gsl_matrix_scale(&Pk.matrix,tempInvlambda);

		   //PrintMatrix(Pk);


			/* Compute temp1 = J*htheta */

           gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
                               1.0, &J.matrix, &htheta.matrix,
                                0.0, &temp1.matrix);

		   //PrintMatrix(temp1);

		   /*temp2 = deltaf - J*htheta(temp1)*/

		   gsl_matrix_memcpy(&temp2.matrix,&deltaf.matrix);

    	   //PrintMatrix(temp2);

		   gsl_matrix_sub(&temp2.matrix,&temp1.matrix);
	
		   //PrintMatrix(temp2);

		   /*temp3 = htheta'*Pk*/

		   gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
                               1.0, &hthetaT.matrix, &Pk.matrix,
                                0.0, &temp3.matrix);

		   //PrintMatrix(temp3);

	   
		   /*calculating (deltaf - J*htheta)*htheta'*Pk => temp4 = temp2*temp3*/

		   gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
                               1.0, &temp2.matrix, &temp3.matrix,
                                0.0, &temp4.matrix);

		   //PrintMatrix(temp4);

		   /*calculating Pk*htheta => temp5 = Pk*htheta*/

		   gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
                               1.0, &Pk.matrix, &htheta.matrix,
                                0.0, &temp5.matrix);

		   //PrintMatrix(temp5);

		   /*calculating htheta'*Pk*htheta => temp6 = htheta'*temp5*/

		   gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
                               1.0, &hthetaT.matrix, &temp5.matrix,
                                0.0, &temp6.matrix);

		   //PrintMatrix(temp6);

		   /*calculating lambda+htheta'*Pk*htheta => temp7 = lambda + temp6*/

           temp7 = lambda + gsl_matrix_get(&temp6.matrix, 0, 0);

		   /*calculating 1/(lambda+htheta'*Pk*htheta) => temp7 = 1/(lambda+htheta'*Pk*htheta)*/

		   temp7 = 1.00/temp7;

		   /*calculating ((deltaf - J*htheta)*htheta'*Pk)/(lambda+htheta'*Pk*htheta) => temp4 = temp4*inv(temp7)*/
      
		   gsl_matrix_scale(&temp4.matrix,temp7);

		   //PrintMatrix(temp4);

		   /* calculating J = J + ((deltaf - J*htheta)*htheta'*Pk)/(lambda+htheta'*Pk*htheta)  => J = J + temp4*/

		   //PrintMatrix(J);

		   gsl_matrix_add(&J.matrix, &temp4.matrix);

		   //PrintMatrix(J);



		  }

		  // matrix_pseudo_inverse(gsl_matrix *dest, const gsl_matrix *src);
		   matrix_pseudo_inverse(&pinvJ.matrix,&J.matrix);

		   //PrintMatrix(pinvJ);

		   // deltaq = pinv(J)*f

		   gsl_blas_dgemm (CblasNoTrans, CblasNoTrans,
                               1.0, &pinvJ.matrix, &f.matrix,
                                0.0, &deltaq.matrix);

		   //PrintMatrix(deltaq);	

		   //Clamping the deltaq values

		   matrix_clamp(&deltaq.matrix, &deltaq.matrix);

		   //PrintMatrix(deltaq);

		  //calculating q =  q - deltaq;

		   gsl_matrix_sub(&q.matrix,&deltaq.matrix);
	
		   //PrintMatrix(q);

		
//		}



		


	
	     return 0;
	}


	void matrix_clamp(gsl_matrix *dest, const gsl_matrix *src){

	
		//matrix max and min to clamp the joint variations

		double qmaxValClamp = 1.5;

		double qminValClamp = -1.5;

		double srcElement = 0.00;

		for(int i = 0; i < src->size1; i++){

		    srcElement = gsl_matrix_get(src, i, 0);

			if(srcElement > qmaxValClamp  ){

				srcElement = qmaxValClamp;

				gsl_matrix_set(dest, i, 0, srcElement);
			
			}

		
		}


		for(int i = 0; i < src->size1; i++){

		    srcElement = gsl_matrix_get(src, i, 0);

			if(srcElement < qminValClamp  ){

				srcElement = qminValClamp;

				gsl_matrix_set(dest, i, 0, srcElement);
			
			}		
		}




	}



	void matrix_pseudo_inverse(gsl_matrix *dest, const gsl_matrix *src)
    {
			 if (src->size1 < src->size2)
					pseudo_inverse_right(dest, src);
			 else if (src->size1 > src->size2)
					pseudo_inverse_left(dest, src);
			 else
					matrix_inverse(dest, src);
    }


	void pseudo_inverse_right(gsl_matrix *dest, const gsl_matrix *src)
   {
              gsl_matrix *trans, *prod, *inv;
 
              trans = gsl_matrix_alloc(src->size2, src->size1);
              prod = gsl_matrix_alloc(src->size1, src->size1);
              inv = gsl_matrix_alloc(src->size1, src->size1);
 
              gsl_matrix_transpose_memcpy(trans, src);
              gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, src, trans, 0.0, prod);
              matrix_inverse(inv, prod);
              gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, trans, inv, 0.0, dest);
 
              gsl_matrix_free(trans);
              gsl_matrix_free(prod);
              gsl_matrix_free(inv);
   }

	 void pseudo_inverse_left(gsl_matrix *dest, const gsl_matrix *src)
    {
              gsl_matrix *trans, *prod, *inv;
 
              trans = gsl_matrix_alloc(src->size2, src->size1);
              prod = gsl_matrix_alloc(src->size2, src->size2);
              inv = gsl_matrix_alloc(src->size2, src->size2);
 
              gsl_matrix_transpose_memcpy(trans, src);
              gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, trans, src, 0.0, prod);
              matrix_inverse(inv, prod);
              gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, inv, trans, 0.0, dest);
 
              gsl_matrix_free(trans);
              gsl_matrix_free(prod);
              gsl_matrix_free(inv);
    }
 
    void matrix_inverse(gsl_matrix *dest, const gsl_matrix *src)
   {
              gsl_matrix *tmp;
              int signum;
              gsl_permutation *p = gsl_permutation_alloc(src->size1);
              tmp = gsl_matrix_alloc(src->size1, src->size2);
              gsl_matrix_memcpy(tmp, src);
 
              gsl_linalg_LU_decomp(tmp, p, &signum);
              gsl_linalg_LU_invert(tmp, p, dest);
 
              gsl_permutation_free(p);
              gsl_matrix_free(tmp);
    }

	double mean_matrix(const gsl_matrix *src)
	{
	
      double meanValue = 0.00;

	  for (int i = 0; i < src->size1; i++)
	  {
	  
	       meanValue += gsl_matrix_get(src, 0, i);
	  }

	  meanValue = meanValue/src->size1;
	
	  return meanValue;
	}

	double norm_matrix(gsl_matrix *src){

		 double normValue = 0.00;

		  for (int i = 0; i < src->size1; i++)
		  {
		  
			   normValue += gsl_matrix_get(src, 0, i)*gsl_matrix_get(src, 0, i);
		  }

		  normValue = sqrt(normValue);

		  return normValue;

	}



	virtual void threadRelease()
	{

		portReadFeatures.close();

		delete [] encoders;

        cvReleaseImage(&cvImageCam);

		infileuL.close();

		infilevL.close();

		infileuR.close();

		infilevR.close();

		


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






int main(int argc, char *argv[])
{
	Network yarp;

	if (!yarp.checkNetwork())
		return false;

	GatewayModule mod(yarp);

	return mod.runModule(argc,argv);
}
