/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Marco Randazzo
 * email:  marco.randazzo@iit.it
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
\defgroup laserScannerGui laserScannerGui
 
@ingroup icub_tools
@ingroup icub_guis
 
A simple GUI to display the distance mesaurements of iKart's laser scanner.
 
Copyright (C) 2010 RobotCub Consortium
 
Author: Marco Randazzo

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
A simple GUI to display the distance mesaurements of iKart's laser scanner.
 
\section lib_sec Libraries 
- YARP libraries. 
- OpenCV libraries.

\section parameters_sec Parameters
None.

\section portsa_sec Ports Accessed
None. 
 
\section portsc_sec Ports Created 
The module creates the port /laserScannerGui:i used to receive the laser data.

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files 
None. 
 
\section conf_file_sec Configuration Files
None.
 
\section tested_os_sec Tested OS
Windows, Linux

\author Marco Randazzo
*/ 

#include<iostream>
#include<iomanip>
#include<sstream>
#include<fstream>
#include<string>
#include<stdio.h>

#include<math.h>

#include<cv.h>
#include<highgui.h>

#include <yarp/dev/Drivers.h>
#include <yarp/os/Network.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;

double scale =0.1; //global scale factor 
int robot_radius = 715/2; //mm
int laser_position = 245; //mm

void drawGrid(IplImage *img)
{
const CvScalar color_white = cvScalar(255,255,255);
const CvScalar color_black = cvScalar(0,0,0);

	cvLine(img,cvPoint(0,0),cvPoint(img->width,img->height),color_black);
	cvLine(img,cvPoint(img->width,0),cvPoint(0,img->height),color_black);
	cvLine(img,cvPoint(img->width/2,0),cvPoint(img->width/2,img->height),color_black);
	cvLine(img,cvPoint(0,img->height/2),cvPoint(img->width,img->height/2),color_black);
	const int step = (int)(500.0 * scale); //mm
/*
	for (int xi=0; xi<img->width; xi+=step)
		cvLine(img,cvPoint(xi,0),cvPoint(xi,img->height),color_black);
	for (int yi=0; yi<img->height; yi+=step)
		cvLine(img,cvPoint(0,yi),cvPoint(img->width,yi),color_black);
*/
	cvCircle(img,cvPoint(img->width/2,img->height/2),step*1,color_black);
	cvCircle(img,cvPoint(img->width/2,img->height/2),step*2,color_black);
	cvCircle(img,cvPoint(img->width/2,img->height/2),step*3,color_black);
	cvCircle(img,cvPoint(img->width/2,img->height/2),step*4,color_black);
	cvCircle(img,cvPoint(img->width/2,img->height/2),step*5,color_black);
}

void drawRobot (IplImage *img)
{

	const CvScalar color_black = cvScalar(0,0,0);
	const CvScalar color_gray  = cvScalar(100,100,100);

	//draw a circle
	cvCircle(img,cvPoint(img->width/2,img->height/2),(int)(robot_radius*scale),color_gray,CV_FILLED);
}

void drawLaser(const Vector *v, IplImage *img)
{
	if (!v) return;
const CvScalar color_white = cvScalar(255,255,255);
const CvScalar color_black = cvScalar(0,0,0);
    cvZero(img);
	cvRectangle(img,cvPoint(0,0),cvPoint(img->width,img->height),cvScalar(255,0,0),-1);
	CvPoint center;
	center.x = img->width/2;
	center.y = img->height/2-(int)(laser_position*scale);

	double angle =0;
	double lenght=0;
	static double old_time=0;

	double curr_time=yarp::os::Time::now();
	//fprintf(stderr,"received vector size:%d",v->size());
	fprintf(stderr,"time:%f\n",curr_time-old_time);
	old_time = curr_time;

	for (int i=0; i<1080; i++)
	{
		lenght=(*v)[i];
		angle=i/1080.0*270.0-(90-(360-270)/2);
//			angle=i;
//			lenght=i;
		double x = lenght*scale*cos(angle/180*3.14);
		double y =-lenght*scale*sin(angle/180*3.14);
		CvPoint ray;
		ray.x=int(x);
		ray.y=int(y);
		ray.x += center.x;
		ray.y += center.y;

		int thickness = 2;
		//draw a line
		cvLine(img,center,ray,color_white,thickness);
	}
}


int main(int argc, char *argv[])
{
    YARP_REGISTER_DEVICES(icubmod)

	Network yarp;

    string name;
    if(argc > 1) name = argv[1];
    else name = "/laserScannerGui:i";

    int width = 600;
    int height = 600;

	BufferedPort<yarp::sig::Vector> inPort;
    inPort.open(name.c_str());

    IplImage *img  = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,3);
	IplImage *img2 = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,3);
    cvNamedWindow("Laser Scanner GUI",CV_WINDOW_AUTOSIZE);

    bool exit = false;
    while(!exit)
    {
        yarp::sig::Vector *v = inPort.read(true);
        {
            //your drawing func.
            drawLaser(v,img);
			drawRobot(img2);
			drawGrid(img);
            cvAddWeighted(img, 0.7, img2, 0.3, 0.0, img);
            cvShowImage("Laser Scanner GUI",img);
        }
        //if ESC is pressed, exit.
        if(cvWaitKey(1) == 27) exit = true;
    }

    inPort.close();
    cvDestroyAllWindows();
    cvReleaseImage(&img);
}

