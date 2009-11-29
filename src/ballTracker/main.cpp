/* 
 * Copyright (C) 2006 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Lorenzo Natale and Francesco Nori
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

///
/// $Id: main.cpp,v 1.31 2007/07/24 18:40:06 babybot Exp $
/// 
#include <cv.h>
#include <cxtypes.h>


#include <yarp/sig/Image.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/ImageFile.h>
#include <yarp/sig/ImageDraw.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>

#include <iostream>
#include <math.h>

//Circular Hough Transform
#include "cht.h"  

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;

const int SCALE_FACTOR = 2;

const int LOW_PASS_THRESHOLD=70;	  //threshold on the output of the low pass filter
const int MAX_RADIUS=95/SCALE_FACTOR;          //max radius for the ROI
int SAT_MIN = 10;
int VAL_MIN = 10;
 
//Hough parameters
const int CANNY_THRESHOLD1=3000;
const int CANNY_THRESHOLD2=1200;
const int HOUGH_MIN_RADIUS=10/SCALE_FACTOR;
const int HOUGH_MAX_RADIUS=65/SCALE_FACTOR;
const int HOUGH_RSTEPS=2/SCALE_FACTOR;
const int HOUGH_MIN1_VAL=30/SCALE_FACTOR;
const int HOUGH_MIN2_VAL=25/SCALE_FACTOR;

//Gaussian Filter for linear edge detection
const int GAUSSIAN_FILTER_SIZE = 5;
const unsigned char GAUSSIAN_FILTER[GAUSSIAN_FILTER_SIZE] = {154, 187, 127, 67, 100};
const int FILTER_THRESHOLD = 1000;
const int NUMBER_OF_EDGES = 500/SCALE_FACTOR;

//Filter on the position
const double TEMPORAL_POLE_TARGET = 0.05;

//Port names
const char *DEFAULT_NAME = "/histogram";

//Second ball tracker. If false then search for a second ball
bool SLAVE = false;

//Gradient Search
const int Hsize = 31;
#define PI 3.1415926535897932384626433832795
#define min(a,b) (((a)<(b))?(a):(b))
#define max(a,b) (((a)>(b))?(a):(b))

/*
 * Low passes the image radius estimation. The
 * underlying assumption is that the radius does
 * not change very fast so that we can filter out 
 * some noise.
 */  

class firstOrderLowPassFilter
{
private:
	double *state;
	double pole;
public:

    firstOrderLowPassFilter(double p): state(0)
    {
        pole=p;
		state=new double [2];
		state[0] = 0;
		state[1] = 0;
    }

    ~firstOrderLowPassFilter()
	{
		if (state!=0)
			delete [] state;
		state=0;
	}

	double temporalFilter(double input)
	{
		state[0] = state[1];
		state[1] = pole*state[0] + (1-pole)*input;
		return state[1];
	}

	void setState(double s0, double s1)
	{
		state[0] = s0;
		state[1] = s1;
	}

};


/* downsample src to fit dst */
void downsample(const ImageOf<PixelRgb> &src, ImageOf<PixelRgb> &dst)
{
    cvResize((const IplImage *)(src.getIplImage()), 
             (IplImage *) dst.getIplImage(), CV_INTER_NN);
}

/*
 * Sums the first image to the second image
 */

void copyToOutput(ImageOf<PixelMono> &inMono, ImageOf<PixelMono> &outMono)
{
	const int H=outMono.height();
    const int W=outMono.width();

    for(int r=0;r<H;r++)
    {
        unsigned char *img1=inMono.getRow(r);
        unsigned char *img2=outMono.getRow(r);

        for(int c=0;c<W;c++)
        {
            unsigned char tmp=*img1 + *img2;
			if (*img2!=255)
				*img2 = *img1;

			img1++;
			img2++;
        }
    }
}

/*
 * Compute the BY opponence
 */

void computeBY(ImageOf<PixelRgb> &inImg, ImageOf<PixelMono> &outMono)
{
	const int H=outMono.height();
    const int W=outMono.width();

    for(int r=0;r<H;r++)
    {
        unsigned char *img1=inImg.getRow(r);
        unsigned char *img2=outMono.getRow(r);

        for(int c=0;c<W;c++)
        {
			//*img2 = unsigned char (1/2.0*((double) img1[2] - (img1[1] + img1[0])/2)+127.5);
			*img2 = img1[2] - (img1[1] + img1[0])/2;

			img1+=3;
			img2++;
        }
    }
}

void computeBYNew(ImageOf<PixelRgb> &inImg, ImageOf<PixelMono> &outMono)
{
	const int H=outMono.height();
    const int W=outMono.width();

    for(int r=0;r<H;r++)
    {
        unsigned char *img1=inImg.getRow(r);
        unsigned char *img2=outMono.getRow(r);

        for(int c=0;c<W;c++)
        {
			int a = (2*img1[2] - img1[1] - img1[0] +510)/4;
			if (a>255)
				a = 255;
			if (a<0)
				a=0;

			*img2 = a;

			img1+=3;
			img2++;
        }
    }
}

/*
 * Low passes the image in order to filter out
 * small regions. A threshold is applied to
 * eliminate these small regions.
 */

void lowPass(ImageOf<PixelMono> &in, ImageOf<PixelMono> &out)
{
    IplImage *inIpl=(IplImage *)in.getIplImage();
    IplImage *outIpl=(IplImage *)out.getIplImage();

    cvSmooth(inIpl, 
             outIpl,
             CV_GAUSSIAN,3,3); //2x2 gaussian kernel

    /*cvThreshold(outIpl,
                outIpl,
                LOW_PASS_THRESHOLD,
                255,
                CV_THRESH_BINARY);*/
}

void lowPass(ImageOf<PixelRgb> &in, ImageOf<PixelRgb> &out)
{
    IplImage *inIpl=(IplImage *)in.getIplImage();
    IplImage *outIpl=(IplImage *)out.getIplImage();

    cvSmooth(inIpl, 
             outIpl,
             CV_GAUSSIAN,3,3); //2x2 gaussian kernel

    /*cvThreshold(outIpl,
                outIpl,
                LOW_PASS_THRESHOLD,
                255,
                CV_THRESH_BINARY);*/
}


/* Builds the histogram with the following two steps:
 * (-) uses the circular hough until a circular object is found
 * (-) updates the histogram using the pixels inside the circular object
 */

bool locateBall(ImageOf<PixelRgb> &inImg, BufferedPort<ImageOf<PixelMono> > &outPort, int &x, int &y, double &R)
{
	HOUGH_MAP_TYPE maxVal = 0; 

	ImageOf<PixelMono> inMono, inMonoLP;
	ImageOf<PixelMono> inEdge;

	CircularHT hough;

	const int width=inImg.width();
	const int height=inImg.height();

	hough.setParameters(HOUGH_MIN_RADIUS, HOUGH_MAX_RADIUS, HOUGH_RSTEPS, height, width);

	inMono.resize(width, height);
	inMonoLP.resize(width, height);
	inEdge.resize(width, height);


    cvCvtColor((IplImage *) inImg.getIplImage(),
		(IplImage *) inMono.getIplImage(), CV_RGB2GRAY);

	//computeBY(inImg, inMono);
	lowPass(inMono, inMonoLP);

	cvCanny((IplImage *) inMonoLP.getIplImage(), 
		(IplImage *) inEdge.getIplImage(), CANNY_THRESHOLD1, CANNY_THRESHOLD2, 5 );

	hough.computeTransform(inEdge);
	R=hough.getMax(x, y, maxVal);

	ImageOf<PixelMono> &outEdge=outPort.prepare();
	outEdge.resize(width, height);
	outEdge=inEdge;
	draw::addCrossHair(outEdge, PixelMono(255), (int)x, (int)y, (int)R);
	copyToOutput(inMonoLP, outEdge);
	outPort.write();


	//fprintf(stderr, "%d %f \n", maxVal, R);
	if (maxVal > HOUGH_MIN1_VAL && R > 10)
		return false;
	else
		return true;
}

bool locateBall(ImageOf<PixelRgb> &inImg, BufferedPort<ImageOf<PixelMono> > &outPort, BufferedPort<Vector > &inPosSlave, int &x, int &y, double &R)
{
	HOUGH_MAP_TYPE maxVal = 0; 

	ImageOf<PixelMono> inMono, inMonoLP;
	ImageOf<PixelMono> inEdge;

	CircularHT hough;

	const int width=inImg.width();
	const int height=inImg.height();

	//fprintf(stderr, "Image dim: (%d, %d) \n", width, height);
	hough.setParameters(HOUGH_MIN_RADIUS, HOUGH_MAX_RADIUS, HOUGH_RSTEPS, height, width);

	inMono.resize(width, height);
	inMonoLP.resize(width, height);
	inEdge.resize(width, height);


    cvCvtColor((IplImage *) inImg.getIplImage(),
		(IplImage *) inMono.getIplImage(), CV_RGB2GRAY);

	//computeBY(inImg, inMono);
	lowPass(inMono, inMonoLP);

	cvCanny((IplImage *) inMonoLP.getIplImage(), 
		(IplImage *) inEdge.getIplImage(), CANNY_THRESHOLD1, CANNY_THRESHOLD2, 5 );

	Vector *masterPos=inPosSlave.read(true);
	int masterX = (*masterPos)[0]/SCALE_FACTOR + width/2;
	int masterY = (*masterPos)[1]/SCALE_FACTOR + height/2;
	int masterR = (*masterPos)[2]/SCALE_FACTOR;

	//remove Master Ball
	//fprintf(stderr, "Trying to remove circle @ (%d, %d) with radius %d\n", masterX, masterY, masterR);
	for (int k = -masterR/4; k < masterR/4 ; k ++)
		draw::addCircleOutline(inEdge, PixelMono(0), masterX, masterY, (masterR+k));

	hough.computeTransform(inEdge);
	R=hough.getMax(x, y, maxVal);

	ImageOf<PixelMono> &outEdge=outPort.prepare();
	outEdge.resize(width, height);
	outEdge=inEdge;
	draw::addCrossHair(outEdge, PixelMono(255), (int)x, (int)y, (int)R);
	copyToOutput(inMonoLP, outEdge);
	outPort.write();


	//fprintf(stderr, "%d %f \n", maxVal, R);
	if (maxVal > HOUGH_MIN1_VAL && R > 10)
		return false;
	else
		return true;
}


/*
 * Filters the given line with the given filter.
 * If an edge is found the output image is modified
 * so as so mark the given edge
 */
int filter1D(IplImage* image, unsigned char *buffer, CvLineIterator &iter, int count, ImageOf<PixelMono> &outImg)
{
	if (count <=2)
		return 0;

	int i = 0;
	CV_NEXT_LINE_POINT(iter);
	i++;
	CV_NEXT_LINE_POINT(iter);
	i++;


	int filtered_prev = 0;

	int j = 0;
	for( j = 0; j < GAUSSIAN_FILTER_SIZE; j++)
	{
		if ( (j+i) > count)
			filtered_prev = filtered_prev;
		else
			filtered_prev = filtered_prev + buffer[j+i] * (GAUSSIAN_FILTER[j]-127);
	}
	filtered_prev = abs(filtered_prev);


	for( ; i < count; i++ )
	{
		int filtered = 0;
		for( j = 0; j < GAUSSIAN_FILTER_SIZE; j++)
		{
			if ( (j+i) > count)
				filtered = filtered;
			else
				filtered = filtered + buffer[j+i] * (GAUSSIAN_FILTER[j]-127);
		}

		filtered = abs(filtered);
		//fprintf(stderr, "%d ", filtered);

		if ((filtered_prev > FILTER_THRESHOLD) && (filtered_prev > filtered))
		{
			int offset, x, y;
			offset = iter.ptr - (uchar*)(image->imageData);
			y = offset/image->widthStep;
			x = (offset - y*image->widthStep)/(sizeof(uchar));
			//fprintf(stderr, "Current point is: (%d, %d)", x, y);
			PixelMono &pictel = outImg.safePixel(x, y);
			pictel = PixelMono(255);
			return 0;
		}

		CV_NEXT_LINE_POINT(iter);
		filtered_prev = filtered;
	}
	return 0;
}


/* 
 * Tracks the ball given its current position.
 */

bool trackBall(ImageOf<PixelRgb> &inImg, BufferedPort<ImageOf<PixelMono> > &outPort, int &x, int &y, double &R, int colorSp)
{

	ImageOf<PixelRgb> inImgLP;
	ImageOf<PixelMono> inMono, inMonoLP;
	unsigned char buffer[1024];

	for(int i = 0; i<1024; i++)
		buffer[i] = 0;

	const int width=inImg.width();
	const int height=inImg.height();

	inImgLP.resize(width, height);
	inMono.resize(width, height);
	inMonoLP.resize(width, height);

    //cvCvtColor((IplImage *) inImg.getIplImage(),
	//	(IplImage *) inMono.getIplImage(), CV_RGB2GRAY);

	lowPass(inImg, inImgLP);
    if (colorSp==0)
        computeBYNew(inImgLP, inMonoLP);
    else
        cvCvtColor((IplImage *) inImgLP.getIplImage(),
                   (IplImage *) inMonoLP.getIplImage(), CV_RGB2GRAY);

	ImageOf<PixelMono> &outMono=outPort.prepare();
	outMono.resize(width, height);
	outMono.zero();
	

	for (int ii = 0; ii<NUMBER_OF_EDGES; ii++)
	{

		double theta = 2*PI*ii/NUMBER_OF_EDGES;

		CvPoint start; 
		CvPoint stop;

		start.x=x+(int) (0.3*R*sin(theta));
		start.y=y+(int) (0.3*R*cos(theta));

		if (start.x > width-1)
			start.x = width-1;
		if (start.x < 0)
			start.x = 0;
		if (start.y > height-1)
			start.y = height-1;
		if (start.y < 0)
			start.y = 0;

		stop.x=x+(int) (1.8*R*sin(theta));
		stop.y=y+(int) (1.8*R*cos(theta));

		if (stop.x > width-1)
			stop.x = width-1;
		if (stop.x < 0)
			stop.x = 0;
		if (stop.y > height-1)
			stop.y = height-1;
		if (stop.y < 0)
			stop.y = 0;

		//fprintf(stderr, "Start and stop points are: (%d, %d) (%d, %d)\n", start.x, start.y, stop.x, stop.y);

		CvLineIterator iter;
		cvSampleLine( (IplImage *) inMonoLP.getIplImage(), start, stop, buffer);
		int count = cvInitLineIterator( (IplImage *) inMonoLP.getIplImage(), start, stop, &iter);
		filter1D((IplImage *) inMonoLP.getIplImage(), buffer, iter, count, outMono);
	}

	CircularHT hough;

	if ((R-3)>HOUGH_MIN_RADIUS)
		hough.setParameters((int)(R-3), int(R+3), HOUGH_RSTEPS, height, width);
	else
		hough.setParameters(HOUGH_MIN_RADIUS, (int) (R+3), HOUGH_RSTEPS, height, width);

	hough.computeTransform(outMono);
	HOUGH_MAP_TYPE maxVal = 0; 
	R=hough.getMax(x, y, maxVal);

	draw::addCrossHair(outMono, PixelMono(255), (int)x, (int)y, (int) R);
	copyToOutput(inMonoLP, outMono);
	outPort.write();


	if (maxVal > HOUGH_MIN2_VAL)
		return false;
	else
		return true;
}


bool trackBall(ImageOf<PixelRgb> &inImg, BufferedPort<ImageOf<PixelMono> > &outPort, BufferedPort<Vector > &inPosSlave, int &x, int &y, double &R, int colorSp)
{

	ImageOf<PixelRgb> inImgLP;
	ImageOf<PixelMono> inMono, inMonoLP;
	unsigned char buffer[1024];

	for(int i = 0; i<1024; i++)
		buffer[i] = 0;

	const int width=inImg.width();
	const int height=inImg.height();

	inImgLP.resize(width, height);
	inMono.resize(width, height);
	inMonoLP.resize(width, height);

    //cvCvtColor((IplImage *) inImg.getIplImage(),
	//	(IplImage *) inMono.getIplImage(), CV_RGB2GRAY);

	lowPass(inImg, inImgLP);
    if (colorSp==0)
        computeBYNew(inImgLP, inMonoLP);
    else
        cvCvtColor((IplImage *) inImgLP.getIplImage(),
                   (IplImage *) inMonoLP.getIplImage(), CV_RGB2GRAY);

	ImageOf<PixelMono> &outMono=outPort.prepare();
	outMono.resize(width, height);
	outMono.zero();
	

	for (int ii = 0; ii<NUMBER_OF_EDGES; ii++)
	{

		double theta = 2*PI*ii/NUMBER_OF_EDGES;

		CvPoint start; 
		CvPoint stop;

		start.x=x+(int) (0.3*R*sin(theta));
		start.y=y+(int) (0.3*R*cos(theta));

		if (start.x > width-1)
			start.x = width-1;
		if (start.x < 0)
			start.x = 0;
		if (start.y > height-1)
			start.y = height-1;
		if (start.y < 0)
			start.y = 0;

		stop.x=x+(int) (1.8*R*sin(theta));
		stop.y=y+(int) (1.8*R*cos(theta));

		if (stop.x > width-1)
			stop.x = width-1;
		if (stop.x < 0)
			stop.x = 0;
		if (stop.y > height-1)
			stop.y = height-1;
		if (stop.y < 0)
			stop.y = 0;

		//fprintf(stderr, "Start and stop points are: (%d, %d) (%d, %d)\n", start.x, start.y, stop.x, stop.y);

		CvLineIterator iter;
		cvSampleLine( (IplImage *) inMonoLP.getIplImage(), start, stop, buffer);
		int count = cvInitLineIterator( (IplImage *) inMonoLP.getIplImage(), start, stop, &iter);
		filter1D((IplImage *) inMonoLP.getIplImage(), buffer, iter, count, outMono);
	}

	Vector *masterPos=inPosSlave.read(true);
	int masterX = (*masterPos)[0] + width/2;
	int masterY = (*masterPos)[1] + height/2;
	int masterR = (*masterPos)[2];

	//remove Master Ball
	//fprintf(stderr, "Trying to remove circle @ (%d, %d) with radius %d\n", masterX, masterY, masterR);
	for (int k = -masterR/4; k < masterR/4 ; k ++)
		draw::addCircleOutline(outMono, PixelMono(0), masterX, masterY, (masterR+k));


	CircularHT hough;

	if ((R-3)>HOUGH_MIN_RADIUS)
		hough.setParameters((int)(R-3), int(R+3), HOUGH_RSTEPS, height, width);
	else
		hough.setParameters(HOUGH_MIN_RADIUS, (int) (R+3), HOUGH_RSTEPS, height, width);

	hough.computeTransform(outMono);
	HOUGH_MAP_TYPE maxVal = 0; 
	R=hough.getMax(x, y, maxVal);

	draw::addCrossHair(outMono, PixelMono(255), (int)x, (int)y, (int) R);
	copyToOutput(inMonoLP, outMono);
	outPort.write();


	if (maxVal > HOUGH_MIN2_VAL)
		return false;
	else
		return true;
}

void namePorts(int argc, char *argv[], BufferedPort<ImageOf<PixelRgb> > & in, BufferedPort<ImageOf<PixelRgb> > & out, BufferedPort<Vector> & outPos, BufferedPort<ImageOf<PixelMono> > & outMono)
{
    Property p;
    p.fromCommand(argc,argv);

    if (argc<=1) {
        printf("default port names are:\n");
        printf("    /histogram/img/i      (for incoming images)\n");
        printf("    /histogram/img/o      (for outgoing images)\n");
		printf("    /histogram/pos/o      (for streaming position vector output)\n");
		printf("    /histogram/img/mono/o (for initial ball localization)\n");
        printf("can change \"/histogram\" prefix with --name option\n");
		printf("can detect a second ball with the option --slave\n\n");
    }

	ConstString name = p.check("name",Value(DEFAULT_NAME)).asString();
	std::string inImageName = name.c_str();
	inImageName += "/img/i";

	std::string outImageName = name.c_str();
	outImageName += "/img/o";

	std::string posOutName = name.c_str();
	posOutName += "/pos/o";

	std::string outImageMonoName = name.c_str();
	outImageMonoName += "/img/mono/o";

    in.open(inImageName.c_str());
    out.open(outImageName.c_str());
    outPos.open(posOutName.c_str());
    outMono.open(outImageMonoName.c_str());
}

void nameSlavePort(int argc, char *argv[], BufferedPort<Vector> & inSlave)
{
    Property p;
    p.fromCommand(argc,argv);

	ConstString name = p.check("name",Value(DEFAULT_NAME)).asString();
	std::string inPosSlave = name.c_str();
	inPosSlave += "/pos/slave/i";

    inSlave.open(inPosSlave.c_str());
}


void initSlave(int argc, char *argv[])
{

    Property p;
    p.fromCommand(argc,argv);

    if (p.check("slave"))
        SLAVE=true; //detect a second ball
    else
        SLAVE=false; //detect only one ball
}


int initColor(int argc, char *argv[])
{
    int ret;
    if (argc<1)
        {
            printf("You can change color space with --black or --blue (default is blue)");
            return 0;
        }

    Property p;
    p.fromCommand(argc,argv);

    if (p.check("black"))
        ret=1; //black
    else
        ret=0; //blue

    return ret;
}

void initParams(int argc, char *argv[])
{
    Property p;
    p.fromCommand(argc,argv);

	if 	(p.check("minval"))
		VAL_MIN = p.check("minval",Value(VAL_MIN)).asInt();

	if 	(p.check("minsat"))
		SAT_MIN = p.check("minsat",Value(SAT_MIN)).asInt();
}

void sendResults(double x, double y, double R, ImageOf<PixelRgb> inSmall, BufferedPort<ImageOf<PixelRgb> > & out, BufferedPort<Vector> & outPos)
{
  Vector &v=outPos.prepare();
  v.size(3);

  /*
  v[0] = x-inSmall.width()/2;
  v[1] = y-inSmall.height()/2;
  v[2] = R;
  */

  v[0] = (2*x/inSmall.width())-1;
  v[1] = (2*y/inSmall.height())-1;
  v[2] = 0;

  outPos.write();

  ImageOf<PixelRgb> &outImg=out.prepare();
  outImg.resize(inSmall.width(), inSmall.height());
  outImg=inSmall;

  addCircleOutline(outImg, PixelRgb(255, 0, 0), (int)x, (int)y, (int)R);	
  out.write();
}

void computeMap(float * inImg, int sizeX, int sizeY, int border, int * ptsArray)
{
	int x,y;
	int i;
	int sizeI = sizeX*sizeY;
	
	float * DX = new float [sizeI];
	float * DY = new float [sizeI];

	float * mod = new float [sizeI];
	float * the = new float [sizeI];

	int * tVect = new int [Hsize*sizeI];

	memset(tVect,0,Hsize*sizeI*sizeof(int));

	int* vectImg3 = new int [sizeI];
	int* vectImg2 = new int [sizeI];

	memset(vectImg3,0,sizeI*sizeof(int));
	memset(vectImg2,0,sizeI*sizeof(int));

	double the0, the2;
	double dist;

	for (y=border; y<sizeY-border-1; y++)
		for (x=border; x<sizeX-border-1; x++)
		{
			float dx = 0.5f*(inImg[y*sizeX+x+1]  - inImg[y*sizeX+x-1]);
			float dy = 0.5f*(inImg[(y+1)*sizeX+x]- inImg[(y-1)*sizeX+x]);

			DX[y*sizeX+x] = dx;
			DY[y*sizeX+x] = dy;

			mod[y*sizeX+x] = (float)sqrt(dx*dx + dy*dy);
			the[y*sizeX+x] = (float)fmod(atan2(dy, dx) + 2*PI, 2*PI); 
		}
		
	for (y=border; y<sizeY-border; y++)
		for (x=border; x<sizeX-border; x++)
		{
			float MOD = mod[y*sizeX+x];

			if (MOD>1)
			{
				double theta = the[y*sizeX+x];
				theta+= (PI/8.0f);

				if (theta>=2*PI)
					theta -= (2*PI);

				if (theta<0)
					theta += (2*PI);

				int sw = (int)(4*theta/(PI));

				theta -= (PI/8.0f);

				sw = sw%8;

				switch (sw)
				{
				case 0:
					the0 = the[(y-2)*sizeX+x]+3*PI/2;
					the2 = the[(y+2)*sizeX+x]+3*PI/2;
					theta+=(3*PI/2);

					if (fabs(the2-the0)>PI/1000)
						dist = (sin(the0)+sin(the2))/ sin (the2-the0);
					else dist = 0;

					theta-=(float)(3*PI/2);

					break;

				case 1:
					the0 = the[(y-2)*sizeX+x+2]+5*PI/4;
					the2 = the[(y+2)*sizeX+x-2]+5*PI/4;
					theta+=(5*PI/4);

					if (fabs(the2-the0)>PI/1000)
						dist = (sin(the0)+sin(the2))/ sin (the2-the0);
					else dist = 0;

					theta-=(5*PI/4);

					break;

				case 2:
					the0 = the[(y)*sizeX+x+2]+PI;
					the2 = the[(y)*sizeX+x-2]+PI;
					theta+=(PI);

					if (fabs(the2-the0)>PI/1000)
						dist = (sin(the0)+sin(the2))/ sin (the2-the0);
					else dist = 0;

					theta-=(float)(PI);
					
					break;

				case 3:
					the0 = the[(y+2)*sizeX+x+2]+3*PI/4;
					the2 = the[(y-2)*sizeX+x-2]+3*PI/4;
					theta+=(3*PI/4);

					if (fabs(the2-the0)>PI/1000)
						dist = (sin(the0)+sin(the2))/ sin (the2-the0);
					else dist = 0;

					theta-=(3*PI/4);
					break;

				case 4:
					the0 = the[(y+2)*sizeX+x]+PI/2;
					the2 = the[(y-2)*sizeX+x]+PI/2;
					theta+=(PI/2);

					if (fabs(the2-the0)>PI/1000)
						dist = (sin(the0)+sin(the2))/ sin (the2-the0);
					else dist = 0;

					theta-=(float)(PI/2);
					break;

				case 5:
					the0 = the[(y+2)*sizeX+x-2]+(float)PI/4;
					the2 = the[(y-2)*sizeX+x+2]+(float)PI/4;
					theta+=(float)(PI/4);

					if (fabs(the2-the0)>PI/1000)
						dist = (sin(the0)+sin(the2))/ sin (the2-the0);
					else dist = 0;

					theta-=(float)(PI/4);
					break;

				case 6:
					the0 = the[(y)*sizeX+x-2];
					the2 = the[(y)*sizeX+x+2];

					if (fabs(the2-the0)>PI/1000)
						dist = (sin(the0)+sin(the2))/ sin (the2-the0);
					else dist = 0;
					break;

				case 7:
					the0 = the[(y-2)*sizeX+x-2]-PI/4;
					the2 = the[(y+2)*sizeX+x+2]-PI/4;
					theta-=PI/4;

					if (fabs(the2-the0)>PI/1000)
						dist = (sin(the0)+sin(the2))/ sin (the2-the0);
					else dist = 0;
					theta+=(PI/4);
					break;

				default:
					dist = 0;
					break;

				}

				int ilo = 0;
				int ihi = 0;

				if (dist<0)
					ilo = -Hsize+1;
				else if (dist >0)
					ihi =  Hsize-1;

				for (i=ilo; i<=ihi; i++)
				{
					int x0=(int)(x+i*cos(theta));
					int y0=(int)(y+i*sin(theta));

					if (x0<border||x0>sizeX-border-1||y0<border||y0>sizeY-border-1)
						continue;

						if (dist<0)
							tVect[Hsize*(y0*sizeX+x0)-i]++;
						else if (dist >0)
							tVect[Hsize*(y0*sizeX+x0)+i]++;

					vectImg3[y0*sizeX+x0]+=(abs(i));
					vectImg2[y0*sizeX+x0]++;
				
				}
			}
		}

		int Max = 2;

		memset (ptsArray, 0, sizeof(int)*100);

		int index = 2;

		for (int j=0; j<Hsize*sizeI; j++)
		{
			if (tVect[j]==Max)
			{
				if (index<100-1)
				{
					ptsArray[index] = j;
					index++;
				}
			}
			if (tVect[j]>Max)
			{
				index = 1;
				memset (ptsArray, 0, sizeof(int)*100);

				ptsArray[index] = j;
				Max = tVect[j];
					
				index++;

			}
		}


		ptsArray[0] = index;
		ptsArray[100-1] = Max;

	delete [] DX;
	delete [] DY;
	delete [] mod;
	delete [] the;
	delete [] vectImg2;
	delete [] vectImg3;
	delete [] tVect;

}


void imsmooth(float * inImg, float * outImg, int sizeX, int sizeY ,double sig)
{
	int size = sizeX*sizeY;

	float * tmpPtr = outImg;
	
	if(sig > 0.01) 
	{
		int W = (int) ceil(4*sig) ;
		int i,j; 

		float * g0 = new float [2*W+1];
		float * buffer = new float [size];
		float acc = 0.0;
    
		for(j=0; j < 2*W+1; j++) 
		{
			g0[j] = (float)(exp(-0.5 * (j - W)*(j - W)/(sig*sig)));
			acc += g0[j] ;
		}

		for(j = 0; j < 2*W+1; j++) 
		{
			g0[j] /= acc;
	    }
    
    // Convolve along the columns

		for(j = 0 ; j < sizeY ; j++) 
		{
			for(i = 0 ; i < sizeX ; i++) 
			{
				float * start = inImg + max(i-W,0)       + j*sizeX ;        
				float * stop  = inImg + min(i+W,sizeX-1) + j*sizeX + 1 ;
				float * g = g0 + max(0, W-i);
				acc = 0.0 ;
				while(stop != start) 
					acc += (*g++) * (*start++) ;
				*buffer++ = acc ;
			}
		}
		buffer -= size ;
    
    // Convolve along the rows

		for(j = 0 ; j < sizeY ; j++) 
		{
			for(i = 0 ; i < sizeX ; i++) 
			{
				float* start = buffer + i + max(j-W,0)*sizeX;
				float* stop  = buffer + i + min(j+W,sizeY-1)*sizeX+sizeX;
				float* g = g0 + max(0, W-j) ;
				acc = 0.0 ;
				while(stop != start) 
				{ 
					acc += (*g++) * (*start); 
					start+=sizeX;
				}
				*outImg++ = acc ;
		  }
		}    
		delete [] buffer;
		delete [] g0;
	} 
	else 
	{
		memcpy(outImg, inImg, sizeof(float)*size);   
	}

	outImg = tmpPtr;
}


bool findCircles(ImageOf<PixelRgb> &inImg, BufferedPort<ImageOf<PixelMono> > &outPort, int &x, int &y, double &R)
{
	ImageOf<PixelMono> inMono;

	int i,j,k;


	const int width=inImg.width();
	const int height=inImg.height();

	inMono.resize(width, height);

	computeBYNew(inImg, inMono);

	unsigned char * monoImg = inMono.getRow(0);

	float * floatImg = new float [width*height];
	float * floatLPImg = new float [width*height];

	for (j=0; j<width*height; j++)
		floatImg[j] = monoImg[j];

	imsmooth(floatImg,floatLPImg,width,height,1.0);

	int radius;
	int point;

	int * ptsArray = new int [100];

	computeMap(floatLPImg,width,height,2,ptsArray);

	unsigned char * colorImg = inImg.getRow(0);

	double distance;

	bool outcome = true;

	float MaxBlu = 0;
	int bpoint;

	for (k=1; k<ptsArray[0]; k++)
	{
		point = ptsArray[k]/Hsize;
		x = point % width;
		y = point / width;
		radius = ptsArray[k]%Hsize;

		if (floatLPImg[y*width+x]>MaxBlu)
		{
			MaxBlu = floatLPImg[y*width+x]; 
			bpoint = k;
		}

	}

	point = ptsArray[bpoint]/Hsize;
	x = point % width;
	y = point / width;
	radius = ptsArray[bpoint]%Hsize;

	for (j=0; j<height; j++)
	{
		for (i=0; i<width; i++)
		{
			distance = sqrt(double((i-x)*(i-x))+double((j-y)*(j-y)));
			if (distance >= radius)
			{
				colorImg[3*(j*width+i)+0] = (3*colorImg[3*(j*width+i)+0]+255)/4;
				colorImg[3*(j*width+i)+1] = (3*colorImg[3*(j*width+i)+1]+255)/4;
				colorImg[3*(j*width+i)+2] = (3*colorImg[3*(j*width+i)+2]+  0)/4;
			}
			else
			{
				colorImg[3*(j*width+i)+0] = (3*colorImg[3*(j*width+i)+0]+    0)/4;
				colorImg[3*(j*width+i)+1] = (3*colorImg[3*(j*width+i)+1]+    0)/4;
				colorImg[3*(j*width+i)+2] = (3*colorImg[3*(j*width+i)+2]+  255)/4;
			}
		}
	}

	R = (double)radius;

//	sig::file::write(inImg,"Images\\2756088sBall.ppm");

	if (ptsArray[100-1]>12)
		outcome = false;

	if (radius>10)
		outcome = false;

	delete [] floatImg;
	delete [] floatLPImg;
	delete [] ptsArray;

	inMono.zero();

	return outcome;
}

int main(int argc, char *argv[])
{
    Network::init();
    int c=0;
	double radius = 0;
	double pos[2] = {0, 0};

	initSlave(argc, argv);
	initParams(argc, argv);
    int colorSpace=initColor(argc, argv);

    BufferedPort<ImageOf<PixelRgb> > in;
    BufferedPort<ImageOf<PixelRgb> > out;
	BufferedPort<Vector> outPos;
    BufferedPort<ImageOf<PixelMono> > outMono;
	BufferedPort<Vector> inPosSlave;
	if (SLAVE)
	{
		nameSlavePort(argc, argv, inPosSlave);
	}

	namePorts(argc, argv, in, out, outPos, outMono);

    ImageOf< PixelRgb> *imgL=0;
    
    while(imgL==0)
        imgL=in.read(true);

    ImageOf<PixelRgb> inSmall;

    const int width=imgL->width()/SCALE_FACTOR;
    const int height=imgL->height()/SCALE_FACTOR;

    inSmall.resize(width, height);

	//locate the ball
	int xBall, yBall;
	double xBallLP, yBallLP;
	double R;


	firstOrderLowPassFilter filt_xBall(TEMPORAL_POLE_TARGET);
	firstOrderLowPassFilter filt_yBall(TEMPORAL_POLE_TARGET);
	firstOrderLowPassFilter filt_R(TEMPORAL_POLE_TARGET);

    int count = 0;
	bool ballNotFound =true;
	fprintf(stderr, "Trying to locate the ball:");

    double dT=0; //average cycle time
    double prev = 0;
    double now;
	while(true)
	{
		if (ballNotFound)
		{
			imgL=in.read(true);
			downsample(*imgL, inSmall);
			if (imgL!=0)
			{
				if (SLAVE)
					ballNotFound = locateBall(inSmall, outMono, inPosSlave, xBall, yBall, R);
				else
					ballNotFound = locateBall(inSmall, outMono, xBall, yBall, R);
				if (!ballNotFound)
				{
					xBall = SCALE_FACTOR * xBall;
					yBall = SCALE_FACTOR * yBall;
					R	  = SCALE_FACTOR * R;

					filt_xBall.setState(xBall, xBall);
					filt_yBall.setState(yBall, yBall);
					filt_R.setState(R, R);
				}
                else
                    {
                        //send image anyway...
                        ImageOf<PixelRgb> &outImg=out.prepare();
                        outImg.resize(imgL->width(), imgL->height());
                        outImg=*imgL;
                        out.write();
                    }

				if (count%100==0)
                    fprintf(stderr, ".");
                count++;
			}
			else 
			{
				ballNotFound = true;
				fprintf(stderr, "Unable to read images\n");
			}
		}
		else
		{
			now= Time::now();
            dT+=now-prev;
            prev=now;
            if (count%100==0)
                {
                    fprintf(stderr, "Last cycle time is: %.1lf \n", dT*10);
                    dT=0;
                }

			imgL=in.read(true);
			if (imgL!=0)
			{
				if (SLAVE)
					ballNotFound = trackBall(*imgL, outMono, inPosSlave, xBall, yBall, R, colorSpace);
				else
					ballNotFound = trackBall(*imgL, outMono, xBall, yBall, R, colorSpace);

				//ballNotFound = true;
				if (!ballNotFound)
				{
					xBallLP = filt_xBall.temporalFilter(xBall);
					yBallLP = filt_yBall.temporalFilter(yBall);
					R	    = filt_R.temporalFilter(R);
					sendResults(xBallLP, yBallLP, R, *imgL, out, outPos);
                }
				else
                    {
                        //send image anyway...
                        ImageOf<PixelRgb> &outImg=out.prepare();
                        outImg.resize(imgL->width(), imgL->height());
                        outImg=*imgL;
                        out.write();
                        fprintf(stderr, "\nBall Lost! Trying to relocate the ball:");
                    }
			}
			else 
			{
				ballNotFound = true;
				fprintf(stderr, "Unable to read images\n");
			}

            //			if (count%100==0)
            //				fprintf(stderr, "Current cycle time is: %.1lf \n", 1000*(t2-t1));
           count++;
		}
	}

    Network::fini();
	return 0;
}
