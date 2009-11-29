///
///
///       YARP - Yet Another Robotic Platform (c) 2001-2003 
///
///                    #nat#
///
///     "Licensed under the Academic Free License Version 1.0"
///

///
/// $Id: main.cpp,v 1.28 2006/11/27 19:12:38 babybot Exp $
/// 
#include <cv.h>

#include <yarp/sig/Image.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/ImageFile.h>
#include <yarp/sig/ImageDraw.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Property.h>

#include "histogram.h"

#include <iostream>
#include <math.h>

//Circular Hough Transform
#include "cht.h"  

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;

const int SCALE_FACTOR = 2;

const int windowH=15/SCALE_FACTOR;    // window size for building the histogram
const int windowW=15/SCALE_FACTOR;    // window size for building the histogram
int VAL_MIN = 10;					  // min threshold of V in the backprojection/histogram
int SAT_MIN = 10;					  // min threshold of S in the backprojection/histogram

const double HISTO_THRESHOLD=0.1;	  //histogram threshold, for segmentation

const int LOW_PASS_THRESHOLD=70;	  //threshold on the output of the low pass filter
const double TEMPORAL_POLE = 0.9;     //temporal pole for the circle radius
const int MAX_RADIUS=95/SCALE_FACTOR;          //max radius for the ROI
const int POINTS_THRESHOLD=100/SCALE_FACTOR;   // we require a min number of pixels in the seg. region
 
//Hough parameters
const int CANNY_THRESHOLD1=3000;
const int CANNY_THRESHOLD2=1200;
const int HOUGH_MIN_RADIUS=20/SCALE_FACTOR;
const int HOUGH_MAX_RADIUS=65/SCALE_FACTOR;
const int HOUGH_RSTEPS=2/SCALE_FACTOR;
const int HOUGH_MIN_VAL=80/SCALE_FACTOR;
const int HOUGH_RADIUS_REDUCT=10/SCALE_FACTOR;

//Port names
const char *DEFAULT_NAME = "/histogram";

/* downsample src to fit dst */
void downsample(const ImageOf<PixelRgb> &src, ImageOf<PixelRgb> &dst)
{
    cvResize((const IplImage *)(src.getIplImage()), 
             (IplImage *) dst.getIplImage(), CV_INTER_NN);
}

/*
 * Builds the model of the object 
 * estimating its color histogram
 */

void grow(ImageOf<PixelHsv> &hsv, Histogram &histo, ImageOf<PixelRgb> &out, double i, double j, double r)
{
    int H=hsv.height();
    int W=hsv.width();

	double d;					//distance from the center
	double r2 = (float)(r*r);	//square of the radius
	PixelHsv pix;

    for (int ii=((int) (i-r)); ii<=((int)(i+r)); ii++)
	{
		for (int jj=((int) (j-r)); jj<=((int) (j+r)); jj++)
		{
			d = float((ii-i)*(ii-i)+(jj-j)*(jj-j));
            if (d<=r2)
			{
				pix = hsv.pixel(ii,jj);
				if ((pix.v>VAL_MIN) && (pix.s>SAT_MIN))
				{
	                histo.Apply(pix.h, pix.s, 0);
					PixelRgb &pictel = out.safePixel(ii, jj);
					pictel = PixelRgb(255, 0, 0);
				}
			}
		}
	}
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
		(IplImage *) inMono.getIplImage(), CV_BGR2GRAY);

	cvCanny((IplImage *) inMono.getIplImage(), 
		(IplImage *) inEdge.getIplImage(), CANNY_THRESHOLD1, CANNY_THRESHOLD2, 5 );

	hough.computeTransform(inEdge);
	R=hough.getMax(x, y, maxVal);

	ImageOf<PixelMono> &outEdge=outPort.prepare();
	outEdge.resize(width, height);
	outEdge=inEdge;
	draw::addCrossHair(outEdge, PixelMono(255), x, y, 10);
	outPort.write();

	if (maxVal > HOUGH_MIN_VAL)
		return false;
	else
		return true;
}

/*
 * Gives an output image where each pixel
 * is compared with the histogram. Moreover
 * an estimation of a circular region that
 * includes the object is given
 */
void backprojection(Histogram &histo,  ImageOf<PixelHsv> &hsv, ImageOf<PixelRgb> &out, double *mean, double &radius)
{
    int H=hsv.height();
    int W=hsv.width();
	double weights_sum=0;
	double cov[3] = {0, 0, 0};
	double lambda[2];

	mean[0] = 0;
	mean[1] = 0;

    for(int r=0;r<H;r++)
    {
        unsigned char *outTmp=out.getRow(r);
        unsigned char *hsvTmp=hsv.getRow(r);

        for(int c=0;c<W;c++)
        {
			if ((hsvTmp[2]>VAL_MIN) && (hsvTmp[1]>SAT_MIN))
			{

				double tmp=histo.backProjection(hsvTmp[0], hsvTmp[1], 0);
				if (tmp>HISTO_THRESHOLD)
				{
					*outTmp=static_cast<int> (255*tmp+0.5);

					weights_sum = weights_sum + tmp;
					mean[0] = mean[0] + tmp*c;
					mean[1] = mean[1] + tmp*r;
					cov[0] = cov[0] + tmp*c*c;
					cov[1] = cov[1] + tmp*r*r;
					cov[2] = cov[2] + tmp*r*c;
				}
            }
			hsvTmp+=3;
            outTmp+=3;
        }
    }
	if (weights_sum!=0)
	{
		mean[0] = mean[0]/weights_sum;
		mean[1] = mean[1]/weights_sum;

		cov[0] = cov[0]/weights_sum - mean[0]*mean[0];
		cov[1] = cov[1]/weights_sum - mean[1]*mean[1];
		cov[2] = cov[2]/weights_sum - mean[1]*mean[0];

		//compute the geometry of the region of interest
		lambda[0] = 2*((cov[0]+cov[1]) + sqrt(4*cov[2]*cov[2]+(cov[1] - cov[0])*(cov[1] - cov[0])));
		lambda[1] = 2*((cov[0]+cov[1]) - sqrt(4*cov[2]*cov[2]+(cov[1] - cov[0])*(cov[1] - cov[0])));

		if (lambda[0]>lambda[1])
			radius = sqrt(lambda[0]);
		else
			radius = sqrt(lambda[1]);
	}
	else
		radius = 0;
}

/*
 * Gives an output image where each pixel
 * is compared with the histogram. 
 */
void backprojection(Histogram &histo,  ImageOf<PixelHsv> &hsv, ImageOf<PixelMono> &out)
{
    const int H=hsv.height();
    const int W=hsv.width();

    for(int r=0;r<H;r++)
    {
        unsigned char *outTmp=out.getRow(r);
        unsigned char *hsvTmp=hsv.getRow(r);

        for(int c=0;c<W;c++)
        {
			if ((hsvTmp[2]>VAL_MIN) && (hsvTmp[1]>SAT_MIN))
			{
				double tmp=histo.backProjection(hsvTmp[0], hsvTmp[1], 0);

				if (tmp>HISTO_THRESHOLD)
					*outTmp=static_cast<int>(255);
				else
					*outTmp=0;
            }
			else
				*outTmp=0;

			hsvTmp+=3;
            outTmp+=1;
        }
    }
}

/*
 * Gives an estimation of a circular region that
 * includes the object.
 * return the number of pixels in the region
 */
int moments(ImageOf<PixelMono> &in, double *mean, double &radius)
{
    const int H=in.height();
    const int W=in.width();

	double weights_sum=0;
	double cov[3] = {0, 0, 0};
	double lambda[2];

	mean[0] = 0;
	mean[1] = 0;

    for(int r=0;r<H;r++)
        {
            unsigned char *tmp=in.getRow(r);

            for(int c=0;c<W;c++)
                {
                    weights_sum = weights_sum + (*tmp);
                    mean[0] = mean[0] + (*tmp)*c;
                    mean[1] = mean[1] + (*tmp)*r;

                    cov[0] = cov[0] + (*tmp)*c*c;
                    cov[1] = cov[1] + (*tmp)*r*r;
                    cov[2] = cov[2] + (*tmp)*r*c;		

                    tmp++;
                }
        }

	if (weights_sum!=0)
        {
            mean[0] = mean[0]/weights_sum;
            mean[1] = mean[1]/weights_sum;

            cov[0] = cov[0]/weights_sum - mean[0]*mean[0];
            cov[1] = cov[1]/weights_sum - mean[1]*mean[1];
            cov[2] = cov[2]/weights_sum - mean[1]*mean[0];

            //compute the geometry of the region of interest
            lambda[0] = 2*((cov[0]+cov[1]) + sqrt(4*cov[2]*cov[2]+(cov[1] - cov[0])*(cov[1] - cov[0])));
            lambda[1] = 2*((cov[0]+cov[1]) - sqrt(4*cov[2]*cov[2]+(cov[1] - cov[0])*(cov[1] - cov[0])));

            if (lambda[0]>lambda[1])
                radius = sqrt(lambda[0]);
            else
                radius = sqrt(lambda[1]);
        }
	else
		radius = 0;

    return (int) weights_sum;
}

void toHSV(ImageOf<PixelRgb> &in, ImageOf<PixelHsv> &out)
{
    IplImage *inIPL=(IplImage *) in.getIplImage();
    IplImage *hsv=(IplImage *) out.getIplImage();

	cvCvtColor(inIPL, hsv, CV_RGB2HSV);
}

void splitHSV(ImageOf<PixelHsv> &in, ImageOf<PixelMono> &h, ImageOf<PixelMono> &s, ImageOf<PixelMono> &v)
{
    const int H=in.height();
    const int W=in.width();

    for(int r=0;r<H;r++)
    {
        unsigned char *inTmp=in.getRow(r);
        unsigned char *hTmp=h.getRow(r);
        unsigned char *sTmp=s.getRow(r);
        unsigned char *vTmp=v.getRow(r);

        for(int c=0;c<W;c++)
        {
            *hTmp++=*inTmp++;
            *sTmp++=*inTmp++;    
            *vTmp++=*inTmp++;    
        }
    }
} 

void copyToOutput(ImageOf<PixelMono> &outMono, ImageOf<PixelRgb> &outImg)
{
	const int H=outMono.height();
    const int W=outMono.width();

    for(int r=0;r<H;r++)
    {
        unsigned char *img1=outMono.getRow(r);
        unsigned char *img2=outImg.getRow(r);

        for(int c=0;c<W;c++)
        {
            unsigned char tmp=*img1;
			if (tmp==255)
			{
				img2[0]=255;
				img2[1]=0;
				img2[2]=0;
			}

			img1++;
			img2+=3;
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
             CV_GAUSSIAN,5,5); //3x3 gaussian kernel

    cvThreshold(outIpl,
                outIpl,
                LOW_PASS_THRESHOLD,
                255,
                CV_THRESH_BINARY);
}

/*
 * Low passes the image radius estimation. The
 * underlying assumption is that the radius does
 * not change very fast so that we can filter out 
 * some noise.
 */  

double TemporalFilter(double *state, double &radius)

{
	state[0] = state[1];
	state[1] = TEMPORAL_POLE*state[0] + (1-TEMPORAL_POLE)*radius;
	return state[1];
}

 

void initHisto(Histogram &histo)
{
    unsigned char n[3]={20, 20, 1};
    histo.Resize(255, 0, n);
    histo.clean();
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
        printf("can change \"/histogram\" prefix with --name option\n\n");
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

void initParams(int argc, char *argv[])
{
    Property p;
    p.fromCommand(argc,argv);

	if 	(p.check("minval"))
		VAL_MIN = p.check("minval",Value(VAL_MIN)).asInt();

	if 	(p.check("minsat"))
		SAT_MIN = p.check("minsat",Value(SAT_MIN)).asInt();
}

int main(int argc, char *argv[])
{
    Network::init();
    int c=0;
	double radius = 0;
    double radius1=0;
	double pos[2] = {0, 0};
    double pos1[2] = {0, 0};

	//temporal filter state
	double x[2] = {0, 0};
	//temporal filter output
	double radius_lp = 0;

    Histogram histo;
	initHisto(histo);

    BufferedPort<ImageOf<PixelRgb> > in;
    BufferedPort<ImageOf<PixelRgb> > out;
	BufferedPort<Vector> outPos;
    BufferedPort<ImageOf<PixelMono> > outMono;

	namePorts(argc, argv, in, out, outPos, outMono);
	initParams(argc, argv);

    ImageOf< PixelRgb> *imgL=0;
    
    while(imgL==0)
        imgL=in.read(true);

    ImageOf<PixelHsv> hsv;
    ImageOf<PixelRgb> inSmall;
	ImageOf<PixelMono> outImgMono, outImgMonoLp;

    const int width=imgL->width()/SCALE_FACTOR;
    const int height=imgL->height()/SCALE_FACTOR;

    inSmall.resize(width, height);
    hsv.resize(width, height);
	outImgMono.resize(width, height);
	outImgMonoLp.resize(width, height);

	//locate the ball
	int xBall, yBall;
	double R;
	bool ballNotFound =true;
	fprintf(stderr, "Trying to locate the ball:");
	while(ballNotFound)
	{
		imgL=in.read(true);
		downsample(*imgL, inSmall);
		if (imgL!=0)
		{
			ballNotFound = locateBall(inSmall, outMono, xBall, yBall, R);
			fprintf(stderr, ".");
		}
		else 
		{
			ballNotFound = true;
			fprintf(stderr, "Unable to read images\n");
		}
	}
	fprintf(stderr, "Ball located @(%d,%d) \n", xBall, yBall);

	//grow the histogram
	for (int i = 0; i<100; i++)
	{
		imgL=in.read(true);
		downsample(*imgL, inSmall);

        toHSV(inSmall,  hsv);
        ImageOf<PixelRgb> &outImg=out.prepare();

        outImg.resize(width, height);
        outImg=inSmall;

		grow(hsv, histo, outImg,  xBall, yBall, R-HOUGH_RADIUS_REDUCT);
		
		out.write();
	}

	//execute the main loop (backprojection)
    while(true)
    {
        imgL=in.read(true);
        downsample(*imgL, inSmall);

        if (imgL!=0)
        {
            toHSV(inSmall,  hsv);

            ImageOf<PixelRgb> &outImg=out.prepare();

            outImg.resize(width, height);
            outImg=inSmall;

			backprojection(histo, hsv, outImgMono);			

			////
			lowPass(outImgMono, outImgMonoLp);
			copyToOutput(outImgMonoLp, outImg);

			int npoints=moments(outImgMonoLp, pos, radius);
            //  fprintf(stderr, "Pre: %lf %lf %lf\n", pos[0], pos[1], radius);
            //  filter(outImgMonoLp, 1.5*radius, pos1, radius1);  
            //  fprintf(stderr, "Aft: %lf %lf %lf\n", pos[0], pos[1], radius);
            radius_lp = TemporalFilter(x, radius);

            Vector &v=outPos.prepare();
            v.size(3);

            PixelRgb brush;

            if (npoints>POINTS_THRESHOLD)
            {
				brush.r=0;
                brush.g=255;
                brush.b=0;
                            
                v[0] = pos[0];
                v[1] = pos[1];
                if (radius_lp<MAX_RADIUS)
					v[2] = radius_lp;
                else
                    v[2]=MAX_RADIUS;
			}
            else
            {
                brush.r=0;
                brush.g=128;
                brush.b=0;

                v[0] = width/2;
                v[1] = height/2;
                if (radius_lp<MAX_RADIUS)
					v[2] = radius_lp;
                else
                    v[2]=MAX_RADIUS;
			}

            addCircleOutline(outImg, brush, (int)pos[0], (int)pos[1],(int) v[2]);

            out.write();
            outPos.write();
        }
    }

    Network::fini();
	return 0;
}
