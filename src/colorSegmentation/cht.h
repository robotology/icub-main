#ifndef __HOUGH__
#define __HOUGH__

#include <yarp/sig/Image.h>

typedef unsigned int HOUGH_MAP_TYPE;

class Point
{
public:
	Point(int xx, int yy)
	{ x=xx; y=yy;}

	Point(){x=0; y=0;}

	int x;
	int y;
};

inline int cht_round(double f)
{
	return int (f+0.5);
}

const double PI=3.14;

template <class T>
T*** alloc3dC(int depth, int H, int W)
{
	T* p=new T[depth*H*W];
	T*** ppp=new T**[depth];

	for(int k=0;k<depth;k++)
	{
		ppp[k]=new T *[H];
		for(int h=0; h<H; h++)
		{
			ppp[k][h]=p;

			for(int w=0; w<W; w++)
				p[w]=T(0);

			p+=W; //skip line
		}
	}

	return ppp;
}

template <class T>
void dealloc3dC(T *** &ppp, int depth, int H, int W)
{
	T *p=ppp[0][0]; //get the pointer to the first element

	for(int k=0;k<depth;k++)
	{
		delete [] ppp[k];
	}

	delete [] ppp;
	delete [] p;
}

template <class T>
T*** alloc3d(int depth, int H, int W)
{
	T*** ppp=new T** [depth];

	for(int k=0;k<depth;k++)
	{
		ppp[k]=new T *[H];
		for(int h=0; h<H; h++)
		{
			ppp[k][h]=new T[W];
			for(int w=0; w<W; w++)
				ppp[k][h][w]=T(0);
		}
	}

	return ppp;
}

template <class T>
void dealloc3d( T *** &ppp, int depth, int H)
{
	for(int k=0;k<depth;k++)
	{
		for(int h=0; h<H; h++)
			delete [] ppp[k][h];
		delete [] ppp[k];
	}
	delete [] ppp;

	ppp=0;
}

/**
*   This ImageJ plugin shows the Hough Transform Space and search for
*   circles in a binary image. The image must have been passed through
*   an edge detection module and have edges marked in white (background
*   must be in black).
*/
class CircularHT {
public:
    int radiusMin;  // Find circles with radius grater or equal radiusMin
    int radiusMax;  // Find circles with radius less or equal radiusMax
    int radiusInc;  // Increment used to go from radiusMin to radiusMax
    int maxCircles; // Numbers of circles to be found
    double threshold; // An alternative to maxCircles. All circles with
    // a value in the hough space greater then threshold are marked. Higher thresholds
    // results in fewer circles.
    unsigned char *imageValues; // Raw image (returned by ip.getPixels())
    HOUGH_MAP_TYPE ***houghValues; // Hough Space Values
    int width; // Hough Space width (depends on image width)
    int height;  // Hough Space heigh (depends on image height)
    int depth;  // Hough Space depth (depends on radius interval)
    int offset; // Image Width
    int roi_offx;   // ROI x offset
    int roi_offy;   // ROI y offset
	int roi_w;
	int roi_h;
	int incDen;
	int lutSize;
    Point *centerPoint; // Center Points of the Circles Found.
    int vectorMaxSize;
    bool useThreshold;
    int ***lut; // LookUp Table for rsin e rcos values
	int *radLut;
	int nRadLut;
	Point maxPoint;
	unsigned int maxHough;
    unsigned int maxR;

	CircularHT();
	~CircularHT();
	void resize(int, int, int);

	void alloc();
	void dealloc();

	void computeTransform(yarp::sig::ImageOf<yarp::sig::PixelMono> &ip);
	void getCenterPointsByThreshold(double threshold, yarp::sig::ImageOf<yarp::sig::PixelMono> &h);
	void getFullMap(yarp::sig::ImageOf<yarp::sig::PixelMono> &map);
	
	int buildLookUpTable();
	bool setParameters(int rmin, int rmax, int rinc, int height, int width);
    void houghTransform ();
	void getCenterPointsByThreshold (double threshold);
	void getCenterPoints (int maxCircles);
    void clearNeighbours(int x,int y, int radius);

    // Convert Values in Hough Space to an 8-Bit Image Space.
    void createHoughPixels (unsigned char *img, unsigned char *houghPixels, int R);
	void drawCircles(unsigned char *circlespixels);

	void drawCircle(unsigned char *img, int x, int y);
	void createFullMap(unsigned char *out);

	void setROI(int x, int y, int w, int h);

	HOUGH_MAP_TYPE getMax(int &x, int &y, HOUGH_MAP_TYPE &val);

	inline bool outOfBounds(int y,int x) {
        if(x >= width)
            return true;
        if(x <= 0)
            return true;
        if(y >= height)
            return true;
        if(y <= 0)
            return true;
        return false;
	}
	
	inline Point nthMaxCenter (int i) {
        return centerPoint[i];
    }
};
#endif
