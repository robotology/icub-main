#include "cht.h"

#include <stdio.h>
#include <math.h>
#include <memory.h>
#include <yarp/sig/Image.h>

using namespace yarp::sig;

CircularHT::CircularHT():
threshold(-1),
vectorMaxSize(500),
useThreshold(false),
houghValues(0),
lut(0),
radLut(0),
maxR(0),
maxHough(0)
{
	//constructor
}

CircularHT::~CircularHT()
{
	dealloc();
}

void CircularHT::alloc()
{
	dealloc();

	incDen = cht_round(8.0 * radiusMin);  // increment denominator
	lut = alloc3dC<int>(2, incDen, depth);
	houghValues = alloc3dC<HOUGH_MAP_TYPE>(depth, height, width);
	radLut=new int [radiusInc*(radiusMax-radiusMin)];
}

void CircularHT::dealloc()
{
	if (lut!=0)
		dealloc3dC<int> (lut, 2, incDen, depth);

	if (houghValues!=0)
		dealloc3dC<HOUGH_MAP_TYPE> (houghValues, depth, height, width);

	if (radLut!=0)
		delete [] radLut;

	radLut=0;
}

void CircularHT::computeTransform(yarp::sig::ImageOf<yarp::sig::PixelMono> &ip)
{
	imageValues = ip.getRawImage();
	houghTransform();
}

void CircularHT::getCenterPointsByThreshold(double threshold, ImageOf<PixelMono> &h)
{
	getCenterPointsByThreshold(threshold);
	drawCircles(h.getRawImage());
}

void CircularHT::getFullMap(ImageOf<PixelMono> &map)
{
	createFullMap(map.getRawImage());
}

bool CircularHT::setParameters(int rmin, int rmax, int rinc, int h, int w)
{
	height=h;
	width=w;
	offset=width;
	roi_offx=0;
	roi_offy=0;
	roi_w=w;
	roi_h=h;

	radiusMin = rmin;
	radiusMax = rmax;
	radiusInc = rinc;
	depth = ((radiusMax-radiusMin)/radiusInc)+1;
	
	alloc();
	lutSize = buildLookUpTable();

	return true;
}

/** The parametric equation for a circle centered at (a,b) with
radius r is:

  a = x - r*cos(theta)
  b = y - r*sin(theta)
  
    In order to speed calculations, we first construct a lookup
    table (lut) containing the rcos(theta) and rsin(theta) values, for
    theta varying from 0 to 2*PI with increments equal to
    1/8*r. As of now, a fixed increment is being used for all
    different radius (1/8*radiusMin). This should be corrected in
    the future.
	
	  Return value = Number of angles for each radius
	  
*/
int CircularHT::buildLookUpTable() 
{
	int i = 0;
	
	int k=0;
	int radius;
	for(radius = radiusMin;radius < radiusMax;radius += radiusInc)
	{
		int indexR = (radius-radiusMin)/radiusInc;
		radLut[k]=indexR;
		k++;
	}
	nRadLut=k;

	k=0;
	for(radius = radiusMin;radius < radiusMax;radius += radiusInc)
	{
		i = 0;
		for(int incNun = 0; incNun < incDen; incNun++)
		{
			double angle = (2*PI * (double)incNun) / (double)incDen;
			int indexR = radLut[k];//(radius-radiusMin)/radiusInc;
			int rcos = cht_round((double)radius * cos (angle));
			int rsin = cht_round((double)radius * sin (angle));
		
			if((i == 0) || (rcos != lut[0][i][indexR]) && (rsin != lut[1][i][indexR]))
				{
					lut[0][i][indexR] = rcos;
					lut[1][i][indexR] = rsin;
					i++;
				}
		}
		k++;
	}

	return i;
}

void CircularHT::houghTransform ()
{
	const int k = roi_w - 1;
	const int l = roi_h - 1;
    
	maxHough=0;

	memset(houghValues[0][0], 0, sizeof(unsigned int)*depth*height*width);

	for(int y = 1; y <= l; y++) {
		for(int x = 1; x <= k; x++) {
			if( imageValues[(x+roi_offx)+(y+roi_offy)*offset] != 0 )  
			{
				for(int k=0;k<nRadLut;k++)
				{
					int indexR = radLut[k];
					for(int i = 0; i < lutSize; i++) 
					{
						int a = x + lut[1][i][indexR]; 
						int b = y + lut[0][i][indexR]; 
						if((b >= 0) && (b < height) && (a >= 0) && (a < width))
						{ 
							unsigned int &tmp=houghValues[indexR][b][a];

							tmp+=1;
							if (tmp>maxHough)
							{
								maxHough=tmp;
                                maxR=indexR;
								maxPoint.x=a;
								maxPoint.y=b;
							}
						}
					}
				}
			}
		}
	}
}


// Convert Values in Hough Space to an 8-Bit Image Space.
void CircularHT::createHoughPixels (unsigned char *img, unsigned char *houghPixels, int R)
{
	double d = -1.0;
	for(int j = 0; j<height; j++)
	{
		for(int k = 0; k < width; k++)
			if(houghValues[j][k][R]> d)
			{
				d = houghValues[R][j][k];
			}
			
	}
	
	for(int l = 0; l < height; l++) 
	{
		for(int i = 0; i < width; i++)
		{
			houghPixels[i + l * width] = cht_round((houghValues[R][l][i] * 255.0) / d);
			houghPixels[i+l*width]+=cht_round(0.5*img[i+l*width]);
		}
		
	}
}

// Convert Values in Hough Space to an 8-Bit Image Space.
void CircularHT::createFullMap (unsigned char *houghPixels)
{
	double *tmpImg=new double[height*width];
	memset(tmpImg, 0, sizeof(double)*height*width);
	memset(houghPixels, 0, sizeof(unsigned char)*height*width);
	
	double max=0.0;
	int x;
	int y;
	for(int radius = radiusMin;radius < radiusMax;radius += radiusInc)
	{
		int indexR = (radius-radiusMin)/radiusInc;
		for(y = 0; y < height; y++)
		{
			for(x=0; x < width; x++) 
			{
					tmpImg[y*width+x]+=houghValues[indexR][y][x];
			}
		}
	}

	for(y = 0; y < height; y++) {
		for(x= 0; x < width; x++) {
				if (tmpImg[y*width+x]>max)
					max=tmpImg[y*width+x];
			}
		}

	for(y = 0; y < height; y++) {
		for(x= 0; x < width; x++) {
			houghPixels[(y+roi_offy)*width+x+roi_offx]=cht_round((tmpImg[y*width+x]/max)*255);			
			}
		}

	delete [] tmpImg;
}

/** Search circles having values in the hough space higher than a threshold

  @param threshold The threshold used to select the higher point of Hough Space
*/
void CircularHT::getCenterPointsByThreshold (double threshold)
{
	centerPoint = new Point[vectorMaxSize];
	int xMax = 0;
	int yMax = 0;
	int countCircles = 0;

		double thr=threshold;//*radius;
	//	printf("%.2lf ", thr);
		for(int y = 0; y < height; y++) {
			for(int x= 0; x < width; x++) {
				 for(int radius = radiusMin;radius < radiusMax;radius += radiusInc) {
					int indexR = (radius-radiusMin)/radiusInc;
					if(houghValues[indexR][y][x]>= thr) {
					if(countCircles < vectorMaxSize) {
						centerPoint[countCircles] = Point (x, y);
						clearNeighbours(xMax,yMax,radius);
						++countCircles;
					} 
					else
						break;
				}
			}
		}
	}

	maxCircles = countCircles;
}

HOUGH_MAP_TYPE CircularHT::getMax(int &ox, int &oy, HOUGH_MAP_TYPE &omaxval)
{
 	ox=maxPoint.x+roi_offx;
 	oy=maxPoint.y+roi_offy;

	Point center(0, 0);
	const int H=roi_h;
	const int W=roi_w;

	omaxval=maxHough;
	return maxR*radiusInc+radiusMin;

	/*for(int y = 0; y < H; y++) 	{
		for(int x = 0; x < W; x++) {
			for(int k=0;k<nRadLut;k++) {
				int indexR = radLut[k];
				HOUGH_MAP_TYPE tmp=houghValues[indexR][y][x];
				if(tmp>omaxval) 
				{
					maxR=indexR;
					omaxval=tmp;
					center.x=x;
					center.y=y;
				}
			}
		}
	}

	ox=center.x+roi_offx;
	oy=center.y+roi_offy;
	return maxR*radiusInc+radiusMin;*/
}

/** Clear, from the Hough Space, all the counter that are near (radius/2) a previously found circle C.
@param x The x coordinate of the circle C found.
@param x The y coordinate of the circle C found.
@param x The radius of the circle C found.
*/
void CircularHT::clearNeighbours(int x,int y, int radius) {
	// The following code just clean the points around the center of the circle found.
	
    double halfRadius = radius / 2.0F;
	double halfSquared = halfRadius*halfRadius;
	
	int y1 = (int)((double)y - halfRadius);
	int y2 = (int)cht_round((double)y + halfRadius) + 1;
	int x1 = (int)((double)x - halfRadius);
	int x2 = (int)cht_round((double)x + halfRadius) + 1;
	
	if(y1 < 0)
		y1 = 0;
	if(y2 > height)
		y2 = height;
	if(x1 < 0)
		x1 = 0;
	if(x2 > width)
		x2 = width;
	
	for(int r = radiusMin;r < radiusMax;r = r+radiusInc) {
		int indexR = (r-radiusMin)/radiusInc;
		for(int i = y1; i < y2; i++) {
			for(int j = x1; j < x2; j++) {	      	     
				if(pow (j - x, 2) + pow (i - y, 2) < halfSquared) {
					houghValues[indexR][i][j]= 0;
				}
			}
		}
	}
	
}


// Draw the circles found in the original image.
void CircularHT::drawCircles(unsigned char *circlespixels)
{
	// Copy original image to the circlespixels image.
	// Changing pixels values to 100, so that the marked
	// circles appears more clear. Must be improved in
	// the future to show the resuls in a colored image.
	
	for(int i = 0; i < width*height ;++i ) {
		if(imageValues[i] != 0 )
			circlespixels[i] = 100;
		else
			circlespixels[i] = 0;
	}
	
	char cor = -1;
	
	for(int l = 0; l < maxCircles; l++) {
		
		int i = centerPoint[l].x;
		int j = centerPoint[l].y;
		
		
		int k;
		// Draw a gray cross marking the center of each circle.
		for(k = -10 ; k <= 10 ; ++k ) {
			if(!outOfBounds(j+k+roi_offy,i+roi_offx))
				circlespixels[(j+k+roi_offy)*offset + (i+roi_offx)] = cor;
			if(!outOfBounds(j+roi_offy,i+k+roi_offx))
				circlespixels[(j+roi_offy)*offset   + (i+k+roi_offx)] = cor;
		}
		
		for(k = -2 ; k <= 2 ; ++k ) {
			if(!outOfBounds(j-2+roi_offy,i+k+roi_offx))
				circlespixels[(j-2+roi_offy)*offset + (i+k+roi_offx)] = cor;
			if(!outOfBounds(j+2+roi_offy,i+k+roi_offx))
				circlespixels[(j+2+roi_offy)*offset + (i+k+roi_offx)] = cor;
			if(!outOfBounds(j+k+roi_offy,i-2+roi_offx))
				circlespixels[(j+k+roi_offy)*offset + (i-2+roi_offx)] = cor;
			if(!outOfBounds(j+k+roi_offy,i+2+roi_offx))
				circlespixels[(j+k+roi_offy)*offset + (i+2+roi_offx)] = cor;
		}
	}
}

void CircularHT::setROI(int x, int y, int w, int h)
{
	roi_offx=x;
	roi_offy=y;
	roi_w=w;
	roi_h=h;
}

void CircularHT::drawCircle(unsigned char *img, int x, int y)
{
	int i;
	int j;
	for(i = 0; i < width*height ;++i ) {
		if(imageValues[i] != 0 )
			img[i] = 100;
		else
			img[i] = 0;
	}

	i = x;
	j = y;
	
	unsigned char cor=0xFF; //255
		
	int k;
	// Draw a gray cross marking the center of each circle.
	for(k = -10 ; k <= 10 ; ++k ) {
		if(!outOfBounds(j+k+roi_offy,i+roi_offx))
			img[(j+k+roi_offy)*offset + (i+roi_offx)] = cor;
		if(!outOfBounds(j+roi_offy,i+k+roi_offx))
			img[(j+roi_offy)*offset   + (i+k+roi_offx)] = cor;
	}
		
	for(k = -2 ; k <= 2 ; ++k ) {
		if(!outOfBounds(j-2+roi_offy,i+k+roi_offx))
			img[(j-2+roi_offy)*offset + (i+k+roi_offx)] = cor;
		if(!outOfBounds(j+2+roi_offy,i+k+roi_offx))
			img[(j+2+roi_offy)*offset + (i+k+roi_offx)] = cor;
		if(!outOfBounds(j+k+roi_offy,i-2+roi_offx))
			img[(j+k+roi_offy)*offset + (i-2+roi_offx)] = cor;
			if(!outOfBounds(j+k+roi_offy,i+2+roi_offx))
			img[(j+k+roi_offy)*offset + (i+2+roi_offx)] = cor;
	}
}


/** Search for a fixed number of circles.
@param maxCircles The number of circles that should be found.  
*/
void CircularHT::getCenterPoints (int maxCircles)
{
	centerPoint = new Point[maxCircles];
	int xMax = 0;
	int yMax = 0;
	int rMax = 0;
	
	for(int c = 0; c < maxCircles; c++) {
		double counterMax = -1;
		for(int radius = radiusMin;radius < radiusMax;radius = radius+radiusInc) 
		{
			int indexR = (radius-radiusMin)/radiusInc;
			for(int y = 0; y < height; y++) {
				for(int x = 0; x <  width; x++) {
					if(houghValues[indexR][y][x] > counterMax) {
						counterMax = houghValues[indexR][y][x];
						xMax = x;
						yMax = y;
						rMax = radius;
					}
				}
				
			}
		}
		
		centerPoint[c] = Point (xMax, yMax);
		clearNeighbours(xMax,yMax,rMax);
	}
}

