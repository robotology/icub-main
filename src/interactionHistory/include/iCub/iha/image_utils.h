#ifndef __IMAGE_UTILS_H_
#define __IMAGE_UTILS_H_

#include <yarp/sig/Image.h>
#include <algorithm>
#include <iCub/iha/debug.h>

using yarp::sig::ImageOf;
using yarp::sig::PixelRgb;
using namespace std;
using namespace iCub::iha;

/**
 * Pixelate an image to a give size. Place the average luminance in the given array.
 * \param img pointer to yarp image to pixelate
 * \param sarray pointer to an interger array (size pixX x pixY) for holding resulting sensor values
 * \param pixX number of sensors (pixelated pixels) across
 * \param pixY number of sensors (pixelated pixels) down
 */
void imageToSensorArray(ImageOf<PixelRgb> *img, int *sensorArray, int pixX, int pixY) {
	const int iw=img->width();
	const int ih=img->height();
	const double bsw=((double)iw)/((double)pixX);
	const double bsh=((double)ih)/((double)pixY);

	double lum[pixX * pixY];
	int pixelCount[pixX * pixY];
	memset(lum, 0, sizeof(lum));
	memset(pixelCount, 0, sizeof(pixelCount));

	double luminanceSum = 0;
	IhaDebug::pmesg(DBGL_DEBUG2,"imageToSensorArray imgw/h %d,%d bsw/h %f,%f\n",iw,ih,bsw,bsh);
	for (int x=0;x<iw;x++) {
		for (int y=0;y<ih;y++) {
			PixelRgb &pix=img->safePixel(x,y);
			double luminance = 0.3 * pix.r + 0.59 * pix.g + 0.11 * pix.b;
			luminanceSum += luminance;
			
			int destX = (int)( ((double) x) / bsw); // % pixelateSize;
			int destY = (int)( ((double) y) / bsh); // % pixelateSize;
			int destIndex = destX * pixX + destY; 
			//printf("iw=%d ih=%d x=%d y=%d bsw=%f bsh=%f destX=%d destY=%d destIndex=%d\n",iw,ih,x,y,bsw,bsh,destX,destY,destIndex);
			lum[destIndex] += luminance;
			pixelCount[destIndex]++;
		}
	}
	const double averageLuminance=luminanceSum/(iw*ih);
	// adjust according to average
	for (int y=0;y<pixY;y++) {
		for (int x=0;x<pixX;x++) {
			const int i = x * pixX + y;
			// sensorArray[i] = lum[i];
			const int n = pixelCount[i];
			sensorArray[i] = ((n == 0) ? 0 : max(0,min((int)(lum[i]/n/averageLuminance*128),255)));
			//sensorArray[i] = ((n == 0) ? 0 : max(0,min((int)(lum[i]/n),255)));
		}
	}
}

/**
 * Display the given array on an image as regions of greyscale.
 * \param img pointer to yarp image that can be written on
 * \param sarray pointer to an interger array (size pixX x pixY) with sensor values
 * \param pixX number of sensors across
 * \param pixY number of sensors down
 */
void showPixelatedImage(ImageOf<PixelRgb> *img, int *sarray, int pixX, int pixY) {
	int iw=img->width();
	int ih=img->height();
	const double bsw=((double)iw)/((double)pixX);
	const double bsh=((double)ih)/((double)pixY);

	for (int x=0;x<iw;x++) {
		for (int y=0;y<ih;y++) {
			
			int destX = (int)( ((double) x) / bsw); 
			int destY = (int)( ((double) y) / bsh); 
			int destIndex = destX * pixX + destY; 
			PixelRgb &pix=img->safePixel(x,y);
			pix = PixelRgb(0,0,sarray[destIndex]);
		}
	}
}

#endif

