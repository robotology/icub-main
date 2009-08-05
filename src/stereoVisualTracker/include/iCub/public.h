/*
	Type definitions, public inclusions and some arguably useful macros

	Copyright (C) 2007 basilio noris

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*!
 *	\file public.h
 *	\author Basilio Noris
 *	\date 25-11-06
 */

#ifndef _PUBLIC_H_
#define _PUBLIC_H_

#ifdef WIN32
#pragma warning(disable:4996) // avoids the sprintf_s and fopen_s warning which dont exist on linux ;)
#endif

// opencv includes
#include "cv.h"
#include "cvaux.h"
#include "highgui.h"

// type definitions, u <- unsigned, s <- signed, f <- floating point, X: number of bits
#define u8 unsigned char
#define u16 unsigned short int
#define u32 unsigned int
#define u64 unsigned long
#define s8 char
#define s16 short int
#define s32 int
#define s64 long
#define f32 float
#define f64 double

#ifndef WIN32
#define max(a,b) (a > b ? a : b)
#define min(a,b) (a < b ? a : b)
#endif


#ifdef OPENCV_0_9_7
#define cvDrawLine(a,b,c,d) cvLine(a,b,c,d)
#endif

// fancy macros
#define FOR(i,length) for(u32 i=0; i<(u32)(length); i++) 
#define SWAP(x,y) x^=y^=x^=y // fast swap for pointers or integers
#define MAX3(a,b,c) max(a,max(b,c))
#define MIN3(a,b,c) min(a,min(b,c))
#define trim(a,b,c) min(max(a,b),c)
// delete macros
#define KILL(a) if(a){delete [] a; a=NULL;} // safe delete []
#define IMKILL(a) if(a) {cvReleaseImage(&(a)); (a)=NULL;} // safe delete image
#define DEL(a) if(a){delete a; a=NULL;} // safe delete pointer
// reading pixels from an image
#define rgb(image,i) (image->imageData[i])
#define to255(x) ((x<<8)-x) // fast multiply *255

// definitions for the matlab-style tic toc
#define TICTOC u64 tic_time = 0;
#define tic tic_time = (u64)(cvGetTickCount()/cvGetTickFrequency())
#define toc printf("Elapsed time is %.3f seconds.\n",((u64)(cvGetTickCount()/cvGetTickFrequency()) - tic_time)/1000000.0f)
#define etoc ((u64)(cvGetTickCount()/cvGetTickFrequency()) - tic_time)

/*!
 * The Filter class, one of the main tools to process images
 */
class Filter
{
public:
	/*!
		The main Processing function, takes an input image and if necessary can make modifications over it
		\param image the input image
	*/
	virtual void Apply(IplImage *image){};

	/*!
		The secondary Processing function, usually does the same as the Apply()
		function but returns the modifications as an output image, leaving the source image intact
		\param image the input image
		\return returns the filtered image
	*/
	virtual IplImage *Process(IplImage *image){return NULL;};

	/*!
		The Configuration function, can be useful for certain filters
		\param image the input image
		\param selection the region of the image the filter will be configured from
	*/
	virtual void Config(IplImage *image, CvRect selection, IplImage *mask=0){};
};

/*!
 * The Frame Grabber virtual class
 */
class FrameGrabber
{
public:
	/*!
		Grabs the current frame
		\param frame the pointer to the frame that will be filled by the grabber
		\param index can be the index to a particular frame or to one of many streams (in case of stereo grabbers)
	*/
	virtual void GrabFrame(IplImage **frame, u32 index=0){};
	
	/*!
		Get the resolution of the frame grabber (or of the current frame)
		\return returns the size of the current frame
	*/
	virtual CvSize GetSize(){return cvSize(0,0);};

	/*!
		Kills the grabber, freeing and releasing up the memory allocated by the grabber
	*/
	virtual void Kill(){};

	virtual bool Init(char *s1){return true;};
	virtual bool Init(char *s1, char *s2){return true;};
	virtual double GetTime(){return 0.0;};
};

#endif //_PUBLIC_H_
