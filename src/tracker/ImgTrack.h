// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#ifndef _YARPImageTrackTool_INC
#define _YARPImageTrackTool_INC

#include <yarp/sig/all.h>

// 128 is a good number.
const int ISIZE = 128;
const int IMG_H = ISIZE;
const int IMG_W = ISIZE;

typedef unsigned char ImgInt[IMG_H][IMG_W];
typedef unsigned char ImgInt3[IMG_H][IMG_W][3];


// What are the YARP classes de jour?
// They seem to change every year...
typedef yarp::sig::ImageOf<yarp::sig::PixelMono> MonoImage;
typedef yarp::sig::ImageOf<yarp::sig::PixelRgb> ColorImage;



class YARPImageTrackTool
{
private:
    int tx;
    int ty;
    float scaled_tx, scaled_ty;
    int mw, mh;

    MonoImage prevImg;
    ColorImage prevImg3;

    int first;
    int delta;
    int blockSize;
    int blockXSize;
    int blockYSize;
    int windowSize;
    int windowXSize;
    int windowYSize;
    int windowXOffset;
    int windowYOffset;
    int normalize;
    float quality;

	void Apply(ImgInt frame1, ImgInt frame2, int x, int y);
	void Apply(MonoImage& src);
	void Apply(MonoImage& src, MonoImage& dest, int x, int y);
	void Apply(ImgInt3 frame1, ImgInt3 frame2, int x, int y);

	void SetXY(int x, int y)
    {
		tx = x; ty = y;
		delta = 1;
    }
    

public:
	YARPImageTrackTool()
    {
		ResetXY();
		quality = 0;
		first = 1;
		blockSize = 9;
		blockXSize = blockSize;
		blockYSize = blockSize;
		windowSize = 5;
		windowXSize = windowSize;
		windowYSize = windowSize;
		windowXOffset = 0;
		windowYOffset = 0;
		normalize = 0;
        scaled_tx = 0;
        scaled_ty = 0;
        mw = mh = 128;
    }
  
	void SetBlockSize(int size)
    {
		blockSize = size;
		blockXSize = blockSize;
		blockYSize = blockSize;
    }
  
	void SetBlockSize(int dx, int dy)
    {
		blockXSize = dx;
		blockYSize = dy;
    }

	// search will be from -size to +size pixels from the search center
	void SetSearchWindowSize(int size) { windowSize = windowXSize = windowYSize = size; }
	void SetSearchWindowOffset(int dx, int dy)
    {
		windowXOffset = dx;
		windowYOffset = dy;
    }

	void SetSearchWindowSize(int xsize, int ysize)
    {
		windowXSize = xsize;
		windowYSize = ysize;
		windowSize = xsize;
    }
  
	void SetNormalize(int flag=1) { normalize = flag; }
	//int GetX() { return tx; }
	//int GetY() { return ty; }
	float GetX() { return scaled_tx; }
	float GetY() { return scaled_ty; }
    float GetQuality() { return quality; }
  
	void Restart() { first = 1; }
	void ResetXY() { 
        SetXY(IMG_W/2,IMG_H/2); 
        scaled_tx = (((float)tx*mw)/128);
        scaled_ty = (((float)ty*mh)/128);
    }

	void Apply(ColorImage& src) {
        ApplyHelper(src,false);
    }

	void ApplyHelper(ColorImage& src, bool recursive = false);

    void SetXYScaled(int x, int y) {
        tx = int(((float)x*128)/mw);
        ty = int(((float)y*128)/mh);
        scaled_tx = x;
        scaled_ty = y;
    }
};




#endif
