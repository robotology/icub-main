// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2007 Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "ImgTrack.h"

using namespace yarp::sig;

// YARP method names de jour
#define GetWidth width
#define GetHeight height
#define PeerCopy copy
#define GetRawBuffer getRawImage

struct Pixel3 
{ 
	unsigned char b, g, r; 
};

static float ComparePixel(ImgInt s1, int r1, int c1, ImgInt s2, int r2, int c2, int BLOCK_SIZEX, int BLOCK_SIZEY, int NORMALIZE)
{
	int delx = BLOCK_SIZEX;
	int dely = BLOCK_SIZEY;
	int i, j;
	float d, diff = 0;

	if (!NORMALIZE)
        {
            for (i = -dely; i <= dely; i++)
                {
                    for (j = -delx; j <= delx; j++)
                        {
                            d = (float)(s1[r1+i][c1+j] - s2[r2+i][c2+j]);
                            diff += (d > 0) ? d : -d;
                        }
                }
        }
	else
        {
            float t1=0, t2=0;
            float mu1, mu2;
            float s12 = 1;
            for (i = -dely; i <= dely; i++)
                {
                    for (j = -delx; j <= delx; j++)
                        {
                            t1 += s1[r1+i][c1+j];
                            t2 += s2[r2+i][c2+j];
                        }
                }

            float area = (float)((2*delx+1)*(2*dely+1));
            mu1 = t1/area;
            mu2 = t2/area;
            float theta = 0.1f;

            if (t1>theta && t2>theta)
                {
                    s12 = t2/t1;
                }

            for (i=-dely; i<=dely; i++)
                {
                    for (j=-delx; j<=delx; j++)
                        {
                            d = (s1[r1+i][c1+j]*s12)-(s2[r2+i][c2+j]);
                            diff += (d > 0) ? d : -d;
                        }
                }
        }

	return diff;
}


template <class T>
T max2(T x, T y)
{ 
	return (x > y) ? x : y; 
}


static float ComparePixel(ImgInt3 s1, int r1, int c1, ImgInt3 s2, int r2, int c2,int BLOCK_SIZEX, int BLOCK_SIZEY, int NORMALIZE)
{
	int delx = BLOCK_SIZEX;
	int dely = BLOCK_SIZEY;
	int i, j;
	float d, diff=0;

	if (NORMALIZE)
        {
            for (i=-dely; i<=dely; i++)
                {
                    for (j=-delx; j<=delx; j++)
                        {
                            d = (float)(s1[r1+i][c1+j][0] - s2[r2+i][c2+j][0]);
                            diff += (d>0)?d:-d;
                            d = (float)(s1[r1+i][c1+j][1] - s2[r2+i][c2+j][1]);
                            diff += (d>0)?d:-d;
                            d = (float)(s1[r1+i][c1+j][2] - s2[r2+i][c2+j][2]);
                            diff += (d>0)?d:-d;
                        }
                }
        }
	else
        {
            Pixel3 *p1, *p2;
            for (i = -dely; i <= dely; i++)
                {
                    for (j = -delx; j <= delx; j++)
                        {
                            p1 = (Pixel3*) s1[r1+i][c1+j];
                            p2 = (Pixel3*) s2[r2+i][c2+j];
                            d = (float)(max2(abs(p1->r - p1->g), abs(p2->r - p2->g)) * abs((p1->r - p1->g) - (p2->r - p2->g)));
                            d += max2(abs(p1->r - p1->b),abs(p2->r - p2->b)) * abs((p1->r - p1->b) - (p2->r - p2->b));
                            d *= abs(p1->r + p1->g + p1->b - (p2->r + p2->g + p2->b));
                            diff += d;
                        }
                }
        }

	return diff;
}


template <class ImgType>
static float FindPixel(ImgType s1, int r1, int c1, 
					   ImgType s2, int *pr2, int *pc2,
					   int BLOCK_SIZEX, int BLOCK_SIZEY, 
					   int SEARCH_SIZEX, int SEARCH_SIZEY, 
					   int OFFSET_X, int OFFSET_Y, int NORMALIZE)
{
	int r2=(*pr2) + OFFSET_Y, c2=(*pc2) + OFFSET_X;
	float best = 999999999999.0f, current, central=0;
	float worst = 0;
	int i, j;
	int delx = SEARCH_SIZEX;
	int dely = SEARCH_SIZEY;
	
	for (i = -dely+OFFSET_Y; i <= dely+OFFSET_Y; i++)
        {
            for (j = -delx+OFFSET_X; j <= delx+OFFSET_X; j++)
                {
                    int r1d = (*pr2)+i;
                    int c1d = (*pc2)+j;
                    if (c1 + BLOCK_SIZEX < IMG_W && c1 - BLOCK_SIZEX >= 0 &&
                        r1 + BLOCK_SIZEY < IMG_H && r1 - BLOCK_SIZEY >= 0 &&
                        c1d + BLOCK_SIZEX < IMG_W && c1d - BLOCK_SIZEX >= 0 &&
                        r1d + BLOCK_SIZEY < IMG_H && r1d - BLOCK_SIZEY >= 0)
                        {
                            current = ComparePixel(s1,r1,c1,s2,r1d,c1d,BLOCK_SIZEX,BLOCK_SIZEY,NORMALIZE);
                            if (i==0 && j==0)
                                {
                                    central = current;
                                }

                            if (current<best || (current==best && i==0 && j==0))
                                {
                                    r2 = (*pr2)+i;
                                    c2 = (*pc2)+j;
                                    best = current;
                                }

                            if (current > worst)
                                {
                                    worst = current;
                                }
                        }
                }
        }

	*pr2 = r2;
	*pc2 = c2;

	float safe = 10;
	return (fabs(worst-best)+safe)/(best+safe);
}

void YARPImageTrackTool::Apply(ImgInt frame1, ImgInt frame2, int x, int y)
{
	quality = FindPixel(frame1, y, x, frame2, &ty, &tx, blockXSize, blockYSize, windowXSize, windowYSize, windowXOffset, windowYOffset, normalize);
}

void YARPImageTrackTool::Apply(ImgInt3 frame1, ImgInt3 frame2, int x, int y)
{
	quality = FindPixel(frame1, y, x, frame2, &ty, &tx, blockXSize, blockYSize, windowXSize, windowYSize, windowXOffset, windowYOffset, normalize);
}

void YARPImageTrackTool::Apply(MonoImage& src)
{
	int ox, oy;

	if (src.GetWidth() != IMG_W || src.GetHeight()!=IMG_H)
        {
            printf("Image tracking code is old, and specific to %dx%d images\n", IMG_W, IMG_H);
            exit(1);
        }

	//nextImg.Refer(src);
	if (first)
        {
            first = 0;
            ResetXY();
            prevImg.PeerCopy(src);
            delta = 0;
        }
	else
        {
            ox = tx;
            oy = ty;
            ImgInt& ii1 = *((ImgInt *)prevImg.GetRawBuffer());
            ImgInt& ii2 = *((ImgInt *)src.GetRawBuffer());
            Apply(ii1, ii2, tx, ty);
		
            if (ox != tx || oy != ty || delta)
                {
                    prevImg.PeerCopy(src);
                    delta = 0;
                }
        }
}

void YARPImageTrackTool::Apply(MonoImage& src, MonoImage& dest, 
                               int x, int y)
{
	if ((src.GetWidth()!=IMG_W||src.GetHeight()!=IMG_H) || 
		(dest.GetWidth()!=IMG_W||dest.GetHeight()!=IMG_H))
        {
            printf("Image tracking code is old, and specific to %dx%d images\n", IMG_W, IMG_H);
            exit(1);
        }

	tx = x; ty = y;
	ImgInt& ii1 = *((ImgInt *)src.GetRawBuffer());
	ImgInt& ii2 = *((ImgInt *)dest.GetRawBuffer());
	Apply(ii1, ii2, tx, ty);
}

void YARPImageTrackTool::ApplyHelper(ColorImage& src, bool recursive)
{
    if (!recursive) {
        mh = src.height();
        mw = src.width();
    }
	int ox, oy;

	if (src.GetWidth() != IMG_W || src.GetHeight() != IMG_H)
        {
            ColorImage cp;
            cp.copy(src,128,128);
            ApplyHelper(cp,true);
            scaled_tx = (((float)tx*src.width())/128);
            scaled_ty = (((float)ty*src.height())/128);
            return;
        }

	if (first)
        {
            first = 0;
            ResetXY();
            prevImg3.PeerCopy(src);
            delta = 0;
        }
	else
        {
            ox = tx;
            oy = ty;
            ImgInt3& ii1 = *((ImgInt3 *)prevImg3.GetRawBuffer());
            ImgInt3& ii2 = *((ImgInt3 *)src.GetRawBuffer());
            Apply(ii1, ii2, tx, ty);
            if (ox != tx || oy != ty || delta)
                {
                    prevImg3.PeerCopy(src);
                    delta = 0;
                }
            scaled_tx = (float)tx;
            scaled_ty = (float)ty;
        }
}

