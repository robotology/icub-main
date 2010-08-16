// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea, Giorgio Metta and Francesco Orabona
 * email:   francesco.rea@iit.it
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
 * @file ColorVQ.h
 * @brief implementation of the color quantization class.
 */

#include <iCub/ColorVQ.h>
#include <cstdlib>
#define DBGPF1 if (0)

using namespace yarp::sig;
//using namespace yarp::os;

void ColorVQ::Resize(int x, int y, int fovea) {
	tmp1.resize(x, y);
	tmp2.resize(x, y);

	width=x;
	height=y;
}


void ColorVQ::Variance(ImageOf<PixelMono> &src, ImageOf<PixelInt> &dst, int size) {
	tmp2 = tmp1;
}

void ColorVQ::Compactness(ImageOf<PixelMono> &src, int fovea, int val, int eps)
{
	int area=0;
	int perimetro=0;

	int w=src.width();
	int h=src.height();

	for (int y=fovea; y<h-1; y++) {
		// first column
		int x=0;
		if (abs(src(x,y)-val)<eps) {
			area++;
			if (abs(src(w-1,y-1)-val)<eps)
				perimetro--;
			if (abs(src(x+1,y+1)-val)<eps)
				perimetro--;
			if (abs(src(w-1,y+1)-val)<eps)
				perimetro--;
			if (abs(src(x+1,y-1)-val)<eps)
				perimetro--;
		}

		for (x=1; x<w-1; x++)
			if (abs(src(x,y)-val)<eps) {
				area++;
				if (abs(src(x-1,y-1)-val)<eps)
					perimetro--;
				if (abs(src(x+1,y+1)-val)<eps)
					perimetro--;
				if (abs(src(x-1,y+1)-val)<eps)
					perimetro--;
				if (abs(src(x+1,y-1)-val)<eps)
					perimetro--;
			}

		// last column
		x=w-1;
		if (abs(src(x,y)-val)<eps) {
			area++;
			if (abs(src(x-1,y-1)-val)<eps)
				perimetro--;
			if (abs(src(0,y+1)-val)<eps)
				perimetro--;
			if (abs(src(x-1,y+1)-val)<eps)
				perimetro--;
			if (abs(src(0,y-1)-val)<eps)
				perimetro--;
		}
	}

	perimetro+=4*area;
}


void ColorVQ::DominantQuantization(ImageOf<PixelBgr> &src, ImageOf<PixelBgr> &dst, unsigned char t)
{
	for (int r=0; r<height; r++)
		for (int c=0; c<width; c++) {
			unsigned int qr,qg,qb;
			
			unsigned char ored=src(c,r).r;
			unsigned char ogreen=src(c,r).g;
			unsigned char oblue=src(c,r).b;

			if ( (0.25*255<ored) && (ored<0.75*255) && (0.25*255<ogreen) && (ogreen<0.75*255) &&  (0.25*255<oblue) && (oblue<0.75*255) ){
				int dr=abs(ored-ogreen)+abs(ored-oblue);
				int dg=abs(ored-ogreen)+abs(ogreen-oblue);
				int db=abs(ogreen-oblue)+abs(oblue-ored);

				if ((dr>dg+t) && (dr>db+t)) {
					if ((ored>ogreen) && (ored>oblue))
						qr=255;
					else if ((ored<ogreen) && (ored<oblue))
						qr=0;
					else
						qr=127;
				} else
					qr=127;
				
				if ((dg>dr+t) && (dg>db+t)) {
					if ((ogreen>ored) && (ogreen>oblue))
						qg=255;
					else if ((ogreen<ored) && (ogreen<oblue))
						qg=0;
					else
						qg=127;
				} else
					qg=127;

				if ((db>dg+t) && (db>dr+t)) {
					if ((oblue>ogreen) && (oblue>ored))
						qb=255;
					else if ((oblue<ogreen) && (oblue<ored))
						qb=0;
					else
						qb=127;
				} else
					qb=127;
			} else {
				if (ored>0.75*255) qr=255;
				else if (ored<0.25*255) qr=0;
				else qr=127;

				if (ogreen>0.75*255) qg=255;
				else if (ogreen<0.25*255) qg=0;
				else qg=127;

				if (oblue>0.75*255) qb=255;
				else if (oblue<0.25*255) qb=0;
				else qb=127;
			}
			
			dst(c,r).r=qr;
			dst(c,r).g=qg;
			dst(c,r).b=qb;
		}
}


void ColorVQ::DominantQuantization(PixelBgr src, PixelBgr &dst, unsigned char t)
{
	unsigned int qr,qg,qb;
	
	unsigned char ored=src.r;
	unsigned char ogreen=src.g;
	unsigned char oblue=src.b;

	if ( (0.25*255<ored) && (ored<0.75*255) &&
		 (0.25*255<ogreen) && (ogreen<0.75*255) &&
		 (0.25*255<oblue) && (oblue<0.75*255)) {
		int dr=abs(ored-ogreen)+abs(ored-oblue);
		int dg=abs(ored-ogreen)+abs(ogreen-oblue);
		int db=abs(ogreen-oblue)+abs(oblue-ored);

		if ((dr>dg+t) && (dr>db+t)) {
			if ((ored>ogreen) && (ored>oblue))
				qr=255;
			else if ((ored<ogreen) && (ored<oblue))
				qr=0;
			else
				qr=127;
		} else
			qr=127;
		
		if ((dg>dr+t) && (dg>db+t)) {
			if ((ogreen>ored) && (ogreen>oblue))
				qg=255;
			else if ((ogreen<ored) && (ogreen<oblue))
				qg=0;
			else
				qg=127;
		} else
			qg=127;

		if ((db>dg+t) && (db>dr+t)){
			if ((oblue>ogreen) && (oblue>ored))
				qb=255;
			else if ((oblue<ogreen) && (oblue<ored))
				qb=0;
			else
				qb=127;
		} else
			qb=127;
	} else {
		if (ored>0.75*255) qr=255;
		else if (ored<0.25*255) qr=0;
		else qr=127;

		if (ogreen>0.75*255) qg=255;
		else if (ogreen<0.25*255) qg=0;
		else qg=127;

		if (oblue>0.75*255) qb=255;
		else if (oblue<0.25*255) qb=0;
		else qb=127;
	}
	
	dst.r=qr;
	dst.g=qg;
	dst.b=qb;
}
