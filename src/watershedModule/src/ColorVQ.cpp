// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//#include <yarp/YARPConfig.h>
#include <ace/config.h>
//#include <ipl/ipl.h>

#include <iCub/ColorVQ.h>

#define DBGPF1 if (0)

void ColorVQ::Resize(int x, int y, int fovea)
{
	tmp1.resize(x, y);
	tmp2.resize(x, y);

	width=x;
	height=y;
}


void ColorVQ::Variance(ImageOf<PixelMono> &src, ImageOf<PixelInt> &dst, int size)
{
	//__IPLDEPRECATED__iplConvert((IplImage*) src, (IplImage*) tmp1);
	tmp2=tmp1;
	
	//__IPLDEPRECATED__iplSquare((IplImage*) tmp1, (IplImage*) tmp1);
	//__IPLDEPRECATED__iplBlur((IplImage*) tmp1, (IplImage*) tmp1, size, size, (size-1)/2, (size-1)/2);

	//__IPLDEPRECATED__iplBlur((IplImage*) tmp2, (IplImage*) tmp2, size, size, (size-1)/2, (size-1)/2);
	//__IPLDEPRECATED__iplSquare((IplImage*) tmp2, (IplImage*) tmp2);

	//__IPLDEPRECATED__iplSubtract((IplImage*) tmp1, (IplImage*) tmp2, (IplImage*) dst);
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
			
			unsigned char or=src(c,r).r;
			unsigned char og=src(c,r).g;
			unsigned char ob=src(c,r).b;

			if ( (0.25*255<or) && (or<0.75*255) && (0.25*255<og) && (og<0.75*255) &&  (0.25*255<ob) && (ob<0.75*255) ){
				int dr=abs(or-og)+abs(or-ob);
				int dg=abs(or-og)+abs(og-ob);
				int db=abs(og-ob)+abs(ob-or);

				if ((dr>dg+t) && (dr>db+t)) {
					if ((or>og) && (or>ob))
						qr=255;
					else if ((or<og) && (or<ob))
						qr=0;
					else
						qr=127;
				} else
					qr=127;
				
				if ((dg>dr+t) && (dg>db+t)) {
					if ((og>or) && (og>ob))
						qg=255;
					else if ((og<or) && (og<ob))
						qg=0;
					else
						qg=127;
				} else
					qg=127;

				if ((db>dg+t) && (db>dr+t)) {
					if ((ob>og) && (ob>or))
						qb=255;
					else if ((ob<og) && (ob<or))
						qb=0;
					else
						qb=127;
				} else
					qb=127;
			} else {
				if (or>0.75*255) qr=255;
				else if (or<0.25*255) qr=0;
				else qr=127;

				if (og>0.75*255) qg=255;
				else if (og<0.25*255) qg=0;
				else qg=127;

				if (ob>0.75*255) qb=255;
				else if (ob<0.25*255) qb=0;
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
	
	unsigned char or=src.r;
	unsigned char og=src.g;
	unsigned char ob=src.b;

	if ( (0.25*255<or) && (or<0.75*255) &&
		 (0.25*255<og) && (og<0.75*255) &&
		 (0.25*255<ob) && (ob<0.75*255)) {
		int dr=abs(or-og)+abs(or-ob);
		int dg=abs(or-og)+abs(og-ob);
		int db=abs(og-ob)+abs(ob-or);

		if ((dr>dg+t) && (dr>db+t)) {
			if ((or>og) && (or>ob))
				qr=255;
			else if ((or<og) && (or<ob))
				qr=0;
			else
				qr=127;
		} else
			qr=127;
		
		if ((dg>dr+t) && (dg>db+t)) {
			if ((og>or) && (og>ob))
				qg=255;
			else if ((og<or) && (og<ob))
				qg=0;
			else
				qg=127;
		} else
			qg=127;

		if ((db>dg+t) && (db>dr+t)){
			if ((ob>og) && (ob>or))
				qb=255;
			else if ((ob<og) && (ob<or))
				qb=0;
			else
				qb=127;
		} else
			qb=127;
	} else {
		if (or>0.75*255) qr=255;
		else if (or<0.25*255) qr=0;
		else qr=127;

		if (og>0.75*255) qg=255;
		else if (og<0.25*255) qg=0;
		else qg=127;

		if (ob>0.75*255) qb=255;
		else if (ob<0.25*255) qb=0;
		else qb=127;
	}
	
	dst.r=qr;
	dst.g=qg;
	dst.b=qb;
}
