// include header
#include <Tracking.h>
#include <cv.h>
#include <highgui.h>
#include <math.h>

#include "LabelImage.h"

// namespaces
using namespace halode::IO;
using namespace halode::gui;
using namespace halode::tracking;

using std::vector;

// ***************************************************************************

/**
 *
 * Implementation of HandTracker class
 *
 */

// Init constructor 

HandTracker::HandTracker(Parameters *_params_ptr) {
	
	//printf("Start:\t[HandTracker]\n");
	// Declarations & Initializing
	this->param_ptr				= _params_ptr;

	this->roi.col.x				= (int)(this->param_ptr->img_width/2);
	this->roi.col.y				= (int)(this->param_ptr->img_width/2);
	this->roi.row.x				= (int)(this->param_ptr->img_height/2);
	this->roi.row.y				= (int)(this->param_ptr->img_height/2);

	//this->isReady				= false;

	this->trackImg_ptr			= new ImageOf<PixelRgb>;
	//this->BGRimg_ptr			= new ImageOf<PixelBgr>;
	this->trackImg_ptr->resize(this->param_ptr->img_width, this->param_ptr->img_height);
	this->trackImg_ptr->zero();

	//this->cnt					= 0;
	//this->pos					= new iPoint3D [5];
	//this->objTracker			= new CObjectTracker(this->param_ptr);


}

// Destructor
HandTracker::~HandTracker() {
	//delete this->objTracker;
	//delete [] this->pos;
	//printf("Quit:\t[PatchTracker]\n");
}

void HandTracker::trackObject(ImageOf<PixelRgb> *_img_ptr) {
}

iPoint2D HandTracker::searchContrasts(ImageOf<PixelRgb> *_img_ptr, string _color) {
  ImageOf<PixelInt> mask, label;
	PixelRgb p_1, p, p1, q;
	iPoint2D pos;
	int i, j;
	this->trackImg_ptr->zero();
	this->xEllipse.clear();
	this->yEllipse.clear();

	mask.resize(*_img_ptr);
	mask.zero();
	label.resize(*_img_ptr);
	
	/*for (j = 1; j< _img_ptr->height()-1; j++) {
		for (i = 1; i< _img_ptr->width()-1; i++) {
			p = _img_ptr->pixel(i,j);
			// red
			if ((p.r > 3*p.g) && (p.r > 2*p.b) && (p.r>30)) {
				this->trackImg_ptr->safePixel(i,j) = p;	
			}
			// yellow
			else if (((p.g > 0.75*p.r) || (p.r > 0.75*p.g)) && (p.g > 1.5*p.b)) {
				this->trackImg_ptr->safePixel(i,j) = p;	
			}
			// magenta
			else if ((p.r>3*p.g) && (p.b > 2*p.g)) {
				
			}
			// light blue
			else if ((p.b > 2.5*p.r) && (p.b > 1.5*p.g) && (p.b>70)) {
				this->trackImg_ptr->safePixel(i,j) = p;	
			}			
		}
	}*/
	int offset = 3;
	bool cont;
	double xAve = 0;
	double yAve = 0;
	Gui g;
	int count = 0;
	/*for (j = offset; j< _img_ptr->height()-offset; j=j+(int)((double)offset/2.0)) {
		for (i = offset; i< _img_ptr->width()-offset; i=i+(int)((double)offset/2.0)) {
	for (j = offset; j< _img_ptr->height()-offset; j=j++) {
		for (i = offset; i< _img_ptr->width()-offset; i=i++) {*/
	for (j = offset; j< _img_ptr->height()-offset; j=j+2) {
		for (i = offset; i< _img_ptr->width()-offset; i=i+2) {

			p	= _img_ptr->pixel(i,j);

			// if the pixel has anyway not the color wanted, do not any computation
			if ((this->isYellow(&p)) || (this->isRed(&p))) {

				p_1 = _img_ptr->pixel(i,j-offset);
				p1	= _img_ptr->pixel(i,j+offset);

				cont = transition(&p_1, &p1, "yellowred");
				//cont = transition(p_1, p1, "redblue");

				if (cont) {
					int offset2 = 7;
				  for (int dx=-offset2; dx<=offset2; dx++) {
				    for (int dy=-offset2; dy<=offset2; dy++) {
				      mask.safePixel(i+dx,j+dy) = 255;
				    }
				  }
					xAve += i;
					yAve += j;
					count++;

					if (count<199) {
						xEllipse.push_back(i);
						yEllipse.push_back(j);
					}

					this->trackImg_ptr->safePixel(i,j-offset)	= p_1;
					this->trackImg_ptr->safePixel(i,j-2)	= _img_ptr->pixel(i,j-2);
					this->trackImg_ptr->safePixel(i,j)		= p;
					this->trackImg_ptr->safePixel(i,j+2)	= _img_ptr->pixel(i,j+2);
					this->trackImg_ptr->safePixel(i,j+offset)	= p1;
				}

				p_1 = _img_ptr->pixel(i-offset,j);
				p1	= _img_ptr->pixel(i+offset,j);

				cont = transition(&p_1, &p1, "yellowred");
				//cont = transition(p_1, p1, "redblue");

				if (cont) {
					this->trackImg_ptr->safePixel(i-offset,j)	= p_1;
					this->trackImg_ptr->safePixel(i-2,j)	= _img_ptr->pixel(i-2,j);
					this->trackImg_ptr->safePixel(i,j)		= p;
					this->trackImg_ptr->safePixel(i+2,j)	= _img_ptr->pixel(i+2,j);
					this->trackImg_ptr->safePixel(i+offset,j)	= p1;
				}
			} // end if is specified color
		} // end for i
	} // end for j

	if (count>0) {
		xAve /= count;
		yAve /= count;
	}

	LabelImage labeller;
	int biggest = labeller.Apply(mask,label);
	int xlo = this->param_ptr->img_width;
	int xhi = 0;
	int ylo = this->param_ptr->img_height;
	int yhi = 0;
	IMGFOR(*(this->trackImg_ptr),xx,yy) {
	  if (label(xx,yy)==biggest) {
	    //addCircle(*(this->trackImg_ptr),PixelRgb(0,0,255),
	    //      xx, yy, 4);
		xlo = MIN(xlo,xx);
		xhi = MAX(xhi,xx);
		ylo = MIN(ylo,yy);
		yhi = MAX(yhi,yy);
	  }
	}

	bgRange b;
	b.col.x = xlo;
	b.col.y = xhi;
	b.row.x = ylo;
	b.row.y = yhi;
	this->roi = b;
	pos.x = (int)xAve;
	pos.y = (int)yAve;
	return pos;
}

bgRange* HandTracker::getROI() {
	return &this->roi;
}

bool HandTracker::transition(PixelRgb *_p1_ptr, PixelRgb *_p2_ptr, string _fromTo) {
	bool transition = false;
	bool R1, R2, Y1, Y2;
	int transitionDiffGreen = 80;
	
	// red <-> yellow
	if (abs(_p1_ptr->g-_p2_ptr->g) > transitionDiffGreen) {
		if (_fromTo == "yellowred" ) {
			Y1 = this->isYellow(_p1_ptr);
			R1 = this->isRed(_p1_ptr);
 			if (R1) {
				Y2 = this->isYellow(_p2_ptr);
 				if (Y2) {
					transition = true;	
					//printf("rp.g - yp.g = %d\n", _p1.g-_p2.g); 
				}
			}
			else if (Y1) {
				R2 = this->isRed(_p2_ptr);
				if (R2) {
					transition = true;	
					//printf("rp.g - yp.g = %d\n", _p1.g-_p2.g); 
				}
			}
		}
	}
	// red <-> lightblue
	/*else if (_fromTo == "redblue") {
		B1 = ((_p1.b > 2.5*_p1.r) && (_p1.b > 1.5*_p1.g) && (_p1.b>70));
		B2 = ((_p2.b > 2.5*_p2.r) && (_p2.b > 1.5*_p2.g) && (_p2.b>70));
 		if (R1) {
 			if (B2) {
				transition = true;	
			}
		}
 		else if (B1) {
 			if (R2) {
				transition = true;	
			}
		}
	}
	else {
		transition = false;
	}*/
	return transition;
}

bool HandTracker::isYellow(PixelRgb *_p1_ptr) {
	if (((_p1_ptr->g > 0.75*_p1_ptr->r) || (_p1_ptr->r > 0.75*_p1_ptr->g)) && (_p1_ptr->g > 1.5*_p1_ptr->b)) {
		return true;
	}
	else {
		return false;
	}
}

bool HandTracker::isRed(PixelRgb *_p1_ptr) {
	if ((_p1_ptr->r > 3*_p1_ptr->g) && (_p1_ptr->r > 2*_p1_ptr->b) && (_p1_ptr->r>30)) {
		return true;
	}
	else {
		return false;
	}
}

bool HandTracker::isBlue(PixelRgb *_p1_ptr) {
	if ((_p1_ptr->b > 2.5*_p1_ptr->r) && (_p1_ptr->b > 1.5*_p1_ptr->g) && (_p1_ptr->b>70)) {
		return true;
	}
	else {
		return false;
	}
}

