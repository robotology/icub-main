// include header
#include <imageprocessing.h>

// namespaces
using namespace thesis::imageprocessing::imgproc_helpers;

// ***************************************************************************

/**
 * Implementation of Helpers class
 *
 * This class is a pool of methods /functionality that can serve every module. 
 *
 */

Helpers::Helpers() {}

Helpers::~Helpers() {}

PixelRgb Helpers::binThreshold(PixelRgb _p, int _threshold) {
	PixelRgb p = _p;
	int sum = 0;
	sum = ROUNDTOINT((p.r + p.g + p.b)/3);
	if (sum < _threshold) {
		p.r = 0;
		p.g = 0;
		p.b = 0;
	} else {
		p.r = 255;
		p.g = 255;
		p.b = 255;
	}
	return p;
}

PixelRgb Helpers::threshold(PixelRgb _p, int _threshold) {
	PixelRgb p = _p;
	int sum = 0;
	sum = ROUNDTOINT((p.r + p.g + p.b)/3);
	if (sum < _threshold) {
		p.r = 0;
		p.g = 0;
		p.b = 0;
	}
	return p;
}

bool Helpers::withinBorders(int _i, int _j, int _w, int _h) {
	bool res = false;
	if (( _j >= 0 ) & ( _j < _h)) {
		if (( _i >= 0 ) & ( _i < _w)) {
			res = true;
		}
	}
	return res;
}

bool Helpers::withinBorders(iPoint2D _pos, iPoint2D _size ) {
	bool res = false;
	if (( _pos.y >= 0 ) & ( _pos.y < _size.y)) {
		if (( _pos.x >= 0 ) & ( _pos.x < _size.x)) {
			res = true;
		}
	}
	return res;
}


