// include header
#include <Mathematics.h>

// namespaces
using namespace thesis::mathematics;

using std::sort;

// ***************************************************************************

/**
 * Implementation of Geometry class
 *
 * This "module" is working with .
 */

// Constructor
Geometry::Geometry() {
	//printf("Start:\t[Geometry]\n");
	this->double_tmp = 0.0;
	this->int_tmp = 0;
	this->bottle_tmp = Bottle ();
	this->bottle_tmp.clear();
}

// Destructor
Geometry::~Geometry() {
	//printf("Quit:\t[Geometry]\n");
}

double Geometry::median(double *_array_ptr, int _array_size) {
	double res;
	//printf("<sort:= ");
	sort(_array_ptr, _array_ptr + _array_size);
	//for (int i=0; i<_array_size; i++) {
	//	printf("[%f] ", _array_ptr[i]);
	//}
	//printf(">\n");
	if ((_array_size % 2) == 1) {
		res = _array_ptr[(_array_size-1)/2];
	}
	else {
		double i1, i2;
		i1 = _array_ptr[(_array_size)/2 -1];
		i2 = _array_ptr[(_array_size)/2 ];
		res = (i1 + i2)/2;
		printf("array size = %d, i1 = %f, i2 = %f, sum = %f, res = %f \n", _array_size,  i1, i2, i1 + i2, res);
	}
	printf("return: %f\n", res);
	return res;
}

double Geometry::median(Bottle *_bottle_ptr) {
	double res;
	//printf("<sort:= ");
	double *_array_ptr = new double [_bottle_ptr->size()];
	for (int i=0; i<_bottle_ptr->size(); i++) {
		_array_ptr[i] = _bottle_ptr->get(i).asDouble();
		//printf("[%f] ", _array_ptr[i]);
	}
	//printf(">\n");
	int _array_size = _bottle_ptr->size();
	sort(_array_ptr, _array_ptr + _array_size);
	if ((_array_size % 2) == 1) {
		res = _array_ptr[(_array_size-1)/2];
	}
	else {
		double i1, i2;
		i1 = _array_ptr[(_array_size)/2 -1];
		i2 = _array_ptr[(_array_size)/2 ];
		res = (i1 + i2)/2;
		//printf("array size = %d, i1 = %f, i2 = %f, sum = %f, res = %f \n", _array_size,  i1, i2, i1 + i2, res);
	}
	//printf("return: %f\n", res);
	delete [] _array_ptr;
	return res;
}

Bottle Geometry::lowPass(double *_array_ptr, int _array_size) {
	double val;
	Bottle b;
	for (int i = 0; i<_array_size; i++) {
		if ((i >= 1) & (i < _array_size-1)) {
			val = (double)(_array_ptr[i-1] + _array_ptr[i] + _array_ptr[i+1])/3;
		}
		else if (i == 0) {
			val = (double)(_array_ptr[i] + _array_ptr[i+1])/2;
		}
		else {
			val = (double)(_array_ptr[i-1] + _array_ptr[i])/2;
		}
		b.addDouble(val);
		//printf("[%f] ", b.get(i).asDouble());
	}
	//printf("\n");
	this->bottle_tmp.copy(b);
	return b;
}

int Geometry::getAngle(dPoint2D _v1, dPoint2D _v2) {
	double angle = 0.0;
	double amount_v1, amount_v2, y_, scalar_prod;
	int angle_grad = 0;
	dPoint2D delta;

	delta.x = _v1.x - _v2.x;
	delta.y = _v1.y - _v2.y;

	if ((delta.x != 0.0) | ( delta.y != 0.0)) {
		scalar_prod = (double)((_v1.x*_v2.x)+(_v1.y*_v2.y));
		amount_v1	= (double)sqrt((_v1.x*_v1.x + _v1.y*_v1.y));
		amount_v2	= (double)sqrt((_v2.x*_v2.x + _v2.y*_v2.y));
		y_			= scalar_prod / (amount_v1 * amount_v2);
		angle = acos(y_);
		angle_grad = ROUNDTOINT(180*angle/PI);
	}
	return angle_grad;
}

int Geometry::getAngle(iPoint2D _v1, iPoint2D _v2) {
	double angle = 0.0;
	double amount_v1, amount_v2, y_, scalar_prod;
	int angle_grad = 0;
	iPoint2D delta;

	delta.x = _v1.x - _v2.x;
	delta.y = _v1.y - _v2.y;

	if ((delta.x != 0) | ( delta.y != 0)) {
		scalar_prod = (double)((_v1.x*_v2.x)+(_v1.y*_v2.y));
		amount_v1	= (double)sqrt((_v1.x*_v1.x + _v1.y*_v1.y));
		amount_v2	= (double)sqrt((_v2.x*_v2.x + _v2.y*_v2.y));
		y_			= scalar_prod / (amount_v1 * amount_v2);
		angle = acos(y_);
		angle_grad = ROUNDTOINT(180*angle/PI);
	}

	return angle_grad;
}

int Geometry::getAngle(iPoint2D _v1) {
	double angle = 0.0;
	double amount_v1, amount_v2, y_, scalar_prod;
	int angle_grad = 0;
	iPoint2D delta, _v2;
	_v2.x = 1;
	_v2.y = 0;

	delta.x = _v1.x - _v2.x;
	delta.y = _v1.y - _v2.y;

	if ((delta.x != 0) | ( delta.y != 0)) {
		scalar_prod = (double)((_v1.x*_v2.x)+(_v1.y*_v2.y));
		amount_v1	= (double)sqrt((_v1.x*_v1.x + _v1.y*_v1.y));
		amount_v2	= (double)sqrt((_v2.x*_v2.x + _v2.y*_v2.y));
		y_			= scalar_prod / (amount_v1 * amount_v2);
		angle = acos(y_);
		angle_grad = ROUNDTOINT(180*angle/PI);
	}

	return angle_grad;
}

int Geometry::getAngle(dPoint2D _v1) {
	double angle = 0.0;
	double amount_v1, amount_v2, y_, scalar_prod;
	int angle_grad = 0;
	dPoint2D delta, _v2;
	_v2.x = 1.0;
	_v2.y = 0.0;

	delta.x = _v1.x - _v2.x;
	delta.y = _v1.y - _v2.y;

	if ((delta.x != 0.0) | ( delta.y != 0.0)) {
		scalar_prod = (double)((_v1.x*_v2.x)+(_v1.y*_v2.y));
		amount_v1	= (double)sqrt((_v1.x*_v1.x + _v1.y*_v1.y));
		amount_v2	= (double)sqrt((_v2.x*_v2.x + _v2.y*_v2.y));
		y_			= scalar_prod / (amount_v1 * amount_v2);
		angle = acos(y_);
		angle_grad = ROUNDTOINT(180*angle/PI);
	}

	return angle_grad;
}

double Geometry::getArcCos(dPoint2D _v1) {
	double angle = 0.0;
	double amount_v1, amount_v2, y_, scalar_prod;
	int angle_grad = 0;
	dPoint2D delta, _v2;
	_v2.x = 1.0;
	_v2.y = 0.0;

	delta.x = _v1.x - _v2.x;
	delta.y = _v1.y - _v2.y;

	if ((delta.x != 0.0) | ( delta.y != 0.0)) {
		scalar_prod = (double)((_v1.x*_v2.x)+(_v1.y*_v2.y));
		amount_v1	= (double)sqrt((_v1.x*_v1.x + _v1.y*_v1.y));
		amount_v2	= (double)sqrt((_v2.x*_v2.x + _v2.y*_v2.y));
		y_			= scalar_prod / (amount_v1 * amount_v2);
	}

	return y_;
}

