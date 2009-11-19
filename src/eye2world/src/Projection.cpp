#include "iCub/vislab/Projection.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

using namespace ctrl;

using namespace vislab::yarp;

//#define VERBOSE
#define DEBUG

#if defined (VERBOSE) || defined(DEBUG)
#include <vislab/yarp/util/all.h>
using namespace vislab::yarp::util;
#endif

namespace vislab {
namespace math {


Projection::Projection(const CameraCalibration& c, const Vector* offset) {
	this->offset = offset;
	cameraCalibration = c;
	heightOffset = 0.0;
	forceRecomputation = true;
	H = eye(3, 3);
	T_rt = eye(4, 4);
	T_cr = eye(4, 4);
}

void Projection::setHeightOffset(double d, bool forceRecompuation) {
	if (heightOffset != d) {
		heightOffset = d;
		if (forceRecompuation) {
			setBaseTransformation(T_cr);
		} else {
			this->forceRecomputation = true;
		}
	}
}

void Projection::setBaseTransformation(const Matrix& T) { // T: cam <- robot
	if (T_cr != T || forceRecomputation) {
		// body <- table transformation
		T_rt = eye(4, 4);
		if (offset != NULL && offset->size() >= 3) {
			T_rt[0][3] = -(*offset)[0];
			T_rt[1][3] = -(*offset)[1];
			T_rt[2][3] = -(*offset)[2];
		}
		T_rt[2][3] -= heightOffset;
		T_rt = SE3inv(T_rt);

		Matrix T_ct = T * T_rt; // T_cr
		// cancel 3rd column...
		for (int r = 0; r < T_ct.rows(); r++) {
			T_ct[r][2] = T_ct[r][3];
		}
		// ...and 3rd line due to coordinate definitions
		T_ct = T_ct.submatrix(0, 2, 0, 2);

		H = luinv(T_ct);
		T_cr = T;

#ifdef VERBOSE
		printMatrix(T_ct);
		printMatrix(T_ct);
		printMatrix(H);
		printMatrix(this->T);
#endif
	}
}

void Projection::project(const Vector &in, Vector &out) {

	if (in.size() < 2) {
		throw "The input vector has to contain 2D coordinates";
	}

	Vector p_c(3);
	p_c[0] = (in[0] - cameraCalibration.cx) / cameraCalibration.fx; // x_c/z_c
	p_c[1] = (in[1] - cameraCalibration.cy) / cameraCalibration.fy; // y_c/z_c
	p_c[2] = 1; // z_c/z_c

	Vector p_t = H * p_c;
	p_t.resize(p_t.length() + 1);

	// points at the "additional offset plane"
	p_t[0] = p_t[0] / p_t[2];
	p_t[1] = p_t[1] / p_t[2];
	p_t[2] = 0;
	p_t[3] = 1;

	out = T_rt * p_t;
	out.resize(3);

#ifdef VERBOSE
	printVector(p_c);
	printVector(p_t);
	printVector(out);
#endif

}

}
}
