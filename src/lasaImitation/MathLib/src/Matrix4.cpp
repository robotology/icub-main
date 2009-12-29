#include "Matrix4.h"
#ifdef USE_MATHLIB_NAMESPACE
using namespace MathLib;
#endif

const Matrix4 Matrix4::ZERO(
	R_ZERO, R_ZERO, R_ZERO, R_ZERO,
	R_ZERO, R_ZERO, R_ZERO, R_ZERO,
	R_ZERO, R_ZERO, R_ZERO, R_ZERO,
	R_ZERO, R_ZERO, R_ZERO, R_ZERO);

const Matrix4 Matrix4::IDENTITY(
	R_ONE , R_ZERO, R_ZERO, R_ZERO,
	R_ZERO, R_ONE , R_ZERO, R_ZERO,
	R_ZERO, R_ZERO, R_ONE , R_ZERO,
	R_ZERO, R_ZERO, R_ZERO, R_ONE );

