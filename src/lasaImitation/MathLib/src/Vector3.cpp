#include "Vector3.h"
#ifdef USE_MATHLIB_NAMESPACE
using namespace MathLib;
#endif

const Vector3 Vector3::ZERO(R_ZERO, R_ZERO, R_ZERO);
const Vector3 Vector3::EX(R_ONE , R_ZERO, R_ZERO);
const Vector3 Vector3::EY(R_ZERO, R_ONE , R_ZERO);
const Vector3 Vector3::EZ(R_ZERO, R_ZERO, R_ONE );

