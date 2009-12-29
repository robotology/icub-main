#ifndef __MATHLIB_H__
#define __MATHLIB_H__


#define  USE_T_EXTENSIONS

#include "MathLibCommon.h"

#include "Macros.h"

#include "Vector.h"

#ifdef  USE_T_EXTENSIONS
#include "TVector.h"
#include "Vector3.h"
#endif


#include "Matrix.h"

#include "Quaternion.h"
/*
#include "SpatialVector.h"
#include "SpatialMatrix.h"
#include "SpatialFrame.h"
#include "SpatialForce.h"
#include "SpatialInertia.h"
#include "SpatialVelocity.h"
*/
#ifdef USE_T_EXTENSIONS
#include "TMatrix.h"
#include "Matrix3.h"
#include "Matrix4.h"
#include "Referential.h"
#include "ReferenceFrame.h"
#endif

#include "Differentiator.h"
#include "SplineFit.h"
#include "Regression.h"
#include "DTW.h"

#endif

