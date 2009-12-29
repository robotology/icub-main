#include "Matrix3.h"
#include <math.h>
#ifdef USE_MATHLIB_NAMESPACE
using namespace MathLib;
#endif


const Matrix3 Matrix3::ZERO(
	R_ZERO, R_ZERO, R_ZERO,
	R_ZERO, R_ZERO, R_ZERO,
	R_ZERO, R_ZERO, R_ZERO);

const Matrix3 Matrix3::IDENTITY(
	R_ONE , R_ZERO, R_ZERO,
	R_ZERO, R_ONE , R_ZERO,
	R_ZERO, R_ZERO, R_ONE );


Matrix3& Matrix3::RotationX(REALTYPE angleX)
{
	REALTYPE sx, cx;
	sx = sinf(angleX);
	cx = cosf(angleX);
	_[0][0] = R_ONE;
	_[0][1] = R_ZERO;
	_[0][2] = R_ZERO;
	_[1][0] = R_ZERO;
	_[1][1] = cx;
	_[1][2] = -sx;
	_[2][0] = R_ZERO;
	_[2][1] = sx;
	_[2][2] = cx;
  return *this;  
}

Matrix3& Matrix3::RotationY(REALTYPE angleY)
{
	REALTYPE sy, cy;
	sy = sinf(angleY);
	cy = cosf(angleY);
	_[0][0] = cy;
	_[0][1] = R_ZERO;
	_[0][2] = sy;
	_[1][0] = R_ZERO;
	_[1][1] = R_ONE;
	_[1][2] = R_ZERO;
	_[2][0] = -sy;
	_[2][1] = R_ZERO;
	_[2][2] = cy;
  return *this;
}

Matrix3& Matrix3::RotationZ(REALTYPE angleZ)
{
	REALTYPE sz, cz;
	sz = sinf(angleZ);
	cz = cosf(angleZ);
	_[0][0] = cz;
	_[0][1] = -sz;
	_[0][2] = R_ZERO;
	_[1][0] = sz;
	_[1][1] = cz;
	_[1][2] = R_ZERO;
	_[2][0] = R_ZERO;
	_[2][1] = R_ZERO;
	_[2][2] = R_ONE;
  return *this;
}


Matrix3& Matrix3::RotationYXZ(REALTYPE angleX, REALTYPE angleY, REALTYPE angleZ)
{
  REALTYPE sx, cx, sy, cy, sz, cz;
  sx = sinf(angleX);
  cx = cosf(angleX);
  sy = sinf(angleY);
  cy = cosf(angleY);
  sz = sinf(angleZ);
  cz = cosf(angleZ);
  _[0][0] = cy * cz + sx * sy * sz;
  _[0][1] = -cy * sz + sx * sy * cz;
  _[0][2] = cx * sy;
  _[1][0] = cx * sz;
  _[1][1] = cx * cz;
  _[1][2] = -sx;
  _[2][0] = -sy * cz + sx * cy * sz;
  _[2][1] = sy * sz + sx * cy * cz;
  _[2][2] = cx * cy;
  return *this;
}

Matrix3& Matrix3::RotationV(REALTYPE angle, const Vector3 &axis)
{    
  REALTYPE n = axis.Norm();
  if(n<=EPSILON){
    Identity();      
  }else{
    n = R_ONE/n;  
    const REALTYPE c = cosf(angle);
    const REALTYPE s = sinf(angle);
    const REALTYPE u = (1-c);
    const REALTYPE x = axis._[0]*n;
    const REALTYPE y = axis._[1]*n;
    const REALTYPE z = axis._[2]*n;
    
    _[0][0] = x*x*u+  c;
    _[0][1] = x*y*u-z*s;
    _[0][2] = x*z*u+y*s;
    _[1][0] = x*y*u+z*s;
    _[1][1] = y*y*u+  c;
    _[1][2] = y*z*u-x*s;
    _[2][0] = x*z*u-y*s;
    _[2][1] = y*z*u+x*s;
    _[2][2] = z*z*u+  c;
  }
  return *this;
}

Vector3& Matrix3::GivensRotationPlane(REALTYPE a, REALTYPE b, Vector3 & result, int path){
  REALTYPE p = (path==0?1.0:-1.0);
  if(b == R_ZERO){    
    result[0] = SIGN(a);
    result[1] = R_ZERO;
    result[2] = fabs(a);
  }else if(a== R_ZERO){
    result[0] = R_ZERO;
    result[1] = SIGN(b);
    result[2] = fabs(b);    
  }else if (fabs(b)>fabs(a)){
    REALTYPE t  = a/b;
    REALTYPE u  = p*SIGN(b)*sqrt(R_ONE+t*t);
    result[1]   = R_ONE/u;
    result[0]   = result[1]*t;
    result[2]   = b*u;
  }else{
    REALTYPE t = b/a;
    REALTYPE u = p*SIGN(a)*sqrt(R_ONE+t*t);
    result[0]  = R_ONE/u;
    result[1]  = result[0]*t;
    result[2]  = a*u;
  }

  return result;
}

Vector3& Matrix3::GetEulerAnglesGeneric(int i, int neg, int alt, int rev, Vector3& result, int path){
  int j,k,h;
  j = (i+neg +1)%3;
  k = 3-i-j;
  h = ((k+ ((1^neg)^alt))+1)%3;

  //cout <<"("<<i<<" "<< neg<<" "<< alt<<") ";  
  //cout << i<<" "<< j<<" "<< k<<" "<< h<<endl;
  Vector3 v;
  v[0] = _[0][i]; v[1] = _[1][i]; v[2] = _[2][i];
    
  Vector3 givens; 
  GivensRotationPlane(v[h],v[k],givens,path);
  v[h] = givens[2];    
  REALTYPE s1 = givens[0] * _[k][j] - givens[1] * _[h][j];
  REALTYPE c1 = givens[0] * _[k][k] - givens[1] * _[h][k];
  
  result[0] = atan2(       s1,        c1);  
  result[1] = atan2(     v[j],      v[i]);
  result[2] = atan2(givens[1], givens[0]);
  
  if(alt==1) result[2] = -result[2];
  if(neg==1) result    = -result;
  if((rev==1)){
    REALTYPE tmp = result[0];
    result[0] = result[2];
    result[2] = tmp;  
  }
  return result;
}

Matrix3& Matrix3::EulerRotation(int axis0, int axis1, int axis2, const Vector3& angles){
  Matrix3 R0,R1,R2;
  switch(axis0) {
  case 1:  RotationY(angles.cx()); break;
  case 2:  RotationZ(angles.cx()); break;
  case 0: 
  default: RotationX(angles.cx()); break;
  }  
  switch(axis1) {
  case 1:  R0.RotationY(angles.cy()); break;
  case 2:  R0.RotationZ(angles.cy()); break;
  case 0: 
  default: R0.RotationX(angles.cy()); break;
  }  
  Mult(R0,R1);  
  switch(axis2) {
  case 1:  R0.RotationY(angles.cz()); break;
  case 2:  R0.RotationZ(angles.cz()); break;
  case 0: 
  default: R0.RotationX(angles.cz()); break;
  }  
  R1.Mult(R0,*this);
  return *this;
}
