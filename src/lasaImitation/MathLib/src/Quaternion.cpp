#include "Quaternion.h"
#ifdef USE_MATHLIB_NAMESPACE
using namespace MathLib;
#endif

const Quaternion Quaternion::ZERO(R_ZERO, R_ZERO, R_ZERO, R_ZERO);
const Quaternion Quaternion::IDENTITY(R_ONE, R_ZERO, R_ZERO, R_ZERO);


Quaternion& Quaternion::Rotation(const Vector3& axis, REALTYPE angle)
{
	REALTYPE s, c;
	angle = angle * 0.5;
    s   = sin(angle);
    c   = cos(angle);
    REALTYPE n = axis.Norm();
    if(n<1e-6){
        Identity();    
    }else{
        s = s/n;
    	for (int i = 0; i < 3; i++)
    		_[i] = axis._[i] * s;
    	w = c;
    }
    return *this;
}
Quaternion& Quaternion::Rotation(const Vector3& axisAngle){
    REALTYPE angle = axisAngle.Norm();
    if(angle<1e-6){
        Identity();    
    }else{
        REALTYPE iAngle = 1.0/angle;
        REALTYPE s = iAngle * sin(angle*0.5);
        REALTYPE c = cos(angle*0.5);
        for (int i = 0; i < 3; i++)
            _[i] = axisAngle._[i] * s;
        _[3] = c;
    }
}

Quaternion& Quaternion::RotationX(REALTYPE angle)
{
	angle = angle * 0.5;
	x = sin(angle);
	y = R_ZERO;
	z = R_ZERO;
	w = cos(angle);
    return *this;
}

Quaternion& Quaternion::RotationY(REALTYPE angle)
{
	angle = angle * 0.5;
	x = R_ZERO;
	y = sin(angle);
	z = R_ZERO;
	w = cos(angle);
    return *this;
}

Quaternion& Quaternion::RotationZ(REALTYPE angle)
{
	angle = angle * 0.5;
	x = R_ZERO;
    y = R_ZERO;
	z = sin(angle);
	w = cos(angle);
    return *this;
}

Quaternion& Quaternion::Rotation(const Matrix3 &m)
{
    Vector3 v;
    m.GetExactRotationAxis(v);
    Rotation(v);
    return *this;
}

Quaternion& Quaternion::Slerp(const Quaternion &target, REALTYPE t, Quaternion &result) const{
    t = TRUNC(t,R_ZERO,R_ONE);
    
    Quaternion cTarget(target);
    
    REALTYPE c = Dot(target);
    if(c<R_ZERO){
        cTarget.SMinus();
        c = -c;    
    }
    
    REALTYPE s0,s1;
    if((R_ONE-c)<1e-6){
        s0 = R_ONE-t;
        s1 = t;        
    }else{
        REALTYPE angle = acos(c);
        REALTYPE s = R_ONE / sin(angle);
        s0 = sin((R_ONE-t) * angle) * s; 
        s1 = sin( t * angle) * s; 
    }
    for (int i = 0; i < 4; i++)
        result._[i] = _[i] * s0 + target._[i] * s1;
    
    return result;
}

