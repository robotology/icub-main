#pragma once

#ifndef M_PI
	#define M_PI 3.14159265358979323846264338327
#endif

inline double radians(double degAngle)
{
	return degAngle * M_PI / 180;
}

inline double degrees(double radAngle)
{
	return radAngle * 180 / M_PI;
}

inline double orientedAngle(const Vector &v1, const Vector &v2)
{
	double perpDot = v1[0] * v2[1] - v1[1] * v2[0];
	double dot = v1[0] * v2[0] + v1[1] * v2[1];
 
	return degrees(atan2(perpDot,dot));
}

inline void Divide(Vector &v, double d)
{
	if(d == 0)
	{
		return;
	}
	for(int i=0; i<v.size(); ++i)
	{
		v[i] /= d;
	}
}

inline void Normalize(Vector &v)
{
	if(v[0] == 0 && v[1] == 0)
	{
		return;
	}
	double d = norm(v);
	Divide(v, d);
}

inline Vector Normalize(const Vector &v)
{
	Vector normalizedV(v);
	Divide(normalizedV, norm(normalizedV));
	return normalizedV;
}