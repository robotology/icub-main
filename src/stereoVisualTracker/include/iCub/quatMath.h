#ifndef _QUAT_MATH_H_
#define _QUAT_MATH_H_

#include <cmath>
#include <cstdlib>

#ifdef PIf
#undef PIf
#endif 
#define PIf 3.141592653589793f

#ifdef vec3
	#undef vec3
#endif

class vec3
{
public:
	union{
		struct {
			float x, y, z;
		};
		float _[3];
	};
	vec3() : x(0), y(0), z(0){};
	vec3(float x, float y, float z) : x(x), y(y), z(z){};
	//vec3(int x, int y, int z) : x(float(x)), y(float(y)), z(float(z)){};
	//vec3(double x, double y, double z) : x(float(x)), y(float(y)), z(float(z)){};

	//operators
	vec3 operator+ (const vec3 &v) const { return vec3(x + v.x, y + v.y, z + v.z); }
	vec3 operator- (const vec3 &v) const { return vec3(x - v.x, y - v.y, z - v.z); }
	vec3 operator* (const vec3 &v) const { return vec3(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x); }
	vec3& operator+= (const vec3 &v) { x += v.x; y += v.y; z += v.z; return *this; }
	vec3& operator-= (const vec3 &v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
	vec3& operator*= (const vec3 &v) { vec3 nv = vec3(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x); x = nv.x; y = nv.y; z = nv.z; return *this; }

	vec3 operator+ (const float &d) const { return vec3(x + d, y + d, z + d); }
	vec3 operator- (const float &d) const { return vec3(x - d, y - d, z - d); }
	vec3 operator* (const float &d) const { return vec3(x * d, y * d, z * d); }
	vec3 operator/ (const float &d) const { return vec3(x / d, y / d, z / d); }
	vec3& operator+= (const float &d) { x += d; y += d; z += d; return *this; }
	vec3& operator-= (const float &d) { x -= d; y -= d; z -= d; return *this; }
	vec3& operator*= (const float &d) { x *= d; y *= d; z *= d; return *this; }
	vec3& operator/= (const float &d) { x /= d; y /= d; z /= d; return *this; }

	float dot(const vec3 &v) const { return x*v.x + y*v.y + z*v.z; }
	float norm() const { return sqrtf(x*x + y*y + z*z); }
	float sqnorm() const { return x*x + y*y + z*z; }
	void normalize() { float n = norm(); x /= n;	y /= n;	z /= n; }
	vec3 normalized() { float n = norm(); return vec3(x / n, y / n, z / n); }

	static vec3 randvec() { return vec3( (rand()/(float)RAND_MAX)*2-1, (rand()/(float)RAND_MAX)*2-1, (rand()/(float)RAND_MAX)*2-1 ); }
};

class Quat
{
public:
	union{
		struct {
			float w, x, y, z;
		};
		float _[3];
	};
	Quat() : w(0), x(0), y(0), z(0){};
	Quat(int w, int x, int y, int z) : w(float(w)), x(float(x)), y(float(y)), z(float(z)){};
	Quat(float w, float x, float y, float z) : w(w), x(x), y(y), z(z){};
	Quat(double w, double x, double y, double z) : w(float(w)), x(float(x)), y(float(y)), z(float(z)){};
	Quat(vec3 v): w(0), x(v.x), y(v.y), z(v.z){};
	//Quat(vec3 v, float theta): w(cosf(theta/2)), x(v.x*sinf(theta/2)), y(v.y*sinf(theta/2)), z(v.z*sinf(theta/2)){
	Quat(vec3 v, float theta){
		w = cosf(theta/2);
		x = v.x*sinf(theta/2);
		y = v.y*sinf(theta/2);
		z = v.z*sinf(theta/2);
	}

	Quat operator+ (const Quat &q) const { return Quat(w+q.w, x+q.x, y+q.y, z+q.z); }
	Quat operator- (const Quat &q) const { return Quat(w-q.w, x-q.x, y-q.y, z-q.z); }
	Quat& operator+= (const Quat &q) { w += q.w; x += q.x; y += q.y; z += q.z; return *this; }
	Quat& operator-= (const Quat &q) { w -= q.w; x -= q.x; y -= q.y; z -= q.z; return *this; }

	Quat operator* (const Quat &q) const {
		return Quat(
			w*q.w - x*q.x - y*q.y - z*q.z,
			w*q.x + x*q.w + y*q.z - z*q.y,
			w*q.y - x*q.z + y*q.w + z*q.x,
			w*q.z + x*q.y - y*q.x + z*q.w
			);
	}

	Quat& operator*= (const Quat &q) {
		Quat nq = Quat(
			w*q.w - x*q.x - y*q.y - z*q.z,
			w*q.x + x*q.w + y*q.z - z*q.y,
			w*q.y - x*q.z + y*q.w + z*q.x,
			w*q.z + x*q.y - y*q.x + z*q.w
			);
		w = nq.w; x=nq.x; y=nq.y; z=nq.z;
		return *this;
	}

	Quat operator+ (const float &d) const { return Quat(w + d, x + d, y + d, z + d); }
	Quat operator- (const float &d) const { return Quat(w - d, x - d, y - d, z - d); }
	Quat operator* (const float &d) const { return Quat(w * d, x * d, y * d, z * d); }
	Quat operator/ (const float &d) const { return Quat(w / d, x / d, y / d, z / d); }
	Quat& operator+= (const float &d) { w += d; x += d; y += d; z += d; return *this; }
	Quat& operator-= (const float &d) { w -= d; x -= d; y -= d; z -= d; return *this; }
	Quat& operator*= (const float &d) { w *= d; x *= d; y *= d; z *= d; return *this; }
	Quat& operator/= (const float &d) { w /= d; x /= d; y /= d; z /= d; return *this; }

	float dot(const Quat &q) const { return w*q.w + x*q.x + y*q.y + z*q.z; }
	Quat conj() const{ return Quat(w, -x, -y, -z); }
	float norm() const { return sqrtf(w*w + x*x + y*y + z*z); }
	float sqnorm() const { return w*w + x*x + y*y + z*z; }
	void normalize() { float n = norm(); w /= n;	x /= n;	y /= n;	z /= n; }
	Quat normalized() { float n = norm(); return Quat(w / n, x / n, y / n, z / n); }
	vec3 vec() const { return vec3(x,y,z); }

	// rotate current quaternion by another quaternion
	vec3 rotate(const vec3 &v){if(w==0) return v; Quat nq = (*this)*Quat(v)*(*this).conj(); return vec3(nq.x, nq.y, nq.z); }

	void toMatrix(float *out){
	  float x2,y2,z2,xx,yy,zz,xy,yz,xz,wx,wy,wz;
	  x2 = 2 * x; xx = x * x2;  xy = x * y2; wx = w * x2;
	  y2 = 2 * y; yy = y * y2;  yz = y * z2; wy = w * y2;
	  z2 = 2 * z; zz = z * z2;  xz = z * x2; wz = w * z2;
	  out[0] = 1.0f - (yy + zz);
	  out[4] = 1.0f - (xx + zz);
	  out[8] = 1.0f - (xx + yy);
	  out[3] = xy + wz;
	  out[6] = xz - wy;
	  out[1] = xy - wz;
	  out[7] = yz + wx;
	  out[2] = xz + wy;
	  out[5] = yz - wx;
	}

	float fromMatrix(float *m){
	  float angle,f;
	  f =  0.5*(m[0]+m[4]+m[8]-1);
	  if(abs(f)>1){return 0;}
	  angle = acos(f);
	  if(abs(angle)>0){
	    f = sin(0.5*angle)/(2*sin(angle));
	    x = (m[7]-m[5])*f;
	    y = (m[2]-m[6])*f;
	    z = (m[3]-m[1])*f;
	    w = 1-x*x-y*y-z*z;
	  }
	  return angle;
	}
};

#endif //_QUAT_MATH_H_
