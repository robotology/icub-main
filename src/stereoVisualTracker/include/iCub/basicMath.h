/*!
 * \file basicMath.h
 * \author Basilio Noris
 * \date 27-11-06
 */
#ifndef _BASICMATH_H_
#define _BASICMATH_H_

#define PIf	3.1415926535897932384626433832795f
//#define Vec2 CvPoint2D32f
#define Vec3 CvPoint3D32f
#define vec2(a,b) cvPoint2D32f(a,b)
#define vec3(a,b,c) cvPoint3D32f(a,b,c)
#define Truncate(a, b) a = (a>=b ? b : a)
#define point2(a) cvPoint((int)a.x, (int)a.y)

class Vec2 : public CvPoint2D32f{
public:
	float x, y;
	// constructors
	Vec2() : x(0), y(0) {}
	Vec2(int a, int b): x((float)a), y((float)b) {}
	Vec2(float a, float b): x(a), y(b) {}
	Vec2(double a, double b): x((float)a), y((float)b) {}
	Vec2(CvPoint a): x((float)a.x), y((float)a.y) {}
	Vec2(CvPoint2D32f a): x(a.x), y(a.y) {}
	CvPoint2D32f to32f(){return cvPoint2D32f(x,y);}
	CvPoint to2d(){return cvPoint((int)x,(int)y);}

	//operators
	Vec2 operator+(const Vec2 &v) const {
		return Vec2(x + v.x, y + v.y);
	}

	Vec2& operator+=(const Vec2 &v) {
		x += v.x;
		y += v.y;
		return *this;
	}

	Vec2 operator-(const Vec2 &v) const {
		return Vec2(x - v.x, y - v.y);
	}

	Vec2& operator-=(const Vec2 &v) {
		x -= v.x;
		y -= v.y;
		return *this;
	}

	Vec2 operator*(const float &d) const {
		return Vec2(x * d, y * d);
	}

	Vec2& operator*=(const float &d) {
		x *= d;
		y *= d;
		return *this;
	}

	Vec2 operator/(const float &d) const {
		return Vec2(x / d, y / d);
	}

	Vec2& operator/=(const float &d) {
		assert(d!=0);
		float inv = 1.f / d;
		x *= inv;
		y *= inv;
		return *this;
	}

	bool operator==(const Vec2 &v) const {
		return x == v.x && y == v.y;
	}

	Vec2 operator-() const {
		return Vec2(-x, -y);
	}

	// other functions

	Vec2 normalize() { 
		assert(length() > 0);
		float l = length();
		x /= l;
		y /= l;
		return *this;
	}

	float lengthSquared() {
		return x * x + y * y;
	}

	float length() {
		return sqrt(lengthSquared());
	}

	float dot(const Vec2 &v) const {
		return x * v.x + y * v.y;
	}
};

static float Dot(Vec3 a, Vec3 b)
{
	return a.x*b.x + a.y*b.y + a.z*b.z;
}

static Vec3 Mul(Vec3 a, Vec3 b)
{
	return vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

static Vec3 MulS(Vec3 a, float b)
{
	return vec3(a.x * b, a.y * b, a.z * b);
}

static Vec3 Add(Vec3 a, Vec3 b)
{
	return vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

static Vec3 AddS(Vec3 a, float b)
{
	return vec3(a.x + b, a.y + b, a.z + b);
}

static Vec3 Sub(Vec3 a, Vec3 b)
{
	return vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

static float Dot(Vec2 a, Vec2 b)
{
	return a.x*b.x + a.y*b.y;
}

static CvPoint2D32f MulS(Vec2 a, float b)
{
	return vec2(a.x * b, a.y * b);
}

static CvPoint2D32f Add(Vec2 a, Vec2 b)
{
	return vec2(a.x + b.x, a.y + b.y);
}

static CvPoint2D32f AddS(Vec2 a, float b)
{
	return vec2(a.x + b, a.y + b);
}

static CvPoint2D32f Sub(Vec2 a, Vec2 b)
{
	return vec2(a.x - b.x, a.y - b.y);
}

static float Norm(Vec2 a)
{
	return cvSqrt(Dot(a,a));
}

static float Norm(Vec3 a)
{
	return cvSqrt(Dot(a,a));
}

static CvPoint2D32f Normalize(Vec2 a)
{
	float n = Norm(a);
	return vec2(a.x/n, a.y/n);
}

static Vec3 Normalize(Vec3 a)
{
	float n = Norm(a);
	return vec3(a.x/n, a.y/n, a.z/n);
}

static CvPoint2D32f Inv(Vec2 a)
{
	return vec2(-a.x, -a.y);
}

static Vec3 Inv(Vec3 a)
{
	return vec3(-a.x, -a.y, -a.z);
}

static Vec3 Inv(Vec3 a,int b)
{
	return vec3(b==0 ? -a.x : a.x, b==1 ? -a.y : a.y, b==2 ? -a.z : a.z);
}

static Vec3 Swap(Vec3 a, int b, int c)
{
	if (b==0)
	{
		if(c==1) return vec3(a.y, a.x, a.z);
		if(c==2) return vec3(a.z, a.y, a.x);
	}
	if (b==1)
	{
		if(c==0) return vec3(a.y, a.x, a.z);
		if(c==2) return vec3(a.x, a.z, a.y);
	}
	if (b==2)
	{
		if(c==0) return vec3(a.z, a.y, a.x);
		if(c==1) return vec3(a.x, a.z, a.y);
	}
	return a;
}

static Vec3 Cross(Vec3 a, Vec3 b)
{
    return vec3( a.y*b.z - a.z*b.y,
                -a.x*b.z + a.z*b.x,
                 a.x*b.y - a.y*b.x);
}

static int Equals(CvPoint2D32f a, CvPoint2D32f b)
{
	return a.x == b.x && a.y == b.y;
}

static int Equals(Vec3 a, Vec3 b)
{
	return a.x == b.x && a.y == b.y && a.z == b.z;
}

static CvPoint2D32f Floor(CvPoint2D32f a)
{
	return vec2((int)a.x, (int)a.y);
}

static Vec3 Floor(Vec3 a)
{
	return vec3((int)a.x, (int)a.y, (int)a.z);
}

static CvPoint2D32f Boundary(CvPoint2D32f a)
{
	return vec2(MIN(319,MAX(a.x, 0)), MIN(239,MAX(a.y, 0)));
}

static CvPoint2D32f Unzero(CvPoint2D32f a)
{
	return vec2(MAX(a.x, 0), MAX(a.y, 0));
}

static Vec3 Unzero(Vec3 a)
{
	return vec3(MAX(a.x, 0), MAX(a.y, 0), MAX(a.z, 0));
}

static float Distance(CvPoint2D32f a, CvPoint2D32f b)
{
	return (float)sqrt((b.x - a.x)*(b.x - a.x) + (b.y - a.y)*(b.y - a.y));
}

static float Distance(Vec3 a, Vec3 b)
{
	return (float)sqrt((b.x - a.x)*(b.x - a.x) + (b.y - a.y)*(b.y - a.y) + (b.z - a.z)*(b.z - a.z));
}

static u32 *FindMatches(CvPoint2D32f *src, u32 srccnt, CvPoint2D32f *dst, u32 dstcnt){
	f32 *distances = new f32[dstcnt*srccnt];
	FOR(i, dstcnt){
		f32 distotal = 0;
		FOR(j, srccnt){
			distances[i*srccnt + j] = Distance(src[j],dst[i]);
			distotal += distances[i*srccnt + j];
		}
		FOR(j,srccnt){
			distances[i*srccnt + j] /= distotal;
		}
	}
	u32 *indices = new u32[dstcnt];
	FOR(j,dstcnt){
		u32 minind = 0;
		FOR(i,dstcnt*srccnt){
			if(distances[i] < distances[minind]) minind = i;
		}
		indices[minind/srccnt] = minind%srccnt;
		FOR(i,srccnt){
			distances[(minind/srccnt)*srccnt + i] = 1.0f;
		}
		FOR(i,dstcnt){
			distances[i*srccnt + minind%srccnt] = 1.0f;
		}
	}
	delete [] distances;
	return indices;
}

static void gram_schmit(Vec3 v1, Vec3  v2, float *m)
{
	//v3 = v1 x v2;
	//e3 = v3 / |v3|;
	//e2 = v2 - e3 * (v2 * e3) / |v2 - e3 * (v2 * e3)|;
	//e1 = v1 - e3 * (v1 * e3) - e2*(v1*e2);
	Vec3 v3 = Mul(v1, v2);
	Vec3 e3 = Normalize(v3);
	Vec3 e2 = Normalize(Sub(v2, MulS(e3, Dot(v2, e3))));
	Vec3 asd = MulS(e3, Dot(v1, e3));
	Vec3 e1 = Sub(Sub(v1, MulS(e3, Dot(v1, e3))), MulS(e2, Dot(v1, e2)));

	m[0] = e1.x;	m[1] = e2.x;	m[2] = e3.x;	m[3] = 0;
	m[4] = e1.y;	m[5] = e2.y;	m[6] = e3.y;	m[7] = 0;
	m[8] = e1.z;	m[9] = e2.z;	m[10] = e3.z;	m[11] = 0;
	m[12] = 0;		m[13] = 0;		m[14] = 0;		m[15] = 1;
}

static float *Transpose(float *m)
{
	float *n = new float[16];
	n[0] = m[0];	n[1] = m[4];	n[2] = m[8];	n[3] = m[3];
	n[4] = m[1];	n[5] = m[5];	n[6] = m[9];	n[7] = m[7];
	n[8] = m[2];	n[9] = m[6];	n[10] = m[10];	n[11] = m[11];
	n[12] = m[12];	n[13] = m[13];	n[14] = m[14];	n[15] = m[15];
	return n;
}

static float Angle(Vec3 a, Vec3 b)
{
	// acos( a.b / |a|.|b|)
	float phi = Dot(a,b) / (Norm(a)*Norm(b));
	return acos(phi);
}


/*!
 * Cubic interpolation class adapted from an algorithm by Marino 'Mutilate' Alge
 * works with Vec2 and Vec3 types
 */
template <class T>
class CCubic
{
private:
	T a, b, c, d;

public:
	/*!
		Constructor, takes 4 points as input for the interpolation
		\param x0-x3 the input points for interpolation
	*/
	CCubic(T x0, T x1, T x2, T x3)
	{
//		a = x1;
//		b = (x2 - x0)*0.5f;
//		c = x0 - x1*2.5f + x2*2.0f - x3*0.5f;
//		d = (x3 - x0)*0.5f - (x2 - x1)*1.5f;
		a = x1;
		b = MulS(Sub(x2,x0),0.5f);
		c = Add(Sub(x0,MulS(x1,2.5f)),Sub(MulS(x2,2.0f),MulS(x3,0.5f)));
		d = Sub(MulS(Sub(x3,x0),0.5f),MulS(Sub(x2,x1),1.5f));
	}

	/*!
		The interpolation prediction function
		\param t the input time
		\return returns the value expected at time t
	*/
	T X(float t) const
	{
		//return a + (b + (c + d*t)*t)*t;
		return Add(a,MulS(Add(b,MulS(Add(c,MulS(d,t)),t)),t));
	}

	/*!
		All in one interpolation function
		\param x0-x3 the input points for interpolation
		\param t the input time
		\return returns the value expected at time t, using points x0-x3 for interpolation
	*/
	static T X(T x0, T x1, T x2, T x3, float t)
	{
		//return x1 + ((x2 - x0)*0.5f + (x0 - x1*2.5f + x2*2.0f - x3*0.5f + (x3 - x0)*0.5f - (x2 - x1)*1.5f*t)*t)*t;
		T asd7 = Sub(x0, MulS(x1,2.5f));
		T asd6 = Sub(MulS(x2,2.0f), MulS(x3,0.5f));
		T asd5 = Sub(MulS(Sub(x3,x0),0.5f), MulS(Sub(x2,x1),1.5f*t));
		T asd4 = Add(asd5,Add(asd6,asd7));
		T asd3 = MulS(asd4,t);
		T asd2 = Add(MulS(Sub(x2,x0),0.5f), asd3);
		T asd  = MulS(asd2,t);
		return Add(x1, asd);
	}
};

#endif // _BASICMATH_H_
