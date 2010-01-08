///////////////////////////////////////////////////////////////////////////////////////////////////
// OpenGL Mathematics Copyright (c) 2005 - 2009 G-Truc Creation (www.g-truc.net)
///////////////////////////////////////////////////////////////////////////////////////////////////
// Created : 2005-12-21
// Updated : 2009-04-29
// Licence : This source is under MIT License
// File    : glm/gtx/transform.inl
///////////////////////////////////////////////////////////////////////////////////////////////////

namespace glm{
namespace gtx{
namespace transform
{
    template <typename T> 
    inline detail::tmat4x4<T> translate(
		T x, T y, T z)
    {
		return gtc::matrix_transform::translate(
			detail::tmat4x4<T>(1.0f), 
			detail::tvec3<T>(x, y , z));
    }

    template <typename T> 
    inline detail::tmat4x4<T> translate(
		detail::tvec3<T> const & v)
    {
		return gtc::matrix_transform::translate(
			detail::tmat4x4<T>(1.0f), v);
    }

    template <typename T> 
    inline detail::tmat4x4<T> translate(
		detail::tmat4x4<T> const & m, 
		T x, T y, T z)
    {
        return gtc::matrix_transform::translate(
			m, detail::tvec3<T>(x, y , z));
    }

    template <typename T> 
    inline detail::tmat4x4<T> rotate(
		T angle, 
		detail::tvec3<T> const & v)
    {
		return gtc::matrix_transform::rotate(
			detail::tmat4x4<T>(1), angle, v);
    }

    template <typename T> 
    inline detail::tmat4x4<T> rotate(
		T angle, 
		T x, T y, T z)
    {
		return gtc::matrix_transform::rotate(
			detail::tmat4x4<T>(1), angle, detail::tvec3<T>(x, y, z));
    }

    template <typename T> 
    inline detail::tmat4x4<T> rotate(
		detail::tmat4x4<T> const & m, 
		T angle, 
		T x, T y, T z)
    {
		return gtc::matrix_transform::rotate(
			m, angle, detail::tvec3<T>(x, y, z));
    }

    template <typename T> 
    inline detail::tmat4x4<T> scale(T x, T y, T z)
    {
		return gtc::matrix_transform::scale(
			detail::tmat4x4<T>(1), detail::tvec3<T>(x, y, z));
    }

    template <typename T> 
    inline detail::tmat4x4<T> scale(
		detail::tmat4x4<T> const & m, 
		T x, T y, T z)
    {
        return gtc::matrix_transform::scale(
			m, detail::tvec3<T>(x, y, z));
    }

    template <typename T> 
    inline detail::tmat4x4<T> scale(
		detail::tmat4x4<T> const & m, 
		detail::tvec3<T> const & v)
    {
        return gtc::matrix_transform::scale(
			m, v);
    }
/*
    /////////////////////////////////////////////////
    template <typename T> 
    inline detail::tmat4x4<T> translate3D(
		const detail::tmat4x4<T>& m, 
		T x, T y, T z)
    {
        detail::tmat4x4<T> r(1);
		r[3] = detail::tvec4<T>(x, y, z, T(1));
        //r[0] = m[0];
        //r[1] = m[1];
        //r[2] = m[2];
        //r[3][0] = m[0][0] * x + m[1][0] * y + m[2][0] * z + m[3][0];
        //r[3][1] = m[0][1] * x + m[1][1] * y + m[2][1] * z + m[3][1];
        //r[3][2] = m[0][2] * x + m[1][2] * y + m[2][2] * z + m[3][2];
        //r[3][3] = m[0][3] * x + m[1][3] * y + m[2][3] * z + m[3][3];
        return m * r;
    }

    template <typename T> 
    inline detail::tmat4x4<T> translate3D(
		const detail::tmat4x4<T>& m, 
		const detail::tvec3<T>& v)
    {
        return translate3D(m, v.x, v.y, v.z);
    }

    template <typename T> 
    inline detail::tmat4x4<T> rotate3D(
		const detail::tmat4x4<T>& m, 
		T angle, 
		T x, T y, T z)
    {
        T a = radians(angle);
        T c = cos(a);
        T s = sin(a);
        detail::tmat4x4<T> Result;

        detail::tvec3<T> axis = normalize(detail::tvec3<T>(x, y, z));

        Result[0][0] = c + (1 - c) * axis.x * axis.x;
	    Result[0][1] = (1 - c) * axis.x * axis.y + s * axis.z;
	    Result[0][2] = (1 - c) * axis.x * axis.z - s * axis.y;
	    Result[0][3] = 0;

	    Result[1][0] = (1 - c) * axis.y * axis.x - s * axis.z;
	    Result[1][1] = c + (1 - c) * axis.y * axis.y;
	    Result[1][2] = (1 - c) * axis.y * axis.z + s * axis.x;
	    Result[1][3] = 0;

	    Result[2][0] = (1 - c) * axis.x * axis.z + s * axis.y;
	    Result[2][1] = (1 - c) * axis.y * axis.z - s * axis.x;
	    Result[2][2] = c + (1 - c) * axis.z * axis.z;
	    Result[2][3] = 0;

        Result[3] = detail::tvec4<T>(0, 0, 0, 1);

        return m * Result;
    }

    template <typename T> 
    inline detail::tmat4x4<T> rotate3D(
		const detail::tmat4x4<T>& m, 
		T angle, 
		const detail::tvec3<T>& v)
    {
        return rotate3D(m, angle, v.x, v.y, v.z);
    }

    template <typename T> 
    inline detail::tmat3x3<T> rotate3D(
		const detail::tmat3x3<T>& m, 
		T angle, 
		T x, T y, T z)
    {
        T a = radians(angle);
        T c = cos(a);
        T s = sin(a);
        detail::tmat3x3<T> Result;

        detail::tvec3<T> axis = normalize(detail::tvec3<T>(x, y, z));

        Result[0][0] = c + (1 - c) * axis.x * axis.x;
	    Result[0][1] = (1 - c) * axis.x * axis.y + s * axis.z;
	    Result[0][2] = (1 - c) * axis.x * axis.z - s * axis.y;

	    Result[1][0] = (1 - c) * axis.y * axis.x - s * axis.z;
	    Result[1][1] = c + (1 - c) * axis.y * axis.y;
	    Result[1][2] = (1 - c) * axis.y * axis.z + s * axis.x;

	    Result[2][0] = (1 - c) * axis.x * axis.z + s * axis.y;
	    Result[2][1] = (1 - c) * axis.y * axis.z - s * axis.x;
	    Result[2][2] = c + (1 - c) * axis.z * axis.z;

        return m * Result;
    }

    template <typename T> 
    inline detail::tmat3x3<T> rotate3D(
		const detail::tmat3x3<T>& m, 
		T angle, 
		const detail::tvec3<T>& v)
    {
        return rotate3D(m, angle, v.x, v.y, v.z);
    }

    template <typename T> 
    inline detail::tmat4x4<T> scale3D(
		const detail::tmat4x4<T>& m, 
		T x, T y, T z)
    {
        detail::tmat4x4<T> r;
        r[0] = m[0] * x;
        r[1] = m[1] * y;
        r[2] = m[2] * z;
        r[3] = m[3];
        return r;
    }

    template <typename T> 
    inline detail::tmat4x4<T> scale3D(
		const detail::tmat4x4<T>& m, 
		const detail::tvec3<T>& v)
    {
        return scale3D(m, v.x, v.y, v.z);
    }

    template <typename T> 
    inline detail::tmat3x3<T> scale3D(
		const detail::tmat3x3<T>& m, 
		T x, T y, T z)
    {
        detail::tmat3x3<T> r;
        r[0] = m[0] * x;
        r[1] = m[1] * y;
        r[2] = m[2] * z;
        return r;
    }

    template <typename T> 
    inline detail::tmat3x3<T> scale3D(
		const detail::tmat3x3<T>& m, 
		const detail::tvec3<T>& v)
    {
        return scale3D(m, v.x, v.y, v.z);
    }

    template <typename T> 
    inline detail::tmat3x3<T> translate2D(
		const detail::tmat3x3<T>& m, 
		T x, T y)
    {
        detail::tmat3x3<T> r;
        r[0] = m[0];
        r[1] = m[1];
        r[2] = m[2];
        r[2][0] = m[0][0] * x + m[1][0] * y + m[2][0];
        r[2][1] = m[0][1] * x + m[1][1] * y + m[2][1];
        r[2][2] = m[0][2] * x + m[1][2] * y + m[2][2];
        return r;
    }

    template <typename T> 
    inline detail::tmat3x3<T> translate2D(
		const detail::tmat3x3<T>& m, 
		const detail::tvec2<T>& v)
    {
        return translate2D(m, v.x, v.y);
    }

    template <typename T> 
    inline detail::tmat3x3<T> rotate2D(
		const detail::tmat3x3<T>& m, 
		T angle)
    {
        T a = radians(angle);
        T c = cos(a);
        T s = sin(a);
        detail::tmat3x3<T> Result;

        Result[0][0] = c;
	    Result[0][1] = -s;
	    Result[0][2] = 0;

	    Result[1][0] = s;
	    Result[1][1] = c;
	    Result[1][2] = 0;

        Result[2] = detail::tvec3<T>(0, 0, 1);

        return m * Result;
    }

    template <typename T> 
    inline detail::tmat2x2<T> rotate2D(
		const detail::tmat2x2<T>& m, 
		T angle)
    {
        T a = radians(angle);
        T c = cos(a);
        T s = sin(a);
        detail::tmat2x2<T> Result;

        Result[0][0] = c;
	    Result[0][1] = -s;

	    Result[1][0] = s;
	    Result[1][1] = c;

        return m * Result;
    }

    template <typename T> 
    inline detail::tmat3x3<T> scale2D(
		const detail::tmat3x3<T>& m, 
		T x, T y)
    {
        detail::tmat3x3<T> r;
        r[0] = m[0] * x;
        r[1] = m[1] * y;
        r[2] = m[2];
        return r;
    }

    template <typename T> 
    inline detail::tmat3x3<T> scale2D(
		const detail::tmat3x3<T>& m, 
		const detail::tvec2<T>& v)
    {
        return scale2D(m, v.x, v.y);
    }

    template <typename T> 
    inline detail::tmat2x2<T> scale2D(
		const detail::tmat2x2<T>& m, 
		T x, T y)
    {
        detail::tmat2x2<T> r;
        r[0] = m[0] * x;
        r[1] = m[1] * y;
        return r;
    }

    template <typename T> 
    inline detail::tmat2x2<T> scale2D(
		const detail::tmat2x2<T>& m, 
		const detail::tvec2<T>& v)
    {
        return scale2D(m, v.x, v.y);
    }
*/	
}//namespace transform
}//namespace gtx
}//namespace glm
