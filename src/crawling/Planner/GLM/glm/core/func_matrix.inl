///////////////////////////////////////////////////////////////////////////////////////////////////
// OpenGL Mathematics Copyright (c) 2005 - 2009 G-Truc Creation (www.g-truc.net)
///////////////////////////////////////////////////////////////////////////////////////////////////
// Created : 2008-03-08
// Updated : 2008-03-08
// Licence : This source is under MIT License
// File    : glm/core/func_matrix.inl
///////////////////////////////////////////////////////////////////////////////////////////////////

namespace glm
{
	namespace core{
	namespace function{
	namespace matrix{

    // matrixCompMult
    template <typename valType>
    inline detail::tmat2x2<valType> matrixCompMult
	(
		detail::tmat2x2<valType> const & x, 
		detail::tmat2x2<valType> const & y
	)
    {
		GLM_STATIC_ASSERT(detail::type<valType>::is_float);

        detail::tmat2x2<valType> result;
        for(int j = 0; j < 2; ++j)
            for(int i = 0; i < 2; ++i)
                result[j][i] = x[j][i] * y[j][i];
        return result;
    }

    template <typename valType>
	inline detail::tmat3x3<valType> matrixCompMult
	(
		detail::tmat3x3<valType> const & x, 
		detail::tmat3x3<valType> const & y
	)
    {
		GLM_STATIC_ASSERT(detail::type<valType>::is_float);

        detail::tmat3x3<valType> result;
        for(int j = 0; j < 3; ++j)
            for(int i = 0; i < 3; ++i)
                result[j][i] = x[j][i] * y[j][i];
        return result;
    }

    template <typename valType>
    inline detail::tmat4x4<valType> matrixCompMult
	(
		detail::tmat4x4<valType> const & x, 
		detail::tmat4x4<valType> const & y
	)
    {
		GLM_STATIC_ASSERT(detail::type<valType>::is_float);

        detail::tmat4x4<valType> result;
        for(int j = 0; j < 4; ++j)
            for(int i = 0; i < 4; ++i)
                result[j][i] = x[j][i] * y[j][i];
        return result;
    }

    template <typename valType>
    inline detail::tmat2x3<valType> matrixCompMult
	(
		detail::tmat2x3<valType> const & x, 
		detail::tmat2x3<valType> const & y
	)
    {
		GLM_STATIC_ASSERT(detail::type<valType>::is_float);

        detail::tmat2x3<valType> result;
        for(int j = 0; j < 2; ++j)
            for(int i = 0; i < 3; ++i)
                result[j][i] = x[j][i] * y[j][i];
        return result;
    }

    template <typename valType>
    inline detail::tmat3x2<valType> matrixCompMult
	(
		detail::tmat3x2<valType> const & x, 
		detail::tmat3x2<valType> const & y
	)
    {
		GLM_STATIC_ASSERT(detail::type<valType>::is_float);

        detail::tmat3x2<valType> result;
        for(int j = 0; j < 3; ++j)
            for(int i = 0; i < 2; ++i)
                result[j][i] = x[j][i] * y[j][i];
        return result;
    }

    template <typename valType>
    inline detail::tmat2x4<valType> matrixCompMult
	(
		detail::tmat2x4<valType> const & x, 
		detail::tmat2x4<valType> const & y
	)
    {
		GLM_STATIC_ASSERT(detail::type<valType>::is_float);

        detail::tmat2x4<valType> result;
        for(int j = 0; j < 2; ++j)
            for(int i = 0; i < 4; ++i)
                result[j][i] = x[j][i] * y[j][i];
        return result;
    }

    template <typename valType>
    inline detail::tmat4x2<valType> matrixCompMult
	(
		detail::tmat4x2<valType> const & x, 
		detail::tmat4x2<valType> const & y
	)
    {
		GLM_STATIC_ASSERT(detail::type<valType>::is_float);

        detail::tmat4x2<valType> result;
        for(int j = 0; j < 4; ++j)
            for(int i = 0; i < 2; ++i)
                result[j][i] = x[j][i] * y[j][i];
        return result;
    }

    template <typename valType>
    inline detail::tmat3x4<valType> matrixCompMult
	(
		detail::tmat3x4<valType> const & x, 
		detail::tmat3x4<valType> const & y
	)
    {
		GLM_STATIC_ASSERT(detail::type<valType>::is_float);

        detail::tmat3x4<valType> result;
        for(int j = 0; j < 3; ++j)
            for(int i = 0; i < 4; ++i)
                result[j][i] = x[j][i] * y[j][i];
        return result;
    }

    template <typename valType>
    inline detail::tmat4x3<valType> matrixCompMult
	(
		detail::tmat4x3<valType> const & x, 
		detail::tmat4x3<valType> const & y
	)
    {
		GLM_STATIC_ASSERT(detail::type<valType>::is_float);

        detail::tmat4x3<valType> result;
        for(int j = 0; j < 4; ++j)
            for(int i = 0; i < 3; ++i)
                result[j][i] = x[j][i] * y[j][i];
        return result;
    }

	// outerProduct
    template <typename valType>
    inline detail::tmat2x2<valType> outerProduct
	(
		detail::tvec2<valType> const & c, 
		detail::tvec2<valType> const & r
	)
    {
		GLM_STATIC_ASSERT(detail::type<valType>::is_float);

		detail::tmat2x2<valType> m;
		m[0][0] = c.x * r.x;
		m[0][1] = c.y * r.x;
		m[1][0] = c.x * r.y;
		m[1][1] = c.y * r.y;
        return m;
    }

    template <typename valType>
    inline detail::tmat3x3<valType> outerProduct
	(
		detail::tvec3<valType> const & c, 
		detail::tvec3<valType> const & r
	)
    {
		GLM_STATIC_ASSERT(detail::type<valType>::is_float);

		detail::tmat3x3<valType> m;
		m[0][0] = c.x * r.x;
		m[0][1] = c.y * r.x;
		m[0][2] = c.z * r.x;
		m[1][0] = c.x * r.y;
		m[1][1] = c.y * r.y;
		m[1][2] = c.z * r.y;
		m[2][0] = c.x * r.z;
		m[2][1] = c.y * r.z;
		m[2][2] = c.z * r.z;
        return m;
    }

    template <typename valType>
    inline detail::tmat4x4<valType> outerProduct
	(
		detail::tvec4<valType> const & c, 
		detail::tvec4<valType> const & r
	)
    {
		GLM_STATIC_ASSERT(detail::type<valType>::is_float);

		detail::tmat4x4<valType> m;
		m[0][0] = c.x * r.x;
		m[0][1] = c.y * r.x;
		m[0][2] = c.z * r.x;
		m[0][3] = c.w * r.x;
		m[1][0] = c.x * r.y;
		m[1][1] = c.y * r.y;
		m[1][2] = c.z * r.y;
		m[1][3] = c.w * r.y;
		m[2][0] = c.x * r.z;
		m[2][1] = c.y * r.z;
		m[2][2] = c.z * r.z;
		m[2][3] = c.w * r.z;
		m[3][0] = c.x * r.w;
		m[3][1] = c.y * r.w;
		m[3][2] = c.z * r.w;
		m[3][3] = c.w * r.w;
        return m;
    }

    template <typename valType>
	inline detail::tmat2x3<valType> outerProduct
	(
		detail::tvec3<valType> const & c, 
		detail::tvec2<valType> const & r
	)
	{
		GLM_STATIC_ASSERT(detail::type<valType>::is_float);

		detail::tmat2x3<valType> m;
		m[0][0] = c.x * r.x;
		m[0][1] = c.y * r.x;
		m[0][2] = c.z * r.x;
		m[1][0] = c.x * r.y;
		m[1][1] = c.y * r.y;
		m[1][2] = c.z * r.y;
		return m;
	}

    template <typename valType>
	inline detail::tmat3x2<valType> outerProduct
	(
		detail::tvec2<valType> const & c, 
		detail::tvec3<valType> const & r
	)
	{
		GLM_STATIC_ASSERT(detail::type<valType>::is_float);

		detail::tmat3x2<valType> m;
		m[0][0] = c.x * r.x;
		m[0][1] = c.y * r.x;
		m[1][0] = c.x * r.y;
		m[1][1] = c.y * r.y;
		m[2][0] = c.x * r.z;
		m[2][1] = c.y * r.z;
		return m;
	}

	template <typename valType>
	inline detail::tmat2x4<valType> outerProduct
	(
		detail::tvec2<valType> const & c, 
		detail::tvec4<valType> const & r
	)
	{
		GLM_STATIC_ASSERT(detail::type<valType>::is_float);

		detail::tmat2x4<valType> m;
		m[0][0] = c.x * r.x;
		m[0][1] = c.y * r.x;
		m[0][2] = c.z * r.x;
		m[0][3] = c.w * r.x;
		m[1][0] = c.x * r.y;
		m[1][1] = c.y * r.y;
		m[1][2] = c.z * r.y;
		m[1][3] = c.w * r.y;
		return m;
	}

	template <typename valType>
	inline detail::tmat4x2<valType> outerProduct
	(
		detail::tvec4<valType> const & c, 
		detail::tvec2<valType> const & r
	)
	{
		GLM_STATIC_ASSERT(detail::type<valType>::is_float);

		detail::tmat4x2<valType> m;
		m[0][0] = c.x * r.x;
		m[0][1] = c.y * r.x;
		m[1][0] = c.x * r.y;
		m[1][1] = c.y * r.y;
		m[2][0] = c.x * r.z;
		m[2][1] = c.y * r.z;
		m[3][0] = c.x * r.w;
		m[3][1] = c.y * r.w;
		return m;
	}

	template <typename valType>
	inline detail::tmat3x4<valType> outerProduct
	(
		detail::tvec4<valType> const & c, 
		detail::tvec3<valType> const & r
	)
	{
		GLM_STATIC_ASSERT(detail::type<valType>::is_float);

		detail::tmat3x4<valType> m;
		m[0][0] = c.x * r.x;
		m[0][1] = c.y * r.x;
		m[0][2] = c.z * r.x;
		m[0][3] = c.w * r.x;
		m[1][0] = c.x * r.y;
		m[1][1] = c.y * r.y;
		m[1][2] = c.z * r.y;
		m[1][3] = c.w * r.y;
		m[2][0] = c.x * r.z;
		m[2][1] = c.y * r.z;
		m[2][2] = c.z * r.z;
		m[2][3] = c.w * r.z;
		return m;
	}

	template <typename valType>
	inline detail::tmat4x3<valType> outerProduct
	(
		detail::tvec3<valType> const & c, 
		detail::tvec4<valType> const & r
	)
	{
		GLM_STATIC_ASSERT(detail::type<valType>::is_float);

		detail::tmat4x3<valType> m;
		m[0][0] = c.x * r.x;
		m[0][1] = c.y * r.x;
		m[0][2] = c.z * r.x;
		m[1][0] = c.x * r.y;
		m[1][1] = c.y * r.y;
		m[1][2] = c.z * r.y;
		m[2][0] = c.x * r.z;
		m[2][1] = c.y * r.z;
		m[2][2] = c.z * r.z;
		m[3][0] = c.x * r.w;
		m[3][1] = c.y * r.w;
		m[3][2] = c.z * r.w;
		return m;
	}

    template <typename valType>
    inline detail::tmat2x2<valType> transpose
	(
		detail::tmat2x2<valType> const & m
	)
    {
		GLM_STATIC_ASSERT(detail::type<valType>::is_float);

        detail::tmat2x2<valType> result;
        result[0][0] = m[0][0];
        result[0][1] = m[1][0];
        result[1][0] = m[0][1];
        result[1][1] = m[1][1];
        return result;
    }

    template <typename valType>
    inline detail::tmat3x3<valType> transpose
	(
		detail::tmat3x3<valType> const & m
	)
    {
		GLM_STATIC_ASSERT(detail::type<valType>::is_float);

        detail::tmat3x3<valType> result;
        result[0][0] = m[0][0];
        result[0][1] = m[1][0];
        result[0][2] = m[2][0];

        result[1][0] = m[0][1];
        result[1][1] = m[1][1];
        result[1][2] = m[2][1];

        result[2][0] = m[0][2];
        result[2][1] = m[1][2];
        result[2][2] = m[2][2];
        return result;
    }

    template <typename valType>
    inline detail::tmat4x4<valType> transpose
	(
		detail::tmat4x4<valType> const & m
	)
    {
		GLM_STATIC_ASSERT(detail::type<valType>::is_float);

        detail::tmat4x4<valType> result;
        result[0][0] = m[0][0];
        result[0][1] = m[1][0];
        result[0][2] = m[2][0];
        result[0][3] = m[3][0];

        result[1][0] = m[0][1];
        result[1][1] = m[1][1];
        result[1][2] = m[2][1];
        result[1][3] = m[3][1];

        result[2][0] = m[0][2];
        result[2][1] = m[1][2];
        result[2][2] = m[2][2];
        result[2][3] = m[3][2];

        result[3][0] = m[0][3];
        result[3][1] = m[1][3];
        result[3][2] = m[2][3];
        result[3][3] = m[3][3];
        return result;
    }

    template <typename valType>
    inline detail::tmat2x3<valType> transpose
	(
		detail::tmat3x2<valType> const & m
	)
    {
		GLM_STATIC_ASSERT(detail::type<valType>::is_float);

        detail::tmat2x3<valType> result;
        result[0][0] = m[0][0];
        result[0][1] = m[1][0];
		result[0][2] = m[2][0];
        result[1][0] = m[0][1];
        result[1][1] = m[1][1];
		result[1][2] = m[2][1];
        return result;
    }

    template <typename valType>
    inline detail::tmat3x2<valType> transpose
	(
		detail::tmat2x3<valType> const & m
	)
    {
		GLM_STATIC_ASSERT(detail::type<valType>::is_float);

        detail::tmat3x2<valType> result;
        result[0][0] = m[0][0];
        result[0][1] = m[1][0];
        result[1][0] = m[0][1];
        result[1][1] = m[1][1];
        result[2][0] = m[0][2];
        result[2][1] = m[1][2];
        return result;
    }

    template <typename valType>
	inline detail::tmat2x4<valType> transpose
	(
		detail::tmat4x2<valType> const & m
	)
	{
		GLM_STATIC_ASSERT(detail::type<valType>::is_float);

		detail::tmat2x4<valType> result;
        result[0][0] = m[0][0];
        result[0][1] = m[1][0];
		result[0][2] = m[2][0];
		result[0][3] = m[3][0];
        result[1][0] = m[0][1];
        result[1][1] = m[1][1];
		result[1][2] = m[2][1];
		result[1][3] = m[3][1];
		return result;
	}

    template <typename valType>
	inline detail::tmat4x2<valType> transpose
	(
		detail::tmat2x4<valType> const & m
	)
	{
		GLM_STATIC_ASSERT(detail::type<valType>::is_float);

        detail::tmat4x2<valType> result;
        result[0][0] = m[0][0];
        result[0][1] = m[1][0];
        result[1][0] = m[0][1];
        result[1][1] = m[1][1];
        result[2][0] = m[0][2];
        result[2][1] = m[1][2];
        result[3][0] = m[0][3];
        result[3][1] = m[1][3];
        return result;
	}

    template <typename valType>
	inline detail::tmat3x4<valType> transpose
	(
		detail::tmat4x3<valType> const & m
	)
	{
		GLM_STATIC_ASSERT(detail::type<valType>::is_float);

		detail::tmat3x4<valType> result;
        result[0][0] = m[0][0];
        result[0][1] = m[1][0];
		result[0][2] = m[2][0];
		result[0][3] = m[3][0];
        result[1][0] = m[0][1];
        result[1][1] = m[1][1];
		result[1][2] = m[2][1];
		result[1][3] = m[3][1];
        result[2][0] = m[0][2];
        result[2][1] = m[1][2];
		result[2][2] = m[2][2];
		result[2][3] = m[3][2];
		return result;
	}

    template <typename valType>
	inline detail::tmat4x3<valType> transpose
	(
		detail::tmat3x4<valType> const & m
	)
	{
		GLM_STATIC_ASSERT(detail::type<valType>::is_float);

        detail::tmat4x3<valType> result;
        result[0][0] = m[0][0];
        result[0][1] = m[1][0];
		result[0][2] = m[2][0];
        result[1][0] = m[0][1];
        result[1][1] = m[1][1];
		result[1][2] = m[2][1];
        result[2][0] = m[0][2];
        result[2][1] = m[1][2];
		result[2][2] = m[2][2];
        result[3][0] = m[0][3];
        result[3][1] = m[1][3];
		result[3][2] = m[2][3];
        result[4][0] = m[0][4];
        result[4][1] = m[1][4];
		result[4][2] = m[2][4];
        return result;
	}

	}//namespace matrix
	}//namespace function
	}//namespace core
}//namespace glm
