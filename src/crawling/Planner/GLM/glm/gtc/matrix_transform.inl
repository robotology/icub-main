///////////////////////////////////////////////////////////////////////////////////////////////////
// OpenGL Mathematics Copyright (c) 2005 - 2009 G-Truc Creation (www.g-truc.net)
///////////////////////////////////////////////////////////////////////////////////////////////////
// Created : 2009-04-29
// Updated : 2009-04-29
// Licence : This source is under MIT License
// File    : glm/gtc/matrix_transform.inl
///////////////////////////////////////////////////////////////////////////////////////////////////

namespace glm{
namespace gtc{
namespace matrix_transform
{
    template <typename valType> 
    inline detail::tmat4x4<valType> translate(
		detail::tmat4x4<valType> const & m,
		detail::tvec3<valType> const & v)
    {
        detail::tmat4x4<valType> Result(valType(1));
        Result[3] = detail::tvec4<valType>(v, valType(1));
        return m * Result;
    }
		
    template <typename valType> 
    inline detail::tmat4x4<valType> rotate(
		detail::tmat4x4<valType> const & m,
		valType const & angle, 
		detail::tvec3<valType> const & v)
    {
        valType a = radians(angle);
        valType c = cos(a);
        valType s = sin(a);
        detail::tmat4x4<valType> Result;

        detail::tvec3<valType> axis = normalize(v);

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

        Result[3] = detail::tvec4<valType>(0, 0, 0, 1);
        return m * Result;
    }

    template <typename valType> 
    inline detail::tmat4x4<valType> scale(
		detail::tmat4x4<valType> const & m,
		detail::tvec3<valType> const & v)
    {
        detail::tmat4x4<valType> Result(valType(1));
        Result[0][0] = v.x;
        Result[1][1] = v.y;
        Result[2][2] = v.z;
        return m * Result;
    }

}//namespace matrix_transform
}//namespace gtc
}//namespace glm
