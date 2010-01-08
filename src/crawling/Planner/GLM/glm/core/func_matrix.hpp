///////////////////////////////////////////////////////////////////////////////////////////////////
// OpenGL Mathematics Copyright (c) 2005 - 2009 G-Truc Creation (www.g-truc.net)
///////////////////////////////////////////////////////////////////////////////////////////////////
// Created : 2008-08-03
// Updated : 2008-08-03
// Licence : This source is under MIT License
// File    : glm/core/func_matrix.h
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef glm_core_func_matrix
#define glm_core_func_matrix

namespace glm
{
	namespace test{
		void main_core_func_matrix();
	}//namespace test

	namespace core{
	namespace function{
	//! Define all matrix functions from Section 8.5 of GLSL 1.30.8 specification. Included in glm namespace.
	namespace matrix{

	//! Multiply matrix x by matrix y component-wise, i.e., 
	//! result[i][j] is the scalar product of x[i][j] and y[i][j].
	//! (From GLSL 1.30.08 specification, section 8.6)
	template <typename matType> 
	matType matrixCompMult(matType const & x, matType const & y);

	//! Treats the first parameter c as a column vector 
	//! and the second parameter r as a row vector
	//! and does a linear algebraic matrix multiply c * r.
	//! (From GLSL 1.30.08 specification, section 8.6)
	template <typename vecType, typename matType> 
	matType outerProduct(vecType const & c, vecType const & r);

	//template <typename T> detail::tmat2x2<T> outerProduct(const detail::tvec2<T>& c, const detail::tvec2<T>& r); //!< \brief Treats the first parameter c as a column vector and the second parameter r as a row vector and does a linear algebraic matrix multiply c * r. (From GLSL 1.20.6 specification) 
	//template <typename T> detail::tmat3x3<T> outerProduct(const detail::tvec3<T>& c, const detail::tvec3<T>& r); //!< \brief Treats the first parameter c as a column vector and the second parameter r as a row vector and does a linear algebraic matrix multiply c * r. (From GLSL 1.20.6 specification) 
	//template <typename T> detail::tmat4x4<T> outerProduct(const detail::tvec4<T>& c, const detail::tvec4<T>& r); //!< \brief Treats the first parameter c as a column vector and the second parameter r as a row vector and does a linear algebraic matrix multiply c * r. (From GLSL 1.20.6 specification) 
	//template <typename T> detail::tmat2x3<T> outerProduct(const detail::tvec3<T>& c, const detail::tvec2<T>& r); //!< \brief Treats the first parameter c as a column vector and the second parameter r as a row vector and does a linear algebraic matrix multiply c * r. (From GLSL 1.20.6 specification) 
	//template <typename T> detail::tmat3x2<T> outerProduct(const detail::tvec2<T>& c, const detail::tvec3<T>& r); //!< \brief Treats the first parameter c as a column vector and the second parameter r as a row vector and does a linear algebraic matrix multiply c * r. (From GLSL 1.20.6 specification) 
	//template <typename T> detail::tmat2x4<T> outerProduct(const detail::tvec2<T>& c, const detail::tvec4<T>& r); //!< \brief Treats the first parameter c as a column vector and the second parameter r as a row vector and does a linear algebraic matrix multiply c * r. (From GLSL 1.20.6 specification) 
	//template <typename T> detail::tmat4x2<T> outerProduct(const detail::tvec4<T>& c, const detail::tvec2<T>& r); //!< \brief Treats the first parameter c as a column vector and the second parameter r as a row vector and does a linear algebraic matrix multiply c * r. (From GLSL 1.20.6 specification) 
	//template <typename T> detail::tmat3x4<T> outerProduct(const detail::tvec4<T>& c, const detail::tvec3<T>& r); //!< \brief Treats the first parameter c as a column vector and the second parameter r as a row vector and does a linear algebraic matrix multiply c * r. (From GLSL 1.20.6 specification) 
	//template <typename T> detail::tmat4x3<T> outerProduct(const detail::tvec3<T>& c, const detail::tvec4<T>& r); //!< \brief Treats the first parameter c as a column vector and the second parameter r as a row vector and does a linear algebraic matrix multiply c * r. (From GLSL 1.20.6 specification) 

	//! Returns the transposed matrix of x
	//! (From GLSL 1.30.08 specification, section 8.6)
	template <typename matType> 
	typename matType::transpose_type transpose(matType const & x);
	
	}//namespace matrix
	}//namespace function
	}//namespace core

	using namespace core::function::matrix;
}//namespace glm

#include "func_matrix.inl"

#endif//glm_core_func_matrix
