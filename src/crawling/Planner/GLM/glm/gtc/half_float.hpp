///////////////////////////////////////////////////////////////////////////////////////////////////
// OpenGL Mathematics Copyright (c) 2005 - 2009 G-Truc Creation (www.g-truc.net)
///////////////////////////////////////////////////////////////////////////////////////////////////
// Created : 2009-04-29
// Updated : 2009-04-29
// Licence : This source is under MIT License
// File    : glm/gtc/half_float.hpp
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef glm_gtc_half_float
#define glm_gtc_half_float

// Dependency:
#include "../glm.hpp"

namespace glm
{
	namespace test{
		void main_ext_gtc_half_float();
	}//namespace test

	namespace detail
	{
#ifndef GLM_USE_ANONYMOUS_UNION
		template <>
		struct tvec2<half>
		{
			//////////////////////////////////////
			// Typedef (Implementation details)

			typedef half value_type;
			typedef half& value_reference;
			typedef half* value_pointer;
			typedef tvec2<bool> bool_type;

			typedef sizeType size_type;
			static size_type value_size();
			static bool is_vector();

			typedef tvec2<half> type;
			typedef tvec2<half>* pointer;
			typedef const tvec2<half>* const_pointer;
			typedef const tvec2<half>*const const_pointer_const;
			typedef tvec2<half>*const pointer_const;
			typedef tvec2<half>& reference;
			typedef const tvec2<half>& const_reference;
			typedef const tvec2<half>& param_type;

			//////////////////////////////////////
			// Data

			half x, y;

			//////////////////////////////////////
			// Accesses

			half& operator[](size_type i);
			half operator[](size_type i) const;

			//////////////////////////////////////
			// Address (Implementation details)

			const half* _address() const{return (value_type*)(this);}
			half* _address(){return (value_type*)(this);}

			//////////////////////////////////////
			// Implicit basic constructors

			tvec2();
			tvec2(const tvec2<half>& v);

			//////////////////////////////////////
			// Explicit basic constructors

			explicit tvec2(half s);
			explicit tvec2(half s1, half s2);

			//////////////////////////////////////
			// Swizzle constructors

			tvec2(const tref2<half>& r);

			//////////////////////////////////////
			// Convertion scalar constructors

			//! Explicit converions (From section 5.4.1 Conversion and scalar constructors of GLSL 1.30.08 specification)
			template <typename U> 
			explicit tvec2(U x);
			//! Explicit converions (From section 5.4.1 Conversion and scalar constructors of GLSL 1.30.08 specification)
			template <typename U, typename V> 
			explicit tvec2(U x, V y);			

			//////////////////////////////////////
			// Convertion vector constructors

			//! Explicit conversions (From section 5.4.1 Conversion and scalar constructors of GLSL 1.30.08 specification)
			template <typename U> 
			explicit tvec2(const tvec2<U>& v);
			//! Explicit conversions (From section 5.4.1 Conversion and scalar constructors of GLSL 1.30.08 specification)
			template <typename U> 
			explicit tvec2(const tvec3<U>& v);
			//! Explicit conversions (From section 5.4.1 Conversion and scalar constructors of GLSL 1.30.08 specification)
			template <typename U> 
			explicit tvec2(const tvec4<U>& v);

			//////////////////////////////////////
			// Unary arithmetic operators

			tvec2<half>& operator= (const tvec2<half>& v);

			tvec2<half>& operator+=(half s);
			tvec2<half>& operator+=(tvec2<half> const & v);
			tvec2<half>& operator-=(half s);
			tvec2<half>& operator-=(tvec2<half> const & v);
			tvec2<half>& operator*=(half s);
			tvec2<half>& operator*=(tvec2<half> const & v);
			tvec2<half>& operator/=(half s);
			tvec2<half>& operator/=(tvec2<half> const & v);
			tvec2<half>& operator++();
			tvec2<half>& operator--();

			//////////////////////////////////////
			// Swizzle operators

			half swizzle(comp X) const;
			tvec2<half> swizzle(comp X, comp Y) const;
			tvec3<half> swizzle(comp X, comp Y, comp Z) const;
			tvec4<half> swizzle(comp X, comp Y, comp Z, comp W) const;
			tref2<half> swizzle(comp X, comp Y);
		};

		template <>
		struct tvec3<half>
		{
			//////////////////////////////////////
			// Typedef (Implementation details)

			typedef half value_type;
			typedef half& value_reference;
			typedef half* value_pointer;
			typedef tvec3<bool> bool_type;

			typedef glm::sizeType size_type;
			static size_type value_size();
			static bool is_vector();

			typedef tvec3<half> type;
			typedef tvec3<half>* pointer;
			typedef const tvec3<half>* const_pointer;
			typedef const tvec3<half>*const const_pointer_const;
			typedef tvec3<half>*const pointer_const;
			typedef tvec3<half>& reference;
			typedef const tvec3<half>& const_reference;
			typedef const tvec3<half>& param_type;

			//////////////////////////////////////
			// Data

			half x, y, z;

			//////////////////////////////////////
			// Accesses

			half& operator[](size_type i);
			half operator[](size_type i) const;

			//////////////////////////////////////
			// Address (Implementation details)

			const value_type* _address() const{return (value_type*)(this);}
			value_type* _address(){return (value_type*)(this);}

			//////////////////////////////////////
			// Implicit basic constructors

			tvec3();
			tvec3(const tvec3<half>& v);

			//////////////////////////////////////
			// Explicit basic constructors

			explicit tvec3(half s);
			explicit tvec3(half s1, half s2, half s3);

			//////////////////////////////////////
			// Swizzle constructors

			tvec3(const tref3<half>& r);

			//////////////////////////////////////
			// Convertion scalar constructors

			//! Explicit converions (From section 5.4.1 Conversion and scalar constructors of GLSL 1.30.08 specification)
			template <typename U> 
			explicit tvec3(U x);
			//! Explicit converions (From section 5.4.1 Conversion and scalar constructors of GLSL 1.30.08 specification)
			template <typename U, typename V, typename W> 
			explicit tvec3(U x, V y, W z);			

			//////////////////////////////////////
			// Convertion vector constructors

			//! Explicit conversions (From section 5.4.1 Conversion and scalar constructors of GLSL 1.30.08 specification)
			template <typename A, typename B> 
			explicit tvec3(const tvec2<A>& v, B s);
			//! Explicit conversions (From section 5.4.1 Conversion and scalar constructors of GLSL 1.30.08 specification)
			template <typename A, typename B> 
			explicit tvec3(A s, const tvec2<B>& v);
			//! Explicit conversions (From section 5.4.1 Conversion and scalar constructors of GLSL 1.30.08 specification)
			template <typename U> 
			explicit tvec3(const tvec3<U>& v);
			//! Explicit conversions (From section 5.4.1 Conversion and scalar constructors of GLSL 1.30.08 specification)
			template <typename U> 
			explicit tvec3(const tvec4<U>& v);

			//////////////////////////////////////
			// Unary arithmetic operators

			tvec3<half>& operator= (const tvec3<half>& v);

			tvec3<half>& operator+=(half s);
			tvec3<half>& operator+=(const tvec3<half>& v);
			tvec3<half>& operator-=(half s);
			tvec3<half>& operator-=(const tvec3<half>& v);
			tvec3<half>& operator*=(half s);
			tvec3<half>& operator*=(const tvec3<half>& v);
			tvec3<half>& operator/=(half s);
			tvec3<half>& operator/=(const tvec3<half>& v);
			tvec3<half>& operator++();
			tvec3<half>& operator--();

			//////////////////////////////////////
			// Unary bit operators

			tvec3<half>& operator%=(half s);
			tvec3<half>& operator%=(const tvec3<half>& v);
			tvec3<half>& operator&=(half s);
			tvec3<half>& operator&=(const tvec3<half>& v);
			tvec3<half>& operator|=(half s);
			tvec3<half>& operator|=(const tvec3<half>& v);
			tvec3<half>& operator^=(half s);
			tvec3<half>& operator^=(const tvec3<half>& v);
			tvec3<half>& operator<<=(half s);
			tvec3<half>& operator<<=(const tvec3<half>& v);
			tvec3<half>& operator>>=(half s);
			tvec3<half>& operator>>=(const tvec3<half>& v);

			//////////////////////////////////////
			// Swizzle operators

			half swizzle(comp X) const;
			tvec2<half> swizzle(comp X, comp Y) const;
			tvec3<half> swizzle(comp X, comp Y, comp Z) const;
			tvec4<half> swizzle(comp X, comp Y, comp Z, comp W) const;
			tref3<half> swizzle(comp X, comp Y, comp Z);
		};

		template <>
		struct tvec4<half>
		{
			//////////////////////////////////////
			// Typedef (Implementation details)

			typedef half value_type;
			typedef half& value_reference;
			typedef half* value_pointer;
			typedef tvec4<bool> bool_type;

			typedef glm::sizeType size_type;
			static size_type value_size();
			static bool is_vector();

			typedef tvec4<half> type;
			typedef tvec4<half>* pointer;
			typedef const tvec4<half>* const_pointer;
			typedef const tvec4<half>*const const_pointer_const;
			typedef tvec4<half>*const pointer_const;
			typedef tvec4<half>& reference;
			typedef const tvec4<half>& const_reference;
			typedef const tvec4<half>& param_type;

			//////////////////////////////////////
			// Data

			half x, y, z, w;

			//////////////////////////////////////
			// Accesses

			half& operator[](size_type i);
			half operator[](size_type i) const;

			//////////////////////////////////////
			// Address (Implementation details)

			const value_type* _address() const{return (value_type*)(this);}
			value_type* _address(){return (value_type*)(this);}

			//////////////////////////////////////
			// Implicit basic constructors

			tvec4();
			tvec4(tvec4<half> const & v);

			//////////////////////////////////////
			// Explicit basic constructors

			explicit tvec4(half s);
			explicit tvec4(half s0, half s1, half s2, half s3);

			//////////////////////////////////////
			// Swizzle constructors

			tvec4(const tref4<half>& r);

			//////////////////////////////////////
			// Convertion scalar constructors

			//! Explicit converions (From section 5.4.1 Conversion and scalar constructors of GLSL 1.30.08 specification)
			template <typename U> 
			explicit tvec4(U x);
			//! Explicit converions (From section 5.4.1 Conversion and scalar constructors of GLSL 1.30.08 specification)
			template <typename A, typename B, typename C, typename D> 
			explicit tvec4(A x, B y, C z, D w);			

			//////////////////////////////////////
			// Convertion vector constructors

			//! Explicit conversions (From section 5.4.1 Conversion and scalar constructors of GLSL 1.30.08 specification)
			template <typename A, typename B, typename C> 
			explicit tvec4(const tvec2<A>& v, B s1, C s2);
			//! Explicit conversions (From section 5.4.1 Conversion and scalar constructors of GLSL 1.30.08 specification)
			template <typename A, typename B, typename C> 
			explicit tvec4(A s1, const tvec2<B>& v, C s2);
			//! Explicit conversions (From section 5.4.1 Conversion and scalar constructors of GLSL 1.30.08 specification)
			template <typename A, typename B, typename C> 
			explicit tvec4(A s1, B s2, const tvec2<C>& v);
			//! Explicit conversions (From section 5.4.1 Conversion and scalar constructors of GLSL 1.30.08 specification)
			template <typename A, typename B> 
			explicit tvec4(const tvec3<A>& v, B s);
			//! Explicit conversions (From section 5.4.1 Conversion and scalar constructors of GLSL 1.30.08 specification)
			template <typename A, typename B> 
			explicit tvec4(A s, const tvec3<B>& v);
			//! Explicit conversions (From section 5.4.1 Conversion and scalar constructors of GLSL 1.30.08 specification)
			template <typename A, typename B> 
			explicit tvec4(const tvec2<A>& v1, const tvec2<B>& v2);
			//! Explicit conversions (From section 5.4.1 Conversion and scalar constructors of GLSL 1.30.08 specification)
			template <typename U> 
			explicit tvec4(const tvec4<U>& v);

			//////////////////////////////////////
			// Unary arithmetic operators

			tvec4<half>& operator= (const tvec4<half>& v);

			tvec4<half>& operator+=(half s);
			tvec4<half>& operator+=(const tvec4<half>& v);
			tvec4<half>& operator-=(half s);
			tvec4<half>& operator-=(const tvec4<half>& v);
			tvec4<half>& operator*=(half s);
			tvec4<half>& operator*=(const tvec4<half>& v);
			tvec4<half>& operator/=(half s);
			tvec4<half>& operator/=(const tvec4<half>& v);
			tvec4<half>& operator++();
			tvec4<half>& operator--();

			//////////////////////////////////////
			// Unary bit operators

			tvec4<half>& operator%= (half s);
			tvec4<half>& operator%= (const tvec4<half>& v);
			tvec4<half>& operator&= (half s);
			tvec4<half>& operator&= (const tvec4<half>& v);
			tvec4<half>& operator|= (half s);
			tvec4<half>& operator|= (const tvec4<half>& v);
			tvec4<half>& operator^= (half s);
			tvec4<half>& operator^= (const tvec4<half>& v);
			tvec4<half>& operator<<=(half s);
			tvec4<half>& operator<<=(const tvec4<half>& v);
			tvec4<half>& operator>>=(half s);
			tvec4<half>& operator>>=(const tvec4<half>& v);

			//////////////////////////////////////
			// Swizzle operators

			half swizzle(comp X) const;
			tvec2<half> swizzle(comp X, comp Y) const;
			tvec3<half> swizzle(comp X, comp Y, comp Z) const;
			tvec4<half> swizzle(comp X, comp Y, comp Z, comp W) const;
			tref4<half> swizzle(comp X, comp Y, comp Z, comp W);
		};
#endif//GLM_USE_ANONYMOUS_UNION
	}
	//namespace detail

	namespace gtc{
	//! GLM_GTC_half_float extension: Add support for half precision floating-point types
	namespace half_float
	{
		//! Type for half-precision floating-point numbers. 
		//! From GLM_GTC_half_float extension.
		typedef detail::half					half;

		//! Vector of 2 half-precision floating-point numbers. 
		//! From GLM_GTC_half_float extension.
		typedef detail::tvec2<detail::half>		hvec2;

		//! Vector of 3 half-precision floating-point numbers.
		//! From GLM_GTC_half_float extension.
		typedef detail::tvec3<detail::half>		hvec3;

		//! Vector of 4 half-precision floating-point numbers. 
		//! From GLM_GTC_half_float extension.
		typedef detail::tvec4<detail::half>		hvec4;

		//! 2 * 2 matrix of half-precision floating-point numbers.
		//! From GLM_GTC_half_float extension.
		typedef detail::tmat2x2<detail::half>	hmat2;

		//! 3 * 3 matrix of half-precision floating-point numbers.
		//! From GLM_GTC_half_float extension.
		typedef detail::tmat3x3<detail::half>	hmat3;

		//! 4 * 4 matrix of half-precision floating-point numbers.
		//! From GLM_GTC_half_float extension.
		typedef detail::tmat4x4<detail::half>	hmat4;

	}//namespace half_float
	}//namespace gtc
}//namespace glm

#define GLM_GTC_half_float namespace gtc::half_float
#ifndef GLM_GTC_GLOBAL
namespace glm {using GLM_GTC_half_float;}
#endif//GLM_GTC_GLOBAL

#include "half_float.inl"

#endif//glm_gtc_half_float
