///////////////////////////////////////////////////////////////////////////////////////////////////
// OpenGL Mathematics Copyright (c) 2005 - 2009 G-Truc Creation (www.g-truc.net)
///////////////////////////////////////////////////////////////////////////////////////////////////
// Created : 2008-08-17
// Updated : 2008-08-17
// Licence : This source is under MIT License
// File    : glm/core/type_half.h
///////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef glm_core_type_half
#define glm_core_type_half

#include <cstdlib>

namespace glm
{
	namespace detail
	{
		typedef short hdata;

		float toFloat32(hdata value);
		hdata toFloat16(float value);

		class half
		{
		public: 
			// Constructors
			half();
			half(const half& s);
			
			template <typename U>
			half(U const & s);

			// Cast
			//operator float();
			operator float() const;
			//operator double();
			//operator double() const;

			// Unary updatable operators
			half& operator= (half s);
			half& operator+=(half s);
			half& operator-=(half s);
			half& operator*=(half s);
			half& operator/=(half s);
			half& operator++();
			half& operator--();
	
			float toFloat() const{return toFloat32(data);}

			hdata _data() const{return data;}

		private:
			hdata data;
		};

		void test_half_type();
	}//namespace detail

	detail::half operator- (detail::half s1, detail::half s2);

	detail::half operator* (detail::half s1, detail::half s2);

	detail::half operator/ (detail::half s1, detail::half s2);

	// Unary constant operators
	detail::half operator- (detail::half s);

	detail::half operator-- (detail::half s, int);

	detail::half operator++ (detail::half s, int);

}//namespace glm

glm::detail::half operator+ (glm::detail::half s1, glm::detail::half s2);

#include "type_half.inl"

#endif//glm_core_type_half
