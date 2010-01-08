///////////////////////////////////////////////////////////////////////////////////////////////////
// OpenGL Mathematics Copyright (c) 2005 - 2009 G-Truc Creation (www.g-truc.net)
///////////////////////////////////////////////////////////////////////////////////////////////////
// Created : 2008-08-23
// Updated : 2008-09-09
// Licence : This source is under MIT License
// File    : glm/core/type_tvec4.inl
///////////////////////////////////////////////////////////////////////////////////////////////////

namespace glm
{
	namespace detail
	{
		template <typename valType>
		typename tvec4<valType>::size_type tvec4<valType>::value_size()
		{
			return typename tvec4<valType>::size_type(4);
		}

		template <typename valType> 
		bool tvec4<valType>::is_vector()
		{
			return true;
		}

		//////////////////////////////////////
		// Accesses

		template <typename valType>
		inline valType& tvec4<valType>::operator[](typename tvec4<valType>::size_type i)
		{
			assert( i >= typename tvec4<valType>::size_type(0) && 
					i < tvec4<valType>::value_size());

			return (&x)[i];
		}

		template <typename valType>
		inline valType const & tvec4<valType>::operator[](typename tvec4<valType>::size_type i) const
		{
			assert( i >= typename tvec4<valType>::size_type(0) && 
					i < tvec4<valType>::value_size());

			return (&x)[i];
		}

		//////////////////////////////////////
		// Implicit basic constructors

		template <typename valType>
		inline tvec4<valType>::tvec4() :
			x(valType(0)),
			y(valType(0)),
			z(valType(0)),
			w(valType(0))
		{}

		template <typename valType>
		inline tvec4<valType>::tvec4(const tvec4<valType>& v) :
			x(v.x),
			y(v.y),
			z(v.z),
			w(v.w)
		{}

		//////////////////////////////////////
		// Explicit basic constructors

		template <typename valType>
		inline tvec4<valType>::tvec4(valType s) :
			x(s),
			y(s),
			z(s),
			w(s)
		{}

		template <typename valType>
		inline tvec4<valType>::tvec4(valType s1, valType s2, valType s3, valType s4) :
			x(s1),
			y(s2),
			z(s3),
			w(s4)
		{}

		//////////////////////////////////////
		// Swizzle constructors

		template <typename valType>
		inline tvec4<valType>::tvec4(const tref4<valType>& r) :
			x(r.x),
			y(r.y),
			z(r.z),
			w(r.w)
		{}

		//////////////////////////////////////
		// Convertion scalar constructors
		
		template <typename valType>
		template <typename U> 
		inline tvec4<valType>::tvec4(U x) :
			x(valType(x)),
			y(valType(x)),
			z(valType(x)),
			w(valType(x))
		{}

		template <typename valType>
		template <typename A, typename B, typename C, typename D> 
		inline tvec4<valType>::tvec4(A x, B y, C z, D w) :
			x(valType(x)),
			y(valType(y)),
			z(valType(z)),
			w(valType(w))
		{}

		//////////////////////////////////////
		// Convertion vector constructors

		template <typename valType>
		template <typename A, typename B, typename C> 
		inline tvec4<valType>::tvec4(const tvec2<A>& v, B s1, C s2) :
			x(valType(v.x)),
			y(valType(v.y)),
			z(valType(s1)),
			w(valType(s2))
		{}

		template <typename valType>
		template <typename A, typename B, typename C> 
		inline tvec4<valType>::tvec4
		(
			A s1, 
			tvec2<B> const & v, 
			C s2
		) :
			x(valType(s1)),
			y(valType(v.x)),
			z(valType(v.y)),
			w(valType(s2))
		{}

		template <typename valType>
		template <typename A, typename B, typename C> 
		inline tvec4<valType>::tvec4
		(
			A s1, 
			B s2, 
			tvec2<C> const & v
		) :
			x(valType(s1)),
			y(valType(s2)),
			z(valType(v.x)),
			w(valType(v.y))
		{}

		template <typename valType>
		template <typename A, typename B> 
		inline tvec4<valType>::tvec4
		(
			tvec3<A> const & v, 
			B s
		) :
			x(valType(v.x)),
			y(valType(v.y)),
			z(valType(v.z)),
			w(valType(s))
		{}

		template <typename valType>
		template <typename A, typename B> 
		inline tvec4<valType>::tvec4
		(
			A s, 
			tvec3<B> const & v
		) :
			x(valType(s)),
			y(valType(v.x)),
			z(valType(v.y)),
			w(valType(v.z))
		{}

		template <typename valType>
		template <typename A, typename B> 
		inline tvec4<valType>::tvec4
		(
			tvec2<A> const & v1, 
			tvec2<B> const & v2
		) :
			x(valType(v1.x)),
			y(valType(v1.y)),
			z(valType(v2.x)),
			w(valType(v2.y))
		{}

		template <typename valType>
		template <typename U> 
		inline tvec4<valType>::tvec4(const tvec4<U>& v) :
			x(valType(v.x)),
			y(valType(v.y)),
			z(valType(v.z)),
			w(valType(v.w))
		{}

		//////////////////////////////////////
		// Unary arithmetic operators

		template <typename valType>
		inline tvec4<valType>& tvec4<valType>::operator= (const tvec4<valType>& v)
		{
			this->x = v.x;
			this->y = v.y;
			this->z = v.z;
			this->w = v.w;
			return *this;
		}

		template <typename valType>
		inline tvec4<valType>& tvec4<valType>::operator+=(valType s)
		{
			this->x += s;
			this->y += s;
			this->z += s;
			this->w += s;
			return *this;
		}

		template <typename valType>
		inline tvec4<valType>& tvec4<valType>::operator+=(const tvec4<valType>& v)
		{
			this->x += v.x;
			this->y += v.y;
			this->z += v.z;
			this->w += v.w;
			return *this;
		}

		template <typename valType>
		inline tvec4<valType>& tvec4<valType>::operator-=(valType s)
		{
			this->x -= s;
			this->y -= s;
			this->z -= s;
			this->w -= s;
			return *this;
		}

		template <typename valType>
		inline tvec4<valType>& tvec4<valType>::operator-=(const tvec4<valType>& v)
		{
			this->x -= v.x;
			this->y -= v.y;
			this->z -= v.z;
			this->w -= v.w;
			return *this;
		}

		template <typename valType>
		inline tvec4<valType>& tvec4<valType>::operator*=(valType s)
		{
			this->x *= s;
			this->y *= s;
			this->z *= s;
			this->w *= s;
			return *this;
		}

		template <typename valType>
		inline tvec4<valType>& tvec4<valType>::operator*=(const tvec4<valType>& v)
		{
			this->x *= v.x;
			this->y *= v.y;
			this->z *= v.z;
			this->w *= v.w;
			return *this;
		}

		template <typename valType>
		inline tvec4<valType>& tvec4<valType>::operator/=(valType s)
		{
			this->x /= s;
			this->y /= s;
			this->z /= s;
			this->w /= s;
			return *this;
		}

		template <typename valType>
		inline tvec4<valType>& tvec4<valType>::operator/=(const tvec4<valType>& v)
		{
			this->x /= v.x;
			this->y /= v.y;
			this->z /= v.z;
			this->w /= v.w;
			return *this;
		}

		template <typename valType>
		inline tvec4<valType>& tvec4<valType>::operator++()
		{
			++this->x;
			++this->y;
			++this->z;
			++this->w;
			return *this;
		}

		template <typename valType>
		inline tvec4<valType>& tvec4<valType>::operator--()
		{
			--this->x;
			--this->y;
			--this->z;
			--this->w;
			return *this;
		}

		//////////////////////////////////////
		// Unary bit operators

		template <typename valType>
		inline tvec4<valType>& tvec4<valType>::operator%=(valType s)
		{
			this->x %= s;
			this->y %= s;
			this->z %= s;
			this->w %= s;
			return *this;
		}

		template <typename valType>
		inline tvec4<valType>& tvec4<valType>::operator%=(const tvec4<valType>& v)
		{
			this->x %= v.x;
			this->y %= v.y;
			this->z %= v.z;
			this->w %= v.w;
			return *this;
		}

		template <typename valType>
		inline tvec4<valType>& tvec4<valType>::operator&=(valType s)
		{
			this->x &= s;
			this->y &= s;
			this->z &= s;
			this->w &= s;
			return *this;
		}

		template <typename valType>
		inline tvec4<valType>& tvec4<valType>::operator&=(const tvec4<valType>& v)
		{
			this->x &= v.x;
			this->y &= v.y;
			this->z &= v.z;
			this->w &= v.w;
			return *this;
		}

		template <typename valType>
		inline tvec4<valType>& tvec4<valType>::operator|=(valType s)
		{
			this->x |= s;
			this->y |= s;
			this->z |= s;
			this->w |= s;
			return *this;
		}

		template <typename valType>
		inline tvec4<valType>& tvec4<valType>::operator|=(const tvec4<valType>& v)
		{
			this->x |= v.x;
			this->y |= v.y;
			this->z |= v.z;
			this->w |= v.w;
			return *this;
		}

		template <typename valType>
		inline tvec4<valType>& tvec4<valType>::operator^=(valType s)
		{
			this->x ^= s;
			this->y ^= s;
			this->z ^= s;
			this->w ^= s;
			return *this;
		}

		template <typename valType>
		inline tvec4<valType>& tvec4<valType>::operator^=(const tvec4<valType>& v)
		{
			this->x ^= v.x;
			this->y ^= v.y;
			this->z ^= v.z;
			this->w ^= v.w;
			return *this;
		}

		template <typename valType>
		inline tvec4<valType>& tvec4<valType>::operator<<=(valType s)
		{
			this->x <<= s;
			this->y <<= s;
			this->z <<= s;
			this->w <<= s;
			return *this;
		}

		template <typename valType>
		inline tvec4<valType>& tvec4<valType>::operator<<=(const tvec4<valType>& v)
		{
			this->x <<= v.x;
			this->y <<= v.y;
			this->z <<= v.z;
			this->w <<= v.w;
			return *this;
		}

		template <typename valType>
		inline tvec4<valType>& tvec4<valType>::operator>>=(valType s)
		{
			this->x >>= s;
			this->y >>= s;
			this->z >>= s;
			this->w >>= s;
			return *this;
		}

		template <typename valType>
		inline tvec4<valType>& tvec4<valType>::operator>>=(const tvec4<valType>& v)
		{
			this->x >>= v.x;
			this->y >>= v.y;
			this->z >>= v.z;
			this->w >>= v.w;
			return *this;
		}

		//////////////////////////////////////
		// Swizzle operators

		template <typename valType>
		inline valType tvec4<valType>::swizzle(comp x) const
		{
			return (*this)[x];
		}

		template <typename valType>
		inline tvec2<valType> const tvec4<valType>::swizzle(comp x, comp y) const
		{
			return tvec2<valType>(
				(*this)[x],
				(*this)[y]);
		}

		template <typename valType>
		inline tvec3<valType> const tvec4<valType>::swizzle(comp x, comp y, comp z) const
		{
			return tvec3<valType>(
				(*this)[x],
				(*this)[y],
				(*this)[z]);
		}

		template <typename valType>
		inline tvec4<valType> const tvec4<valType>::swizzle(comp x, comp y, comp z, comp w) const
		{
			return tvec4<valType>(
				(*this)[x],
				(*this)[y],
				(*this)[z],
				(*this)[w]);
		}

		template <typename valType>
		inline tref4<valType> tvec4<valType>::swizzle(comp x, comp y, comp z, comp w)
		{
			return tref4<valType>(
				(*this)[x],
				(*this)[y],
				(*this)[z],
				(*this)[w]);
		}

		//////////////////////////////////////
		// Binary arithmetic operators

		template <typename T> 
		inline tvec4<T> operator+ (const tvec4<T>& v, T s)
		{
			return tvec4<T>(
				v.x + s,
				v.y + s,
				v.z + s,
				v.w + s);
		}

		template <typename T> 
		inline tvec4<T> operator+ (T s, const tvec4<T>& v)
		{
			return tvec4<T>(
				s + v.x,
				s + v.y,
				s + v.z,
				s + v.w);
		}

		template <typename T> 
		inline tvec4<T> operator+ (const tvec4<T>& v1, const tvec4<T>& v2)
		{
			return tvec4<T>(
				v1.x + v2.x,
				v1.y + v2.y,
				v1.z + v2.z,
				v1.w + v2.w);
		}

		//operator-
		template <typename T> 
		inline tvec4<T> operator- (const tvec4<T>& v, T s)
		{
			return tvec4<T>(
				v.x - s,
				v.y - s,
				v.z - s,
				v.w - s);
		}

		template <typename T> 
		inline tvec4<T> operator- (T s, const tvec4<T>& v)
		{
			return tvec4<T>(
				s - v.x,
				s - v.y,
				s - v.z,
				s - v.w);
		}

		template <typename T> 
		inline tvec4<T> operator- (const tvec4<T>& v1, const tvec4<T>& v2)
		{
			return tvec4<T>(
				v1.x - v2.x,
				v1.y - v2.y,
				v1.z - v2.z,
				v1.w - v2.w);
		}

		//operator*
		template <typename T> 
		inline tvec4<T> operator* (const tvec4<T>& v, T s)
		{
			return tvec4<T>(
				v.x * s,
				v.y * s,
				v.z * s,
				v.w * s);
		}

		template <typename T> 
		inline tvec4<T> operator* (T s, const tvec4<T>& v)
		{
			return tvec4<T>(
				s * v.x,
				s * v.y,
				s * v.z,
				s * v.w);
		}

		template <typename T> 
		inline tvec4<T> operator* (const tvec4<T>& v1, const tvec4<T> & v2)
		{
			return tvec4<T>(
				v1.x * v2.x,
				v1.y * v2.y,
				v1.z * v2.z,
				v1.w * v2.w);
		}

		//operator/
		template <typename T> 
		inline tvec4<T> operator/ (const tvec4<T>& v, T s)
		{
			return tvec4<T>(
				v.x / s,
				v.y / s,
				v.z / s,
				v.w / s);
		}

		template <typename T> 
		inline tvec4<T> operator/ (T s, const tvec4<T>& v)
		{
			return tvec4<T>(
				s / v.x,
				s / v.y,
				s / v.z,
				s / v.w);
		}

		template <typename T> 
		inline tvec4<T> operator/ (const tvec4<T>& v1, const tvec4<T>& v2)
		{
			return tvec4<T>(
				v1.x / v2.x,
				v1.y / v2.y,
				v1.z / v2.z,
				v1.w / v2.w);
		}

		// Unary constant operators
		template <typename T> 
		inline tvec4<T> operator- (const tvec4<T>& v)
		{
			return tvec4<T>(
				-v.x, 
				-v.y, 
				-v.z, 
				-v.w);
		}

		template <typename T> 
		inline tvec4<T> operator++ (const tvec4<T>& v, int)
		{
			return tvec4<T>(
				v.x + T(1), 
				v.y + T(1), 
				v.z + T(1), 
				v.w + T(1));
		}

		template <typename T> 
		inline const tvec4<T> operator-- (const tvec4<T>& v, int)
		{
			return tvec4<T>(
				v.x - T(1), 
				v.y - T(1), 
				v.z - T(1), 
				v.w - T(1));
		}

		//////////////////////////////////////
		// Binary bit operators

		template <typename T>
		inline tvec4<T> operator% (const tvec4<T>& v, T s)
		{
			return tvec4<T>(
				v.x % s,
				v.y % s,
				v.z % s,
				v.w % s);
		}

		template <typename T>
		inline tvec4<T> operator% (T s, const tvec4<T>& v)
		{
			return tvec4<T>(
				s % v.x,
				s % v.y,
				s % v.z,
				s % v.w);
		}

		template <typename T>
		inline tvec4<T> operator% (const tvec4<T>& v1, const tvec4<T>& v2)
		{
			return tvec4<T>(
				v1.x % v2.x,
				v1.y % v2.y,
				v1.z % v2.z,
				v1.w % v2.w);
		}

		template <typename T>
		inline tvec4<T> operator& (const tvec4<T>& v, T s)
		{
			return tvec4<T>(
				v.x & s,
				v.y & s,
				v.z & s,
				v.w & s);
		}

		template <typename T>
		inline tvec4<T> operator& (T s, const tvec4<T>& v)
		{
			return tvec4<T>(
				s & v.x,
				s & v.y,
				s & v.z,
				s & v.w);
		}

		template <typename T>
		inline tvec4<T> operator& (const tvec4<T>& v1, const tvec4<T>& v2)
		{
			return tvec4<T>(
				v1.x & v2.x,
				v1.y & v2.y,
				v1.z & v2.z,
				v1.w & v2.w);
		}

		template <typename T>
		inline tvec4<T> operator| (const tvec4<T>& v, T s)
		{
			return tvec4<T>(
				v.x | s,
				v.y | s,
				v.z | s,
				v.w | s);
		}

		template <typename T>
		inline tvec4<T> operator| (T s, const tvec4<T>& v)
		{
			return tvec4<T>(
				s | v.x,
				s | v.y,
				s | v.z,
				s | v.w);
		}

		template <typename T>
		inline tvec4<T> operator| (const tvec4<T>& v1, const tvec4<T>& v2)
		{
			return tvec4<T>(
				v1.x | v2.x,
				v1.y | v2.y,
				v1.z | v2.z,
				v1.w | v2.w);
		}
		
		template <typename T>
		inline tvec4<T> operator^ (const tvec4<T>& v, T s)
		{
			return tvec4<T>(
				v.x ^ s,
				v.y ^ s,
				v.z ^ s,
				v.w ^ s);
		}

		template <typename T>
		inline tvec4<T> operator^ (T s, const tvec4<T>& v)
		{
			return tvec4<T>(
				s ^ v.x,
				s ^ v.y,
				s ^ v.z,
				s ^ v.w);
		}

		template <typename T>
		inline tvec4<T> operator^ (const tvec4<T>& v1, const tvec4<T>& v2)
		{
			return tvec4<T>(
				v1.x ^ v2.x,
				v1.y ^ v2.y,
				v1.z ^ v2.z,
				v1.w ^ v2.w);
		}

		template <typename T>
		inline tvec4<T> operator<< (const tvec4<T>& v, T s)
		{
			return tvec4<T>(
				v.x << s,
				v.y << s,
				v.z << s,
				v.w << s);
		}

		template <typename T>
		inline tvec4<T> operator<< (T s, const tvec4<T>& v)
		{
			return tvec4<T>(
				s << v.x,
				s << v.y,
				s << v.z,
				s << v.w);
		}

		template <typename T>
		inline tvec4<T> operator<< (const tvec4<T>& v1, const tvec4<T>& v2)
		{
			return tvec4<T>(
				v1.x << v2.x,
				v1.y << v2.y,
				v1.z << v2.z,
				v1.w << v2.w);
		}

		template <typename T>
		inline tvec4<T> operator>> (const tvec4<T>& v, T s)
		{
			return tvec4<T>(
				v.x >> s,
				v.y >> s,
				v.z >> s,
				v.w >> s);
		}

		template <typename T>
		inline tvec4<T> operator>> (T s, const tvec4<T>& v)
		{
			return tvec4<T>(
				s >> v.x,
				s >> v.y,
				s >> v.z,
				s >> v.w);
		}

		template <typename T>
		inline tvec4<T> operator>> (const tvec4<T>& v1, const tvec4<T>& v2)
		{
			return tvec4<T>(
				v1.x >> v2.x,
				v1.y >> v2.y,
				v1.z >> v2.z,
				v1.w >> v2.w);
		}

		template <typename T> 
		inline const tvec4<T> operator~ (const tvec4<T>& v)
		{
			return tvec4<T>(
				~v.x,
				~v.y,
				~v.z,
				~v.w);
		}

		//////////////////////////////////////
		// tref definition

		template <typename T> 
		tref4<T>::tref4(T& x, T& y, T& z, T& w) :
			x(x),
			y(y),
			z(z),
			w(w)
		{}

		template <typename T> 
		tref4<T>::tref4(const tref4<T>& r) :
			x(r.x),
			y(r.y),
			z(r.z),
			w(r.w)
		{}

		template <typename T> 
		tref4<T>::tref4(const tvec4<T>& v) :
			x(v.x),
			y(v.y),
			z(v.z),
			w(v.w)
		{}

		template <typename T> 
		tref4<T>& tref4<T>::operator= (const tref4<T>& r)
		{
			x = r.x;
			y = r.y;
			z = r.z;
			w = r.w;
			return *this;
		}

		template <typename T> 
		tref4<T>& tref4<T>::operator= (const tvec4<T>& v)
		{
			x = v.x;
			y = v.y;
			z = v.z;
			w = v.w;
			return *this;
		}

	}//namespace detail
}//namespace glm
