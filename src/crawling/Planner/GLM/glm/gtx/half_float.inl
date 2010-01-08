///////////////////////////////////////////////////////////////////////////////////////////////////
// OpenGL Mathematics Copyright (c) 2005 - 2009 G-Truc Creation (www.g-truc.net)
///////////////////////////////////////////////////////////////////////////////////////////////////
// Created : 2005-12-21
// Updated : 2008-10-02
// Licence : This source is under MIT License
// File    : glm/gtx/half.inl
///////////////////////////////////////////////////////////////////////////////////////////////////

namespace glm{
namespace detail{

#ifndef GLM_USE_ANONYMOUS_UNION

//////////////////////////////////////
// hvec2

inline tvec2<half>::size_type tvec2<half>::value_size()
{
	return tvec2<half>::size_type(2);
}

inline bool tvec2<half>::is_vector()
{
	return true;
}

//////////////////////////////////////
// Accesses

inline half& tvec2<half>::operator[](tvec2<half>::size_type i)
{
	assert( i >= tvec2<half>::size_type(0) && 
			i < tvec2<half>::value_size());
	return (&x)[i];
}

inline half tvec2<half>::operator[](tvec2<half>::size_type i) const
{
	assert( i >= tvec2<half>::size_type(0) && 
			i < tvec2<half>::value_size());
	return (&x)[i];
}

//////////////////////////////////////
// Implicit basic constructors

inline tvec2<half>::tvec2() :
	x(half(0.f)),
	y(half(0.f))
{}

inline tvec2<half>::tvec2(tvec2<half> const & v) :
	x(v.x),
	y(v.y)
{}

//////////////////////////////////////
// Explicit basic constructors

inline tvec2<half>::tvec2(half s) :
	x(s),
	y(s)
{}

inline tvec2<half>::tvec2(half s1, half s2) :
	x(s1),
	y(s2)
{}

//////////////////////////////////////
// Swizzle constructors

inline tvec2<half>::tvec2(tref2<half> const & r) :
	x(r.x),
	y(r.y)
{}

//////////////////////////////////////
// Convertion scalar constructors

template <typename U> 
inline tvec2<half>::tvec2(U x) :
	x(half(x)),
	y(half(x))
{}

template <typename U, typename V> 
inline tvec2<half>::tvec2(U x, V y) :
	x(half(x)),
	y(half(y))
{}

//////////////////////////////////////
// Convertion vector constructors

template <typename U> 
inline tvec2<half>::tvec2(tvec2<U> const & v) :
	x(half(v.x)),
	y(half(v.y))
{}

template <typename U> 
inline tvec2<half>::tvec2(tvec3<U> const & v) :
	x(half(v.x)),
	y(half(v.y))
{}

template <typename U> 
inline tvec2<half>::tvec2(tvec4<U> const & v) :
	x(half(v.x)),
	y(half(v.y))
{}

//////////////////////////////////////
// Unary arithmetic operators

inline tvec2<half>& tvec2<half>::operator= (tvec2<half> const & v)
{
	this->x = v.x;
	this->y = v.y;
	return *this;
}

inline tvec2<half>& tvec2<half>::operator+=(half s)
{
	this->x += s;
	this->y += s;
	return *this;
}

inline tvec2<half>& tvec2<half>::operator+=(tvec2<half> const & v)
{
	this->x += v.x;
	this->y += v.y;
	return *this;
}

inline tvec2<half>& tvec2<half>::operator-=(half s)
{
	this->x -= s;
	this->y -= s;
	return *this;
}

inline tvec2<half>& tvec2<half>::operator-=(tvec2<half> const & v)
{
	this->x -= v.x;
	this->y -= v.y;
	return *this;
}

inline tvec2<half>& tvec2<half>::operator*=(half s)
{
	this->x *= s;
	this->y *= s;
	return *this;
}

inline tvec2<half>& tvec2<half>::operator*=(tvec2<half> const & v)
{
	this->x *= v.x;
	this->y *= v.y;
	return *this;
}

inline tvec2<half>& tvec2<half>::operator/=(half s)
{
	this->x /= s;
	this->y /= s;
	return *this;
}

inline tvec2<half>& tvec2<half>::operator/=(tvec2<half> const & v)
{
	this->x /= v.x;
	this->y /= v.y;
	return *this;
}

inline tvec2<half>& tvec2<half>::operator++()
{
	++this->x;
	++this->y;
	return *this;
}

inline tvec2<half>& tvec2<half>::operator--()
{
	--this->x;
	--this->y;
	return *this;
}

//////////////////////////////////////
// Swizzle operators

inline half tvec2<half>::swizzle(comp x) const
{
	return (*this)[x];
}

inline tvec2<half> tvec2<half>::swizzle(comp x, comp y) const
{
	return tvec2<half>(
		(*this)[x],
		(*this)[y]);
}

inline tvec3<half> tvec2<half>::swizzle(comp x, comp y, comp z) const
{
	return tvec3<half>(
		(*this)[x],
		(*this)[y],
		(*this)[z]);
}

inline tvec4<half> tvec2<half>::swizzle(comp x, comp y, comp z, comp w) const
{
	return tvec4<half>(
		(*this)[x],
		(*this)[y],
		(*this)[z],
		(*this)[w]);
}

inline tref2<half> tvec2<half>::swizzle(comp x, comp y)
{
	return tref2<half>(
		(*this)[x],
		(*this)[y]);
}

//////////////////////////////////////
// hvec3

inline tvec3<half>::size_type tvec3<half>::value_size()
{
	return tvec3<half>::size_type(3);
}

inline bool tvec3<half>::is_vector()
{
	return true;
}

//////////////////////////////////////
// Accesses

inline half& tvec3<half>::operator[](tvec3<half>::size_type i)
{
	assert( i >= tvec3<half>::size_type(0) && 
			i < tvec3<half>::value_size());

	return (&x)[i];
}

inline half tvec3<half>::operator[](tvec3<half>::size_type i) const
{
	assert( i >= tvec3<half>::size_type(0) && 
			i < tvec3<half>::value_size());

	return (&x)[i];
}

//////////////////////////////////////
// Implicit basic constructors

inline tvec3<half>::tvec3() :
	x(half(0)),
	y(half(0)),
	z(half(0))
{}

inline tvec3<half>::tvec3(tvec3<half> const & v) :
	x(v.x),
	y(v.y),
	z(v.z)
{}

//////////////////////////////////////
// Explicit basic constructors

inline tvec3<half>::tvec3(half s) :
	x(s),
	y(s),
	z(s)
{}

inline tvec3<half>::tvec3(half s0, half s1, half s2) :
	x(s0),
	y(s1),
	z(s2)
{}

//////////////////////////////////////
// Swizzle constructors

inline tvec3<half>::tvec3(tref3<half> const & r) :
	x(r.x),
	y(r.y),
	z(r.z)
{}

//////////////////////////////////////
// Convertion scalar constructors

template <typename U> 
inline tvec3<half>::tvec3(U x) :
	x(half(x)),
	y(half(x)),
	z(half(x))
{}

template <typename A, typename B, typename C> 
inline tvec3<half>::tvec3(A x, B y, C z) :
	x(half(x)),
	y(half(y)),
	z(half(z))
{}

//////////////////////////////////////
// Convertion vector constructors

template <typename A, typename B> 
inline tvec3<half>::tvec3(tvec2<A> const & v, B s) :
	x(half(v.x)),
	y(half(v.y)),
	z(half(s))
{}

template <typename A, typename B> 
inline tvec3<half>::tvec3(A s, tvec2<B> const & v) :
	x(half(s)),
	y(half(v.x)),
	z(half(v.y))
{}

template <typename U> 
inline tvec3<half>::tvec3(tvec3<U> const & v) :
	x(half(v.x)),
	y(half(v.y)),
	z(half(v.z))
{}

template <typename U> 
inline tvec3<half>::tvec3(tvec4<U> const & v) :
	x(half(v.x)),
	y(half(v.y)),
	z(half(v.z))
{}

//////////////////////////////////////
// Unary arithmetic operators

inline tvec3<half>& tvec3<half>::operator= (tvec3<half> const & v)
{
	this->x = v.x;
	this->y = v.y;
	this->z = v.z;
	return *this;
}

inline tvec3<half>& tvec3<half>::operator+=(half s)
{
	this->x += s;
	this->y += s;
	this->z += s;
	return *this;
}

inline tvec3<half>& tvec3<half>::operator+=(tvec3<half> const & v)
{
	this->x += v.x;
	this->y += v.y;
	this->z += v.z;
	return *this;
}

inline tvec3<half>& tvec3<half>::operator-=(half s)
{
	this->x -= s;
	this->y -= s;
	this->z -= s;
	return *this;
}

inline tvec3<half>& tvec3<half>::operator-=(tvec3<half> const & v)
{
	this->x -= v.x;
	this->y -= v.y;
	this->z -= v.z;
	return *this;
}

inline tvec3<half>& tvec3<half>::operator*=(half s)
{
	this->x *= s;
	this->y *= s;
	this->z *= s;
	return *this;
}

inline tvec3<half>& tvec3<half>::operator*=(tvec3<half> const & v)
{
	this->x *= v.x;
	this->y *= v.y;
	this->z *= v.z;
	return *this;
}

inline tvec3<half>& tvec3<half>::operator/=(half s)
{
	this->x /= s;
	this->y /= s;
	this->z /= s;
	return *this;
}

inline tvec3<half>& tvec3<half>::operator/=(tvec3<half> const & v)
{
	this->x /= v.x;
	this->y /= v.y;
	this->z /= v.z;
	return *this;
}

inline tvec3<half>& tvec3<half>::operator++()
{
	++this->x;
	++this->y;
	++this->z;
	return *this;
}

inline tvec3<half>& tvec3<half>::operator--()
{
	--this->x;
	--this->y;
	--this->z;
	return *this;
}

//////////////////////////////////////
// Swizzle operators

inline half tvec3<half>::swizzle(comp x) const
{
	return (*this)[x];
}

inline tvec2<half> tvec3<half>::swizzle(comp x, comp y) const
{
	return tvec2<half>(
		(*this)[x],
		(*this)[y]);
}

inline tvec3<half> tvec3<half>::swizzle(comp x, comp y, comp z) const
{
	return tvec3<half>(
		(*this)[x],
		(*this)[y],
		(*this)[z]);
}

inline tvec4<half> tvec3<half>::swizzle(comp x, comp y, comp z, comp w) const
{
	return tvec4<half>(
		(*this)[x],
		(*this)[y],
		(*this)[z],
		(*this)[w]);
}

inline tref3<half> tvec3<half>::swizzle(comp x, comp y, comp z)
{
	return tref3<half>(
		(*this)[x],
		(*this)[y],
		(*this)[z]);
}

//////////////////////////////////////
// hvec4

inline tvec4<half>::size_type tvec4<half>::value_size()
{
	return tvec4<half>::size_type(4);
}

inline bool tvec4<half>::is_vector()
{
	return true;
}

//////////////////////////////////////
// Accesses

inline half& tvec4<half>::operator[](tvec4<half>::size_type i)
{
	assert( i >= tvec4<half>::size_type(0) && 
			i < tvec4<half>::value_size());

	return (&x)[i];
}

inline half tvec4<half>::operator[](tvec4<half>::size_type i) const
{
	assert( i >= tvec4<half>::size_type(0) && 
			i < tvec4<half>::value_size());

	return (&x)[i];
}

//////////////////////////////////////
// Implicit basic constructors

inline tvec4<half>::tvec4() :
	x(half(0)),
	y(half(0)),
	z(half(0)),
	w(half(0))
{}

inline tvec4<half>::tvec4(const tvec4<half>& v) :
	x(v.x),
	y(v.y),
	z(v.z),
	w(v.w)
{}

//////////////////////////////////////
// Explicit basic constructors

inline tvec4<half>::tvec4(half s) :
	x(s),
	y(s),
	z(s),
	w(s)
{}

inline tvec4<half>::tvec4(half s1, half s2, half s3, half s4) :
	x(s1),
	y(s2),
	z(s3),
	w(s4)
{}

//////////////////////////////////////
// Swizzle constructors

inline tvec4<half>::tvec4(tref4<half> const & r) :
	x(r.x),
	y(r.y),
	z(r.z),
	w(r.w)
{}

//////////////////////////////////////
// Convertion scalar constructors

template <typename U> 
inline tvec4<half>::tvec4(U x) :
	x(half(x)),
	y(half(x)),
	z(half(x)),
	w(half(x))
{}

template <typename A, typename B, typename C, typename D> 
inline tvec4<half>::tvec4(A x, B y, C z, D w) :
	x(half(x)),
	y(half(y)),
	z(half(z)),
	w(half(w))
{}

//////////////////////////////////////
// Convertion vector constructors

template <typename A, typename B, typename C> 
inline tvec4<half>::tvec4(const tvec2<A>& v, B s1, C s2) :
	x(half(v.x)),
	y(half(v.y)),
	z(half(s1)),
	w(half(s2))
{}

template <typename A, typename B, typename C> 
inline tvec4<half>::tvec4(A s1, const tvec2<B>& v, C s2) :
	x(half(s1)),
	y(half(v.x)),
	z(half(v.y)),
	w(half(s2))
{}

template <typename A, typename B, typename C> 
inline tvec4<half>::tvec4(A s1, B s2, const tvec2<C>& v) :
	x(half(s1)),
	y(half(s2)),
	z(half(v.x)),
	w(half(v.y))
{}

template <typename A, typename B> 
inline tvec4<half>::tvec4(const tvec3<A>& v, B s) :
	x(half(v.x)),
	y(half(v.y)),
	z(half(v.z)),
	w(half(s))
{}

template <typename A, typename B> 
inline tvec4<half>::tvec4(A s, const tvec3<B>& v) :
	x(half(s)),
	y(half(v.x)),
	z(half(v.y)),
	w(half(v.z))
{}

template <typename A, typename B> 
inline tvec4<half>::tvec4(const tvec2<A>& v1, const tvec2<B>& v2) :
	x(half(v1.x)),
	y(half(v1.y)),
	z(half(v2.x)),
	w(half(v2.y))
{}

template <typename U> 
inline tvec4<half>::tvec4(const tvec4<U>& v) :
	x(half(v.x)),
	y(half(v.y)),
	z(half(v.z)),
	w(half(v.w))
{}

//////////////////////////////////////
// Unary arithmetic operators

inline tvec4<half>& tvec4<half>::operator= (const tvec4<half>& v)
{
	this->x = v.x;
	this->y = v.y;
	this->z = v.z;
	this->w = v.w;
	return *this;
}

inline tvec4<half>& tvec4<half>::operator+=(half s)
{
	this->x += s;
	this->y += s;
	this->z += s;
	this->w += s;
	return *this;
}

inline tvec4<half>& tvec4<half>::operator+=(const tvec4<half>& v)
{
	this->x += v.x;
	this->y += v.y;
	this->z += v.z;
	this->w += v.w;
	return *this;
}

inline tvec4<half>& tvec4<half>::operator-=(half s)
{
	this->x -= s;
	this->y -= s;
	this->z -= s;
	this->w -= s;
	return *this;
}

inline tvec4<half>& tvec4<half>::operator-=(const tvec4<half>& v)
{
	this->x -= v.x;
	this->y -= v.y;
	this->z -= v.z;
	this->w -= v.w;
	return *this;
}

inline tvec4<half>& tvec4<half>::operator*=(half s)
{
	this->x *= s;
	this->y *= s;
	this->z *= s;
	this->w *= s;
	return *this;
}

inline tvec4<half>& tvec4<half>::operator*=(const tvec4<half>& v)
{
	this->x *= v.x;
	this->y *= v.y;
	this->z *= v.z;
	this->w *= v.w;
	return *this;
}

inline tvec4<half>& tvec4<half>::operator/=(half s)
{
	this->x /= s;
	this->y /= s;
	this->z /= s;
	this->w /= s;
	return *this;
}

inline tvec4<half>& tvec4<half>::operator/=(const tvec4<half>& v)
{
	this->x /= v.x;
	this->y /= v.y;
	this->z /= v.z;
	this->w /= v.w;
	return *this;
}

inline tvec4<half>& tvec4<half>::operator++()
{
	++this->x;
	++this->y;
	++this->z;
	++this->w;
	return *this;
}

inline tvec4<half>& tvec4<half>::operator--()
{
	--this->x;
	--this->y;
	--this->z;
	--this->w;
	return *this;
}

//////////////////////////////////////
// Swizzle operators

inline half tvec4<half>::swizzle(comp x) const
{
	return (*this)[x];
}

inline tvec2<half> tvec4<half>::swizzle(comp x, comp y) const
{
	return tvec2<half>(
		(*this)[x],
		(*this)[y]);
}

inline tvec3<half> tvec4<half>::swizzle(comp x, comp y, comp z) const
{
	return tvec3<half>(
		(*this)[x],
		(*this)[y],
		(*this)[z]);
}

inline tvec4<half> tvec4<half>::swizzle(comp x, comp y, comp z, comp w) const
{
	return tvec4<half>(
		(*this)[x],
		(*this)[y],
		(*this)[z],
		(*this)[w]);
}

inline tref4<half> tvec4<half>::swizzle(comp x, comp y, comp z, comp w)
{
	return tref4<half>(
		(*this)[x],
		(*this)[y],
		(*this)[z],
		(*this)[w]);
}

#endif//GLM_USE_ANONYMOUS_UNION

}//namespace detail
}//namespace glm
