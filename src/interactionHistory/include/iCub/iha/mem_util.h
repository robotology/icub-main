#ifndef __IHA__MEM_UTIL_H__
#define __IHA__MEM_UTIL_H__

#include <ace/OS.h>
#include <ace/Log_Msg.h>

/*
 * simple helper template to alloc memory.
 */
template <class T>
inline T* allocAndCheck(int size)
{
	T* t = new T[size];
	ACE_ASSERT (t != 0);
	ACE_OS::memset(t, 0, sizeof(T) * size);
	return t;
}

/*
 *
 */
template <class T>
inline void checkAndDestroy(T* &p) {
	if (p!=0) {
		delete [] p;
		p = 0;
	}
}

#endif
