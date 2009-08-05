#ifndef _MATRICES_CACHE_H
#define _MATRICES_CACHE_H

#ifdef __cplusplus
extern "C" {
#endif


#include "libmmx.h"

typedef struct {
	MMXMatrix ** M;
	unsigned int nmatrices;
	MM_U_32 ** V;
	unsigned int nvectors;
	void ** G;
	unsigned int nvoidptr;
} MMXMatricesCache;

#ifndef DEPTHMAP_H
// already defined in depthmap.h
typedef void* MMXMatricesCachePtr;

extern MMXMatricesCachePtr MMXMatricesCacheAlloc(unsigned int nmat,
		unsigned int nvect, unsigned int nvoid);

extern void MMXMatricesCacheFree(MMXMatricesCachePtr cache);
#endif

#ifdef __cplusplus
}
#endif


#endif // _MATRICES_CACHE_H
