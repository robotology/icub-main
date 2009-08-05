#include <stdlib.h>
#include <stdio.h>

#include "mmx.h"
#include "matrices_cache.h"



MMXMatricesCachePtr MMXMatricesCacheAlloc(unsigned int nmat, 
		unsigned int nvect, unsigned int nvoid) 
{
	unsigned int i;
	MMXMatricesCache *cache = (MMXMatricesCache*)(malloc(sizeof(MMXMatricesCache)));
	if (cache == NULL) return NULL;
	cache->M = NULL; cache->V = NULL; cache->G = NULL;
	cache->nmatrices = cache->nvectors = cache->nvoidptr = 0;

	if (nmat > 0) {
		cache->M = (MMXMatrix**)(malloc(nmat*sizeof(MMXMatrix*)));
		if (cache->M == NULL) {goto clean_exit;}
		for (i=0;i<nmat;i++) cache->M[i] = NULL;
		cache->nmatrices = nmat;
	}
	
	if (nvect > 0) {
		cache->V = (MM_U_32**)(malloc(nvect*sizeof(MM_U_32*)));
		if (cache->V == NULL) {goto clean_exit;}
		for (i=0;i<nvect;i++) cache->V[i] = NULL;
		cache->nvectors = nvect;
	}

	if (nvoid > 0) {
		cache->G = (void**)(malloc(nvoid*sizeof(MM_U_32*)));
		if (cache->G == NULL) {goto clean_exit;}
		for (i=0;i<nvoid;i++) cache->G[i] = NULL;
		cache->nvoidptr = nvoid;
	}

	return cache;
clean_exit:
	free(cache->M);
	free(cache->V);
	free(cache->G);
	free(cache);
	return NULL;
}

void MMXMatricesCacheFree(MMXMatricesCachePtr vcache) 
{
	MMXMatricesCache* cache = (MMXMatricesCache*)vcache;
	if (cache == NULL) return;
	unsigned int i;
	for (i=0;i<cache->nmatrices;i++) 
		if (cache->M[i]!=NULL) MMXMatrixFree(cache->M[i]);
	free(cache->M);
	
	for (i=0;i<cache->nvectors;i++) 
		if (cache->V[i]!=NULL) free(cache->V[i]);
	free(cache->V);
	
	for (i=0;i<cache->nvoidptr;i++) 
		if (cache->G[i]!=NULL) free(cache->G[i]);
	free(cache->G);
	
	free(cache);
}


