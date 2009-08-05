#include <stdio.h>
#include <stdlib.h>
#include <unistd.h> // for _exit
#include <sys/types.h>

#include "MMXMatrix.h"

extern void *malloc(size_t size);
extern void *realloc(void *ptr, size_t size);

/*
 * Safe versions of malloc and realloc that will warn and exit if out of memory
 */

void *
Alloc(size_t size)
{
  void *p;

  if ((p = malloc(size)) == NULL)
  {
    fprintf(stderr, "Alloc: could not allocate %1d bytes", size);
    _exit(1);
  }
  return (p);
}

void *
Realloc(void *p, size_t size)
{
  if ((p = realloc(p, size)) == NULL)
  {
    fprintf(stderr, "Realloc: could not allocate %1d bytes", size);
    _exit(1);
  }
  return (p);
}



