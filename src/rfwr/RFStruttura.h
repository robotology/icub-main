#ifndef STRUTT1_H
#define STRUTT1_H


#include "Rf.h"

struct indicaRf 
{
  Rf * point;
  int ind;
};


void setIndRf(indicaRf & ww, Rf * punt, int index);
bool wMinore(indicaRf a , indicaRf b);

#endif
