#include "RFStruttura.h"

//********************************************************************
// Struttura per wv ---> indicaRf
//********************************************************************


void setIndRf(indicaRf & ww, Rf * punt, int index)
{
	ww.ind = index;
	ww.point = punt;
}


bool wMinore(indicaRf a , indicaRf b)
{
	return (a.point ->getW() < b.point -> getW());
}
