#ifndef __identification_h__
#define __identification_h__

#include "controller.h"
#include "controller.h"

//exports the following variables
extern Int16 _fout[JN];
extern float wt[JN];
extern bool openloop_identif;

double sin					(double rad);
void compute_identif_wt		(int j);
void compute_sweep_wt		(int j);
void reset_identif			(int j);



#endif