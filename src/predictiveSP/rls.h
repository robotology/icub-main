/* Recursive Least Square (RLS) adaptive FIR filter

Copyright (C) 2001-2002 Andrew Rogers

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA */

#include <stdlib.h>

class RLS
{
private:
	double *W;
        double *X;
        double *P;
		double *Pfinal;
		double *Wfinal;
		double *WfinalImp;
		double *PfinalX;
		double *PXX;
		double *PXXP;
		double *XP;
		
       
        double *PX;
        double error;
		double *G;
        int n;
		bool w_final_impost;
        double ff;
public:
	RLS(int order, double mu ,double *coeffs=NULL, double *init=NULL);
        double sample(double *x, double e);
        double * getCoeffs(double *coeffs);
		void  setX(double *valx);
		void setParW(double *valw);
		void setWfinal(double *w);
		void resetWfinal( );
		double RLS::setW(double *valw, double *valx);
        ~RLS();
};
