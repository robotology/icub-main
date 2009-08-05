#include "Filtro.h"



Filtro::Filtro(float alfa)
{
	errorP_1=0.0;
	this->alfa=alfa;
	initialized=false;

}



Filtro::~Filtro()
{

	errorP_1=0;
	this->alfa=alfa;

}


double Filtro::filter(double error_C)
{

	double filteredError=0.0;

	if (initialized==false) 
	{
		initialized=true;		
		return error_C;
	}
	else
	{
		filteredError=alfa*errorP_1+error_C*(1-alfa);
		errorP_1=filteredError;
		return filteredError;
	}

}
