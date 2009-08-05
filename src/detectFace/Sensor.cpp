#include "Sensor.h"

Sensor::Sensor()//Class constructor
{
	for(int i=0;i<10;i++)
		feature[i]=0.0;
}


Sensor::~Sensor()//Class Destructor
{
}

int Sensor::get_nr_of_features()
{
	return nr_of_features;
}

double Sensor::get_feature(int feature_nr)
{
	return feature[feature_nr];
}
	

