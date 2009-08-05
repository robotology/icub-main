#include "SoundSourceEstimation.h"

SoundSourceEstimation::SoundSourceEstimation()
{
	pan=0.0;
	tilt=0.0;

	map[0] = 1.5;//1.1;
	map[1] = 0.0; 
	map[2] = 0.0;
	map[3] = 0.0; 
	map[4] = 3.57;
	map[5] = -5.50; //-5.50;

}

int SoundSourceEstimation::set_map(double new_map[])
{
	for(int i=0;i<6;i++)
		map[i]=new_map[i];

	return 0;
}

SoundSourceEstimation::~SoundSourceEstimation()
{

}

int SoundSourceEstimation::control(double ITD, double notch)
{
	double position[2], limits[4];
	double features[2];

	features[0]=ITD;
	features[1]=notch;
	position[0]=pan;
	position[1]=tilt;
	pos.Position_Finder(features, map, position, limits);
	pan=position[0];
	tilt=position[1];

	return 0;
}


double SoundSourceEstimation::get_pan()
{
	return pan;
}

double SoundSourceEstimation::get_tilt()
{
	return tilt;
}
