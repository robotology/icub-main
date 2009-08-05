#include "PositionEstimator.h"	//Class header file

PositionEstimator::PositionEstimator()//Class constructor
{	
	FILE *fp;
	// Open for write 
	if( (fp = fopen( "setup_serneck.txt", "r" )) == NULL )
		printf( "The file 'setup_serneck.txt' was not opened\n" );
	else
		printf( "The file 'setup_serneck.txt' was opened\n" );
	
	fscanf(fp,"%f ",&tilt_low);
	fscanf(fp,"%f ",&tilt_high);
	fscanf(fp,"%f ",&pan_low);
	fscanf(fp,"%f ",&pan_high);
	fclose(fp);

}


PositionEstimator::~PositionEstimator()//Class Destructor
{}


void PositionEstimator::Position_Finder(double *features, double *map, double *position, double *limits)
{
	double itd, notch;
	double pan, tilt;
	itd = features[0];
	notch = features[1];
//	printf("features[0] = %0.2f	features[1] = %0.2f\n", features[0], features[1]);
//	printf("map = %.2f	%.2f	%.2f\n	%.2f	%.2f	%.2f",map[0], map[1], map[2], map[3], map[4], map[5]);

	pan = map[0]*itd + map[1]*notch/10000 + map[2];
	tilt =(0.8-sqrt(pan*pan))*(map[3]*itd + map[4]*notch/10000 + map[5])/0.8;

position[0] = pan;
position[1] = tilt;
/*
	
	
printf("\npan = %.2f	tilt = %.2f\n", pan, tilt);
////	pan = pan;
////	tilt = tilt;
printf("position_pan_old = %0.2f\n",position[0]);
position[0] = position[0] + pan;
printf("position_pan_new = %0.2f\n",position[0]);
	if (position[0]<pan_low+0.05)
		position[0]=pan_low+0.05;
	if (position[0]>pan_high-0.05)
		position[0]=pan_high-0.05;

printf("position_tilt_old = %0.2f\n",position[1]);
position[1] =  position[1] + tilt;
printf("position_tilt_new = %0.2f\n",position[1]);
	if (position[1]<tilt_low+0.05)
		position[1]=tilt_low+0.05;
	if (position[1]>tilt_high-0.05)
		position[1]=tilt_high-0.05;

*/
	limits[0] = pan_low;
	limits[1] = pan_high;
	limits[2] = tilt_low;
	limits[3] = tilt_high;

	//printf("\nnotch = %0.3f		Tilt Diff = %0.3f\n",notch,tilt);
	//printf("\nPosition[0] = %0.3f		Position[1] = %0.3f\n",position[0],position[1]);
}

