// kinicub.cpp : Defines the entry point for the console application.
//
#include <stdio.h>

#include <iCub/head/iCubHeadKinematics.h>
#include <iCub/arm/iCubArmKinematics.h>

#include <iCub/kinematics/auxfuncs.h>
#include <iCub/kinematics/gsl_aux.h>

using namespace iCub::contrib;

int main(int argc, char* argv[])
{
double par[]={0,0,0,0,0,0,0};

/*
iCubHeadKinematics R;

	RobMatrix T05 = R.fkine( par, 'l' );
	T05.print();

	double tp[] = {28.5039,6.9342,18.2053,1};
	double neckpos[] = {0.,0.,0.};
	double eyes[3];
	double neck[3];

	R.invkin_eyes( tp, neckpos,eyes);
	printf("\n invkin_eyes %e %e \n", eyes[0],eyes[1]);

	R.invkin_neck( tp, 0, neck);
	printf("\n invkin_neck %e %e \n", neck[0],neck[2]);

	tp[0] = 1;	tp[1] = 1;	tp[2] = 1;
	R.invkin_eyes2( tp, neckpos,eyes);
	printf("\n invkin_eyes2 %e %e \n", eyes[0],eyes[1]);

	RobMatrix RG = R.getRotationFromAzimuthElev( 45, 0);

	RG.print();

	*/

	printf( "Arm Kinematics\n");
	
	iCubArmKinematics R;
	RobMatrix T06 = R.fkine( par, 'l' );
	T06.print();

	gsl_matrix *J = gsl_matrix_calloc( 6, 7);
	R.jacob0( par, J);

	gsl_print_matrix( J, "Jacobian");

	getchar();

	gsl_matrix_free( J );

	return 0;
}
