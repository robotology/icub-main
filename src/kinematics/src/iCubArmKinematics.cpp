/**
 iCubArmKinematics.cpp: 

*/
#include <stdio.h>
#include <string>
#include <iostream>

using namespace std;

#include <iCub/arm/iCubArmKinematics.h>
#include <iCub/kinematics/auxfuncs.h>
#include <iCub/kinematics/gsl_aux.h>

#include <yarp/sig/Matrix.h>

/**
	Construct the kinematic structure using modified Denavitt-Hartenberg parameters.
	Compatible with Matlab robotics toolbox.
*/
iCubArmKinematics::iCubArmKinematics()
{
  Rl = new Robot(7);

  Rl->SetJoint(0,	M_PI/2,		0,		0,			109.8);
  Rl->SetJoint(1,	-M_PI/2,	0,		-M_PI/2,	0);
  Rl->SetJoint(2,	M_PI/2,		0,		0,			-151.46);
  Rl->SetJoint(3,	M_PI/2,		-15,	0,			0);
  Rl->SetJoint(4,	M_PI/2,	    0,		0,			137.5);
  Rl->SetJoint(5,	M_PI/2,	    0,		M_PI/2,		0);
  Rl->SetJoint(6,	0,			0,		0,			0);


  Rr = new Robot(7);

  Rl->SetJoint(0,	M_PI/2,		0,		0,			109.8);
  Rl->SetJoint(1,	-M_PI/2,	0,		-M_PI/2,	0);
  Rl->SetJoint(2,	M_PI/2,		0,		0,			-151.46);
  Rl->SetJoint(3,	M_PI/2,		-15,	0,			0);
  Rl->SetJoint(4,	M_PI/2,	    0,		0,			137.5);
  Rl->SetJoint(5,	M_PI/2,	    0,		M_PI/2,		0);
  Rl->SetJoint(6,	0,			0,		0,			0);
}

iCubArmKinematics::~iCubArmKinematics()
{
	delete Rl;
	delete Rr;

}




RobMatrix iCubArmKinematics::fkine(const double *neckposition, Robot *R)
{
	// angle conversion
    double auxneckposition[7];

	for(int cnt=0;cnt<7;cnt++)
	  auxneckposition[cnt] = neckposition[cnt]*M_PI/180;

	RobMatrix T06 = R->fk( auxneckposition );


	return T06;
}

RobMatrix   iCubArmKinematics::fkine(const double *neckposition, char arm)
{
	if( arm == 'l' )
		return fkine( neckposition, Rl);
	else if( arm == 'r' )
		return fkine( neckposition, Rr);
	else 
		printf( " ERROR!!! valid options are 0 - left arm and 1 - right arm\n\n" );

	return RobMatrix( 0 );
}

RobMatrix iCubArmKinematics::fkine(const double *neckposition, int joint)
{

	if (joint == -1)
		return fkine( neckposition, 'l' );
	else if(joint < Rl->Getm_njoints() )
	{
		// angle conversion
		double auxneckposition[7];

		for(int cnt=0;cnt<7;cnt++)
		  auxneckposition[cnt] = neckposition[cnt]*M_PI/180;

		RobMatrix T = Rl->fk( auxneckposition, joint );

		return T;
	}
	else
		return RobMatrix( 0 );
}

	
void iCubArmKinematics::jacob0( double *pos, gsl_matrix *J)
{
double pos_rad[7];

for(int cnt=0;cnt<7;cnt++)
  pos_rad[cnt] = pos[cnt]*M_PI/180;


	//compute jacobian	
	Rl->Jacob0( pos_rad, J);

}
