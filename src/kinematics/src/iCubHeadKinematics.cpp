/**
 iCubHeadKinematics.cpp: 

*/
#include <stdio.h>
#include <string>
#include <iostream>

using namespace std;

#include <iCub/head/iCubHeadKinematics.h>
#include <iCub/kinematics/auxfuncs.h>
#include <iCub/kinematics/gsl_aux.h>

#include <yarp/sig/Matrix.h>

/**
	Construct the kinematic structure using modified Denavitt-Hartenberg parameters.
	Compatible with Matlab robotics toolbox.
*/
iCubHeadKinematics::iCubHeadKinematics()
{
// the limits should be read from a configuration file
// and maybe also the geometry

  Rl = new Robot(5);

  Rl->SetJoint(0,	0,			0,		0,		0, -20 * M_PI/180, 15 * M_PI/180);
  Rl->SetJoint(1,	M_PI/2,		33.0,	M_PI/2,	0, -45 * M_PI/180, 45 * M_PI/180);
  Rl->SetJoint(2,	M_PI/2,		0,		M_PI/2,	82.5, -40 * M_PI/180, 40 * M_PI/180);
  Rl->SetJoint(3,	M_PI/2,		54.0,	0,		34.0, -24 * M_PI/180, 24 * M_PI/180);
  Rl->SetJoint(4,	M_PI/2,	    0.0,	0,		0, -24 * M_PI/180, 24 * M_PI/180);

  Rr = new Robot(5);

  Rr->SetJoint(0,	0,			0,		0,		0, -20 * M_PI/180, 15 * M_PI/180);
  Rr->SetJoint(1,	M_PI/2,		33.0,	M_PI/2,	0, -45 * M_PI/180, 45 * M_PI/180);
  Rr->SetJoint(2,	M_PI/2,		0,		M_PI/2,	82.5, -40 * M_PI/180, 40 * M_PI/180);
  Rr->SetJoint(3,	M_PI/2,		54.0,	0,		-34.0, -24 * M_PI/180, 24 * M_PI/180);
  Rr->SetJoint(4,	M_PI/2,	    0.0,	0,		0, -24 * M_PI/180, 24 * M_PI/180);

  // dynamics for the controllers
  A = gsl_matrix_calloc( 7, 7);
  B = gsl_matrix_calloc( 7, 2);
  C = gsl_matrix_calloc( 2, 7);

  L = gsl_matrix_calloc( 7, 2);
  
  gsl_matrix_set( L, 4, 0, 1);
  gsl_matrix_set( L, 5, 0, 1);
  gsl_matrix_set( L, 6, 0, 1);

  Xaz = gsl_vector_calloc( 3);
  Xel = gsl_vector_calloc( 3);

  double T = 0.05;

  gsl_matrix_set_identity( A );
  gsl_matrix_set( A, 0, 1, T);
  gsl_matrix_set( A, 1, 1, -0*T+1);
  gsl_matrix_set( A, 2, 3, T);
  gsl_matrix_set( A, 3, 3, -0*T+1);
  gsl_matrix_set( A, 4, 1, -T);
  gsl_matrix_set( A, 4, 3, -T);
  
  gsl_matrix_set( A, 4, 5, -1*T);
  gsl_matrix_set( A, 4, 6, 1*T);

  gsl_matrix_set( B, 1, 0, 1*T);
  gsl_matrix_set( B, 3, 1, 1*T);

  gsl_matrix_set( C, 0, 4, 1);
  gsl_matrix_set( C, 1, 3, 1);

	neckvel = gsl_vector_calloc( 3 );
	eyevel = gsl_vector_calloc( 2 );

	wn = gsl_vector_calloc( 3 );
	wo = gsl_vector_calloc( 3 );
	J = gsl_matrix_calloc( 6, Rl->Getm_njoints());
	J45inv = gsl_matrix_calloc( 2, 3);
	J13inv = gsl_matrix_calloc( 3, 3);
	J13 = gsl_matrix_calloc( 3, 3);
	J45 = gsl_matrix_calloc( 3, 2);

	Ksp = gsl_matrix_calloc( 2, 3);
/*
	gsl_matrix_set( Ksp, 0, 0, 0.5105);
	gsl_matrix_set( Ksp, 0, 2, -2.4334);
	gsl_matrix_set( Ksp, 1, 0, -0.2672);
	gsl_matrix_set( Ksp, 1, 2, -0.5105);
*/
	gsl_matrix_set( Ksp, 0, 0, 0.5154);
	gsl_matrix_set( Ksp, 0, 2, -1.0630);
	gsl_matrix_set( Ksp, 1, 0, -0.3028);
	gsl_matrix_set( Ksp, 1, 2, -0.7274);

	//initialize the integral term
	intfaz = 0;
	intfel = 0;
}

iCubHeadKinematics::~iCubHeadKinematics()
{
	delete Rl;
	delete Rr;

	gsl_matrix_free( A );
	gsl_matrix_free( B );
	gsl_matrix_free( C );

	gsl_matrix_free( L );

	gsl_vector_free( Xaz );
	gsl_vector_free( Xel );

	gsl_matrix_free( Ksp );

	gsl_matrix_free(J);
	gsl_matrix_free(J13);
	gsl_matrix_free(J45);
	gsl_matrix_free(J13inv);
	gsl_matrix_free(J45inv);
	gsl_vector_free(neckvel);
	gsl_vector_free(eyevel);
	gsl_vector_free(wn);
	gsl_vector_free(wo);
}



int iCubHeadKinematics::invkin_neck(const double *targetpoint,const double swing, double *neck)
{
double a,b,c;
double t1,t3;
double s[2];
int ret = -1;
double x,y,z;
double t2;

	// angle conversion
	t2 = swing * M_PI/180;
	
	x = targetpoint[0];	y = targetpoint[1];	z = targetpoint[2];

	a = cos(t2)*x; b = cos(t2)*y; c = 1.*sin(t2)*z + 2.5 + 2.5*cos(t2);

	if (myasbccsol( a, b, c, s))
		return ret;
	else
		ret = 0;

	t1 = s[0]; // there are two solutions and here we select one

	a = (cos(t2)*z+x*cos(t1)*sin(t2)+y*sin(t1)*sin(t2)-2.5*sin(t2));
	b = (-1.*y*cos(t1)+x*sin(t1));
	t3 = atan( -a/b );

	// angle conversion
	neck[0] = t1 * 180/M_PI;	neck[1] = t2 * 180/M_PI;	neck[2] = t3 * 180/M_PI;

	return ret;
}


int iCubHeadKinematics::invkin_eyes(const double *targetpoint, const double *neckposition, double *eyes)
{
	// angle conversion
    double auxneckposition[6];
	for(int cnt=0;cnt<5;cnt++)
	  auxneckposition[cnt] = neckposition[cnt]*M_PI/180;

	
	RobMatrix T34 = *Rl->frametransf( 3, 0);
	RobMatrix T45 = *Rl->frametransf( 4, 0);
	RobMatrix T35 = T34 * T45;
	
	
	gsl_vector *P3ORG5 = T35.P();

	gsl_print_vector( P3ORG5, "P3ORG5");

	RobMatrix T03 = Rl->fk(auxneckposition,3);

	gsl_matrix *T03inv = RobMatrix2Gsl( &T03.Inv() );

	gsl_print_matrix( T03inv, "T03inv");

	//P3 =  T03^(-1) * Ps;
	gsl_vector *P3 = gsl_vector_alloc( 4 );
	gsl_vector *tp = gsl_vector_calloc( 4 );
	gsl_vector_set(tp, 0, targetpoint[0]);gsl_vector_set(tp, 1, targetpoint[1]);
	gsl_vector_set(tp, 2, targetpoint[2]);gsl_vector_set(tp, 3, 1);

	gsl_blas_dgemv ( CblasNoTrans, 1, T03inv, tp, 0, P3);

	gsl_print_vector( P3, "P3");
	//dir = P3 - P3ORG5;
	gsl_vector_sub ( P3, P3ORG5);

	gsl_print_vector( P3, "P3b");

 eyes[0] = atan(-gsl_vector_get(P3,2)/gsl_vector_get(P3,0));

 double aux = sin( eyes[0] ) * gsl_vector_get(P3,2) - cos(eyes[0])  * gsl_vector_get(P3,0);
 
 eyes[1] = atan( -gsl_vector_get(P3,1)/aux );

 // angle conversion
 eyes[0] = eyes[0] * 180/M_PI;
 eyes[1] = eyes[1] * 180/M_PI;

 gsl_vector_free( P3ORG5 );
 gsl_matrix_free( T03inv );

 return 0;
}

int iCubHeadKinematics::opticalaxisdirection(const double *neckposition, double *gaze)
{

	// angle conversion
    double auxneckposition[6];
	for(int cnt=0;cnt<5;cnt++)
	  auxneckposition[cnt] = neckposition[cnt]*M_PI/180;

    cout << "j0" << auxneckposition[0] << "j1" << auxneckposition[1] << "j2" << auxneckposition[2] << "j3" << auxneckposition[3] << endl;

	RobMatrix *T05 = &Rl->fk( auxneckposition );
    T05->print();
	double d = sqrt( T05->M[1][0] * T05->M[1][0] +  T05->M[1][1] * T05->M[1][1]);

	gaze[0] = atan2( -T05->M[1][2], d);

	//check if gaze[0] is around 90 todo

	double f = cos( gaze[0] );

	gaze[1] = atan2( T05->M[0][2]/f, T05->M[2][2]/f);

	gaze[2] = atan2( T05->M[1][0]/f, T05->M[1][1]/f);

	// angle conversion
	gaze[0] = gaze[0] * 180/M_PI;	gaze[1] = gaze[1] * 180/M_PI;    gaze[2] = gaze[2] * 180/M_PI;
 
	return 0;

}


RobMatrix iCubHeadKinematics::fkine(const double *neckposition, Robot *R)
{
	// angle conversion
    double auxneckposition[6];

	for(int cnt=0;cnt<5;cnt++)
	  auxneckposition[cnt] = neckposition[cnt]*M_PI/180;

	RobMatrix T05 = R->fk( auxneckposition );

/*
	Matrix M = Matrix( 4, 4);

	for(int cl=0;cl<4;cl++)
		for(int cc=0;cc<4;cc++)
			M(cl,cc) = T05.M[cl][cc];
    
    return M;
*/
	return T05;
}

RobMatrix   iCubHeadKinematics::fkine(const double *neckposition, char eye)
{
	double neckposition2[6];
	double aux;
	for(int cnt = 0;cnt<6;cnt++)
		neckposition2[cnt]=neckposition[cnt];

	aux=neckposition2[4];
	neckposition2[4]=neckposition2[5];
	neckposition2[5]=aux;

	if( eye == 'l' )
		return fkine( neckposition2, Rl);
	else if( eye == 'r' )
		return fkine( neckposition, Rr);
	else 
		printf( " ERROR!!! valid options are 0 - left eye and 1 - right eye\n\n" );

	return RobMatrix( 0 );
}

int iCubHeadKinematics::invkin_eyes2(const double *direction, const double *eyeposition, double *eyes)
{

	RobMatrix T34 = *Rl->frametransf( 3, eyeposition[0]);
	RobMatrix T45 = *Rl->frametransf( 4, eyeposition[1]);
	RobMatrix T35 = T34 * T45;

	double dirmod;
	dirmod = sqrt(direction[0]*direction[0]+direction[1]*direction[1]+direction[2]*direction[2]);

	RobMatrix V5 = RobMatrix(direction[0]/dirmod,direction[1]/dirmod,direction[2]/dirmod);
	V5.M[3][3] = 0;

	RobMatrix V3 = T35.Inv() * V5;

	eyes[0] = atan2( V3.M[2][3], fabs( V3.M[0][3] ) );

	double aux = -sin( eyes[0] ) * V3.M[2][3] - cos( eyes[0] ) * V3.M[0][3];

	eyes[1] = atan2( -V3.M[1][3], -aux);

	eyes[0] = eyes[0] * 180/M_PI;
	eyes[1] = eyes[1] * 180/M_PI;

	return 1;
}

void iCubHeadKinematics::gazevector2azyelev(gsl_vector *gaze, double *azy, double *elev)
{
double x,y,z;
	
	x = gsl_vector_get( gaze, 0);
	y = gsl_vector_get( gaze, 1);
	z = gsl_vector_get( gaze, 2);

	*elev = atan2( x, sqrt( y*y + z*z) );
	*elev = *elev * 180/M_PI;

	//This makes the head turn as a poltergeist
	//*azy = atan2( -y, -z) - M_PI/2;
	//*azy = - *azy * 180/M_PI;
	*azy = atan2( -z, -y);
	*azy = *azy * 180/M_PI;



	//cout << "X:" << x << " Y:" << y << " Z:" << z << " E:" << *elev << " A:" << *azy << endl; 
}

RobMatrix iCubHeadKinematics::fkine(const double *neckposition, int joint)
{

	if (joint == -1)
		return fkine( neckposition, 'l' );
	else if(joint < Rl->Getm_njoints() )
	{
		// angle conversion
		double auxneckposition[6];

		for(int cnt=0;cnt<5;cnt++)
		  auxneckposition[cnt] = neckposition[cnt]*M_PI/180;

		RobMatrix T = Rl->fk( auxneckposition, joint );

		return T;
	}
	else
		return RobMatrix( 0 );
}

	
double iCubHeadKinematics::HeadSaccade( double desazy, double deselev, double *desposition, double *headpos, double error)
{
double err = error;
int state = 0;

desazy = desazy * M_PI/180;
deselev = deselev * M_PI/180;

if(err == 1000)
	state = 0;
else if(err == 30)
	state = 1;
else if(err == 20)
	state = 2;

	switch( state ) {
		case 0:
			{
			double eyes[2];
			double direction[3];
	
				// without this it works as incremental
				double azy ,elev;
				Gaze( &azy, &elev, headpos);
				azy *= M_PI/180;		elev *= M_PI/180;
				desazy = desazy - azy;
				deselev = deselev - elev;

				direction[0] = cos(desazy)*cos(deselev);
				direction[1] = -sin(desazy)*cos(deselev);
				direction[2] = -sin(deselev);
				double eyeposition[2];
				eyeposition[0] = headpos[3] * M_PI/180;	eyeposition[1] = headpos[4] * M_PI/180;

				invkin_eyes2( direction, eyeposition, eyes );

				//output
				desposition[0] = headpos[0];
				desposition[1] = headpos[1];
				desposition[2] = headpos[2];

				desposition[3] = eyes[0];
				desposition[4] = eyes[1];
				desposition[5] = eyes[1];

				err = (desazy*desazy + deselev*deselev);
			}
			break;
/*
		case 1:
			{
			double neck[3];
			double direction[3];
			
				direction[0] = 1000* sin(deselev);
				direction[1] = -1000* cos(desazy)*cos(deselev);
				direction[2] = -1000* sin(desazy)*cos(deselev);

				if( invkin_neck( direction, 0, neck) )
					printf("NO SOLUTION FOUND FOR THE NECK\n");

				//desposition[0] = neck[0];	desposition[1] = 0;	desposition[2] = neck[2];

				desposition[3] = 0;	desposition[4] = 0;	desposition[5] = 0;

				err = 20;

				desposition[0] = headpos[0];
				desposition[1] = headpos[1];
				desposition[2] = headpos[2];
				desposition[3] = headpos[3];
				desposition[4] = headpos[4];

			}
			break;
*/

		default:
			printf("\n error wrong HEADSACCADE\n");
			break;
	}

	printf("saccade state %d error %f\n", state, err);
	
	return err;
}


double iCubHeadKinematics::HeadGazeController( double desazy, double deselev,
										  gsl_vector *X, gsl_vector *pert, gsl_vector *oldW,
										  double *velocity, double *headpos, gsl_vector *inertial,
										  double disparity, int controltype)
{
double eye_azy,eye_elev, eye_azy_oe,eye_elev_oe,neck_azy,neck_elev;
gsl_vector *Uaz;
gsl_vector *Uel;

// angle conversion
double headpos_rad[6];


	Uaz= gsl_vector_calloc( 2 );	Uel= gsl_vector_calloc( 2 );

	HeadGaze( &neck_azy, &neck_elev, headpos);
	Gaze( &eye_azy, &eye_elev, headpos);
	Gaze( &eye_azy_oe, &eye_elev_oe, headpos);

	// the robot class uses radians
for(int cnt=0;cnt<5;cnt++)
  headpos_rad[cnt] = headpos[cnt]*M_PI/180;

	headpos_rad[5] = headpos_rad[4];
	
	//compute jacobian	
	Rl->Jacob0( headpos_rad, J);
	
	gsl_copy_submatrix( J13, J, 3, 0, 3, 3);
	gsl_copy_submatrix( J45, J, 3, 3, 3, 2);

	gsl_matrix_pseudoinv( J13, J13inv, 0.00001);
	gsl_matrix_pseudoinv( J45, J45inv, 0.00001);

	//controller
	gsl_vector_set( Xaz, 0, (eye_azy + eye_azy_oe)/2 - neck_azy); //to guarantee symetric vergence
	gsl_vector_set( Xaz, 1, neck_azy);
	gsl_vector_set( Xaz, 2, desazy - eye_azy );
	
	gsl_vector_set( Xel, 0, (eye_elev + eye_elev_oe)/2-neck_elev);
	gsl_vector_set( Xel, 1, neck_elev);
	gsl_vector_set( Xel, 2, deselev - eye_elev );

	//integral term
	if( controltype & pidON ) {
		intfaz += gsl_vector_get( Xaz, 2) * 0.01;
		if(intfaz>10)
			intfaz = 10;
		else if(intfaz<-10)
			intfaz = -10;

		intfel += gsl_vector_get( Xel, 2) * 0.01;
		if(intfel>10)
			intfel = 10;
		else if(intfel<-10)
			intfel = -10;
	}

	//for azy
	gsl_vector_memcpy( X, Xaz);
	gsl_vector_set( X, 2, gsl_vector_get( Xaz, 2) + intgain * intfaz);
	gsl_blas_dgemv( CblasNoTrans, -framerate, Ksp, X, 0, Uaz);

	//for elev
	gsl_vector_memcpy( X, Xel);
	gsl_vector_set( X, 2, gsl_vector_get( Xel, 2) + intgain * intfel);
	gsl_blas_dgemv( CblasNoTrans, -framerate, Ksp, X, 0, Uel);

	//desired neck and eye velocities, written in base coordinates

	gsl_vector_set_zero( wn );
	gsl_vector_set_zero( wo );

	// USE VISION
	// USE VISION
	// USE VISION
	if( controltype & visON )
	{
		gsl_vector_set( wn, 0, gsl_vector_get( Uaz, 1) );
		gsl_vector_set( wn, 2, gsl_vector_get( Uel, 1) );

		gsl_vector_set( wo, 0, gsl_vector_get( Uaz, 0) );
		gsl_vector_set( wo, 2, gsl_vector_get( Uel, 0) );
	}

	//INERTIAL
	//INERTIAL
	//INERTIAL
	//INERTIAL
	//compensate for the body motion obtained from the inertial sensor
	
	if( controltype & vorON )	// if compensate
	{

		//convert inertial measure to base ^0_3R * inertial
		RobMatrix inertial3 = RobMatrix( gsl_vector_get(inertial,0),
							gsl_vector_get(inertial,1),
							gsl_vector_get(inertial,2),
							0);
		RobMatrix T03 = Rl->fk( headpos_rad, 3);
		RobMatrix inertialm0 = T03 * inertial3;
		
		gsl_vector *inertial0 = inertialm0.getvector(0,3,3);

		gsl_vector_sub( inertial0, oldW);
		gsl_vector_sub( wo, inertial0);
			
		gsl_vector_free( inertial0 );
	}
	
	// conversion to joint angles
	
	gsl_blas_dgemv ( CblasNoTrans, 1, J13inv, wn, 0.0, neckvel);
	gsl_vector_set(neckvel, 1, 0); // no swing
	gsl_blas_dgemv ( CblasNoTrans, 1, J45inv, wo, 0.0, eyevel);

	//gsl_print_matrix( J45inv, "J45inv");
	//gsl_print_vector( wo, "wo");
	//gsl_print_vector( wn, "wn");

	gsl_blas_dgemv ( CblasNoTrans, 1, J13, neckvel, 0.0, oldW);

	//gsl_print_vector( Xaz, "Xaz");	gsl_print_vector( Xel, "Xel");
	//gsl_print_vector( Uaz, "Uaz (wo, wn)");	gsl_print_vector( Uel, "Uel (wo, wn)");
	//gsl_print_vector( eyevel, "eyevel");	gsl_print_vector( neckvel, "neckvel");
	
	velocity[0] = gsl_vector_get(neckvel, 0);
	velocity[1] = gsl_vector_get(neckvel, 1);
	velocity[2] = gsl_vector_get(neckvel, 2);
	velocity[3] = gsl_vector_get(eyevel, 0);

double vergcontrol;
//double eye_azy_oe, eye_elev_oe; 	Gaze( &eye_azy_oe, &eye_elev_oe, headpos, 'r');
	
	vergcontrol = 80 * disparity;

	//CHANGE WHEN BOARDS CHANGE
	// this works with separated eyes (as in JrKerr driver) NOT THE iCub STANDARD
	//velocity[4] = gsl_vector_get(eyevel, 1) + vergcontrol/2;
	//velocity[5] = gsl_vector_get(eyevel, 1) - vergcontrol/2;

    // DANGEROUS HACK JUST FOR VERGENCE -- ALEX 21/07/07 
    //velocity[4] = vergcontrol/2;
	//velocity[5] = -vergcontrol/2;
    	
	
	//this works with the standard boards
	velocity[4] = gsl_vector_get(eyevel, 1);
	//velocity[5] = vergcontrol;
	velocity[5] = 0;
	
	gsl_vector_free( Uaz );
	gsl_vector_free( Uel );

	double err = ( gsl_vector_get(Xaz, 2)*gsl_vector_get(Xaz, 2) + gsl_vector_get(Xel, 2) * gsl_vector_get(Xel, 2));

	return err;
}


int iCubHeadKinematics::StateEstimator(float *observ, float *state, float *pertvelb, float *pertvelt)
{


	return 0;
}

int iCubHeadKinematics::StateEstimator(gsl_vector *Y, gsl_vector *X, gsl_vector *U, gsl_vector *pert)
{
gsl_vector *bu;
gsl_vector *lycx;
gsl_vector *ycx;
gsl_vector *ax;
	
	bu   = gsl_vector_calloc( 7 );
	lycx = gsl_vector_calloc( 7 );
	ycx = gsl_vector_calloc( 2 );
	ax = gsl_vector_calloc( 7 );

   	// bu = B U
	gsl_blas_dgemv ( CblasNoTrans, 1, B, U, 0, bu);

	gsl_print_vector( bu, "bu");
	gsl_print_matrix( B, "B");
	gsl_print_vector( U, "U");
	gsl_print_matrix( C, "C");
	gsl_print_vector( Xaz, "Xaz");
	gsl_print_vector( X, "X");
	gsl_print_matrix( L, "L");
	// lycx = L(Y-CX)
	gsl_blas_dgemv ( CblasNoTrans, -1, C, Xaz, 0, ycx);
	gsl_vector_add( ycx, Y);
	gsl_blas_dgemv ( CblasNoTrans, 1, L, ycx, 0, lycx);
	
	// bu = BU + L(Y-CX)
	gsl_vector_add( bu, lycx);

	
    // X = A X
	gsl_blas_dgemv ( CblasNoTrans, 1, A, Xaz, 0, Xaz);

	//X = AX + BU + L(Y-CX)
	gsl_vector_add( Xaz, bu);

	gsl_vector_set( pert, 0, gsl_vector_get( Xaz, 5) );
	gsl_vector_set( pert, 1, gsl_vector_get( Xaz, 6) );

	gsl_vector_free( ax );
	gsl_vector_free( lycx );
	gsl_vector_free( ycx );
	gsl_vector_free( bu );

	gsl_vector_memcpy( X, Xaz);

	return 0;

}


int iCubHeadKinematics::HeadGaze(double *hgazy, double *hgelev, double *headpos)
{
double headpos_rad[6];
for(int cnt=0;cnt<5;cnt++)
  headpos_rad[cnt] = headpos[cnt]*M_PI/180;


	RobMatrix T03 = Rl->fk( headpos_rad, 3);
	//T03.print();

	gsl_vector *neckgaze;
	neckgaze = T03.getvector(0,0,3);
	
	gazevector2azyelev( neckgaze, hgazy, hgelev);

	gsl_vector_free( neckgaze );

	return 0;
}
int iCubHeadKinematics::Gaze(double *gazazy, double *gazelev, double *headpos, char weye)
{
	
	double aux;
	aux = headpos[5];
	//we need this because the eyes are coupled
	headpos[5]=headpos[4];
	RobMatrix T05 = fkine( headpos, weye );
	headpos[5] = aux;

	gsl_vector *eyegaze;
	eyegaze = T05.getvector(0,0,3);

	gazevector2azyelev( eyegaze, gazazy, gazelev);

	gsl_vector_free( eyegaze );

	return 0;
}

RobMatrix iCubHeadKinematics::getRotationFromAzimuthElev( double az, double el)
{

	az = az * M_PI/180;
	el = el * M_PI/180;
/*
	RobMatrix R = RobMatrix(
							cos(el), sin(el)*cos(az), sin(el)*sin(az),
							-sin(el), cos(el)*cos(az), cos(el)*sin(az),
							0,        -sin(az),         cos(az) );
*/

	RobMatrix R =                         RobMatrix( cos(el)*cos(az),         -sin(az), -sin(el)*cos(az),
							 cos(el)*sin(az),          cos(az), -sin(el)*sin(az),
							 sin(el),                  0,        cos(el) );

	return R;
}

/**
 * getRotationFromAzimuthElev is not doing what we want, therefore we created a solution:
 * 
 * Computes the transformation of camera frame of reference rotated by azimuth around z, and elevation around y.
 * This is suited to represent azimuth and elevation in a camera frame where x is pointed foward, y to the right, and z down (representation of the camera frames in the iCubHeadKinematics modules)
 * Elevation is UP
 * Azimuth is RIGHT
 * 
 * 
 */
RobMatrix iCubHeadKinematics::getRotationFromAzimuthElev2( double az, double el)
{

    az = az * M_PI/180;
    el = el * M_PI/180;

    RobMatrix R = RobMatrix( cos(el)*cos(az),          -sin(az),  sin(el)*cos(az),
                             cos(el)*sin(az),          cos(az),    sin(el)*sin(az),
                            -sin(el),                  0,        cos(el)        );

    return R;
}


void iCubHeadKinematics::setGazeControllerGain(double FrameRate)
{
	framerate = FrameRate;
}

void iCubHeadKinematics::setGazeControllerGain(gsl_matrix *K)
{
	gsl_matrix_memcpy( Ksp, K);

}

int iCubHeadKinematics::pred( const double *pos, double *reachable, int resolution )
{
double pos_rad[6];
double daz,del,ddir;
double g;
int cnt;

gsl_vector *w = gsl_vector_calloc( 3 );
gsl_vector *nvel = gsl_vector_calloc( 3 );
gsl_vector *evel = gsl_vector_calloc( 2 );
gsl_vector *j = gsl_vector_calloc( 6 );

for(cnt=0;cnt<6;cnt++)
  pos_rad[cnt] = pos[cnt]*M_PI/180;
	
	Rl->Jacob0( pos_rad, J);

	gsl_copy_submatrix( J13, J, 3, 0, 3, 3);
	gsl_matrix_pseudoinv( J13, J13inv, 0.00001);

	gsl_copy_submatrix( J45, J, 3, 3, 3, 2);
	gsl_matrix_pseudoinv( J45, J45inv, 0.00001);

	//printf( "reach dir > ");

	for(cnt = 0; cnt < resolution; cnt++)
	{
		ddir = cnt * 45 * M_PI / 180;
		
		daz = sin( -ddir );
		del = cos( ddir );
	
		gsl_vector_set( w, 0, daz );
		gsl_vector_set( w, 1, 0 );
		gsl_vector_set( w, 2, del );

		gsl_blas_dgemv ( CblasNoTrans, 1, J13inv, w, 0.0, nvel);
		gsl_vector_set(nvel, 1, 0); // no swing
		gsl_blas_dgemv ( CblasNoTrans, 1, J45inv, w, 0.0, evel);

		g = 0.1;

		gsl_vector_set( j, 0, pos_rad[0] + g * gsl_vector_get(nvel, 0));
		gsl_vector_set( j, 1, pos_rad[1] + g * gsl_vector_get(nvel, 1));
		gsl_vector_set( j, 2, pos_rad[2] + g * gsl_vector_get(nvel, 2));
		gsl_vector_set( j, 3, pos_rad[3] + g * gsl_vector_get(evel, 0));
		gsl_vector_set( j, 4, pos_rad[4] + g * gsl_vector_get(evel, 1));
		gsl_vector_set( j, 5, pos_rad[5] + g * gsl_vector_get(evel, 1));

		//gsl_print_vector( j, "j");

		reachable[cnt] = Rl->ValidPosition( j );
		//printf( " %f ", reachable);
	}
	//printf( "\n", reachable);

	printf(" %2.0f %2.0f %2.0f \n", reachable[7], reachable[0], reachable[1]);
	printf(" %2.0f * %2.0f \n", reachable[6],  reachable[2]);
	printf(" %2.0f %2.0f %2.0f \n", reachable[5], reachable[4], reachable[3]);
	
	gsl_vector_free( w );
	gsl_vector_free( nvel );
	gsl_vector_free( evel );
	gsl_vector_free( j );

	return -1;
}

int iCubHeadKinematics::isvaliddirection( double *desvec, const double *vec, const int size )
{
	double angle;
	int desstep = 360/size;
	int desdir;

		angle = atan2( desvec[1], desvec[0]);
		desdir = (int) angle/desstep;

		return (int) vec[desdir];
}

void iCubHeadKinematics::jacob0( double *pos, gsl_matrix *J)
{
double pos_rad[7];

for(int cnt=0;cnt<7;cnt++)
  pos_rad[cnt] = pos[cnt]*M_PI/180;


	//compute jacobian	
	Rl->Jacob0( pos_rad, J);

}
