/**
iCubHeadKinematics - kinematics of the iCub head

 @author Manuel Lopes  macl@isr.ist.utl.pt
*/
// iCubHeadKinematics.h: interface for the iCubHeadKinematics class.
//
//////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <string>
#include <iostream>
using namespace std;



#include <iCub/kinematics/robmatrix.h>
#include <iCub/kinematics/robot.h>

#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>


using namespace yarp::sig;



#if !defined(_iCubHeadKinematics_H_)
#define _iCubHeadKinematics_H_

namespace iCub{
	namespace contrib{
		class iCubHeadKinematics;
	}
}

using namespace iCub::contrib;

#define	pidON	1
#define	vorON	2
#define visON	4

class iCub::contrib::iCubHeadKinematics  
{
public:
	void setGazeControllerGain( gsl_matrix *K );
	void setGazeControllerGain( double FrameRate );
	int StateEstimator( float *observ, float *state, float *pertvelb, float *pertvelt);
	void setintgain( double gain) { intgain = gain; };

    /**
    forward kinematics for the left/right eye

    @param neckposition - head joints position (degrees)
	@param eye			- l - left; r - right
    @return             - RobMatrix with frame of the eye written in the neck base frame
    */
    RobMatrix fkine(const double *neckposition, char eye);
	RobMatrix fkine(const double *neckposition, int joint);
	
	void jacob0( double *pos, gsl_matrix *J);


	
	int HeadGaze(double *hgazy, double *hgelev, double *headpos);
	// This seems to have some +/- switching troubles (?) jonas 081015
	int Gaze(double *gazazy, double *gazelev, double *headpos, char weye = 'l');
	RobMatrix getRotationFromAzimuthElev( double az, double el);
    RobMatrix getRotationFromAzimuthElev2( double az, double el);
	void gazevector2azyelev( gsl_vector *gaze, double *azy, double *elev);
	//For testimg UGO kinematics
	void gazeVector2AzimuthElevation(Vector &gaze, double &azi, double &elev);

	int pred( const double *pos, double *reachable, int resolution );
	int isvaliddirection( double *desvec, const double *vec, const int size );

	iCubHeadKinematics();
	virtual ~iCubHeadKinematics();

	/**
		 opticalaxisdirection - calculates the direction of gaze and corresponding eye rotation
		 
		 @param jointangles - head joints position (degrees)
		 @param gaze		- the 3 gaze angles (azimuth, elevation and rotation)	(degrees)
	*/
	int opticalaxisdirection( const double *neckposition, double *gaze);
	

	/**
	inverse kinematics - for a known neck position it computes the position of the left eye
	to have the target point in the optical axis.

    @param targetpoint  - coordinates of the target point in the neck base frame
	@param neckposition - head joints position (degrees)
	@param eyes			- solution of the kinematics (pan,tilt)

	@return				- 0 is solution found, -1 is no solution
	*/
	int invkin_eyes(const double *targetpoint, const double *neckposition, double *eyes);

	/**
	inverse kinematics - given a desired direction measure with eyeposition it computes the 
	new eye angles to look along that direction

    @param direction	- direction in eye coordinates
	@param eyeposition	- eye joints position (degrees) for the reference direction
	@param eyes			- solution of the kinematics (pan,tilt)

	@return				- 0 is solution found, -1 is no solution
	*/
	int invkin_eyes2( const double *direction, const double *eyeposition, double *eyes );
	
	/**
	inverse kinematics - calculates the neck joint 1 and 3 in order to have the middle
	point of the eyes looking directly at the target point. At this position the eye do
	not need to tilt and the vergence will be symmetric. the user needs to give the desired
	neck swing due to the redundancy.

    @param targetpoint  - coordinates of the target point in the neck base frame
	@param neckposition - head joints position (degrees)
	@param neck			- solution of the kinematics (t0,t1,t2)

    @return				- 0 is solution found, -1 is no solution
	*/
	int invkin_neck(const double *targetpoint, const double swing, double *neck);



	/**

		controllers for head behaviors

	@param target - position of the target in azmuth elevation
	@param velocity - the computed cartesian velocities 
	*/
	double HeadGazeController( double desazy, double deselev, gsl_vector *X, gsl_vector *pert, gsl_vector *oldW, double *velocity, double *headpos, gsl_vector *inertial, double disparity, int controltype);
	double HeadSaccade( double desazy, double deselev, double *desposition, double *headpos, double state);
	


	int StateEstimator(gsl_vector *Y, gsl_vector *X, gsl_vector *U, gsl_vector *pert);


	/**
		structure Robot that includes information about the kinematics, inspired on the matlab robotics toolbox
		 Rl - corresponds to the left eye
		 Rr - corresponds to the right eye
	*/
	Robot* Rl;
	Robot* Rr;

	
	protected:	
	gsl_vector* neckvel;
	gsl_vector* eyevel;
	gsl_vector *wn;
	gsl_vector *wo;
	gsl_matrix* J;
	gsl_matrix* J45inv;
	gsl_matrix* J13inv;
	gsl_matrix* J13;
	gsl_matrix* J45;

    /**
    forward kinematics for the left eye

    @param neckposition - head joints position (degrees)
    @return             - RobMatrix with frame of the eye written in the neck base frame
    */
    RobMatrix   fkine(const double *neckposition, Robot *R);

	/**
	 matrices needed for the controllers
	 */
	gsl_matrix *A;
	gsl_matrix *B;
	gsl_matrix *C;
	
	gsl_matrix *L;

	gsl_vector *Xaz;
	gsl_vector *Xel;

	//smooth pursuit
	gsl_matrix *Ksp;
	double framerate;
	//integral term
	double intfaz;
	double intfel;
	double intgain;


};

#endif // !defined(_iCubHeadKinematics_H_)
