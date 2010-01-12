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

#include <iCub/iKinFwd.h>


using namespace yarp::sig;
using namespace iKin;


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

	//for testing Ugo's classes
protected: 
	iCubEye eyeL; iCubEye eyeR; 
	iCubInertialSensor inerSens; 
public:    
	void initInertialKin(Vector &maxL, Vector &minL)
	{
		for (int j = 0; j < 3; j++) 
			inerSens.releaseLink(j);
			
		iKinChain* inerChain = inerSens.asChain();

		int n = inerSens.getDOF();

		for(int i = 0; i < n; i++)
		{
			(*inerChain)[i].setMin((M_PI/180.0)*minL(i));
			(*inerChain)[i].setMax((M_PI/180.0)*maxL(i));
		}
	}
	void initEyeKin(Vector &maxL, Vector &minL)
	{
		for (int j = 0; j < 3; j++) {
			eyeL.releaseLink(j);
			eyeR.releaseLink(j);
		}

		iKinChain* eyeLChain = eyeL.asChain();
		iKinChain* eyeRChain = eyeR.asChain();
		
		for(int i = 0; i < 8; i++)
		{
			(*eyeLChain)[i].setMin((M_PI/180.0)*minL(i));
			(*eyeRChain)[i].setMin((M_PI/180.0)*minL(i));
			(*eyeLChain)[i].setMax((M_PI/180.0)*maxL(i));
			(*eyeRChain)[i].setMax((M_PI/180.0)*maxL(i));
		}
	};
	Matrix getNeck2WaistTransf(double *torsoEncoders )
	{
		Matrix tNeck2Waist;
		Vector torsoData(6);  //will use the inertial sensor kinematic chain
		// torsoEncoders in degrees
		// torsoData units shall be in radians
		// remind that the torso is in reverse order:
		// their joints are sent assuming the neck as kinematic origin
		// and not the waist, hence we've got to invert them!
		torsoData[0]=(M_PI/180.0)*torsoEncoders[2];	
		torsoData[1]=(M_PI/180.0)*torsoEncoders[1];
		torsoData[2]=(M_PI/180.0)*torsoEncoders[0];
		//the remaining ones no not matter- can be set to zero
		torsoData[3] = 0.0;
		torsoData[4] = 0.0;
		torsoData[5] = 0.0;
		inerSens.setAng(torsoData);
		tNeck2Waist = inerSens.getH(2);
		return tNeck2Waist;
	}

	Matrix getCyclopPose(Vector &cyclopData)
	{
		eyeL.setAng(cyclopData);
		return eyeL.getH();
	}

	//Using Craig's formulae
	Matrix getRotationMatrixFromRollPitchYawAngles(double roll, double pitch, double yaw)
	{
		double gamma = roll*(M_PI/180);
		double beta = pitch*(M_PI/180);
		double alpha = yaw*(M_PI/180);
		double ca = cos(alpha);
		double cb = cos(beta);
		double cg = cos(gamma);
		double sa = sin(alpha);
		double sb = sin(beta);
		double sg = sin(gamma);
		Matrix R(3,3);
		R(0,0) = ca*cb; 
		R(0,1) = ca*sb*sg-sa*cg;
		R(0,2) = ca*sb*cg-sa*sg;
		R(1,0) = sa*cb;
		R(1,1) = sa*sb*sg+ca*cg;
		R(1,2) = sa*sb*cg-ca*sg;
		R(2,0) = -sb;
		R(2,1) = cb*sg;
		R(2,2) = cb*cg;
		return R;
	}
	void getRollPitchYawAnglesFromRotationMatrix( Matrix &R, double &roll, double &pitch, double &yaw )
	{
		double gamma = atan2(R(2,1),R(2,2));
		double beta = atan2(-R(2,0), sqrt(R(0,0)*R(0,0)+R(1,0)*R(1,0)));
		double alpha = atan2(R(1,0),R(0,0));
		roll = gamma*(180/M_PI);
		pitch = beta*(180/M_PI);
		yaw = alpha*(180/M_PI);
	}
	void getHeadRollPitchYawAnglesWRTWaist(double *head_encoders, double *torso_encoders, double &roll, double &pitch, double &yaw)
	{
		Vector headData(8);
		// units shall be in radians
		// remind that the torso is in reverse order:
		// their joints are sent assuming the neck as kinematic origin
		// and not the waist, hence we've got to invert them!
		headData[0]=(M_PI/180.0)*torso_encoders[2];	
		headData[1]=(M_PI/180.0)*torso_encoders[1];
		headData[2]=(M_PI/180.0)*torso_encoders[0];
		// neck part
		headData[3]=(M_PI/180.0)*head_encoders[0];
		headData[4]=(M_PI/180.0)*head_encoders[1];
		headData[5]=(M_PI/180.0)*head_encoders[2];
		// set the joints
		inerSens.setAng(headData);
		//Get Trasformation matriz
		yarp::sig::Matrix headH = inerSens.getH();
		//Get rotation submatrix
		Matrix headR = headH.submatrix(0,2,0,2);
		//Get roll, pitch, yaw angles
		double temp;
		getRollPitchYawAnglesFromRotationMatrix(headR, temp, pitch, yaw);		
	}
};

#endif // !defined(_iCubHeadKinematics_H_)
